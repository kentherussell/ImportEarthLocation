# Runs the imports unattended: asks the mod for them, starts the game through Steam, waits for
# the status file to report done, then closes the game. The installed mod must be current.
# The game stays open from city to city. Only a crash, freeze or stuck import restarts it: that
# city is recorded as crashed and skipped, and the rest of the list continues.
# -All runs every city in tests/cities.csv. -Switches names design changes under trial,
# comma-separated (ImportEarthLocation/Core/DesignSwitches.cs); every city runs with them.
# -GameArguments are passed to the game; none by default. Do not pass --disablePdxSdk: the game
# stops on a fatal error screen right after it initializes (11-11). Test runs go through
# tests/autorun-minimal.ps1, which switches to the Minimal Mods Only playset and back.
# -GameProcessName, -LaunchScript and -PollSeconds exist for tests/autorun-recovery-test.ps1.
param(
    [string[]]$Cities = @('marseille', 'dc'),
    [switch]$All,
    [string]$Switches = '',
    [string]$CitiesFile = (Join-Path $PSScriptRoot 'cities.csv'),
    [int]$MaximumMinutes = 0,
    [int]$StaleHeartbeatMinutes = 20,
    [int]$CityMinutes = 180,
    [int]$MapWaitMinutes = 45,
    [int]$FailedLaunchLimit = 3,
    [int]$RelaunchSeconds = 30,
    [string]$GameProcessName = 'Cities2',
    [string]$LaunchScript = '',
    [int]$PollSeconds = 30,
    [string[]]$GameArguments = @()
)

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'autorun-notify.ps1')

# Marseille, DC and Cairo keep the centres of their earlier captures so runs stay comparable.
$coordinates = @{
    marseille = @('43.296399', '5.377789')
    dc = @('38.895098', '-77.036385')
    cairo = @('30.044388', '31.235726')
}

$listed = @(Import-Csv -LiteralPath $CitiesFile)

foreach ($row in $listed) {
    if (-not $coordinates.ContainsKey($row.City)) {
        $coordinates[$row.City] = @($row.Latitude, $row.Longitude)
    }
}

if ($All) {
    $Cities = @($listed | ForEach-Object { $_.City })
}

foreach ($city in $Cities) {
    if (-not $coordinates.ContainsKey($city)) {
        throw "No coordinates for '$city'; add it to $CitiesFile."
    }
}

if ($MaximumMinutes -le 0) {
    $MaximumMinutes = 60 + 60 * $Cities.Count
}

if (Get-Process -Name $GameProcessName -ErrorAction SilentlyContinue) {
    throw 'The game is already running.'
}

# Older runs moved the local brand packs out of the game's folder to speed up startup. The packs
# are now small enough to stay, so this only puts back any copy an interrupted older run left aside.
$gameData = Join-Path $env:USERPROFILE 'AppData\LocalLow\Colossal Order\Cities Skylines II'
$asideRoot = Join-Path $env:USERPROFILE 'AppData\LocalLow'
$setAside = @{
    (Join-Path $gameData 'BrandPacks') = Join-Path $asideRoot 'ImportEarthLocation-BrandPacks-aside'
    (Join-Path $gameData 'Unreleased Companies') = Join-Path $asideRoot 'ImportEarthLocation-UnreleasedCompanies-aside'
}

function Restore-BrandPacks {
    foreach ($folder in $setAside.Keys) {
        if ((Test-Path $setAside[$folder]) -and -not (Test-Path $folder)) {
            Move-Item -LiteralPath $setAside[$folder] -Destination $folder
        }
    }
}

Restore-BrandPacks

foreach ($folder in $setAside.Keys) {
    if (Test-Path $setAside[$folder]) {
        throw "Both $folder and $($setAside[$folder]) exist; resolve this by hand."
    }
}

$diagnostics = Join-Path $gameData 'ModsData\ImportEarthLocation\diagnostics'
$status = Join-Path $diagnostics 'autorun-status.json'
$request = Join-Path $diagnostics 'autorun-request.json'
$started = Get-Date
$outcome = 'timed out'
$timings = Get-AutorunTimings
$lastPost = Get-Date
$failuresPosted = @{}
$remaining = @($Cities)
$results = [System.Collections.Generic.List[object]]::new()
$failedLaunches = 0

function Send-Step([string]$Text) {
    $minutes = ((Get-Date) - $started).TotalMinutes
    $percent = Get-EstimatedPercent $minutes $Cities $timings
    Send-Discord "$Text | $([int]$minutes) min | about $percent%" -Silent
    Set-Variable -Name lastPost -Value (Get-Date) -Scope Script
}

function Start-Game {
    if ($LaunchScript) {
        & $LaunchScript
        return
    }

    if (-not (Get-Process -Name steam -ErrorAction SilentlyContinue)) {
        Start-Process 'C:\Program Files (x86)\Steam\steam.exe' -ArgumentList '-silent'
        Start-Sleep -Seconds 30
    }

    if ($GameArguments) {
        Start-Process 'C:\Program Files (x86)\Steam\steam.exe' -ArgumentList (@('-applaunch', '949230') + $GameArguments)
    }
    else {
        Start-Process 'steam://rungameid/949230'
    }
}

function Stop-Game {
    $game = Get-Process -Name $GameProcessName -ErrorAction SilentlyContinue

    if ($game) {
        [void]$game.CloseMainWindow()

        if (-not $game.WaitForExit($RelaunchSeconds * 2000)) {
            Stop-Process -Id $game.Id -Force
            [void]$game.WaitForExit(30000)
        }
    }
}

function Read-Status {
    if (-not (Test-Path $status)) {
        return $null
    }

    try {
        return Get-Content -LiteralPath $status -Raw | ConvertFrom-Json
    }
    catch {
        return $null  # The mod was replacing the file, or died doing so.
    }
}

# The game's own logger failing (UnityLogger.Internal_WriteStream) raises an error dialog that
# covers the site shots (report 112). Player.log starts over at each launch, so each city is
# charged with the occurrences added since the previous city finished. The dialog stays open
# until dismissed, so a city's shots are usable only if none occurred since the launch.
$loggerErrors = @{}
$loggerSinceLaunch = @{}
$loggerCharged = 0

function Get-LoggerErrorCount {
    $log = Join-Path $gameData 'Player.log'

    if (-not (Test-Path $log)) {
        return 0
    }

    try {
        return @(Select-String -LiteralPath $log -SimpleMatch 'UnityLogger.Internal_WriteStream').Count
    }
    catch {
        return 0  # The game was writing the file; the next poll counts it.
    }
}

function Update-LoggerErrors($Runs) {
    foreach ($run in $Runs) {
        if ($run.Result -in 'completed', 'failed' -and -not $loggerErrors.ContainsKey($run.Name)) {
            $total = Get-LoggerErrorCount
            $loggerErrors[$run.Name] = [Math]::Max(0, $total - $script:loggerCharged)
            $loggerSinceLaunch[$run.Name] = $total
            $script:loggerCharged = $total
        }
    }
}

# Keeps what a crash can tell us, since the next launch replaces the game's logs.
function Save-CrashEvidence($Run, [datetime]$Since) {
    try {
        $parent = if ($Run.RunFolder -and (Test-Path $Run.RunFolder)) { $Run.RunFolder } else { Join-Path $diagnostics "crash-$($Run.Name)" }
        $target = Join-Path $parent 'crash-evidence'
        New-Item -ItemType Directory -Force -Path $target | Out-Null

        foreach ($file in @((Join-Path $gameData 'Player.log'), (Join-Path $gameData 'Logs\ImportEarthLocation.Mod.log'))) {
            if (Test-Path $file) {
                Copy-Item -LiteralPath $file -Destination $target
            }
        }

        $crashes = Join-Path $env:TEMP 'Colossal Order\Cities Skylines II\Crashes'

        if (Test-Path $crashes) {
            Get-ChildItem -LiteralPath $crashes -Directory |
                Where-Object LastWriteTime -gt $Since |
                ForEach-Object { Copy-Item -LiteralPath $_.FullName -Destination $target -Recurse }
        }
    }
    catch {
        "Crash evidence not saved: $($_.Exception.Message)"
    }
}

try {
    while ($remaining.Count -gt 0 -and ((Get-Date) - $started).TotalMinutes -lt $MaximumMinutes) {
        if (Test-Path $status) {
            Remove-Item -LiteralPath $status -Confirm:$false
        }

        $rows = foreach ($city in $remaining) {
            [pscustomobject]@{
                name = $city
                lat = $coordinates[$city][0]
                lon = $coordinates[$city][1]
                switches = $Switches
            }
        }

        ConvertTo-Json @($rows) | Set-Content -LiteralPath $request -Encoding utf8NoBOM
        $launched = Get-Date
        $loggerCharged = 0
        $outcome = 'timed out'
        $seenGame = $false
        $waitingForMapSince = $null
        $lastStep = ''
        Start-Game
        Send-Step "Game launching: $($remaining.Count) cities to go, starting with $(Format-CityName $remaining[0])"

        while (((Get-Date) - $started).TotalMinutes -lt $MaximumMinutes) {
            Start-Sleep -Seconds $PollSeconds
            $game = Get-Process -Name $GameProcessName -ErrorAction SilentlyContinue

            if ($game) {
                if (-not $seenGame) {
                    Send-Step 'Game started, loading'
                }

                $seenGame = $true
            }
            elseif ($seenGame) {
                $outcome = 'game exited early'
                break
            }
            elseif (((Get-Date) - $launched).TotalMinutes -gt 5) {
                $outcome = 'game never started'
                break
            }

            $current = Read-Status

            if ($null -eq $current) {
                continue
            }

            Update-LoggerErrors $current.runs

            if ($current.state -eq 'failed') {
                $outcome = 'game could not open the editor'
                break
            }

            if ($current.state -eq 'starting' -and ((Get-Date) - $launched).TotalMinutes -gt 45) {
                $outcome = 'game never reached the first import'
                break
            }

            # Numbers change every update, so a step counts as new only when its words change.
            $index = [Math]::Min([int]$current.city, $remaining.Count - 1)
            $cityLabel = "$(Format-CityName $remaining[$index]) ($($results.Count + $index + 1) of $($Cities.Count))"
            $words = "$($current.phase) $($current.step)" -replace '[\d.,:%]+', '' -replace '\s+', ' '
            $words = $words.Substring(0, [Math]::Min(45, $words.Length))

            if ($current.state -ne 'starting' -and $words -ne $lastStep) {
                $lastStep = $words
                $stepText = if ($current.step) { ($current.step -split "`n")[0] } else { $current.phase }
                Send-Step "$cityLabel`: $($stepText.Substring(0, [Math]::Min(120, $stepText.Length)))"
            }
            elseif (((Get-Date) - $lastPost).TotalMinutes -ge 10) {
                Send-Step $(if ($current.state -eq 'starting') { 'Still loading the game' } else { "$cityLabel`: still working" })
            }

            foreach ($run in $current.runs) {
                if ($run.Result -eq 'failed' -and -not $failuresPosted[$run.Name]) {
                    $failuresPosted[$run.Name] = $true
                    Send-Discord "FAILED: $(Format-CityName $run.Name): $($run.Error)"
                }
            }

            if ($current.state -eq 'done') {
                $outcome = 'done'
                break
            }

            $age = (Get-Date).ToUniversalTime() - ([datetime]$current.heartbeatUtc).ToUniversalTime()

            if ($age.TotalMinutes -gt $StaleHeartbeatMinutes -and $current.phase -ne 'WaitingForMap') {
                $outcome = 'game stopped responding'
                break
            }

            # The heartbeat check above skips this phase, so a map that never loads ends here.
            if ($current.state -ne 'starting' -and $current.phase -eq 'WaitingForMap') {
                if (-not $waitingForMapSince) {
                    $waitingForMapSince = Get-Date
                }

                if (((Get-Date) - $waitingForMapSince).TotalMinutes -ge $MapWaitMinutes) {
                    $outcome = 'the next map never loaded'
                    break
                }
            }
            else {
                $waitingForMapSince = $null
            }

            # The heartbeat comes from the panel, so it keeps going while an import is stuck.
            $running = @($current.runs | Where-Object { $_.Result -eq 'running' -and $_.StartedUtc })

            if ($running.Count -gt 0) {
                $runMinutes = ((Get-Date).ToUniversalTime() - ([datetime]$running[0].StartedUtc).ToUniversalTime()).TotalMinutes

                if ($runMinutes -gt $CityMinutes) {
                    $outcome = "city passed the $CityMinutes minute limit"
                    break
                }
            }
        }

        Stop-Game

        # A request the game never read must not fire on the next normal launch.
        if (Test-Path $request) {
            Remove-Item -LiteralPath $request -Confirm:$false
        }

        $final = Read-Status
        $finishedCities = @()
        $blamed = $null

        if ($final) {
            if ($results.Count -eq 0) {
                $firstStart = $final.runs | Where-Object StartedUtc | Select-Object -First 1

                if ($firstStart) {
                    $timings.startup = ([datetime]$firstStart.StartedUtc).ToUniversalTime().Subtract($launched.ToUniversalTime()).TotalMinutes
                }
            }

            Update-LoggerErrors $final.runs

            foreach ($run in $final.runs) {
                if ($loggerErrors.ContainsKey($run.Name)) {
                    $run | Add-Member -NotePropertyName LoggerErrors -NotePropertyValue $loggerErrors[$run.Name] -Force
                    $run | Add-Member -NotePropertyName ShotsUsable -NotePropertyValue ($loggerSinceLaunch[$run.Name] -eq 0) -Force
                }

                if ($run.Result -eq 'running') {
                    $blamed = $run
                    $run.Result = 'crashed'
                    $run.Error = $outcome
                    $run.FinishedUtc = (Get-Date).ToUniversalTime()
                }

                if ($run.Result -ne 'pending') {
                    $results.Add($run)
                    $finishedCities += $run.Name
                }
            }
        }

        $remaining = @($remaining | Where-Object { $finishedCities -notcontains $_ })

        if ($outcome -eq 'done' -or $outcome -eq 'timed out') {
            break
        }

        if ($blamed) {
            Save-CrashEvidence $blamed $launched
            Send-Discord "CRASHED: $(Format-CityName $blamed.Name) skipped ($outcome). $($remaining.Count) cities left."
        }
        else {
            Send-Discord "Game failed before an import started ($outcome)."
        }

        # Only a launch that got nothing done counts: that points at the game, not a map.
        $failedLaunches = if ($finishedCities.Count -gt 0) { 0 } else { $failedLaunches + 1 }

        if ($failedLaunches -ge $FailedLaunchLimit) {
            $outcome = "$failedLaunches launches in a row failed: $outcome"
            break
        }

        if ($remaining.Count -gt 0) {
            # A crash handler still open makes Steam treat the game as running.
            Start-Sleep -Seconds $RelaunchSeconds
            Get-Process -Name UnityCrashHandler64 -ErrorAction SilentlyContinue | Stop-Process -Force
            Start-Sleep -Seconds $RelaunchSeconds
        }
        else {
            $outcome = 'done'
        }
    }
}
finally {
    Restore-BrandPacks
}

# Each launch starts a new status file, so the report reads one rebuilt from every round.
for ($i = 0; $i -lt $results.Count; $i++) {
    $run = $results[$i]

    if ($run.Result -eq 'completed' -and $run.StartedUtc) {
        $end = if ($run.FinishedUtc) { [datetime]$run.FinishedUtc } else { Get-Date }
        $timings[$run.Name] = $end.ToUniversalTime().Subtract(([datetime]$run.StartedUtc).ToUniversalTime()).TotalMinutes
    }
}

foreach ($city in $remaining) {
    $results.Add([pscustomobject]@{ Name = $city; Result = 'pending' })
}

[pscustomobject]@{ state = 'finished'; phase = 'Idle'; city = 0; runs = $results.ToArray() } |
    ConvertTo-Json -Depth 4 |
    Set-Content -LiteralPath $status -Encoding utf8NoBOM

if ($outcome -eq 'done') {
    Save-AutorunTimings $timings
}

& (Join-Path $PSScriptRoot 'autorun-report.ps1') -Outcome $outcome -Minutes ([int]((Get-Date) - $started).TotalMinutes)

if ($outcome -ne 'done') {
    exit 1
}
