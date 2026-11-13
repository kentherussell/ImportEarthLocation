# Checks autorun.ps1's crash recovery without the game. A copy of ping.exe stands in for the game
# and a small script plays the mod, writing status files and killing the "game" on cue. The user
# profile points at a temporary folder, so no real game data or Discord webhook is touched.
$ErrorActionPreference = 'Stop'
$root = Join-Path ([IO.Path]::GetTempPath()) "autorun-test-$([guid]::NewGuid().ToString('N'))"
$diagnostics = Join-Path $root 'AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\diagnostics'
New-Item -ItemType Directory -Force -Path $diagnostics | Out-Null
$fakeGame = Join-Path $root 'FakeGame.exe'
Copy-Item 'C:\Windows\System32\PING.EXE' $fakeGame
$launches = Join-Path $root 'launches.txt'

$modScript = Join-Path $root 'fake-mod.ps1'
@'
param($Diagnostics, $FakeGame)
$request = Join-Path $Diagnostics 'autorun-request.json'
$status = Join-Path $Diagnostics 'autorun-status.json'
$rows = @(Get-Content $request -Raw | ConvertFrom-Json)
Remove-Item $request
$runs = @($rows | ForEach-Object { [pscustomobject]@{ Name = $_.name; Result = 'pending'; RunFolder = $null; Error = $null; StartedUtc = $null; FinishedUtc = $null } })

function Write-Status($State, $Index) {
    [pscustomobject]@{ state = $State; phase = 'Applying'; city = $Index; step = 'working'; heartbeatUtc = (Get-Date).ToUniversalTime().ToString('o'); runs = $runs } |
        ConvertTo-Json -Depth 4 | Set-Content $status
}

if ($runs[0].Name -eq 'nostart') {
    Get-Process FakeGame | Stop-Process -Force
    return
}

for ($i = 0; $i -lt $runs.Count; $i++) {
    $run = $runs[$i]
    $run.Result = 'running'
    $run.StartedUtc = (Get-Date).ToUniversalTime().ToString('o')

    if ($run.Name -eq 'nomap') {
        $run.Result = 'pending'
        $run.StartedUtc = $null
        [pscustomobject]@{ state = 'running'; phase = 'WaitingForMap'; city = $i; step = ''; heartbeatUtc = (Get-Date).ToUniversalTime().ToString('o'); runs = $runs } |
            ConvertTo-Json -Depth 4 | Set-Content $status
        return
    }

    if ($run.Name -eq 'crash') {
        Write-Status 'running' $i
        Start-Sleep -Seconds 2
        Get-Process FakeGame | Stop-Process -Force
        return
    }

    if ($run.Name -eq 'stuck') {
        $run.StartedUtc = (Get-Date).ToUniversalTime().AddMinutes(-500).ToString('o')
        Write-Status 'running' $i
        return
    }

    $run.Result = 'completed'
    $run.FinishedUtc = (Get-Date).ToUniversalTime().ToString('o')
    Write-Status 'running' $i
    Start-Sleep -Seconds 1
}

Write-Status 'done' ($runs.Count - 1)
'@ | Set-Content -LiteralPath $modScript

$launchScript = Join-Path $root 'launch.ps1'
@"
Add-Content -LiteralPath '$launches' 'launch'
Start-Process -FilePath '$fakeGame' -ArgumentList '-n 100000 127.0.0.1' -WindowStyle Hidden
Start-Process -FilePath pwsh -ArgumentList '-NoProfile -File "$modScript" "$diagnostics" "$fakeGame"' -WindowStyle Hidden
"@ | Set-Content -LiteralPath $launchScript

function Invoke-Scenario([string[]]$Cities) {
    Remove-Item $launches -ErrorAction SilentlyContinue
    $csv = Join-Path $root 'cities.csv'
    (@('City,Country,Latitude,Longitude') + ($Cities | ForEach-Object { "$_,Test,1,1" })) | Set-Content $csv
    $realProfile = $env:USERPROFILE
    $env:USERPROFILE = $root

    try {
        $autorun = Join-Path $PSScriptRoot 'autorun.ps1'
        & pwsh -NoProfile -Command "& '$autorun' -Cities $($Cities -join ',') -CitiesFile '$csv' -GameProcessName FakeGame -LaunchScript '$launchScript' -PollSeconds 1 -RelaunchSeconds 1 -MapWaitMinutes 0; exit `$LASTEXITCODE" | Out-Null
        $code = $LASTEXITCODE
    }
    finally {
        $env:USERPROFILE = $realProfile
    }

    $finalStatus = Get-Content (Join-Path $diagnostics 'autorun-status.json') -Raw | ConvertFrom-Json

    return @{
        Code = $code
        Launches = @(Get-Content $launches).Count
        Results = @($finalStatus.runs | ForEach-Object { "$($_.Name)=$($_.Result)" }) -join ' '
    }
}

$failures = 0

function Assert-Same($Name, $Expected, $Actual) {
    if ($Expected -ne $Actual) {
        "FAIL $Name`: expected '$Expected', got '$Actual'"
        Set-Variable -Name failures -Value ($failures + 1) -Scope Script
    }
    else {
        "PASS $Name"
    }
}

try {
    $mixed = Invoke-Scenario @('one', 'crash', 'two', 'stuck', 'three')
    Assert-Same 'continues past a crash and a stuck city' 'one=completed crash=crashed two=completed stuck=crashed three=completed' $mixed.Results
    Assert-Same 'restarts only after the two failures' 3 $mixed.Launches
    Assert-Same 'batch ends as done' 0 $mixed.Code

    $clean = Invoke-Scenario @('one', 'two', 'three')
    Assert-Same 'one launch for a clean batch' 1 $clean.Launches
    Assert-Same 'clean batch ends as done' 0 $clean.Code

    $noMap = Invoke-Scenario @('one', 'nomap')
    Assert-Same 'a map that never loads ends the wait' 'one=completed nomap=pending' $noMap.Results
    Assert-Same 'stops after three launches with no progress' 4 $noMap.Launches

    $broken = Invoke-Scenario @('nostart', 'two')
    Assert-Same 'stops after three failed launches' 3 $broken.Launches
    Assert-Same 'failed batch reports failure' 1 $broken.Code
    Assert-Same 'unstarted cities stay pending' 'nostart=pending two=pending' $broken.Results
}
finally {
    Get-Process FakeGame -ErrorAction SilentlyContinue | Stop-Process -Force
    Remove-Item -LiteralPath $root -Recurse -Force -Confirm:$false
}

exit $(if ($failures -gt 0) { 1 } else { 0 })
