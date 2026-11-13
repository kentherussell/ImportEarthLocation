# Posts an autorun's outcome and each city's numbers to Discord. The webhook lives outside the
# repository, and a failed post never fails the run.
param(
    [Parameter(Mandatory)][string]$Outcome,
    [Parameter(Mandatory)][int]$Minutes,
    [string]$Title = 'Import Earth autorun'
)

. (Join-Path $PSScriptRoot 'autorun-notify.ps1')
$diagnostics = Join-Path $env:USERPROFILE 'AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\diagnostics'
$status = Join-Path $diagnostics 'autorun-status.json'
$webhook = Join-Path $env:USERPROFILE '.import-earth-discord-webhook.txt'
$icons = @{
    completed = [char]::ConvertFromUtf32(0x2705)
    failed = [char]::ConvertFromUtf32(0x274C)
    crashed = [char]::ConvertFromUtf32(0x1F4A5)
}
$waiting = [char]::ConvertFromUtf32(0x23F3)

function Format-City($run) {
    $icon = if ($icons.ContainsKey($run.Result)) { $icons[$run.Result] } else { $waiting }
    $name = Format-CityName $run.Name
    $lines = @("$icon **$name**: $($run.Result)")

    if ($run.StartedUtc -and $run.FinishedUtc) {
        $lines += "- Time: $([int]([datetime]$run.FinishedUtc - [datetime]$run.StartedUtc).TotalMinutes) min"
    }

    if ($run.Error) {
        $lines += "- Error: $($run.Error)"
    }

    if (-not $run.RunFolder -or -not (Test-Path $run.RunFolder)) {
        return $lines -join "`n"
    }

    $manifest = Join-Path $run.RunFolder 'site-shots.json'

    if (Test-Path $manifest) {
        $shots = @(Get-Content -LiteralPath $manifest -Raw | ConvertFrom-Json)
        $failedShots = @($shots | Where-Object { $_.failure }).Count
        $lines += "- Screenshots: $($shots.Count - $failedShots) of $($shots.Count)"
    }

    if ($null -ne $run.LoggerErrors) {
        $lines += if ($run.LoggerErrors -gt 0) {
            "- Game logger errors: $($run.LoggerErrors); screenshots covered by the error dialog"
        }
        else {
            '- Game logger errors: 0'
        }
    }

    $events = Join-Path $run.RunFolder 'events.log'

    if ($run.Result -eq 'running') {
        $stage = switch ($phase) {
            'WaitingForMap' { 'waiting for the new map' }
            'Preparing' { 'preparing: downloading and designing' }
            'Applying' { if (Test-Path $manifest) { 'finishing up' } else { 'building in game, then screenshots' } }
            'Settling' { 'waiting for diagnostics to finish' }
            default { $phase }
        }

        $lines += "- Stage: $stage"

        if ($run.StartedUtc) {
            $lines += "- Running for: $([int]((Get-Date).ToUniversalTime() - ([datetime]$run.StartedUtc).ToUniversalTime()).TotalMinutes) min"
        }

        $taken = @(Get-ChildItem -LiteralPath $run.RunFolder -Filter 'site-*.jpg').Count

        if ($taken -gt 0 -and -not (Test-Path $manifest)) {
            $lines += "- Screenshots so far: $taken"
        }

        if (Test-Path $events) {
            $latest = Get-Content -LiteralPath $events -Tail 400 | Where-Object { $_ -notmatch 'IEL-DIAG|\[WARN\]' } | Select-Object -Last 1

            if ($latest) {
                $text = $latest -replace '\[RunId=[^\]]+\]\s*', '' -replace '\[INFO\]\s*', ''
                $lines += "- Latest step: $($text.Substring(0, [Math]::Min(180, $text.Length)))"
            }
        }
    }

    if (Test-Path $events) {
        $finished = Select-String -LiteralPath $events -Pattern 'Import workflow finished: ([\d,]+) native network definition attempts, ([\d,]+) local omissions' | Select-Object -Last 1

        if ($finished) {
            $lines += "- Road sections sent to the game: $($finished.Matches[0].Groups[1].Value)"
            $lines += "- Sections omitted: $($finished.Matches[0].Groups[2].Value)"
        }

        $review = Select-String -LiteralPath $events -Pattern 'Native acceptance needs review: (.+?)(\.|$)' | Select-Object -Last 1

        if ($review) {
            $lines += "- Needs review (last check): $($review.Matches[0].Groups[1].Value)"
        }

        if ($finished) {
            $lines += "- Built road check: $(& (Join-Path $PSScriptRoot 'road-presence.ps1') -RunFolder $run.RunFolder -Summary)"
        }

        $completeness = Join-Path $run.RunFolder 'capture-completeness.json'

        if (Test-Path $completeness) {
            $lost = (Get-Content -LiteralPath $completeness -Raw | ConvertFrom-Json).artifactFailures

            if ($lost -gt 0) {
                $lines += "- Diagnostics lost to the size limit: $lost files"
            }
        }

        $warnings = (Select-String -LiteralPath $events -Pattern '\[WARN\]').Count
        $errors = (Select-String -LiteralPath $events -Pattern '\[ERROR\]').Count
        $lines += "- Log: $errors errors, $warnings warnings"
    }

    return $lines -join "`n"
}

$heading =
    if ($Outcome -eq 'done') { "$([char]::ConvertFromUtf32(0x1F3C1)) **FINISHED: $Title**`nTotal time: $Minutes min" }
    elseif ($Outcome -like 'Run in progress*') { "**$Title**`n$Outcome after $Minutes min" }
    else { "$([char]::ConvertFromUtf32(0x1F6D1)) **STOPPED: $Title**`n$Outcome after $Minutes min" }

$message = $heading
$game = Get-Process -Name Cities2 -ErrorAction SilentlyContinue

if ($game) {
    $message += "`nGame up $([int]((Get-Date) - $game.StartTime).TotalMinutes) min, using $([Math]::Round($game.WorkingSet64 / 1GB, 1)) GB of memory"
}
else {
    $message += "`nGame not running"
}

if (Test-Path $status) {
    $current = Get-Content -LiteralPath $status -Raw | ConvertFrom-Json
    $phase = $current.phase
    $runs = $current.runs
    $message += "`n`n" + (($runs | ForEach-Object { Format-City $_ }) -join "`n`n")
}

$message

foreach ($part in (Split-DiscordMessage $message)) {
    if ($Outcome -like 'Run in progress*') {
        Send-Discord $part -Silent
    }
    else {
        Send-Discord $part
    }
}
