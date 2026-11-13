# Runs tests/autorun.ps1 on the "Minimal Mods Only" Paradox Mods playset and switches back to the
# playset that was active before, also when the run fails. Every game test goes through this: the
# full playset (about 1,650 packages on 11-11) more than doubles the startup time and loads about
# 200 other code mods into the test.
# All other arguments go to autorun.ps1, for example: -Cities marseille -Switches smooth-roads
param(
    [string]$Playset = 'Minimal Mods Only'
)

# $args keeps parameter names when it is splatted, so they reach autorun.ps1 as written.
$autorunArguments = $args

$ErrorActionPreference = 'Stop'

$configPath = Join-Path $env:USERPROFILE `
    'AppData\LocalLow\Colossal Order\Cities Skylines II\.cache\Mods\playset_config.json'

function Wait-GameClosed {
    # The game writes the playset file when it closes, so a change made earlier would be lost.
    while (Get-Process -Name Cities2 -ErrorAction SilentlyContinue) {
        Start-Sleep -Seconds 5
    }
}

function Set-ActivePlayset([string]$Id) {
    Wait-GameClosed

    # Only the id is replaced, so the rest of the file stays byte for byte as the game wrote it.
    $text = [IO.File]::ReadAllText($configPath)
    $pattern = '("activePlaysetId"\s*:\s*")[^"]*(")'

    if ($text -notmatch $pattern) {
        throw "No activePlaysetId in $configPath"
    }

    [IO.File]::WriteAllText($configPath, ($text -replace $pattern, "`${1}$Id`${2}"))
}

if (Get-Process -Name Cities2 -ErrorAction SilentlyContinue) {
    throw 'The game is running. Close it first.'
}

$config = Get-Content $configPath -Raw | ConvertFrom-Json
$previous = $config.activePlaysetId
$target = @($config.playsets | Where-Object { $_.presentation.name -eq $Playset })

if ($target.Count -ne 1) {
    throw "Expected one playset named '$Playset', found $($target.Count)."
}

Write-Host "Playset: '$Playset' ($($target[0].id)); afterwards back to $previous"
Set-ActivePlayset $target[0].id

try {
    & (Join-Path $PSScriptRoot 'autorun.ps1') @autorunArguments
}
finally {
    Set-ActivePlayset $previous
    Write-Host "Playset back to $previous"
}
