# Discord posting and run-time estimates for the autorun scripts. The webhook lives outside the
# repository, and a failed post never fails the run.
$script:AutorunWebhookFile = Join-Path $env:USERPROFILE '.import-earth-discord-webhook.txt'
$script:AutorunTimings = Join-Path $env:USERPROFILE 'AppData\LocalLow\Colossal Order\Cities Skylines II\ModsData\ImportEarthLocation\diagnostics\autorun-timings.json'

# Silent messages appear in the channel without a push notification (Discord's flag 4096).
function Send-Discord([string]$Message, [switch]$Silent) {
    if (-not (Test-Path $script:AutorunWebhookFile)) {
        return
    }

    try {
        $payload = @{ content = $Message }

        if ($Silent) {
            $payload.flags = 4096
        }

        $body = [Text.Encoding]::UTF8.GetBytes(($payload | ConvertTo-Json))
        $url = (Get-Content -LiteralPath $script:AutorunWebhookFile -Raw).Trim()
        Invoke-RestMethod -Uri $url -Method Post -ContentType 'application/json' -Body $body | Out-Null
    }
    catch {
        "Discord notification failed: $($_.Exception.Message)"
    }
}

# Discord refuses messages over 2,000 characters, so long ones split between paragraphs.
function Split-DiscordMessage([string]$Message) {
    $parts = @()
    $current = ''

    foreach ($block in ($Message -split "`n`n")) {
        if ($current -and ($current.Length + $block.Length + 2) -gt 1900) {
            $parts += $current
            $current = $block
        }
        else {
            $current = if ($current) { "$current`n`n$block" } else { $block }
        }
    }

    return $parts + $current
}

# Measured on 6 Nov; each run then replaces these with its own times.
function Get-AutorunTimings {
    $timings = @{ startup = 25; marseille = 26; dc = 46; other = 45 }

    if (Test-Path $script:AutorunTimings) {
        try {
            (Get-Content -LiteralPath $script:AutorunTimings -Raw | ConvertFrom-Json).PSObject.Properties |
                ForEach-Object { $timings[$_.Name] = [double]$_.Value }
        }
        catch { }
    }

    return $timings
}

function Save-AutorunTimings([hashtable]$Timings) {
    try {
        $Timings | ConvertTo-Json | Set-Content -LiteralPath $script:AutorunTimings -Encoding utf8NoBOM
    }
    catch { }
}

function Format-CityName([string]$Name) {
    if ($Name.Length -le 2) {
        return $Name.ToUpper()
    }

    # Names already capitalised stay as written, as in Rio de Janeiro.
    if ($Name -cne $Name.ToLower()) {
        return $Name
    }

    return (Get-Culture).TextInfo.ToTitleCase($Name)
}

# Elapsed time against the estimate, kept under 100 until the run has really finished.
function Get-EstimatedPercent([double]$ElapsedMinutes, [string[]]$Cities, [hashtable]$Timings) {
    $total = $Timings.startup

    foreach ($city in $Cities) {
        $total += if ($Timings.ContainsKey($city)) { $Timings[$city] } else { $Timings.other }
    }

    return [int][Math]::Min(99, 100 * $ElapsedMinutes / [Math]::Max(1, $total))
}
