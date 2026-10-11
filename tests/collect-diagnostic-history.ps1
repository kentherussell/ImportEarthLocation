param(
    [string[]]$LogPaths = @(
        'C:/Users/kenru/AppData/LocalLow/Colossal Order/Cities Skylines II/Player.log',
        'C:/Users/kenru/AppData/LocalLow/Colossal Order/Cities Skylines II/Player-prev.log',
        'C:/Users/kenru/AppData/LocalLow/Colossal Order/Cities Skylines II/Logs/ImportEarthLocation.Mod.log'
    ),
    [string]$HistoryRoot = 'C:/Users/kenru/AppData/LocalLow/Colossal Order/Cities Skylines II/ModsData/ImportEarthLocation/diagnostics'
)
$ErrorActionPreference = 'Stop'
$snapshots = Join-Path $HistoryRoot 'historical'
New-Item -ItemType Directory -Path $snapshots -Force | Out-Null
foreach ($sourcePath in $LogPaths) {
    if (!(Test-Path -LiteralPath $sourcePath -PathType Leaf)) { continue }
    $source = Get-Item -LiteralPath $sourcePath
    $hash = (Get-FileHash -LiteralPath $sourcePath -Algorithm SHA256).Hash.ToLowerInvariant()
    $destination = Join-Path $snapshots ($source.BaseName + '-' + $hash)
    if (Test-Path -LiteralPath (Join-Path $destination 'source.json')) { continue }
    New-Item -ItemType Directory -Path $destination -Force | Out-Null
    Copy-Item -LiteralPath $source.FullName -Destination (Join-Path $destination 'source.log')
    @{ schemaVersion=1; source=$source.FullName; sha256=$hash; sourceModifiedUtc=$source.LastWriteTimeUtc.ToString('o'); capturedUtc=[DateTime]::UtcNow.ToString('o'); build='unknown unless recorded in source'; note='Snapshots may overlap. Do not add counts across sources or summary phases.' } |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $destination 'source.json') -Encoding utf8
}
$metrics = [System.Collections.Generic.List[object]]::new()
$fields = [System.Collections.Generic.List[object]]::new()
foreach ($snapshot in Get-ChildItem -LiteralPath $snapshots -Directory) {
    $lineNumber=0; $run='unassigned'; $seen=[System.Collections.Generic.HashSet[string]]::new()
    foreach ($line in Get-Content -LiteralPath (Join-Path $snapshot.FullName 'source.log')) {
        $lineNumber++
        # Restrict Player.log extraction to this mod, not unrelated mods or personal game state.
        if ($snapshot.Name.StartsWith('Player') -and $line -notmatch 'IEL-DIAG|ImportEarthLocation') { continue }
        if ($line -match 'PREPARE mapSession=|Preparing [\d.]+ km') { $run='line-' + $lineNumber }
        if (!$seen.Add($line)) { continue }
        $phase = if ($line -match 'IMPORT TOTALS') { 'import-totals' } elseif ($line -match 'Completion statistics:') { 'completion' } elseif ($line -match 'Preparation statistics:') { 'preparation' } else { $null }
        if ($phase) {
            $summary=$line -replace '^.*?(?:statistics:|proof\):)\s*',''
            foreach ($part in ($summary -split '\s*•\s*')) {
                if ($part -match '^(.+):\s*(-?\d+(?:\.\d+)?)$') {
                    $metrics.Add([pscustomobject]@{source=$snapshot.Name;run=$run;line=$lineNumber;phase=$phase;metric=$Matches[1];value=[double]::Parse($Matches[2],[Globalization.CultureInfo]::InvariantCulture)})
                }
            }
        }
        foreach ($match in [regex]::Matches($line,'(?<key>[A-Za-z][A-Za-z0-9_-]*)=(?<value>[^;\s]+)')) {
            $fields.Add([pscustomobject]@{source=$snapshot.Name;run=$run;line=$lineNumber;field=$match.Groups['key'].Value;value=$match.Groups['value'].Value;message=$line})
        }
    }
}
$metrics | Export-Csv -LiteralPath (Join-Path $HistoryRoot 'historical-metrics.csv') -NoTypeInformation -Encoding utf8
$fields | Export-Csv -LiteralPath (Join-Path $HistoryRoot 'historical-fields.csv') -NoTypeInformation -Encoding utf8
$runMetrics = foreach ($runDirectory in Get-ChildItem -LiteralPath (Join-Path $HistoryRoot 'runs') -Directory -ErrorAction SilentlyContinue) {
    $metricPath=Join-Path $runDirectory.FullName 'metrics.json'
    $metadataPath=Join-Path $runDirectory.FullName 'run.json'
    if (!(Test-Path -LiteralPath $metricPath)) { continue }
    $data=Get-Content -LiteralPath $metricPath -Raw | ConvertFrom-Json
    $metadata=Get-Content -LiteralPath $metadataPath -Raw | ConvertFrom-Json
    foreach ($metric in $data.metrics.psobject.Properties) {
        [pscustomobject]@{run=$runDirectory.Name;startedUtc=$data.startedUtc;state=$data.state;build=$metadata.assemblySha256;latitude=$metadata.settings.Latitude;longitude=$metadata.settings.Longitude;mapWidth=$metadata.settings.MapSize.WidthMeters;metric=$metric.Name;value=$metric.Value}
    }
}
$runMetrics | Export-Csv -LiteralPath (Join-Path $HistoryRoot 'run-metrics.csv') -NoTypeInformation -Encoding utf8
Write-Output "Diagnostic history: $HistoryRoot; historical metrics: $($metrics.Count); diagnostic fields: $($fields.Count)."
