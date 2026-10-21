#requires -Version 7.0
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$BaselineDll,
    [Parameter(Mandatory)][string]$CandidateDll,
    [Parameter(Mandatory)][string]$RunDirectory,
    [Parameter(Mandatory)][string]$OutputDirectory,
    [ValidateSet(3,5,7)][int]$Pairs=3
)

$ErrorActionPreference='Stop'
function Median([double[]]$Values) {
    $sorted=@($Values | Sort-Object)
    if(!$sorted.Count){throw 'No measurements'}
    if($sorted.Count % 2){return $sorted[[int][Math]::Floor($sorted.Count/2)]}
    return ($sorted[$sorted.Count/2-1]+$sorted[$sorted.Count/2])/2
}

$baseline=(Resolve-Path -LiteralPath $BaselineDll).Path
$candidate=(Resolve-Path -LiteralPath $CandidateDll).Path
$capture=(Resolve-Path -LiteralPath $RunDirectory).Path
$output=[IO.Path]::GetFullPath($OutputDirectory)
if(Test-Path -LiteralPath $output){throw 'Use a new output directory; benchmark evidence is never overwritten.'}
[IO.Directory]::CreateDirectory($output) | Out-Null
$measurements=[Collections.Generic.List[object]]::new()
$quality=@{}
$versions=@{}
function Measure-Design([string]$Build,[string]$Dll,[string]$Label) {
    if(Get-Process -Name Cities2 -ErrorAction SilentlyContinue){throw 'Close the game before promotion timings. The benchmark does not stop it.'}
    $json=Join-Path $output ($Label+'.json')
    $log=Join-Path $output ($Label+'.log')
    & dotnet $Dll --corrective-timing $capture $json *> $log
    if($LASTEXITCODE -ne 0){throw "Replay failed: $Label; inspect $log"}
    $report=Get-Content -LiteralPath $json -Raw | ConvertFrom-Json
    if(!$report.performanceBenchmark -or !$report.completeWaterAndProtectionContext -or !$report.verticalSolveReplayed -or !$report.finite){throw "Incomplete timing case: $Label"}
    if(![double]::IsFinite([double]$report.designSeconds) -or [double]$report.designSeconds -le 0 -or
        ![double]::IsFinite([double]$report.peakProcessWorkingSetBytes) -or [double]$report.peakProcessWorkingSetBytes -le 0){
        throw "Missing or invalid design time/peak memory: $Label"
    }
    $signature=$report.findings | ConvertTo-Json -Depth 20 -Compress
    if($quality.ContainsKey($Build) -and $quality[$Build] -cne $signature){throw "Nondeterministic findings: $Label"}
    if($versions.ContainsKey($Build) -and $versions[$Build] -ne $report.offlineAssemblyMvid){throw "Assembly changed during benchmark: $Label"}
    $quality[$Build]=$signature
    $versions[$Build]=$report.offlineAssemblyMvid
    [pscustomobject]@{build=$Build;file=$json;seconds=[double]$report.designSeconds;sourceSections=$report.sourceSections;stageSeconds=$report.stageSeconds;peakBytes=$report.peakProcessWorkingSetBytes}
}

# Fresh matching processes include the same per-process JIT/setup policy. The
# warmups warm filesystem/data caches; they do not remove JIT from later runs.
$null=Measure-Design 'baseline' $baseline 'warmup-baseline'
$null=Measure-Design 'candidate' $candidate 'warmup-candidate'
$differences=[Collections.Generic.List[double]]::new()
$memoryChecks=[Collections.Generic.List[object]]::new()
for($i=0;$i -lt $Pairs;$i++) {
    $order=if($i % 2 -eq 0){@('baseline','candidate')}else{@('candidate','baseline')}
    $pair=@{}
    foreach($build in $order) {
        $dll=if($build -eq 'baseline'){$baseline}else{$candidate}
        $row=Measure-Design $build $dll ('pair-'+$i+'-'+$build)
        $measurements.Add($row)
        $pair[$build]=$row
        Write-Host "$build pair $($i+1)/${Pairs}: $($row.seconds)s"
    }
    if($pair.baseline.sourceSections -ne $pair.candidate.sourceSections){throw 'Selected input counts disagree.'}
    $differences.Add(100*($pair.baseline.seconds-$pair.candidate.seconds)/$pair.baseline.seconds)
    $extraBytes=[double]$pair.candidate.peakBytes-[double]$pair.baseline.peakBytes
    $allowedBytes=[Math]::Min([double]128MB,.05*[double]$pair.baseline.peakBytes)
    $memoryChecks.Add([pscustomobject]@{pair=$i;baselinePeakBytes=$pair.baseline.peakBytes;
        candidatePeakBytes=$pair.candidate.peakBytes;extraBytes=$extraBytes;allowedExtraBytes=$allowedBytes;
        passed=($extraBytes -le $allowedBytes)})
}
$medianBaseline=Median @($measurements | Where-Object build -eq baseline | ForEach-Object seconds)
$medianCandidate=Median @($measurements | Where-Object build -eq candidate | ForEach-Object seconds)
$medianDifference=Median $differences.ToArray()
$mad=Median @($differences | ForEach-Object {[Math]::Abs($_-$medianDifference)})
$threshold=[Math]::Max(1.0,3.0*$mad)
$summary=[ordered]@{
    baselineSha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $baseline).Hash
    candidateSha256=(Get-FileHash -Algorithm SHA256 -LiteralPath $candidate).Hash
    baselineMvid=$versions.baseline;candidateMvid=$versions.candidate;capture=$capture;pairs=$Pairs
    processPolicy='Fresh process per case; cache warmups excluded; JIT policy identical'
    medianBaselineSeconds=$medianBaseline;medianCandidateSeconds=$medianCandidate
    pairedImprovementPercent=$differences.ToArray();medianImprovementPercent=$medianDifference
    medianAbsoluteDeviationPoints=$mad;requiredImprovementPoints=$threshold
    speedProved=($medianCandidate -lt $medianBaseline -and $medianDifference -gt $threshold)
    fivePercentTargetMet=($medianDifference -ge 5)
    findingsIdentical=($quality.baseline -ceq $quality.candidate)
    medianBaselinePeakBytes=(Median @($measurements | Where-Object build -eq baseline | ForEach-Object peakBytes))
    medianCandidatePeakBytes=(Median @($measurements | Where-Object build -eq candidate | ForEach-Object peakBytes))
    memoryPairs=$memoryChecks.ToArray()
    memoryGatePassed=(@($memoryChecks | Where-Object {!$_.passed}).Count -eq 0)
    qualityPromotionVerified=$false;nativeAcceptanceVerified=$false
    interpretation='One city only. Physical-domain nonregression and a separate passing comparison for each city are required. Noise rule is not a confidence interval.'
    measurements=$measurements.ToArray()
}
$summary | ConvertTo-Json -Depth 20 | Set-Content -LiteralPath (Join-Path $output 'summary.json')
Write-Host "Saved $(Join-Path $output 'summary.json'); speed proved: $($summary.speedProved)"
