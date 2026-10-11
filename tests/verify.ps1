#requires -Version 7.0
[CmdletBinding()]
param(
    [ValidateSet('Full','Networks','NativeHandoff','Surfaces','Overpass','Vegetation','Water','Terrain','UI')]
    [string]$Focus='Full',
    [ValidateSet('Debug','Release')][string]$Configuration='Debug',
    [ValidateRange(1,8)][int]$MaxParallel=3,
    [string[]]$NativeBatchRun=@(),
    [switch]$IncludeGameBuild,
    [ValidateRange(1,3600)][int]$TimeoutSeconds=300,
    [string]$ManagedPath=$env:CSII_MANAGEDPATH,
    [switch]$SelfTest
)

$ErrorActionPreference='Stop'
$repo=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$reportRoot=Join-Path $repo 'bin/test-results'
[IO.Directory]::CreateDirectory($reportRoot) | Out-Null
# Also protects SurfaceSmokeOnly and ordinary feature builds sharing an output
# folder. Two verification runners must not rebuild each other's live binaries.
try { $verificationLock=[IO.File]::Open((Join-Path $reportRoot 'verification.lock'),[IO.FileMode]::OpenOrCreate,[IO.FileAccess]::ReadWrite,[IO.FileShare]::None) }
catch { throw 'Another verification runner owns the test outputs. Wait for it to finish before starting this runner.' }
$runRoot=Join-Path $reportRoot ((Get-Date).ToUniversalTime().ToString('yyyyMMddTHHmmssfffZ')+'-'+[Guid]::NewGuid().ToString('N').Substring(0,8))
[IO.Directory]::CreateDirectory($runRoot) | Out-Null
$runClock=[Diagnostics.Stopwatch]::StartNew()
$rows=[Collections.Generic.List[object]]::new()
$status='Failed'
$failure=$null
$sourceFingerprint=$null
$maximumActive=0

function New-VerificationJob([string]$Name,[string]$Executable,[string[]]$Arguments,[string]$WorkingDirectory=$repo,[hashtable]$Environment=@{}) {
    [pscustomobject]@{Name=$Name;Executable=$Executable;Arguments=$Arguments;WorkingDirectory=$WorkingDirectory;Environment=$Environment}
}

function Invoke-VerificationJobs([object[]]$Jobs,[string]$Phase,[int]$Limit=$MaxParallel,[int]$Deadline=$TimeoutSeconds) {
    $active=[Collections.Generic.List[object]]::new()
    $results=[Collections.Generic.List[object]]::new()
    $next=0
    try {
        while($next -lt $Jobs.Count -or $active.Count -gt 0) {
            while($next -lt $Jobs.Count -and $active.Count -lt $Limit) {
                $job=$Jobs[$next++]
                $start=[Diagnostics.ProcessStartInfo]::new()
                $start.FileName=$job.Executable
                $start.WorkingDirectory=$job.WorkingDirectory
                $start.UseShellExecute=$false
                $start.CreateNoWindow=$true
                $start.RedirectStandardOutput=$true
                $start.RedirectStandardError=$true
                foreach($argument in $job.Arguments) { $start.ArgumentList.Add($argument) }
                foreach($entry in $job.Environment.GetEnumerator()) { $start.Environment[$entry.Key]=[string]$entry.Value }
                $process=[Diagnostics.Process]::new()
                $process.StartInfo=$start
                if(!$process.Start()) { $process.Dispose(); throw "Could not start $($job.Name)" }
                $active.Add([pscustomobject]@{Job=$job;Process=$process;Clock=[Diagnostics.Stopwatch]::StartNew();Stdout=$process.StandardOutput.ReadToEndAsync();Stderr=$process.StandardError.ReadToEndAsync();TimedOut=$false})
                $script:maximumActive=[Math]::Max($script:maximumActive,$active.Count)
            }
            for($i=$active.Count-1;$i -ge 0;$i--) {
                $item=$active[$i]
                if(!$item.Process.HasExited -and $item.Clock.Elapsed.TotalSeconds -gt $Deadline) {
                    $item.TimedOut=$true
                    $item.Process.Kill($true)
                    $item.Process.WaitForExit()
                }
                if(!$item.Process.HasExited) { continue }
                $item.Process.WaitForExit()
                $item.Clock.Stop()
                $stdout=Join-Path $runRoot ($Phase+'-'+$item.Job.Name+'.stdout.log')
                $stderr=Join-Path $runRoot ($Phase+'-'+$item.Job.Name+'.stderr.log')
                [IO.File]::WriteAllText($stdout,$item.Stdout.GetAwaiter().GetResult())
                [IO.File]::WriteAllText($stderr,$item.Stderr.GetAwaiter().GetResult())
                $row=[pscustomobject]@{phase=$Phase;name=$item.Job.Name;seconds=[Math]::Round($item.Clock.Elapsed.TotalSeconds,3);exitCode=$item.Process.ExitCode;timedOut=$item.TimedOut;stdout=$stdout;stderr=$stderr}
                $results.Add($row)
                $label=if($row.exitCode -eq 0 -and !$row.timedOut){'PASS'}else{'FAIL'}
                Write-Host "$label $Phase/$($row.name) ($($row.seconds)s)"
                $item.Process.Dispose()
                $active.RemoveAt($i)
            }
            if($active.Count -gt 0) { Start-Sleep -Milliseconds 20 }
        }
    }
    finally {
        foreach($item in $active) {
            if(!$item.Process.HasExited) { $item.Process.Kill($true); $item.Process.WaitForExit() }
            $item.Process.Dispose()
        }
    }
    return $results.ToArray()
}

function Assert-VerificationPhase([object[]]$Results) {
    foreach($row in $Results) { $rows.Add($row) }
    $failed=@($Results | Where-Object { $_.exitCode -ne 0 -or $_.timedOut })
    if($failed.Count) {
        foreach($row in $failed) {
            Write-Host "Failure details: $($row.stdout), $($row.stderr)"
            Get-Content -LiteralPath $row.stderr -Tail 20 | ForEach-Object { Write-Host $_ }
            Get-Content -LiteralPath $row.stdout -Tail 20 | ForEach-Object { Write-Host $_ }
        }
        throw "$($failed.Count) required $($failed[0].phase) job(s) failed. No successful final gate is claimed."
    }
}

function Get-VerificationFingerprint {
    $inputs=[Collections.Generic.List[string]]::new()
    foreach($folder in @('ImportEarthLocation','ImportEarthLocation.UI','tests')) {
        # Prune generated/dependency folders before traversal, not after scanning
        # thousands of node_modules entries. Hash the real inputs, not outputs.
        $pending=[Collections.Generic.Stack[string]]::new()
        $pending.Push((Join-Path $repo $folder))
        while($pending.Count) {
            foreach($entry in (Get-ChildItem -LiteralPath $pending.Pop())) {
                if($entry.PSIsContainer) {
                    if($entry.Name -notin @('bin','obj','node_modules','build') -and !($entry.Attributes -band [IO.FileAttributes]::ReparsePoint)) { $pending.Push($entry.FullName) }
                }
                elseif($entry.Extension -in @('.cs','.csproj','.props','.targets','.ts','.tsx','.scss','.cjs','.js','.ps1','.json','.svg','.png')) { $inputs.Add($entry.FullName) }
            }
        }
    }
    $text=[Text.StringBuilder]::new()
    foreach($path in ($inputs | Sort-Object)) {
        [void]$text.AppendLine($path+'|'+(Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash)
    }
    foreach($folder in ($NativeBatchRun | Sort-Object -Unique)) {
        foreach($name in @('native-plan-network.ndjson','last-native-temp-batch.ndjson')) {
            $path=[IO.Path]::GetFullPath((Join-Path $folder $name))
            [void]$text.AppendLine($path+'|'+(Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash)
        }
    }
    if(Test-Path -LiteralPath $ManagedPath) {
        foreach($name in @('Game.dll','Unity.Mathematics.dll','UnityEngine.CoreModule.dll')) {
            $path=Join-Path $ManagedPath $name
            [void]$text.AppendLine($path+'|'+(Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash)
        }
    }
    [Convert]::ToHexString([Security.Cryptography.SHA256]::HashData([Text.Encoding]::UTF8.GetBytes($text.ToString())))
}

try {
    $dotnet=(Get-Command dotnet -CommandType Application | Select-Object -First 1).Source
    $shell=(Get-Process -Id $PID).Path
    if($SelfTest) {
        $jobs=@(1..4 | ForEach-Object { New-VerificationJob "parallel-$_" $shell @('-NoProfile','-Command','Start-Sleep -Milliseconds 100; Write-Output "child-completed"; exit 0') })
        $result=@(Invoke-VerificationJobs $jobs 'runner-self-test' 2)
        Assert-VerificationPhase $result
        if($maximumActive -ne 2 -or $result.Count -ne 4 -or @($result.name | Select-Object -Unique).Count -ne 4) { throw 'Runner concurrency or exactly-once execution failed' }
        $bad=@(Invoke-VerificationJobs @(New-VerificationJob 'exit-seven' $shell @('-NoProfile','-Command','[Console]::Error.WriteLine("intentional failure"); exit 7')) 'runner-negative')
        if($bad.Count -ne 1 -or $bad[0].exitCode -ne 7) { throw 'Runner lost the real child exit code' }
        $rejected=$false
        try { Assert-VerificationPhase $bad } catch { $rejected=$_.Exception.Message -like '*required runner-negative job(s) failed*' }
        foreach($row in $bad) { [void]$rows.Remove($row) }
        if(!$rejected) { throw 'Runner accepted a failed phase and could execute stale binaries' }
        $timed=@(Invoke-VerificationJobs @(New-VerificationJob 'timeout' $shell @('-NoProfile','-Command','Start-Sleep -Seconds 10')) 'runner-negative' 1 1)
        if($timed.Count -ne 1 -or !$timed[0].timedOut -or $timed[0].exitCode -eq 0) { throw 'Runner lost or accepted a timed-out child' }
        $nested=@(Invoke-VerificationJobs @(New-VerificationJob 'output-lock' $shell @('-NoProfile','-File',$PSCommandPath,'-SelfTest')) 'runner-negative')
        if($nested.Count -ne 1 -or $nested[0].exitCode -eq 0 -or (Get-Content -LiteralPath $nested[0].stderr -Raw) -notmatch 'Another verification runner owns') { throw 'Runner allowed overlapping writes to shared test outputs' }
        $pipe=@(Invoke-VerificationJobs @(New-VerificationJob 'both-pipes' $shell @('-NoProfile','-Command','[Console]::Out.Write(("x"*100000)); [Console]::Error.Write(("y"*100000)); exit 0')) 'runner-self-test')
        Assert-VerificationPhase $pipe
        if((Get-Item -LiteralPath $pipe[0].stdout).Length -ne 100000 -or (Get-Item -LiteralPath $pipe[0].stderr).Length -ne 100000) { throw 'Runner truncated or deadlocked redirected output' }
        Write-Host 'PASS runner negative checks: exit code, failed phase, timeout and output lock retained; both pipes drained (intentional failures, not product failures).'
        $status='RunnerSelfTestPassed'
    }
    else {
        if(!$ManagedPath) { $ManagedPath=Join-Path ${env:ProgramFiles(x86)} 'Steam/steamapps/common/Cities Skylines II/Cities2_Data/Managed' }
        $ManagedPath=[IO.Path]::GetFullPath($ManagedPath)
        foreach($folder in $NativeBatchRun) {
            foreach($file in @('native-plan-network.ndjson','last-native-temp-batch.ndjson')) {
                if(!(Test-Path -LiteralPath (Join-Path $folder $file) -PathType Leaf)) { throw "Requested native replay is incomplete: $folder/$file" }
            }
        }
        $selected=switch($Focus) {
            'Full' { @('network','feature','processing','conditioning','terrain','rail-prefab','overpass') }
            'Networks' { @('network','processing','rail-prefab') }
            'NativeHandoff' { @('network') }
            'Surfaces' { @('feature') }
            'Overpass' { @('overpass') }
            'Vegetation' { @('processing','feature') }
            'Water' { @('feature','terrain','conditioning') }
            'Terrain' { @('terrain','conditioning') }
            'UI' { @() }
        }
        if($NativeBatchRun.Count -and 'network' -notin $selected) { $selected=@($selected)+@('network') }
        $sourceFingerprint=Get-VerificationFingerprint
        $builds=[Collections.Generic.List[object]]::new()
        $checks=[Collections.Generic.List[object]]::new()
        foreach($suite in $selected) {
            $project=Join-Path $PSScriptRoot "$suite-tests/$suite-tests.csproj"
            $surfaceOnly=if($Focus -eq 'Surfaces'){'true'}else{'false'}
            $waterOnly=if($Focus -eq 'Water'){'true'}else{'false'}
            $builds.Add((New-VerificationJob $suite $dotnet @('build',$project,'-c',$Configuration,'-v:quiet',"-p:GameManagedPath=$ManagedPath","-p:SurfaceSmokeOnly=$surfaceOnly","-p:WaterSmokeOnly=$waterOnly")))
            $binary=Join-Path $PSScriptRoot "$suite-tests/bin/$Configuration/net9.0/$suite-tests.dll"
            $arguments=@($binary)
            if($suite -eq 'network' -and $Focus -eq 'NativeHandoff') { $arguments+=@('--native-bindings-only') }
            $checks.Add((New-VerificationJob $suite $dotnet $arguments))
        }
        $gameBuild=$Focus -eq 'Full' -or $IncludeGameBuild
        # Keep stock deploy/UI targets disabled; all verification outputs remain
        # under the repository, never the user's installed mod directory.
        $gameOutput=Join-Path $repo 'bin/verification-game/'
        if($gameBuild) {
            $builds.Add((New-VerificationJob 'game-reference' $dotnet @('build',(Join-Path $repo 'ImportEarthLocation/ImportEarthLocation.csproj'),'-c',$Configuration,'-v:quiet','-p:LocalVerificationOnly=true','-p:NeedBuild=false',"-p:ManagedPath=$ManagedPath","-p:OutDir=$gameOutput")))
        }
        Write-Host "Verification focus=$Focus; configuration=$Configuration; parallel limit=$MaxParallel"
        Write-Host 'Build barrier: every selected harness must build successfully before any test binary runs.'
        Assert-VerificationPhase @(Invoke-VerificationJobs $builds.ToArray() 'build')
        if($sourceFingerprint -ne (Get-VerificationFingerprint)) { throw 'Verification inputs changed during build. Re-run against one stable source state.' }
        if($Focus -in @('Full','UI')) {
            $node=(Get-Command node -CommandType Application | Select-Object -First 1).Source
            foreach($script in @('ui-smoke','picker-smoke')) { $checks.Add((New-VerificationJob $script $node @((Join-Path $PSScriptRoot "$script.cjs")))) }
        }
        if($Focus -ne 'Overpass' -and $Focus -ne 'UI') {
            $checks.Add((New-VerificationJob 'native-contract' $shell @('-NoProfile','-File',(Join-Path $PSScriptRoot 'native-contract-smoke.ps1'),'-ManagedPath',$ManagedPath)))
        }
        if($NativeBatchRun.Count) {
            $binary=Join-Path $PSScriptRoot "network-tests/bin/$Configuration/net9.0/network-tests.dll"
            $folders=@($NativeBatchRun | ForEach-Object { [IO.Path]::GetFullPath($_) } | Select-Object -Unique)
            $checks.Add((New-VerificationJob 'native-replays' $dotnet (@($binary,'--audit-native-batches')+$folders)))
        }
        if($gameBuild) {
            $node=(Get-Command node -CommandType Application | Select-Object -First 1).Source
            $userData=$env:CSII_USERDATAPATH
            if(!$userData) { $userData=[IO.Path]::GetFullPath((Join-Path $env:LOCALAPPDATA '../LocalLow/Colossal Order/Cities Skylines II')) }
            $ui=Join-Path $repo 'ImportEarthLocation.UI'
            $checks.Add((New-VerificationJob 'ui-bundle' $node @((Join-Path $ui 'node_modules/webpack/bin/webpack.js'),'--output-path',$gameOutput) $ui @{CSII_USERDATAPATH=$userData}))
        }
        Assert-VerificationPhase @(Invoke-VerificationJobs $checks.ToArray() 'check')
        if($sourceFingerprint -ne (Get-VerificationFingerprint)) { throw 'Verification inputs changed while tests ran. Re-run; stale results cannot qualify this source.' }
        $status=if($Focus -eq 'Full'){'FullOfflineGatePassed'}else{'TargetedChecksPassed'}
        Write-Host "$status. Native replay folders supplied: $($NativeBatchRun.Count). No live endpoints, game launch or deployment performed."
        if($Focus -ne 'Full') { Write-Host 'Targeted feedback only: run -Focus Full before qualifying a candidate.' }
    }
}
catch { $failure=$_.Exception.Message; Write-Host "FAIL: $failure" }
finally {
    $runClock.Stop()
    $summary=[ordered]@{status=$status;focus=$Focus;configuration=$Configuration;sourceFingerprint=$sourceFingerprint;maxParallel=$MaxParallel;maximumActive=$maximumActive;totalSeconds=[Math]::Round($runClock.Elapsed.TotalSeconds,3);nativeReplayFolders=$NativeBatchRun;failure=$failure;jobs=$rows.ToArray();nativeGameE2EVerified=$false;deployed=$false}
    try { [IO.File]::WriteAllText((Join-Path $runRoot 'summary.json'),($summary | ConvertTo-Json -Depth 6)); Write-Host "Report: $runRoot/summary.json" }
    finally { $verificationLock.Dispose() }
}
if($failure) { exit 1 }
