<# .SYNOPSIS Freeze and qualify static startup using only the three current firmware targets offline.
.DESCRIPTION No serial port, probe, debugger, download or motor API is opened.
The retained acceptance manifest binds maintained inputs, logs, renders and exact images.
#>
[CmdletBinding()]
param([Parameter(Mandatory=$true)][ValidatePattern('^[0-9a-f]{40}$')][string]$ReviewedStartupRevision)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$runId = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$evidenceRoot = Join-Path $projectRoot 'output/static_startup_sync_20261020'
[void](New-Item -ItemType Directory -Path $evidenceRoot -Force)
$runDirectory = Join-Path $evidenceRoot $runId
[void](New-Item -ItemType Directory -Path $runDirectory)
$records = [System.Collections.Generic.List[object]]::new()
$buildRecords = [System.Collections.Generic.List[object]]::new()
$failureMessage = $null
$lcdSummary = $null
$sourcesUnchanged = $false
$controlSourcesUnchanged = $false
$scriptHashBefore = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
$pythonExecutable = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$powershellExecutable = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
$controlBaseline = 'd333d276639420637a056f15903e45dcba323d90'
$rtosStackBaseline = 'a9447d70e11a7b80820f822ab5c000cc9f420051'
$env:PYTHONPATH = 'D:\download\TCG\output\can_id_commissioning_20260909\python_deps;D:\download\TCG\Aethor_robo_fw\.worktrees\s3519-adrc-lcd-mit\output\adrc\hardware\python_deps_20261010'
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONUTF8 = '1'

function Get-RevisionInputs {
    <# Enumerate maintained build/test sources without including generated firmware or output. #>
    $paths = @(git -c ('safe.directory=' + $projectRoot.Replace('\','/')) -C $projectRoot ls-files `
        --cached --others --exclude-standard -- .gitignore App Core Ui Drivers Middlewares USB_DEVICE `
        Tests/host Tests/ui Tests/hardware tools Models MDK-ARM CtrBoard-H7_FDCAN.ioc)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate maintained inputs.' }
    foreach ($relativePath in ($paths | Sort-Object -Unique)) {
        if ($relativePath -match '(^|/)build/' -or ($relativePath -ne '.gitignore' -and
            [IO.Path]::GetExtension($relativePath) -notin @('.c','.h','.inc','.s','.m','.ps1','.py','.json','.txt','.uvprojx','.sct','.ioc'))) { continue }
        [ordered]@{path=$relativePath;sha256=(Get-FileHash -LiteralPath (Join-Path $projectRoot $relativePath) -Algorithm SHA256).Hash}
    }
}

function Test-StaticStartupStackConfiguration {
    <# Permit only the reviewed static/POS receive and integrated capacities; preserve every other token and legacy capacity. #>
    param([Parameter(Mandatory=$true)][string]$Text,
        [Parameter(Mandatory=$true)][string]$BaselineText)
    $expectedBaseline = @'
#ifndef AETHOR_TASK_STACKS_H
#define AETHOR_TASK_STACKS_H
#include "debug_ui_config.h"
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (2048U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (512U)
#else
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (1280U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (384U)
#endif
#endif
'@
    $reviewedGates = @'
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
#define AETHOR_CAN_RX_TASK_STACK_WORDS (640U)
#else
#define AETHOR_CAN_RX_TASK_STACK_WORDS (384U)
#endif
#if AETHOR_ADRC_LCD_INTEGRATED || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
'@
    $expectedCurrent = $expectedBaseline.Replace('#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD', $reviewedGates)
    $normalizeStackText = {
        param([string]$Value)
        [regex]::Replace([regex]::Replace($Value, '(?s)/\*.*?\*/|//[^\r\n]*', ''), '\s+', '')
    }
    return ((& $normalizeStackText $BaselineText) -ceq (& $normalizeStackText $expectedBaseline)) -and
        ((& $normalizeStackText $Text) -ceq (& $normalizeStackText $expectedCurrent))
}

function Invoke-RecordedCheck {
    <# Run a child offline check and retain separate stdout/stderr with its actual status. #>
    param([string]$Name,[string]$Executable,[string[]]$Arguments)
    $stdout = Join-Path $runDirectory ($Name+'.log')
    $stderr = Join-Path $runDirectory ($Name+'.stderr.log')
    $started = [DateTime]::UtcNow
    $previousPythonUtf8 = $env:PYTHONUTF8
    try {
        # Builder negative fixtures receive native-encoded Windows PowerShell diagnostics.
        if ($Name -eq 'single_static_build_tests') { $env:PYTHONUTF8 = '0' }
        $child = Start-Process -FilePath $Executable -ArgumentList $Arguments -WorkingDirectory $projectRoot `
            -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    }
    finally { $env:PYTHONUTF8 = $previousPythonUtf8 }
    $records.Add([ordered]@{name=$Name;exit_code=$child.ExitCode;log=$stdout;stderr=$stderr;
        elapsed_seconds=[Math]::Round(([DateTime]::UtcNow-$started).TotalSeconds,3)})
    Write-Output ('CHECK '+$Name+' exit='+$child.ExitCode)
    if ($child.ExitCode -ne 0) { throw ('Offline check failed: '+$Name) }
}

function Save-FirmwareEvidence {
    <# Copy fresh linked images, map, call graph and clean log into immutable run evidence. #>
    param([string]$TargetName,[DateTime]$EarliestTime,[string]$BuildLog)
    if ((Get-Item -LiteralPath $BuildLog).LastWriteTimeUtc -lt $EarliestTime.AddSeconds(-2) -or
        (Get-Content -LiteralPath $BuildLog -Raw) -notmatch '0 Error\(s\), 0 Warning\(s\)') { throw ('Unclean/stale build log: '+$TargetName) }
    $destinations = @{}
    foreach ($extension in @('axf','hex','map','htm')) {
        $source = Join-Path $projectRoot ('MDK-ARM\'+$TargetName+'\'+$TargetName+'.'+$extension)
        if (-not (Test-Path -LiteralPath $source -PathType Leaf) -or
            (Get-Item -LiteralPath $source).LastWriteTimeUtc -lt $EarliestTime.AddSeconds(-2)) { throw ('Stale/missing artifact: '+$source) }
        $destination = Join-Path $runDirectory ($TargetName+'.'+$extension)
        Copy-Item -LiteralPath $source -Destination $destination
        if ((Get-FileHash -LiteralPath $source -Algorithm SHA256).Hash -cne
            (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash) { throw 'Image evidence copy mismatch.' }
        $destinations[$extension] = $destination
    }
    $retainedLog = Join-Path $runDirectory ($TargetName+'.keil.log')
    Copy-Item -LiteralPath $BuildLog -Destination $retainedLog
    $buildRecords.Add([ordered]@{target=$TargetName;errors=0;warnings=0;hex_path=$destinations['hex'];
        elf_path=$destinations['axf'];map_path=$destinations['map'];log_path=$retainedLog})
}

$sourceBefore = @(Get-RevisionInputs)
$startedAt = [DateTime]::UtcNow
Push-Location $projectRoot
try {
    # Every maintained input must belong to the independently reviewed revision.
    # Generated HEX and logs are excluded by the same suffix/path enumeration used for hashing.
    $actualHead = (git -c ('safe.directory='+$projectRoot.Replace('\','/')) rev-parse HEAD).Trim()
    if ($LASTEXITCODE -ne 0 -or $actualHead -cne $ReviewedStartupRevision) { throw 'HEAD is not the reviewed startup revision.' }
    $maintainedNames = @($sourceBefore | ForEach-Object {$_.path})
    $changedNames = @(git -c ('safe.directory='+$projectRoot.Replace('\','/')) diff --name-only $ReviewedStartupRevision --)
    if ($LASTEXITCODE -ne 0 -or @($changedNames | Where-Object {$_ -in $maintainedNames}).Count) {
        throw 'Maintained inputs differ from the independently reviewed startup revision.'
    }
    $untrackedNames = @(git -c ('safe.directory='+$projectRoot.Replace('\','/')) ls-files --others --exclude-standard --)
    if ($LASTEXITCODE -ne 0 -or @($untrackedNames | Where-Object {$_ -in $maintainedNames}).Count) {
        throw 'Unreviewed maintained input exists outside the reviewed commit.'
    }
    $protectedPaths = @('App/Adrc','App/Arm','App/Config','App/Motion','App/Motor','Drivers','Middlewares',
        'USB_DEVICE','Models','Ui','Core/Inc','Core/Src/main.c',
        'MDK-ARM/CtrBoard-H7_FDCAN.uvprojx','MDK-ARM/ADRC-Bench.uvprojx','MDK-ARM/LCD-MIT-ADRC.uvprojx',
        'MDK-ARM/LCD-SingleStatic-Test.uvprojx',':(exclude)Core/Inc/aethor_task_stacks.h')
    git -c ('safe.directory='+$projectRoot.Replace('\','/')) diff --exit-code d857bec -- @protectedPaths | Out-Null
    if ($LASTEXITCODE -ne 0) { throw 'Original controllers, units, default gates, UI, driver or project settings changed.' }
    $stackBaselineLines = git -c ('safe.directory='+$projectRoot.Replace('\','/')) show ($rtosStackBaseline+':Core/Inc/aethor_task_stacks.h')
    if ($LASTEXITCODE -ne 0 -or -not (Test-StaticStartupStackConfiguration `
        -Text (Get-Content -LiteralPath (Join-Path $projectRoot 'Core/Inc/aethor_task_stacks.h') -Raw) `
        -BaselineText ($stackBaselineLines -join "`n"))) {
        throw 'Task-stack exception differs from the exact reviewed capacities and compile gates.'
    }
    $controlSourcesUnchanged = $true
    $suiteStarted = [DateTime]::UtcNow
    Invoke-RecordedCheck 'lcd_suite' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File','tools/verify_debug_ui.ps1')
    $lcdDirectory = Join-Path $projectRoot 'Tests\host\build\verification'
    $lcdSummary = Get-Content -LiteralPath (Join-Path $lcdDirectory 'summary.json') -Raw | ConvertFrom-Json
    if (-not $lcdSummary.first_party_sources_unchanged_during_verification -or $lcdSummary.checks.Count -ne 27 -or
        @($lcdSummary.checks | Where-Object {$_.exit_code -ne 0}).Count -ne 0) { throw 'Expected 27 clean, source-stable host LCD checks.' }
    $lcdRetained = Join-Path $runDirectory 'lcd_evidence'
    [void](New-Item -ItemType Directory -Path $lcdRetained)
    $lcdFiles = @('summary.json') + @($lcdSummary.checks | ForEach-Object {
        [IO.Path]::GetFileName($_.stdout)
        [IO.Path]::GetFileName($_.stderr)
    })
    foreach ($fileName in $lcdFiles) {
        $sourceLog = Join-Path $lcdDirectory $fileName
        if ((Get-Item -LiteralPath $sourceLog).LastWriteTimeUtc -lt $suiteStarted.AddSeconds(-2)) {
            throw ('Stale LCD receipt: '+$fileName)
        }
        Copy-Item -LiteralPath $sourceLog -Destination (Join-Path $lcdRetained $fileName)
    }
    Invoke-RecordedCheck 'single_static_platform' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File','Tests/host/run_single_static_platform_tests.ps1')
    Invoke-RecordedCheck 'load_trials' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File','Tests/host/run_load_trials_tests.ps1')
    Invoke-RecordedCheck 'load_trial_pages' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File','Tests/ui/run_load_trial_pages_tests.ps1')
    Invoke-RecordedCheck 'single_static_build_tests' $pythonExecutable @('-B','Tests/host/test_single_static_build.py')
    Invoke-RecordedCheck 'single_static_deploy_tests' $pythonExecutable @('-B','Tests/hardware/test_single_static_deploy.py')
    Invoke-RecordedCheck 'startup_trace_export_tests' $pythonExecutable @('-B','Tests/host/test_static_startup_trace.py')
    Invoke-RecordedCheck 'trace_map_tests' $pythonExecutable @('-B','Tests/host/test_static_trace_map.py')
    Invoke-RecordedCheck 'startup_sync_tests' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File','Tests/host/run_static_startup_sync_tests.ps1')
    foreach ($target in @('ADRC-Bench','LCD-MIT-ADRC')) {
        $buildStarted = [DateTime]::UtcNow
        $buildLog = Join-Path $runDirectory ($target+'.build.log')
        $buildStderr = Join-Path $runDirectory ($target+'.build.stderr.log')
        $buildStdout = Join-Path $runDirectory ($target+'.build.stdout.log')
        $project = Join-Path $projectRoot ('MDK-ARM\'+$target+'.uvprojx')
        $child = Start-Process -FilePath 'D:\application\keil\UV4\UV4.exe' `
            -ArgumentList @('-r',('"'+$project+'"'),'-t',$target,'-j0','-o',('"'+$buildLog+'"')) -WindowStyle Hidden -Wait -PassThru `
            -RedirectStandardOutput $buildStdout -RedirectStandardError $buildStderr
        $records.Add([ordered]@{name=('keil_'+$target);exit_code=$child.ExitCode;log=$buildLog;stderr=$buildStderr;
            elapsed_seconds=[Math]::Round(([DateTime]::UtcNow-$buildStarted).TotalSeconds,3)})
        if ($child.ExitCode -ne 0) { throw ('Keil failed: '+$target) }
        Save-FirmwareEvidence $target $buildStarted $buildLog
    }
    $testBuildStarted = [DateTime]::UtcNow
    $testBuildEvidence = Join-Path $runDirectory 'single_static_build'
    Invoke-RecordedCheck 'keil_LCD-SingleStatic-Test' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File',
        'tools/build_single_static_test.ps1','-EvidenceDirectory',('"'+$testBuildEvidence+'"'))
    Save-FirmwareEvidence 'LCD-SingleStatic-Test' $testBuildStarted (Join-Path $projectRoot 'Tests\host\build\keil\LCD-SingleStatic-Test.log')
    Invoke-RecordedCheck 'map_single_static' $pythonExecutable @('-B','Tests/host/check_debug_ui_map.py','--target','LCD-SingleStatic-Test','--evidence-directory',('"'+$runDirectory+'"'))
    Copy-Item -LiteralPath (Join-Path $lcdDirectory 'memory-layout.json') -Destination (Join-Path $runDirectory 'single_static_memory_layout.json')
    Invoke-RecordedCheck 'map_current_targets' $pythonExecutable @('-B','Tests/host/check_debug_ui_map.py','--target','LCD-SingleStatic-Test','--target','LCD-MIT-ADRC','--target','ADRC-Bench','--evidence-directory',('"'+$runDirectory+'"'))
    Copy-Item -LiteralPath (Join-Path $lcdDirectory 'memory-layout.json') -Destination (Join-Path $runDirectory 'current_targets_memory_layout.json')
    $renderDirectory = Join-Path $projectRoot 'Tests\ui\artifacts-astra'
    $render = Get-Content -LiteralPath (Join-Path $renderDirectory 'render_manifest.json') -Raw | ConvertFrom-Json
    $baselineText = git -c ('safe.directory='+$projectRoot.Replace('\','/')) show ($controlBaseline+':Tests/ui/artifacts-astra/render_manifest.json')
    if ($LASTEXITCODE -ne 0) { throw 'Cannot read approved visual baseline.' }
    $baseline = ($baselineText -join "`n") | ConvertFrom-Json
    $currentHashes = @{}
    foreach ($entry in $render.images) {$currentHashes[$entry.image] = $entry.rgb_sha256}
    $unrelatedPageCount = 0
    foreach ($entry in $baseline.images) {
        if ($entry.image -notmatch '^([0-9]+)_') { throw 'Visual baseline lacks numeric page ID.' }
        if ([int]$Matches[1] -gt 131) { continue }
        if ($currentHashes[$entry.image] -cne $entry.rgb_sha256) {throw ('Unrelated visual regression: '+$entry.image)}
        ++$unrelatedPageCount
    }
    if ($unrelatedPageCount -ne 131) { throw 'Expected 131 original unrelated visual pages.' }
    if ($render.images.Count -lt 203 -or -not $currentHashes.ContainsKey('194_load_timer_regressive_elapsed.png') -or
        -not $currentHashes.ContainsKey('197_load_timer_duplicate_accepted.png')) { throw 'Missing accepted-report regression renders.' }
    $retainedRender = Join-Path $runDirectory 'render'
    [void](New-Item -ItemType Directory -Path $retainedRender)
    Copy-Item -LiteralPath (Join-Path $renderDirectory 'render_manifest.json') -Destination (Join-Path $retainedRender 'render_manifest.json')
    foreach ($entry in $render.images) {Copy-Item -LiteralPath (Join-Path $renderDirectory $entry.image) -Destination (Join-Path $retainedRender $entry.image)}
    [IO.File]::WriteAllText((Join-Path $runDirectory 'visual_baseline.json'),
        (@{baseline=$controlBaseline;unrelated_pages_identical=131;render_count=$render.images.Count;software_only=$true}|ConvertTo-Json),[Text.UTF8Encoding]::new($false))


}
catch {$failureMessage = $_.Exception.Message}
finally {
    Pop-Location
    $sourceAfter = @(Get-RevisionInputs)
    $sourcesUnchanged = (($sourceBefore|ConvertTo-Json -Depth 4 -Compress) -ceq ($sourceAfter|ConvertTo-Json -Depth 4 -Compress)) -and
        ($scriptHashBefore -ceq (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash)
    $manifest = [ordered]@{schema='single-static-startup-acceptance-v2';acceptance_profile='current-static-startup';target='LCD-SingleStatic-Test';run_id=$runId;
        started_at_utc=$startedAt.ToString('o');completed_at_utc=[DateTime]::UtcNow.ToString('o');
        software_only=$true;hardware_tested=$false;control_baseline=$controlBaseline;reviewed_startup_revision=$ReviewedStartupRevision;control_sources_unchanged=$controlSourcesUnchanged;
        rtos_stack_baseline=$rtosStackBaseline;rtos_stack_override_scope='static-trace-and-integrated-lcd-task-stacks';
        sources_unchanged=$sourcesUnchanged;passed=($null -eq $failureMessage -and $sourcesUnchanged -and $buildRecords.Count -eq 3);
        failure=$failureMessage;lcd_checks=if($null-ne$lcdSummary){$lcdSummary.checks.Count}else{0};
        records=@($records.ToArray());builds=@($buildRecords.ToArray());source_hashes=$sourceAfter;
        verification_script_path=$PSCommandPath;verification_script_sha256=$scriptHashBefore;
        evidence_hashes=@(Get-ChildItem -LiteralPath $runDirectory -Recurse -File|ForEach-Object {
            [ordered]@{path=$_.FullName;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash}})}
    $manifestPath = Join-Path $runDirectory 'manifest.json'
    [IO.File]::WriteAllText($manifestPath,($manifest|ConvertTo-Json -Depth 12),[Text.UTF8Encoding]::new($false))
    Write-Output ('STATIC_TRIALS_MANIFEST='+$manifestPath)
}
if (-not $manifest.passed) {throw ('Offline acceptance failed: '+$failureMessage)}
Write-Output 'STATIC_STARTUP_THREE_TARGET_OFFLINE_ACCEPTANCE_PASSED'
