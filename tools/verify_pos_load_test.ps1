<#
.SYNOPSIS
Run the complete software-only POS acceptance and freeze every maintained input.
.DESCRIPTION
No serial, probe, firmware download or motor request is opened. All old LCD,
POS/MIT/static/STOP and preserved ADRC checks plus real LVGL renders must pass.
Four independently named Keil images must build with zero errors/warnings; their
HEX/AXF/map/callgraph/log bytes and all sources (including new .inc) are hashed.
#>
[CmdletBinding()]
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [string]$KeilPath = 'D:\application\keil\UV4\UV4.exe'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$runId = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
$evidenceRoot = Join-Path $projectRoot 'output\pos_load_auto_20261020'
[void](New-Item -ItemType Directory -Path $evidenceRoot -Force)
$runDirectory = Join-Path $evidenceRoot $runId
if (Test-Path -LiteralPath $runDirectory) { throw 'Acceptance output must be new.' }
[void](New-Item -ItemType Directory -Path $runDirectory)
$records = [System.Collections.Generic.List[object]]::new()
$buildRecords = [System.Collections.Generic.List[object]]::new()
$failureMessage = $null
$lcdSummary = $null
$sourcesUnchanged = $false
$startedAt = [DateTime]::UtcNow
$scriptHashBefore = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
$powershellExecutable = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
$env:PATH = (Split-Path $Compiler) + ';' + (Split-Path $Python) + ';' + $env:PATH
$env:PYTHONDONTWRITEBYTECODE = '1'
$env:PYTHONUTF8 = '1'
# Preserve the established local pure-Python HEX/ELF package locations.
$env:PYTHONPATH = 'D:\download\TCG\output\can_id_commissioning_20260909\python_deps;D:\download\TCG\Aethor_robo_fw\.worktrees\s3519-adrc-lcd-mit\output\adrc\hardware\python_deps_20261010;' + $env:PYTHONPATH

function Get-RevisionInputs {
    <# Enumerate the same git-maintained sources as the independent Python acceptance gate. #>
    $paths = @(git -c ('safe.directory=' + $projectRoot.Replace('\','/')) -C $projectRoot ls-files `
        --cached --others --exclude-standard -- .gitignore App Core Ui Drivers Middlewares USB_DEVICE `
        Tests/host Tests/ui Tests/hardware tools Models MDK-ARM CtrBoard-H7_FDCAN.ioc)
    if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate maintained inputs.' }
    foreach ($relativePath in ($paths | Sort-Object -Unique)) {
        if ($relativePath -match '(^|/)build/' -or ($relativePath -ne '.gitignore' -and
            [IO.Path]::GetExtension($relativePath) -notin @('.c','.h','.inc','.s','.m','.ps1','.py','.json','.txt','.uvprojx','.sct','.ioc'))) { continue }
        [ordered]@{path=$relativePath;sha256=(Get-FileHash -LiteralPath (Join-Path $projectRoot $relativePath) -Algorithm SHA256).Hash.ToLowerInvariant()}
    }
}

function Invoke-RecordedCheck {
    <# Run one offline child with retained stdout/stderr and reject every nonzero status. #>
    param([string]$Name,[string]$Executable,[string[]]$Arguments)
    $stdout = Join-Path $runDirectory ($Name+'.log')
    $stderr = Join-Path $runDirectory ($Name+'.stderr.log')
    $started = [DateTime]::UtcNow
    $previousPythonUtf8 = $env:PYTHONUTF8
    try {
        if ($Name -in @('single_static_build_tests','pos_load_build_tests')) { $env:PYTHONUTF8 = '0' }
        $child = Start-Process -FilePath $Executable -ArgumentList $Arguments -WorkingDirectory $projectRoot `
            -WindowStyle Hidden -Wait -PassThru -RedirectStandardOutput $stdout -RedirectStandardError $stderr
    }
    finally { $env:PYTHONUTF8 = $previousPythonUtf8 }
    $records.Add([ordered]@{name=$Name;exit_code=$child.ExitCode;log=$stdout;stderr=$stderr;
        elapsed_seconds=[Math]::Round(([DateTime]::UtcNow-$started).TotalSeconds,3)})
    Write-Output ('CHECK '+$Name+' exit='+$child.ExitCode)
    if ($null -eq $child.ExitCode -or $child.ExitCode -ne 0) { throw ('Offline check failed: '+$Name) }
}

function Copy-FreshEvidence {
    <# Copy one explicitly named fresh file and reject concurrent source/copy mutation. #>
    param([string]$Source,[string]$Destination,[DateTime]$EarliestTime,[switch]$AllowEmpty)
    $item = Get-Item -LiteralPath $Source
    if ((-not $AllowEmpty -and $item.Length -eq 0) -or $item.LastWriteTimeUtc -lt $EarliestTime -or
        $item.LastWriteTimeUtc -gt [DateTime]::UtcNow) { throw ('Empty or stale evidence: '+$Source) }
    $before = (Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash
    Copy-Item -LiteralPath $Source -Destination $Destination
    if ($before -cne (Get-FileHash -LiteralPath $Destination -Algorithm SHA256).Hash -or
        $before -cne (Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash) { throw ('Changed evidence: '+$Source) }
}

function Save-FirmwareEvidence {
    <# Freeze exact fresh clean HEX/AXF/map/callgraph/log bytes for one independent target. #>
    param([string]$TargetName,[DateTime]$EarliestTime,[string]$BuildLog)
    $log = Get-Content -LiteralPath $BuildLog -Raw
    $summaries = [regex]::Matches($log, '(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)')
    if ($summaries.Count -ne 1 -or $summaries[0].Groups[1].Value -ne '0' -or
        $summaries[0].Groups[2].Value -ne '0') { throw ('Unclean build log: '+$TargetName) }
    $destinations = @{}
    foreach ($extension in @('axf','hex','map','htm')) {
        $source = Join-Path $projectRoot ('MDK-ARM\'+$TargetName+'\'+$TargetName+'.'+$extension)
        $destination = Join-Path $runDirectory ($TargetName+'.'+$extension)
        Copy-FreshEvidence $source $destination $EarliestTime
        $destinations[$extension] = $destination
    }
    $retainedLog = Join-Path $runDirectory ($TargetName+'.keil.log')
    Copy-FreshEvidence $BuildLog $retainedLog $EarliestTime
    $buildRecords.Add([ordered]@{target=$TargetName;errors=0;warnings=0;hex_path=$destinations['hex'];
        elf_path=$destinations['axf'];map_path=$destinations['map'];htm_path=$destinations['htm'];log_path=$retainedLog})
}

$sourceBefore = @(Get-RevisionInputs)
Push-Location $projectRoot
try {
    foreach ($executable in @($Compiler,$Python,$KeilPath)) {
        if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) { throw ('Required executable missing: '+$executable) }
    }
    $suiteStarted = [DateTime]::UtcNow
    Invoke-RecordedCheck 'lcd_suite' $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File',
        'tools/verify_debug_ui.ps1','-Compiler',('"'+$Compiler+'"'),'-Python',('"'+$Python+'"'))
    $lcdDirectory = Join-Path $projectRoot 'Tests\host\build\verification'
    $lcdSummary = Get-Content -LiteralPath (Join-Path $lcdDirectory 'summary.json') -Raw | ConvertFrom-Json
    if (-not $lcdSummary.first_party_sources_unchanged_during_verification -or $lcdSummary.checks.Count -ne 27 -or
        @($lcdSummary.checks | Where-Object {$_.exit_code -ne 0}).Count -ne 0) { throw 'Expected 27 clean source-stable host/LVGL checks.' }
    $lcdRetained = Join-Path $runDirectory 'lcd_evidence'
    [void](New-Item -ItemType Directory -Path $lcdRetained)
    $lcdFiles = @('summary.json') + @($lcdSummary.checks | ForEach-Object {
        [IO.Path]::GetFileName($_.stdout)
        [IO.Path]::GetFileName($_.stderr)
    })
    foreach ($fileName in $lcdFiles) {
        Copy-FreshEvidence (Join-Path $lcdDirectory $fileName) (Join-Path $lcdRetained $fileName) $suiteStarted -AllowEmpty
    }
    $hostChecks = [ordered]@{
        pos_load_app_tests='Tests/host/run_pos_load_app_tests.ps1'
        pos_load_model_tests='Tests/ui/run_pos_load_model_tests.ps1'
        pos_load_lcd_workflow='Tests/host/run_pos_load_lcd_workflow_tests.ps1'
        pos_load_platform_tests='Tests/host/run_pos_load_platform_tests.ps1'
        adrc_shaft='Tests/host/run_adrc_shaft_pilot_tests.ps1'
        adrc_lcd_integration='Tests/host/run_adrc_lcd_integration_tests.ps1'
        single_static_platform='Tests/host/run_single_static_platform_tests.ps1'
        load_trials='Tests/host/run_load_trials_tests.ps1'
        load_trial_pages='Tests/ui/run_load_trial_pages_tests.ps1'
        startup_sync_tests='Tests/host/run_static_startup_sync_tests.ps1'
    }
    foreach ($name in $hostChecks.Keys) {
        Invoke-RecordedCheck $name $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File',
            ('"'+$hostChecks[$name]+'"'),'-Compiler',('"'+$Compiler+'"'))
    }
    $pythonChecks = [ordered]@{
        startup_trace_export_tests='Tests/host/test_static_startup_trace.py'
        trace_map_tests='Tests/host/test_static_trace_map.py'
        single_static_build_tests='Tests/host/test_single_static_build.py'
        single_static_deploy_tests='Tests/hardware/test_single_static_deploy.py'
        pos_load_build_tests='Tests/host/test_pos_load_build.py'
        pos_load_deploy_tests='Tests/hardware/test_pos_load_deploy.py'
    }
    foreach ($name in $pythonChecks.Keys) { Invoke-RecordedCheck $name $Python @('-B',$pythonChecks[$name]) }
    foreach ($target in @('LCD-POS','LCD-MIT','LCD-SingleStatic-Test','LCD-POS-Load-Test')) {
        $buildStarted = [DateTime]::UtcNow
        if ($target -in @('LCD-POS','LCD-MIT')) {
            Invoke-RecordedCheck ('keil_'+$target) $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File',
                'tools/build_debug_ui.ps1','-Target',$target,'-KeilPath',('"'+$KeilPath+'"'))
        } else {
            $builder = if ($target -eq 'LCD-POS-Load-Test') {'tools/build_pos_load_test.ps1'} else {'tools/build_single_static_test.ps1'}
            $buildEvidence = Join-Path $runDirectory ($target+'_build')
            Invoke-RecordedCheck ('keil_'+$target) $powershellExecutable @('-NoProfile','-ExecutionPolicy','Bypass','-File',
                $builder,'-EvidenceDirectory',('"'+$buildEvidence+'"'),'-KeilPath',('"'+$KeilPath+'"'))
        }
        Save-FirmwareEvidence $target $buildStarted (Join-Path $projectRoot ('Tests\host\build\keil\'+$target+'.log'))
    }
    Invoke-RecordedCheck 'map_pos_load' $Python @('-B','Tests/host/check_debug_ui_map.py',
        '--target','LCD-POS','--target','LCD-MIT','--target','LCD-SingleStatic-Test','--target','LCD-POS-Load-Test',
        '--evidence-directory',('"'+$runDirectory+'"'))
    Copy-Item -LiteralPath (Join-Path $lcdDirectory 'memory-layout.json') -Destination (Join-Path $runDirectory 'memory-layout.json')
    $renderDirectory = Join-Path $projectRoot 'Tests\ui\artifacts-astra'
    $render = Get-Content -LiteralPath (Join-Path $renderDirectory 'render_manifest.json') -Raw | ConvertFrom-Json
    if (@($render.images).Count -eq 0) { throw 'Real LVGL render manifest is empty.' }
    $retainedRender = Join-Path $runDirectory 'render'
    [void](New-Item -ItemType Directory -Path $retainedRender)
    Copy-FreshEvidence (Join-Path $renderDirectory 'render_manifest.json') (Join-Path $retainedRender 'render_manifest.json') $suiteStarted
    foreach ($entry in $render.images) {
        if ([IO.Path]::GetFileName($entry.image) -cne $entry.image) { throw 'Invalid LVGL image identity.' }
        Copy-FreshEvidence (Join-Path $renderDirectory $entry.image) (Join-Path $retainedRender $entry.image) $suiteStarted
    }
}
catch { $failureMessage = $_.Exception.Message }
finally {
    Pop-Location
    $sourceAfter = @(Get-RevisionInputs)
    $sourcesUnchanged = (($sourceBefore|ConvertTo-Json -Depth 4 -Compress) -ceq ($sourceAfter|ConvertTo-Json -Depth 4 -Compress)) -and
        ($scriptHashBefore -ceq (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant())
    $manifest = [ordered]@{schema='pos-load-acceptance-v1';target='LCD-POS-Load-Test';
        feature_gate='AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1';trial_mode=2;run_id=$runId;
        started_at_utc=$startedAt.ToString('o');completed_at_utc=[DateTime]::UtcNow.ToString('o');
        software_only=$true;hardware_tested=$false;sources_unchanged=$sourcesUnchanged;
        passed=($null -eq $failureMessage -and $sourcesUnchanged -and $buildRecords.Count -eq 4);
        failure=$failureMessage;lcd_checks=if($null-ne$lcdSummary){$lcdSummary.checks.Count}else{0};
        previous_image_sha256='c333079f935117a0ff3d0260280f4a07dc9f87e5a626a30c8c67ee11392602dc';
        records=@($records.ToArray());builds=@($buildRecords.ToArray());source_hashes=$sourceAfter;
        verification_script_path=$PSCommandPath;verification_script_sha256=$scriptHashBefore;
        evidence_hashes=@(Get-ChildItem -LiteralPath $runDirectory -Recurse -File|ForEach-Object {
            [ordered]@{path=$_.FullName;sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}})}
    $manifestPath = Join-Path $runDirectory 'manifest.json'
    [IO.File]::WriteAllText($manifestPath,($manifest|ConvertTo-Json -Depth 12),[Text.UTF8Encoding]::new($false))
    $manifestHash = (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant()
    Write-Output ('POS_LOAD_MANIFEST='+$manifestPath)
    Write-Output ('POS_LOAD_MANIFEST_SHA256='+$manifestHash)
}
if (-not $manifest.passed) { throw ('Offline acceptance failed: '+$failureMessage+' sources_unchanged='+$sourcesUnchanged) }
# Reopen the frozen bytes through the same strict schema used at deployment; no hardware flags.
& $Python -B (Join-Path $projectRoot 'tools/deploy_pos_load_test.py') --manifest $manifestPath --manifest-sha256 $manifestHash
if ($LASTEXITCODE -ne 0) {
    $manifest.passed = $false
    $manifest.failure = 'Independent frozen POS acceptance verification failed.'
    [IO.File]::WriteAllText($manifestPath,($manifest|ConvertTo-Json -Depth 12),[Text.UTF8Encoding]::new($false))
    Write-Output ('POS_LOAD_REJECTED_MANIFEST_SHA256=' + (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash.ToLowerInvariant())
    throw $manifest.failure
}
Write-Output 'POS_LOAD_FOUR_TARGET_OFFLINE_ACCEPTANCE_PASSED'
