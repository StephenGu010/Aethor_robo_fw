<#
.SYNOPSIS
Runs the reproducible software-only LCD regression suite and optional Keil builds.
.DESCRIPTION
Each child has a separate log and exit-code check. No serial port, probe, motor,
or firmware download is opened. Python and GCC paths can be overridden locally.
#>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [string]$KeilPath = 'D:\application\keil\UV4\UV4.exe',
    [switch]$IncludeKeil
)
$ErrorActionPreference = 'Stop'

function Get-DebugUiSourceFingerprint {
    <# Hash shared first-party build inputs before/after the multi-process verification. #>
    $inputs = foreach ($directory in @('App', 'Ui', 'Core')) {
        Get-ChildItem -LiteralPath (Join-Path $projectRoot $directory) -Recurse -File |
            Where-Object { $_.Extension -in @('.c', '.h', '.inc') }
    }
    $inputs += Get-Item -LiteralPath (Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'),
        (Join-Path $projectRoot 'MDK-ARM\aethor_memory.sct'),
        (Join-Path $projectRoot 'CtrBoard-H7_FDCAN.ioc')
    $entries = foreach ($file in ($inputs | Sort-Object FullName)) {
        [ordered]@{ path = $file.FullName; sha256 = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash }
    }
    return @($entries)
}

function Invoke-RecordedCheck {
    <# Run one bounded test process, retaining stdout/stderr and elapsed time. #>
    param([string]$Name, [string]$Executable, [string[]]$Arguments)
    $stdoutPath = Join-Path $logDirectory ($Name + '.log')
    $stderrPath = Join-Path $logDirectory ($Name + '.stderr.log')
    $startedAt = [DateTime]::UtcNow
    $child = Start-Process -FilePath $Executable -ArgumentList $Arguments `
        -WorkingDirectory $projectRoot -WindowStyle Hidden -Wait -PassThru `
        -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
    $record = [ordered]@{
        name = $Name; exit_code = $child.ExitCode
        elapsed_seconds = [Math]::Round(([DateTime]::UtcNow - $startedAt).TotalSeconds, 3)
        stdout = $stdoutPath; stderr = $stderrPath
    }
    $records.Add($record)
    Write-Output ("CHECK {0} exit={1} seconds={2}" -f $Name, $record.exit_code, $record.elapsed_seconds)
    if ($child.ExitCode -ne 0) {
        Get-Content -LiteralPath $stdoutPath -Tail 12
        Get-Content -LiteralPath $stderrPath -Tail 12
    }
}

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$logDirectory = Join-Path $projectRoot 'Tests\host\build\verification'
[void](New-Item -ItemType Directory -Path $logDirectory -Force)
foreach ($executable in @($Compiler, $Python)) {
    if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
        throw "Required executable was not found: $executable"
    }
}
$env:PATH = (Split-Path $Compiler) + ';' + (Split-Path $Python) + ';' + $env:PATH
$powershell = Join-Path $PSHOME 'powershell.exe'
$records = [System.Collections.Generic.List[object]]::new()
$sourceBefore = Get-DebugUiSourceFingerprint
$testNames = @(
    'check_phase0_architecture', 'test_architecture_publication_check',
    'check_debug_ui_architecture', 'run_phase0_tests', 'run_text_protocol_tests',
    'run_text_protocol_engine_tests', 'run_text_protocol_arm_profile_tests',
    'run_motor_core_tests', 'run_motion_tests', 'run_platform_io_tests',
    'run_protocol_tests', 'run_simulator_tests', 'run_debug_ui_input_tests',
    'run_lcd_transport_tests', 'run_debug_ui_mailbox_tests',
    'run_debug_ui_app_tests', 'run_debug_ui_model_tests', 'run_debug_ui_soak_tests'
)
foreach ($name in $testNames) {
    $testArguments = @('-NoProfile', '-ExecutionPolicy', 'Bypass',
        '-File', ('"' + (Join-Path $projectRoot ("Tests\host\$name.ps1")) + '"'))
    if ($name -like 'run_debug_ui_*' -or $name -eq 'run_lcd_transport_tests') {
        $testArguments += @('-Compiler', ('"' + $Compiler + '"'))
    }
    Invoke-RecordedCheck $name $powershell $testArguments
}
Invoke-RecordedCheck 'test_debug_ui_build_contract' $Python @(
    ('"' + (Join-Path $projectRoot 'Tests\host\test_debug_ui_build_contract.py') + '"'),
    '--compiler', ('"' + $Compiler + '"'))
Invoke-RecordedCheck 'check_numeric_font_subset' $Python @(
    ('"' + (Join-Path $projectRoot 'Ui\fonts\generate_numeric_28.py') + '"'), '--check')
Invoke-RecordedCheck 'run_input_model_integration' $powershell @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', ('"' + (Join-Path $projectRoot 'Tests\ui\run_input_model_integration.ps1') + '"'),
    '-Compiler', ('"' + $Compiler + '"'))
Invoke-RecordedCheck 'run_ui_render_smoke' $powershell @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', ('"' + (Join-Path $projectRoot 'Tests\ui\run_ui_render_smoke.ps1') + '"'),
    '-Compiler', ('"' + $Compiler + '"'), '-Python', ('"' + $Python + '"'))
Invoke-RecordedCheck 'run_ui_review_regressions' $powershell @('-NoProfile', '-ExecutionPolicy', 'Bypass',
    '-File', ('"' + (Join-Path $projectRoot 'Tests\ui\run_review_regressions.ps1') + '"'),
    '-Compiler', ('"' + $Compiler + '"'))
if ($IncludeKeil) {
    foreach ($targetName in @('CtrBoard-H7_FDCAN', 'LCD-ReadOnly', 'LCD-POS', 'LCD-MIT')) {
        Invoke-RecordedCheck ('keil_' + $targetName) $powershell @('-NoProfile', '-ExecutionPolicy',
            'Bypass', '-File', ('"' + (Join-Path $PSScriptRoot 'build_debug_ui.ps1') + '"'),
            '-Target', $targetName, '-KeilPath', ('"' + $KeilPath + '"'))
    }
    Invoke-RecordedCheck 'check_debug_ui_map' $Python @(
        ('"' + (Join-Path $projectRoot 'Tests\host\check_debug_ui_map.py') + '"'))
}
$summaryPath = Join-Path $logDirectory 'summary.json'
$sourceAfter = Get-DebugUiSourceFingerprint
$sourcesUnchanged = (($sourceBefore | ConvertTo-Json -Depth 4 -Compress) -ceq
    ($sourceAfter | ConvertTo-Json -Depth 4 -Compress))
$summary = [ordered]@{
    completed_at_utc = [DateTime]::UtcNow.ToString('o')
    software_only = $true; hardware_tested = $false
    compiler = $Compiler; python = $Python; keil_requested = [bool]$IncludeKeil
    checks = @($records.ToArray())
    first_party_sources_unchanged_during_verification = $sourcesUnchanged
    first_party_source_hashes = $sourceAfter
}
[System.IO.File]::WriteAllText($summaryPath, ($summary | ConvertTo-Json -Depth 6),
    [System.Text.UTF8Encoding]::new($false))
if ((-not $sourcesUnchanged) -or @($records | Where-Object { $_.exit_code -ne 0 }).Count -ne 0) {
    throw "Software verification has failed checks; inspect $summaryPath"
}
Write-Output ("DEBUG_UI_SOFTWARE_VERIFICATION_PASSED checks={0} summary={1}" -f $records.Count, $summaryPath)
