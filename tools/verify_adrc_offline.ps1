<#
.SYNOPSIS
Verify ADRC software modules without opening a serial port or flashing firmware.
.DESCRIPTION
Records fresh host-test logs, ARMCC object compilation and optional full Keil builds.
Compilation and linking do not prove real-time execution or hardware safety.
#>
[CmdletBinding()]
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$ArmCompiler = 'D:\application\keil\ARM\ARMCC\bin\armcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [switch]$IncludeLegacy,
    [switch]$IncludeAdrcKeil,
    [string]$KeilPath = 'D:\application\keil\UV4\UV4.exe'
)
$ErrorActionPreference = 'Stop'

function Get-AdrcVerificationInputs {
    <# Hash maintained inputs to detect edits made while verification is running. #>
    param([string]$ProjectRoot)
    $inputFiles = foreach ($relativeDirectory in @('App', 'Core', 'Ui', 'Drivers', 'Middlewares',
            'USB_DEVICE', 'Tests\host', 'Tests\hardware', 'Tests\ui', 'Models\Adrc', 'tools',
            'output\adrc\shaft_identification_design')) {
        $directory = Join-Path $ProjectRoot $relativeDirectory
        if (Test-Path -LiteralPath $directory) {
            Get-ChildItem -LiteralPath $directory -Recurse -File |
                Where-Object { $_.Extension -in @('.c', '.h', '.s', '.inc', '.m', '.ps1', '.py') -and
                    $_.FullName -notmatch '[\\/]build[\\/]' }
        }
    }
    $inputFiles += Get-Item -LiteralPath $PSCommandPath
    $inputFiles += Get-Item -LiteralPath (Join-Path $ProjectRoot 'Core\Src\freertos.c'),
        (Join-Path $ProjectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'),
        (Join-Path $ProjectRoot 'MDK-ARM\ADRC-Bench.uvprojx'),
        (Join-Path $ProjectRoot 'MDK-ARM\LCD-MIT-ADRC.uvprojx'),
        (Join-Path $ProjectRoot 'MDK-ARM\aethor_memory.sct'),
        (Join-Path $ProjectRoot 'MDK-ARM\startup_stm32h723xx.s'),
        (Join-Path $ProjectRoot 'Ui\fonts\generate_numeric_28.py'),
        (Join-Path $ProjectRoot 'output\adrc\wide_position_20261016\apply_wide_position.py'),
        (Join-Path $ProjectRoot 'output\adrc\wide_position_20261016\test_apply_wide_position.py'),
        (Join-Path $ProjectRoot 'output\adrc\rated70_20261016\rated70_maintenance.py'),
        (Join-Path $ProjectRoot 'output\adrc\rated70_20261016\test_rated70_maintenance.py'),
        (Join-Path $ProjectRoot 'output\adrc\rated70_20261016\apply_rated70.py'),
        (Join-Path $ProjectRoot 'output\adrc\rated70_20261016\deploy_rated70.py'),
        (Join-Path $ProjectRoot 'Drivers\STM32H7xx_HAL_Driver\Inc\stm32h7xx_hal_fdcan.h')
    foreach ($inputFile in ($inputFiles | Sort-Object FullName -Unique)) {
        [ordered]@{ path = $inputFile.FullName
            sha256 = (Get-FileHash -LiteralPath $inputFile.FullName -Algorithm SHA256).Hash }
    }
}

function Invoke-AdrcRecordedCommand {
    <# Run one native tool with structured arguments and retain its actual exit status. #>
    param([string]$Name, [string]$Executable, [string[]]$ToolArguments,
        [string]$LogDirectory, [System.Collections.Generic.List[object]]$Records)
    $startedAt = [DateTime]::UtcNow
    $logPath = Join-Path $LogDirectory ($Name + '.log')
    $savedErrorPreference = $ErrorActionPreference
    try {
        # Windows PowerShell wraps native stderr as ErrorRecord; preserve diagnostics
        # and collect the process exit code before treating failure as terminating.
        $ErrorActionPreference = 'Continue'
        & $Executable @ToolArguments 2>&1 | ForEach-Object { $_.ToString() } |
            Tee-Object -FilePath $logPath | Write-Output
        $nativeExitCode = $LASTEXITCODE
    }
    finally { $ErrorActionPreference = $savedErrorPreference }
    $Records.Add([ordered]@{ name = $Name; exit_code = $nativeExitCode
        elapsed_seconds = [Math]::Round(([DateTime]::UtcNow - $startedAt).TotalSeconds, 3)
        log = $logPath })
    if ($nativeExitCode -ne 0) { throw "Offline check failed: $Name (exit $nativeExitCode)." }
}

function Invoke-AdrcKeilBuild {
    <# Rebuild one experimental target without downloading and bind fresh linked artifacts to its log. #>
    param([string]$ProjectRoot, [string]$TargetName, [string]$LogDirectory,
        [System.Collections.Generic.List[object]]$Records)
    $projectPath = Join-Path $ProjectRoot ('MDK-ARM\' + $TargetName + '.uvprojx')
    $logPath = Join-Path $LogDirectory ($TargetName + '.log')
    $startedAt = [DateTime]::UtcNow
    $buildArguments = @('-r', ('"' + $projectPath + '"'), '-t', $TargetName,
        '-j0', '-o', ('"' + $logPath + '"'))
    $buildProcess = Start-Process -FilePath $KeilPath -ArgumentList $buildArguments `
        -WindowStyle Hidden -Wait -PassThru
    $Records.Add([ordered]@{ name = ('keil_' + $TargetName); exit_code = $buildProcess.ExitCode
        elapsed_seconds = [Math]::Round(([DateTime]::UtcNow - $startedAt).TotalSeconds, 3); log = $logPath })
    if (-not (Test-Path -LiteralPath $logPath -PathType Leaf)) { throw "Missing Keil log: $TargetName" }
    $logItem = Get-Item -LiteralPath $logPath
    $logText = Get-Content -LiteralPath $logPath -Raw
    if ($buildProcess.ExitCode -ne 0 -or $logItem.LastWriteTimeUtc -lt $startedAt.AddSeconds(-2) -or
        $logText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
        throw "Keil target did not finish cleanly: $TargetName; see $logPath"
    }
    foreach ($extension in @('axf', 'hex')) {
        $artifactPath = Join-Path $ProjectRoot ("MDK-ARM\$TargetName\$TargetName.$extension")
        if (-not (Test-Path -LiteralPath $artifactPath -PathType Leaf) -or
            (Get-Item -LiteralPath $artifactPath).LastWriteTimeUtc -lt $startedAt.AddSeconds(-2)) {
            throw "Missing or stale Keil artifact: $artifactPath"
        }
        $retainedPath = Join-Path $LogDirectory ("$TargetName.$extension")
        Copy-Item -LiteralPath $artifactPath -Destination $retainedPath
    }
    Get-Content -LiteralPath $logPath -Tail 5 | Out-Host
}

function Invoke-AdrcOfflineVerification {
    <# Execute the offline acceptance subset and write an honest manifest even after failure. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    foreach ($executable in @($Compiler, $ArmCompiler, $Python)) {
        if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
            throw "Required compiler not found: $executable"
        }
    }
    if ($IncludeAdrcKeil -and -not (Test-Path -LiteralPath $KeilPath -PathType Leaf)) {
        throw "Keil executable not found: $KeilPath"
    }
    $runId = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfffZ')
    $logDirectory = Join-Path $projectRoot ('output\adrc\verification\' + $runId)
    [void](New-Item -ItemType Directory -Path $logDirectory -Force)
    $records = [System.Collections.Generic.List[object]]::new()
    $sourceBefore = @(Get-AdrcVerificationInputs $projectRoot)
    $savedSearchPath = $env:PATH
    $failureMessage = $null
    $powershellExecutable = Join-Path $env:SystemRoot 'System32\WindowsPowerShell\v1.0\powershell.exe'
    Push-Location $projectRoot
    try {
        $env:PATH = (Split-Path $Compiler) + ';' + $savedSearchPath
        Invoke-AdrcRecordedCommand 'armcc_version' $ArmCompiler @('--vsn') $logDirectory $records
        Invoke-AdrcRecordedCommand 'shaft_task_receipt_contract' $Python @('Tests/host/test_adrc_shaft_task_contract.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'shaft_canonical_evidence' $Python @('-m', 'unittest', 'discover',
            '-s', 'output/adrc/shaft_identification_design', '-p', 'test_*.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'wide_position_maintenance_parser' $Python @('-B', '-m', 'unittest', 'discover',
            '-s', 'output/adrc/wide_position_20261016', '-p', 'test_apply_wide_position.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'rated70_disabled_maintenance' $Python @('-B', '-m', 'unittest', 'discover',
            '-s', 'output/adrc/rated70_20261016', '-p', 'test_rated70_maintenance.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'target_evidence_gate' $Python @('Tests/host/test_adrc_target_gate.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'offline_analysis_contract' $Python @('Tests/host/test_adrc_offline_analysis.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'offline_stop_tuning_contract' $Python @('Tests/host/test_adrc_stop_tuning.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'font_line_ending_contract' $Python @('Tests/host/test_numeric_font_line_endings.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'poweroff_deploy_fail_closed' $Python @('Tests/host/test_adrc_poweroff_deploy.py') $logDirectory $records
        foreach ($testName in @('run_adrc_experiment_tests', 'run_adrc_protocol_tests',
                'run_adrc_bench_tests', 'run_adrc_channel_tests', 'run_adrc_app_tests',
                'run_adrc_feedback_adapter_tests', 'run_adrc_simple_trial_tests', 'run_adrc_shaft_pilot_tests', 'test_adrc_shaft_host', 'test_adrc_simple_host', 'run_adrc_lcd_ownership_tests',
                'run_adrc_lcd_integration_tests', 'run_motor_capability_collector_tests', 'test_adrc_capability_host', 'run_platform_io_tests',
                'run_text_protocol_engine_tests', 'run_motor_core_tests', 'run_adrc_wide_position_codec_tests')) {
            $testScript = Join-Path $projectRoot ('Tests\host\' + $testName + '.ps1')
            Invoke-AdrcRecordedCommand $testName $powershellExecutable @('-NoProfile',
                '-ExecutionPolicy', 'Bypass', '-File', $testScript) $logDirectory $records
        }
        # Compile separate and combined static gates without enabling any firmware target.
        foreach ($staticConfiguration in @(
            @{ Name = 'static'; Seven = 1; Single = 0 },
            @{ Name = 'single_static'; Seven = 0; Single = 1 },
            @{ Name = 'both_static'; Seven = 1; Single = 1 })) {
        foreach ($staticSource in @('App/aethor_app.c', 'App/DebugUi/debug_ui_model.c',
                'App/DebugUi/debug_ui_mailbox.c', 'App/Protocol/protocol_engine.c')) {
            $staticName = $staticConfiguration.Name + '_' + [IO.Path]::GetFileNameWithoutExtension($staticSource)
            $staticObject = Join-Path $logDirectory ($staticName + '.o')
            $staticArguments = @('--c99', '--cpu=Cortex-M7', '--fpu=FPv5_D16', '-O3',
                '--diag_error=warning', '--no_multibyte_chars', '-DAETHOR_DEBUG_UI_ENABLE=1',
                '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1',
                ('-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=' + $staticConfiguration.Seven),
                ('-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=' + $staticConfiguration.Single),
                '-IApp', '-IApp/Adrc', '-IApp/DebugUi',
                '-IApp/Arm', '-IApp/Config', '-IApp/Motion', '-IApp/Motor', '-IApp/Protocol',
                '-IApp/Telemetry', '-IApp/Platform', '-c', $staticSource, '-o', $staticObject)
            Invoke-AdrcRecordedCommand ('armcc_' + $staticName) $ArmCompiler $staticArguments $logDirectory $records
            if (-not (Test-Path -LiteralPath $staticObject -PathType Leaf)) {
                throw "Static compiler reported success without creating $staticObject"
            }
        }
        }
        foreach ($source in @('App/Adrc/adrc_experiment.c', 'App/Adrc/adrc_protocol.c', 'App/Adrc/adrc_bench.c',
                'App/Adrc/adrc_simple_trial.c', 'App/Adrc/adrc_shaft_pilot.c', 'App/Adrc/adrc_shaft_pilot_adapter.c', 'App/Adrc/adrc_feedback_adapter.c', 'App/Adrc/adrc_app_bridge.c', 'App/Motor/motor_adrc_command.c', 'App/Motor/motor_capability_collector.c',
                'App/Protocol/protocol_engine.c')) {
            $objectName = [IO.Path]::GetFileNameWithoutExtension($source)
            $objectPath = Join-Path $logDirectory ($objectName + '.o')
            $compileArguments = @('--c99', '--cpu=Cortex-M7', '--fpu=FPv5_D16',
                '--diag_error=warning', '--no_multibyte_chars', '-IApp/Adrc',
                '-IApp/Arm', '-IApp/Config', '-IApp/Motion', '-IApp/Motor',
                '-IApp/Protocol', '-IApp/Telemetry', '-c', $source, '-o', $objectPath)
            Invoke-AdrcRecordedCommand ('armcc_' + $objectName) $ArmCompiler $compileArguments $logDirectory $records
            if (-not (Test-Path -LiteralPath $objectPath -PathType Leaf)) {
                throw "Compiler reported success without creating $objectPath"
            }
        }
        # Compile the actual task and HAL channel with the experimental switch enabled.
        # This checks real vendor headers, but does not link or replace the generated controller.
        $projectFile = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
        [xml]$projectXml = Get-Content -LiteralPath $projectFile -Raw
        $readOnlyTarget = @($projectXml.Project.Targets.Target |
            Where-Object { $_.TargetName -eq 'LCD-ReadOnly' })
        if ($readOnlyTarget.Count -ne 1) { throw 'Expected one LCD-ReadOnly target.' }
        $controls = $readOnlyTarget[0].TargetOption.TargetArmAds.Cads.VariousControls
        $platformArguments = @('--c99', '--cpu=Cortex-M7', '--fpu=FPv5_D16',
            '--diag_error=warning', '--no_multibyte_chars', '-DAETHOR_ADRC_BENCH=1', '-IApp/Adrc')
        foreach ($definition in ([string]$controls.Define -split ',')) {
            if ($definition) { $platformArguments += '-D' + $definition }
        }
        foreach ($include in ([string]$controls.IncludePath -split ';')) {
            if ($include) {
                $platformArguments += '-I' + [IO.Path]::GetFullPath(
                    (Join-Path (Split-Path $projectFile) $include))
            }
        }
        foreach ($source in @('App/Platform/stm32_adrc_channel.c', 'App/Platform/stm32_platform.c',
                'App/Platform/can_rx_inbox.c', 'Core/Src/freertos.c', 'App/aethor_app.c')) {
            $objectName = [IO.Path]::GetFileNameWithoutExtension($source)
            $objectPath = Join-Path $logDirectory ($objectName + '_adrc.o')
            Invoke-AdrcRecordedCommand ('armcc_real_platform_' + $objectName) $ArmCompiler `
                ($platformArguments + @('-c', $source, '-o', $objectPath)) $logDirectory $records
            if (-not (Test-Path -LiteralPath $objectPath -PathType Leaf)) {
                throw "Missing platform object: $objectPath"
            }
        }
        if ($IncludeLegacy) {
            try {
                Invoke-AdrcRecordedCommand 'legacy_full_regression' $powershellExecutable @('-NoProfile',
                    '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'verify_debug_ui.ps1'),
                    '-Compiler', $Compiler, '-Python', $Python, '-KeilPath', $KeilPath,
                    '-IncludeKeil') $logDirectory $records
            }
            finally {
                $legacyLogs = Join-Path $projectRoot 'Tests\host\build\verification'
                $retainedLogs = Join-Path $logDirectory 'legacy_evidence'
                [void](New-Item -ItemType Directory -Path $retainedLogs -Force)
                Get-ChildItem -LiteralPath $legacyLogs -File | ForEach-Object {
                    Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $retainedLogs $_.Name)
                }
            }
        }
        if ($IncludeAdrcKeil) {
            foreach ($targetName in @('ADRC-Bench', 'LCD-MIT-ADRC')) {
                Invoke-AdrcKeilBuild $projectRoot $targetName $logDirectory $records
                # Execute final ARM instructions: host-C success cannot catch target optimizer defects.
                Invoke-AdrcRecordedCommand ('arm_shaft_fixtures_' + $targetName) $Python @(
                    'Tests/host/check_adrc_shaft_arm.py', $targetName, '--compiler', $ArmCompiler) $logDirectory $records
                $shaftEvidence = Join-Path $projectRoot ('output\adrc\host\shaft_arm_' + $targetName)
                $retainedShaftEvidence = Join-Path $logDirectory ('shaft_arm_' + $targetName)
                Copy-Item -LiteralPath $shaftEvidence -Destination $retainedShaftEvidence -Recurse
                Invoke-AdrcRecordedCommand ('arm_execution_' + $targetName) $Python @(
                    'Tests/host/check_adrc_arm_pulse.py',
                    (Join-Path $logDirectory ($targetName + '.axf'))) $logDirectory $records
            }
        }
    }
    catch { $failureMessage = $_.Exception.Message }
    finally {
        Pop-Location
        $env:PATH = $savedSearchPath
    }
    $sourceAfter = @(Get-AdrcVerificationInputs $projectRoot)
    $sourcesUnchanged = (($sourceBefore | ConvertTo-Json -Depth 4 -Compress) -ceq
        ($sourceAfter | ConvertTo-Json -Depth 4 -Compress))
    $passed = ($null -eq $failureMessage) -and $sourcesUnchanged
    $manifest = [ordered]@{
        completed_at_utc = [DateTime]::UtcNow.ToString('o'); offline_subset_passed = $passed
        hardware_tested = $false; generated_controller_verified = $false
        existing_generated_controller_synthetic_tested = [bool](@($records | Where-Object { $_.name -eq 'run_adrc_simple_trial_tests' -and $_.exit_code -eq 0 }).Count -eq 1)
        adrc_firmware_linked = ([bool]$IncludeAdrcKeil -and $passed)
        adrc_keil_requested = [bool]$IncludeAdrcKeil; legacy_requested = [bool]$IncludeLegacy
        failure = $failureMessage; sources_unchanged = $sourcesUnchanged
        checks = @($records.ToArray()); source_hashes = $sourceAfter
        evidence_hashes = @(Get-ChildItem -LiteralPath $logDirectory -File -Recurse | ForEach-Object {
            [ordered]@{ path = $_.FullName; sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
        })
    }
    $manifestPath = Join-Path $logDirectory 'manifest.json'
    [IO.File]::WriteAllText($manifestPath, ($manifest | ConvertTo-Json -Depth 6),
        [Text.UTF8Encoding]::new($false))
    Write-Output "ADRC_OFFLINE_MANIFEST=$manifestPath"
    if (-not $passed) { throw "Offline verification incomplete or inputs changed: $failureMessage" }
    Write-Output 'ADRC_OFFLINE_SUBSET_PASSED (hardware and fresh model generation excluded)'
}

Invoke-AdrcOfflineVerification
