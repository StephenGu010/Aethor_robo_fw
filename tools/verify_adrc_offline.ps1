<#
.SYNOPSIS
Verify ADRC software modules without opening a serial port or flashing firmware.
.DESCRIPTION
Records fresh host-test logs, ARMCC object-compilation results and source hashes.
Object compilation does not prove firmware linking, real-time execution or hardware safety.
#>
[CmdletBinding()]
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$ArmCompiler = 'D:\application\keil\ARM\ARMCC\bin\armcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [switch]$IncludeLegacy
)
$ErrorActionPreference = 'Stop'

function Get-AdrcVerificationInputs {
    <# Hash maintained inputs to detect edits made while verification is running. #>
    param([string]$ProjectRoot)
    $inputFiles = foreach ($relativeDirectory in @('App', 'Tests\host', 'Models\Adrc', 'tools')) {
        $directory = Join-Path $ProjectRoot $relativeDirectory
        if (Test-Path -LiteralPath $directory) {
            Get-ChildItem -LiteralPath $directory -Recurse -File |
                Where-Object { $_.Extension -in @('.c', '.h', '.m', '.ps1', '.py') -and
                    $_.FullName -notmatch '[\\/]build[\\/]' }
        }
    }
    $inputFiles += Get-Item -LiteralPath $PSCommandPath
    $inputFiles += Get-Item -LiteralPath (Join-Path $ProjectRoot 'Core\Src\freertos.c'),
        (Join-Path $ProjectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'),
        (Join-Path $ProjectRoot 'Ui\fonts\generate_numeric_28.py'),
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

function Invoke-AdrcOfflineVerification {
    <# Execute the offline acceptance subset and write an honest manifest even after failure. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
    foreach ($executable in @($Compiler, $ArmCompiler, $Python)) {
        if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) {
            throw "Required compiler not found: $executable"
        }
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
        Invoke-AdrcRecordedCommand 'target_evidence_gate' $Python @('Tests/host/test_adrc_target_gate.py') $logDirectory $records
        Invoke-AdrcRecordedCommand 'font_line_ending_contract' $Python @('Tests/host/test_numeric_font_line_endings.py') $logDirectory $records
        foreach ($testName in @('run_adrc_experiment_tests', 'run_adrc_protocol_tests',
                'run_adrc_bench_tests', 'run_adrc_channel_tests', 'run_adrc_app_tests',
                'run_text_protocol_engine_tests', 'run_motor_core_tests')) {
            $testScript = Join-Path $projectRoot ('Tests\host\' + $testName + '.ps1')
            Invoke-AdrcRecordedCommand $testName $powershellExecutable @('-NoProfile',
                '-ExecutionPolicy', 'Bypass', '-File', $testScript) $logDirectory $records
        }
        foreach ($source in @('App/Adrc/adrc_experiment.c', 'App/Adrc/adrc_protocol.c', 'App/Adrc/adrc_bench.c',
                'App/Adrc/adrc_app_bridge.c', 'App/Motor/motor_adrc_command.c',
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
        foreach ($source in @('App/Platform/stm32_adrc_channel.c', 'Core/Src/freertos.c', 'App/aethor_app.c')) {
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
                    '-Compiler', $Compiler, '-IncludeKeil') $logDirectory $records
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
        adrc_firmware_linked = $false; legacy_requested = [bool]$IncludeLegacy
        failure = $failureMessage; sources_unchanged = $sourcesUnchanged
        checks = @($records.ToArray()); source_hashes = $sourceAfter
    }
    $manifestPath = Join-Path $logDirectory 'manifest.json'
    [IO.File]::WriteAllText($manifestPath, ($manifest | ConvertTo-Json -Depth 6),
        [Text.UTF8Encoding]::new($false))
    Write-Output "ADRC_OFFLINE_MANIFEST=$manifestPath"
    if (-not $passed) { throw "Offline verification incomplete or inputs changed: $failureMessage" }
    Write-Output 'ADRC_OFFLINE_SUBSET_PASSED (hardware and generated controller excluded)'
}

Invoke-AdrcOfflineVerification
