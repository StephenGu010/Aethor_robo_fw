<# .SYNOPSIS Builds and runs 30 minutes of deterministic logical-time readonly integration.
.DESCRIPTION
Uses unchanged production App/input/model with synthetic discovery and decoded CAN
feedback. No hardware, network, RTOS scheduling or wallclock-load claim is made.
The process-only execution policy is supplied by the caller; no policy is changed.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-DebugUiSoakTests {
    <# Compiles strict readonly sources and retains the actual runtime statistics log. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    if (-not (Test-Path -LiteralPath $buildDirectory)) {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }
    $executable = Join-Path $buildDirectory 'debug_ui_soak_tests.exe'
    $logPath = Join-Path $buildDirectory 'debug_ui_soak_tests.log'
    $sourceFiles = @(
        'Tests/host/debug_ui_soak_test_main.c',
        'App/DebugUi/debug_ui_input.c', 'App/DebugUi/debug_ui_model.c', 'App/DebugUi/debug_ui_mailbox.c',
        'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
        'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c',
        'App/Motion/joint_motion.c', 'App/Motion/joint_motion_can.c',
        'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c', 'App/Motor/motor_bank.c',
        'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c', 'App/Motor/s3519_codec.c',
        'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c'
    )
    Push-Location $projectRoot
    try {
        & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -pedantic `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=0' '-DAETHOR_DEBUG_UI_ALLOW_MIT=0' `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
            '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' `
            @sourceFiles -lm -o $executable
        if ($LASTEXITCODE -ne 0) { throw 'Readonly soak compilation failed.' }
        & $executable | Tee-Object -FilePath $logPath
        if ($LASTEXITCODE -ne 0) { throw ('Readonly soak regression failed; inspect ' + $logPath) }
    }
    finally { Pop-Location }
}

Invoke-DebugUiSoakTests
