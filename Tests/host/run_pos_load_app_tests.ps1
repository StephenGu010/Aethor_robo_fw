<# @file run_pos_load_app_tests.ps1. Production-facade POS-only and default-lock regressions; synthetic feedback only. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\pos_load'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sourceFiles = @('Tests/host/pos_load_app_test.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
    'App/Protocol/protocol_engine.c')
Push-Location $projectRoot
try {
    foreach ($gate in @(0, 1)) {
        $executable = Join-Path $buildDirectory ('pos_load_' + $gate + '.exe')
        & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' ('-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=' + $gate) `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
            '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sourceFiles -lm -o $executable
        if ($LASTEXITCODE -ne 0) { throw ('POS load build failed gate=' + $gate) }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw ('POS load regression failed gate=' + $gate) }
    }
}
finally { Pop-Location }
