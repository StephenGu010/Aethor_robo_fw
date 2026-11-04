<# @file run_pos_load_platform_tests.ps1. Actual POS-only platform and HAL-boundary safety regression; no hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\pos_load_platform'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sourceFiles = @('Tests/host/pos_load_platform_test.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
    'App/Protocol/protocol_engine.c', 'App/Platform/can_rx_inbox.c', 'App/Platform/usb_cdc_stream.c')
Push-Location $projectRoot
try {
    $executable = Join-Path $buildDirectory 'pos_load_platform.exe'
    & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MIT=0' '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=0' '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=0' `
        '-ITests/host/single_static_platform_fakes' '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sourceFiles -lm -o $executable
    if ($LASTEXITCODE -ne 0) { throw 'POS-only platform build failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'POS-only platform regression failed.' }
}
finally { Pop-Location }
& (Join-Path $PSScriptRoot 'run_pos_load_core_route_tests.ps1') -Compiler $Compiler
