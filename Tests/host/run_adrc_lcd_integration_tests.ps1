<#
.SYNOPSIS
Builds the one-image LCD-MIT/ADRC application facade against host-only synthetic inputs.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
$testExecutable = Join-Path $buildDirectory 'adrc_lcd_integration_tests.exe'
$sourceFiles = @(
    'Tests/host/adrc_lcd_integration_test_main.c', 'App/Adrc/adrc_lcd_ownership.c',
    'App/Adrc/adrc_app_bridge.c', 'App/Adrc/adrc_bench.c', 'App/Adrc/adrc_protocol.c',
    'App/Adrc/adrc_experiment.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/motor_adrc_command.c', 'App/Motor/s3519_codec.c',
    'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c'
)
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
Push-Location $projectRoot
try {
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' `
        '-DAETHOR_ADRC_LCD_INTEGRATED=1' '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1' '-DAETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1' `
        '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F' '-DAETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F' `
        '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Adrc' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' `
        @sourceFiles '-lm' '-o' $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Integrated application compilation failed.' }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Integrated application assertions failed.' }
}
finally { Pop-Location }
