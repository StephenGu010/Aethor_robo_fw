<# @file Offline torque exposure and real production routing tests; no serial port or probe access. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$evidenceDirectory = Join-Path $projectRoot 'output\torque-envelope'
[void](New-Item -ItemType Directory -Force -Path $evidenceDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sourceFiles = @(
    'Tests/host/torque_envelope_app_test.c', 'App/Adrc/adrc_lcd_ownership.c',
    'App/Adrc/adrc_simple_trial.c', 'App/Adrc/adrc_feedback_adapter.c', 'App/Adrc/adrc_app_bridge.c',
    'App/Adrc/adrc_shaft_pilot.c', 'App/Adrc/adrc_shaft_pilot_adapter.c', 'App/Adrc/adrc_bench.c',
    'App/Adrc/adrc_protocol.c', 'App/Adrc/adrc_experiment.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/motor_adrc_command.c', 'App/Motor/motor_capability_collector.c', 'App/Motor/s3519_codec.c',
    'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c'
)
Push-Location $projectRoot
try {
    & $Compiler -std=c11 -Wall -Wextra -Werror -IApp Tests/host/torque_envelope_test.c -lm `
        -o (Join-Path $evidenceDirectory 'torque_envelope_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Torque policy compilation failed' }
    & (Join-Path $evidenceDirectory 'torque_envelope_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Torque policy assertions failed' }
    & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_ADRC_LCD_INTEGRATED=1' `
        '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1' `
        '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1' '-DAETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1' `
        '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F' '-DAETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F' `
        '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Adrc' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sourceFiles -lm `
        -o (Join-Path $evidenceDirectory 'torque_envelope_app_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Torque adapter compilation failed' }
    & (Join-Path $evidenceDirectory 'torque_envelope_app_test.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Torque adapter assertions failed' }
}
finally { Pop-Location }
