<# .SYNOPSIS Compiles real-facade takeover peak and first-exit regressions; no hardware access. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\static_peak_reason'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sources = @('Tests/host/static_peak_reason_test.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
    'App/Protocol/protocol_engine.c')
$executable = Join-Path $buildDirectory 'static_peak_reason.exe'
Push-Location $projectRoot
try {
    & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1' `
        '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sources -lm -o $executable
    if ($LASTEXITCODE -ne 0) { throw 'Static peak/reason build failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'Static peak/reason regression failed.' }
}
finally { Pop-Location }
