<# .SYNOPSIS Builds the real facade static-HOLD regressions with synthetic evidence; never opens hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\static_load'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sources = @('Tests/host/static_load_app_test.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
    'App/Protocol/protocol_engine.c')
Push-Location $projectRoot
try {
    foreach ($enabled in @(0, 1)) {
        $executable = Join-Path $buildDirectory ("static_load_$enabled.exe")
        & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            "-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=$enabled" `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
            '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sources -lm -o $executable
        if ($LASTEXITCODE -ne 0) { throw "Static HOLD build failed: gate=$enabled" }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw "Static HOLD regression failed: gate=$enabled" }
    }
}
finally { Pop-Location }
