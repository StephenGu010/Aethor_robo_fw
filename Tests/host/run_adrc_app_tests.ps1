<# .SYNOPSIS Builds opt-in ADRC facade tests with an explicitly host-only controller stub. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path -LiteralPath $buildDirectory)) { New-Item -ItemType Directory -Path $buildDirectory | Out-Null }
$executablePath = Join-Path $buildDirectory 'adrc_app_tests.exe'
$sourceFiles = @(
    'Tests/host/adrc_app_test_main.c', 'App/Adrc/adrc_app_bridge.c',
    'App/Adrc/adrc_bench.c', 'App/Adrc/adrc_protocol.c', 'App/Adrc/adrc_experiment.c',
    'App/DebugUi/debug_ui_mailbox.c', 'App/Config/arm_config.c', 'App/Config/build_info.c',
    'App/Telemetry/diagnostics.c', 'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c',
    'App/Motion/joint_motion.c', 'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c',
    'App/Motor/can_tx_scheduler.c', 'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c',
    'App/Motor/motor_runtime.c', 'App/Motor/motor_adrc_command.c', 'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c',
    'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c'
)
$controllerOptions = @('-DADRC_APP_HOST_CONTROLLER_STUB=1')
Push-Location $projectRoot
try {
    & $Compiler -std=c11 -Wall -Wextra -Werror @controllerOptions '-DAETHOR_ADRC_BENCH=1' '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' '-IApp/Adrc' `
        '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' `
        @sourceFiles -lm -o $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'ADRC app host compilation failed.' }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'ADRC app host tests failed.' }
}
finally { Pop-Location }
