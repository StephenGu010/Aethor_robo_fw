<# .SYNOPSIS Builds locked, single-only and combined static gate regressions without hardware access. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\single_static_load'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sources = @('Tests/host/single_static_load_app_test.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c', 'App/Motion/joint_motion.c',
    'App/Motion/joint_motion_can.c', 'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c',
    'App/Motor/motor_bank.c', 'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c',
    'App/Motor/s3519_codec.c', 'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c',
    'App/Protocol/protocol_engine.c')
Push-Location $projectRoot
try {
    foreach ($configuration in @(
        @{ Name = 'locked'; Single = 0; Seven = 0 },
        @{ Name = 'single'; Single = 1; Seven = 0 },
        @{ Name = 'combined'; Single = 1; Seven = 1 })) {
        $executable = Join-Path $buildDirectory ("single_static_load_" + $configuration.Name + '.exe')
        & $Compiler -std=c11 -Wall -Wextra -Werror '-DAETHOR_DEBUG_UI_ENABLE=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            ('-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=' + $configuration.Single) `
            ('-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=' + $configuration.Seven) `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
            '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' @sources -lm -o $executable
        if ($LASTEXITCODE -ne 0) { throw ('Single static HOLD build failed: ' + $configuration.Name) }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw ('Single static HOLD regression failed: ' + $configuration.Name) }
        Write-Output ('SINGLE_STATIC_CONFIG_PASS ' + $configuration.Name)
    }
}
finally { Pop-Location }
