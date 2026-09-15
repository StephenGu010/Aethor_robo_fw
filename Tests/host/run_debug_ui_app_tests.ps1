<# .SYNOPSIS Strict local-control integration tests with host-only explicit profiles. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build'
$compilerPath = $Compiler
$env:PATH = (Split-Path $compilerPath) + ';' + $env:PATH
if (-not (Test-Path -LiteralPath $buildDirectory)) {
    New-Item -ItemType Directory -Path $buildDirectory | Out-Null
}
$sourceFiles = @(
    'Tests/host/debug_ui_app_test_main.c', 'App/DebugUi/debug_ui_mailbox.c',
    'App/Config/arm_config.c', 'App/Config/build_info.c', 'App/Telemetry/diagnostics.c',
    'App/Arm/arm_controller.c', 'App/Arm/joint_reference.c',
    'App/Motion/joint_motion.c', 'App/Motion/joint_motion_can.c',
    'App/Motor/can_frame.c', 'App/Motor/can_tx_scheduler.c', 'App/Motor/motor_bank.c',
    'App/Motor/motor_discovery.c', 'App/Motor/motor_runtime.c', 'App/Motor/s3519_codec.c',
    'App/Protocol/ascii_protocol.c', 'App/Protocol/text_protocol.c', 'App/Protocol/protocol_engine.c'
)
Push-Location $projectRoot
try {
    $configurations = @(
        @{ Name = 'off'; Defines = @() },
        @{ Name = 'readonly'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1') },
        @{ Name = 'seven_readonly'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F') },
        @{ Name = 'pos'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1') },
        @{ Name = 'motor7_pos'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1') },
        @{ Name = 'motor7_mit'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1', '-DAETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1') },
        @{ Name = 'seven_pos'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1', '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F', '-DAETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F') },
        @{ Name = 'seven_mit'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-DAETHOR_DEBUG_UI_MOTOR7_POS_PROFILE=1', '-DAETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE=1', '-DAETHOR_S3519_SAME_MODEL_MASK=0x7F', '-DAETHOR_DEBUG_UI_S3519_PROFILE_MASK=0x7F') },
        @{ Name = 'mit'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1') },
        @{ Name = 'production_readonly'; Defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_ACTIVE_PROFILE=2') }
    )
    foreach ($configuration in $configurations) {
        $executablePath = Join-Path $buildDirectory ('debug_ui_app_' + $configuration.Name + '_tests.exe')
        $configurationDefines = $configuration.Defines
        & $compilerPath -std=c11 -Wall -Wextra -Werror @configurationDefines `
            '-IApp' '-IApp/Arm' '-IApp/Config' '-IApp/DebugUi' '-IApp/Motor' `
            '-IApp/Motion' '-IApp/Protocol' '-IApp/Telemetry' '-IApp/Platform' `
            @sourceFiles -lm -o $executablePath
        if ($LASTEXITCODE -ne 0) { throw ('Debug UI app compilation failed: ' + $configuration.Name) }
        & $executablePath
        if ($LASTEXITCODE -ne 0) { throw ('Debug UI app tests failed: ' + $configuration.Name) }
        Write-Output ('DEBUG_UI_CONFIG_PASS ' + $configuration.Name)
    }
    $invalidConfigurations = @(
        @('-DAETHOR_DEBUG_UI_ALLOW_MOTION=1'),
        @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1'),
        @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_ACTIVE_PROFILE=2'),
        @('-DAETHOR_DEBUG_UI_ENABLE=2')
    )
    foreach ($configurationDefines in $invalidConfigurations) {
        $previousPreference = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        '#include "debug_ui_config.h"' | & $compilerPath -std=c11 -Wall -Wextra -Werror `
            @configurationDefines '-IApp/Config' -x c -c - `
            -o (Join-Path $buildDirectory 'debug_ui_invalid_config.o') 2>$null
        $compileExitCode = $LASTEXITCODE
        $ErrorActionPreference = $previousPreference
        if ($compileExitCode -eq 0) { throw 'An invalid local motion configuration compiled.' }
    }
    Write-Output 'DEBUG_UI_INVALID_CONFIGS_REJECTED'
}
finally { Pop-Location }
