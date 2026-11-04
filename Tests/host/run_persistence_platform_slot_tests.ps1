<# @file run_persistence_platform_slot_tests.ps1
   @brief Compile the actual STM32 platform with deterministic HAL mocks; no device or cleanup operation. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
function Invoke-PersistencePlatformSlotTests {
    <# Build and run strict C11 checks while preserving all existing test/build files. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $projectRoot 'output\repair_20261023\persistence_platform_slot\v2\build'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $testExecutable = Join-Path $buildDirectory 'persistence_platform_slot.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    Push-Location $projectRoot
    try {
        & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Wpedantic `
            '-DAETHOR_MOTOR_PERSISTENCE_TRANSPORT=1' '-DAETHOR_ADRC_LCD_INTEGRATED=1' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1' '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1' `
            '-ITests/host/single_static_platform_fakes' '-IApp/Config' '-IApp/Motor' '-IApp/Platform' `
            'Tests/host/persistence_platform_slot_test.c' 'App/Motor/motor_parameter_persistence.c' `
            'App/Motor/motor_capability_collector.c' 'App/Motor/s3519_codec.c' 'App/Motor/can_frame.c' `
            'App/Motor/can_tx_scheduler.c' 'App/Config/arm_config.c' 'App/Platform/can_rx_inbox.c' `
            'App/Platform/usb_cdc_stream.c' -lm -o $testExecutable
        if ($LASTEXITCODE -ne 0) { throw 'Actual persistence platform slot compilation failed.' }
        & $testExecutable
        if ($LASTEXITCODE -ne 0) { throw ('Actual persistence platform slot assertions failed: ' + $LASTEXITCODE) }
    } finally { Pop-Location }
}
Invoke-PersistencePlatformSlotTests
