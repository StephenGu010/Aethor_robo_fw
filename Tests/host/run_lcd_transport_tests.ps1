<# File: run_lcd_transport_tests.ps1. Build/run isolated LCD tests; no git/deletion/system policy changes. #>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [switch]$CompileArmcc,
    [string]$Armcc = 'D:\application\keil\ARM\ARMCC\bin\armcc.exe'
)
$ErrorActionPreference = 'Stop'

function Invoke-LcdTransportTests {
    <# Compile behavioral tests with warnings as errors, then require a passing process. #>
    $firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'lcd_fakes\build'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $testExecutable = Join-Path $buildDirectory 'lcd_transport_tests.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
        ('-I' + (Join-Path $firmwareRoot 'App\Platform')) `
        (Join-Path $firmwareRoot 'Tests\host\lcd_transport_test_main.c') `
        (Join-Path $firmwareRoot 'App\Platform\lcd_transfer.c') '-o' $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "LCD transport compilation failed: $LASTEXITCODE" }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw "LCD transport behavior failed: $LASTEXITCODE" }
    $platformExecutable = Join-Path $buildDirectory 'lcd_platform_tests.exe'
    & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
        '-DAETHOR_DEBUG_UI_ENABLE=1' '-DLCD_PLATFORM_HOST_TEST=1' `
        ('-I' + (Join-Path $firmwareRoot 'App\Config')) `
        ('-I' + (Join-Path $firmwareRoot 'App\Platform')) `
        ('-I' + (Join-Path $PSScriptRoot 'lcd_fakes')) `
        (Join-Path $PSScriptRoot 'lcd_fakes\platform_test_main.c') `
        (Join-Path $PSScriptRoot 'lcd_fakes\lcd_fake_hal.c') `
        (Join-Path $firmwareRoot 'App\Platform\lcd_st7789.c') `
        (Join-Path $firmwareRoot 'App\Platform\lcd_key_adc.c') `
        (Join-Path $firmwareRoot 'App\Platform\lcd_transfer.c') '-o' $platformExecutable
    if ($LASTEXITCODE -ne 0) { throw "LCD/ADC platform compilation failed: $LASTEXITCODE" }
    & $platformExecutable
    if ($LASTEXITCODE -ne 0) { throw "LCD/ADC platform behavior failed: $LASTEXITCODE" }
}

function Invoke-LcdArmccCompilation {
    <# Compile real platform sources against the current H723 HAL for both UI gate values.
       This is an object-level compatibility check, not the parent firmware link/build. #>
    $firmwareRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'lcd_fakes\build'
    $includeDirectories = @('Core\Inc', 'App\Config', 'App\Platform',
        'Drivers\STM32H7xx_HAL_Driver\Inc', 'Drivers\STM32H7xx_HAL_Driver\Inc\Legacy',
        'Drivers\CMSIS\Device\ST\STM32H7xx\Include', 'Drivers\CMSIS\Include')
    $compileArguments = @('--cpu=Cortex-M7.fp.dp', '--c99', '--diag_error=warning',
        '-DSTM32H723xx', '-DUSE_HAL_DRIVER')
    foreach ($includeDirectory in $includeDirectories) {
        $compileArguments += '-I' + (Join-Path $firmwareRoot $includeDirectory)
    }
    & $Armcc '--vsn'
    if ($LASTEXITCODE -ne 0) { throw "ARMCC version probe failed: $LASTEXITCODE" }
    foreach ($uiEnabled in @(0, 1)) {
        foreach ($sourceName in @('lcd_transfer', 'lcd_st7789', 'lcd_key_adc')) {
            $sourcePath = Join-Path $firmwareRoot ('App\Platform\' + $sourceName + '.c')
            $objectPath = Join-Path $buildDirectory ($sourceName + '_ui' + $uiEnabled + '_armcc.o')
            & $Armcc @compileArguments ('-DAETHOR_DEBUG_UI_ENABLE=' + $uiEnabled) `
                '-c' $sourcePath '-o' $objectPath
            if ($LASTEXITCODE -ne 0) { throw "ARMCC UI$uiEnabled $sourceName failed: $LASTEXITCODE" }
        }
    }
    Write-Output 'PASS: ARMCC 5 H723/current HAL object compilation, UI=0 and UI=1, warnings treated as errors'
}

Invoke-LcdTransportTests
if ($CompileArmcc) { Invoke-LcdArmccCompilation }
