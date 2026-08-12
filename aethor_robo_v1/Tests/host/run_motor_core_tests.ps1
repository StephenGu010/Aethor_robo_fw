<#
.SYNOPSIS
Builds and runs the hardware-independent seven-motor core tests.

.DESCRIPTION
Compiles the static motor bank and bounded CAN scheduler with strict C11
warnings. No HAL, RTOS, USB device, or physical motor is required.
#>

$ErrorActionPreference = 'Stop'

function Get-MotorCoreCompiler {
    <# Returns the first supported host C compiler available on PATH. #>
    $gccCommand = Get-Command 'gcc' -ErrorAction SilentlyContinue
    if ($null -ne $gccCommand)
    {
        return $gccCommand
    }

    $clangCommand = Get-Command 'clang' -ErrorAction SilentlyContinue
    if ($null -ne $clangCommand)
    {
        return $clangCommand
    }

    throw 'Motor core tests require gcc or clang on PATH.'
}

function Invoke-MotorCoreTests {
    <# Builds and executes the allocation-free motor core suite. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'motor_core_tests.exe'
    $compilerCommand = Get-MotorCoreCompiler
    $sourceFiles = @(
        'Tests\host\motor_core_test_main.c',
        'App\Config\arm_config.c',
        'App\Motor\can_frame.c',
        'App\Motor\can_tx_scheduler.c',
        'App\Motor\motor_bank.c',
        'App\Motor\motor_discovery.c',
        'App\Motor\s3519_codec.c'
    )

    if (-not (Test-Path -LiteralPath $buildDirectory))
    {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    Push-Location $projectRoot
    try
    {
        & $compilerCommand.Source `
            '-std=c11' `
            '-Wall' `
            '-Wextra' `
            '-Werror' `
            '-IApp\Config' `
            '-IApp\Motor' `
            @sourceFiles `
            '-lm' `
            '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Motor core compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Motor core tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-MotorCoreTests
