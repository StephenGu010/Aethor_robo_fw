<#
.SYNOPSIS
Builds and runs aethor-text-v1 tests for the formal arm profile.

.DESCRIPTION
Compiles the protocol engine with the production profile selected and validates
that formal seven-axis commands retain the existing bounded command boundary.
#>

$ErrorActionPreference = 'Stop'

function Get-TextProtocolArmTestCompiler {
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

    throw 'Text protocol arm tests require gcc or clang on PATH.'
}

function Invoke-TextProtocolArmHostTests {
    <# Builds and runs the isolated formal-profile protocol test executable. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'text_protocol_arm_tests.exe'
    $compilerCommand = Get-TextProtocolArmTestCompiler

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
            '-DAETHOR_ACTIVE_PROFILE=AETHOR_PROFILE_ARM_PRODUCTION' `
            '-IApp\Arm' `
            '-IApp\Config' `
            '-IApp\Motion' `
            '-IApp\Motor' `
            '-IApp\Protocol' `
            '-IApp\Telemetry' `
            'Tests\host\text_protocol_arm_profile_test_main.c' `
            'App\Protocol\ascii_protocol.c' `
            'App\Protocol\text_protocol.c' `
            'App\Protocol\protocol_engine.c' `
            'App\Config\arm_config.c' `
            'App\Config\build_info.c' `
            'App\Motion\joint_motion.c' `
            '-lm' `
            '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Text protocol arm compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Text protocol arm tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-TextProtocolArmHostTests
