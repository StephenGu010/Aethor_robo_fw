<#
.SYNOPSIS
Builds and runs the aethor-text-v1 protocol-engine lifecycle tests.

.DESCRIPTION
Compiles the strict C11 host suite with both the legacy and new parsers so the
incremental engine migration retains the existing implementation dependencies.
#>

$ErrorActionPreference = 'Stop'

function Get-TextProtocolEngineTestCompiler {
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

    throw 'Text protocol engine tests require gcc or clang on PATH.'
}

function Invoke-TextProtocolEngineHostTests {
    <# Builds and runs the isolated aethor-text-v1 engine test executable. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'text_protocol_engine_tests.exe'
    $compilerCommand = Get-TextProtocolEngineTestCompiler

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
            '-IApp\Arm' `
            '-IApp\Config' `
            '-IApp\Motion' `
            '-IApp\Motor' `
            '-IApp\Protocol' `
            '-IApp\Telemetry' `
            'Tests\host\text_protocol_engine_test_main.c' `
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
            throw "Text protocol engine compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Text protocol engine tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-TextProtocolEngineHostTests
