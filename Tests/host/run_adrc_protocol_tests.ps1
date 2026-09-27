<#
.SYNOPSIS
Builds and runs the ADRC shared replay and owner-task mailbox tests.

.DESCRIPTION
Compiles the strict C11 host suite with both the legacy and new parsers so the
incremental engine migration retains the existing implementation dependencies.
#>

$ErrorActionPreference = 'Stop'

function Get-AdrcProtocolTestCompiler {
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

    throw 'ADRC protocol tests require gcc or clang on PATH.'
}

function Invoke-AdrcProtocolHostTests {
    <# Builds and runs the isolated ADRC gateway test executable. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'adrc_protocol_tests.exe'
    $compilerCommand = Get-AdrcProtocolTestCompiler

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
            '-IApp\Adrc' `
            '-IApp\Arm' `
            '-IApp\Config' `
            '-IApp\Motion' `
            '-IApp\Motor' `
            '-IApp\Protocol' `
            '-IApp\Telemetry' `
            'Tests\host\adrc_protocol_test_main.c' `
            'App\Adrc\adrc_protocol.c' `
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
            throw "ADRC protocol compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "ADRC protocol tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-AdrcProtocolHostTests

