<#
.SYNOPSIS
Builds and runs the hardware-independent aethor-arm-ascii-v1 contract tests.

.DESCRIPTION
Generates a C header from the shared JSON Golden Frames, compiles the strict C11
protocol suite, and stores build-only artifacts under Tests/host/build.
#>

$ErrorActionPreference = 'Stop'

function Get-ProtocolTestCompiler {
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

    throw 'Protocol host tests require gcc or clang on PATH.'
}

function Invoke-ProtocolHostTests {
    <# Generates shared vectors, builds the parser, and runs the protocol suite. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $goldenJsonPath = Join-Path $projectRoot 'Tests\protocol\aethor-arm-ascii-v1-golden.json'
    $goldenHeaderPath = Join-Path $buildDirectory 'protocol_golden_vectors.h'
    $generatorPath = Join-Path $projectRoot 'Tests\protocol\generate_golden_header.ps1'
    $testExecutable = Join-Path $buildDirectory 'protocol_tests.exe'
    $compilerCommand = Get-ProtocolTestCompiler

    if (-not (Test-Path -LiteralPath $buildDirectory))
    {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    & $generatorPath -InputPath $goldenJsonPath -OutputPath $goldenHeaderPath

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
            '-ITests\host\build' `
            'Tests\host\protocol_test_main.c' `
            'App\Protocol\ascii_protocol.c' `
            'App\Protocol\protocol_engine.c' `
            'App\Config\arm_config.c' `
            'App\Config\build_info.c' `
            'App\Motion\joint_motion.c' `
            '-lm' `
            '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Protocol compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Protocol tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-ProtocolHostTests
