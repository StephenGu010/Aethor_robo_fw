<#
.SYNOPSIS
Builds and runs the hardware-independent PRD Phase 0 tests on Windows.

.DESCRIPTION
Selects GCC or Clang, compiles the layered application with strict C11
warnings, and stores the ignored executable under Tests/host/build.
#>

$ErrorActionPreference = 'Stop'

function Get-Phase0Compiler {
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

    throw 'Phase 0 host tests require gcc or clang on PATH.'
}

function Invoke-Phase0HostTests {
    <# Builds and executes the allocation-free Phase 0 test suite. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'phase0_tests.exe'
    $compilerCommand = Get-Phase0Compiler
    $sourceFiles = @(
        'Tests\host\phase0_test_main.c',
        'App\Config\arm_config.c',
        'App\Config\build_info.c'
    )
    $compilerArguments = @(
        '-std=c11',
        '-Wall',
        '-Wextra',
        '-Werror',
        '-IApp\Config'
    ) + $sourceFiles + @('-lm', '-o', $testExecutable)

    if (-not (Test-Path -LiteralPath $buildDirectory))
    {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    Push-Location $projectRoot
    try
    {
        & $compilerCommand.Source @compilerArguments
        if ($LASTEXITCODE -ne 0)
        {
            throw "Phase 0 compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Phase 0 tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-Phase0HostTests
