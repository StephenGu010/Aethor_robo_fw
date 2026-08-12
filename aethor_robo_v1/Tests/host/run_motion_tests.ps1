<#
.SYNOPSIS
Builds and runs the hardware-independent seven-axis motion tests.
#>

$ErrorActionPreference = 'Stop'

function Invoke-MotionHostTests {
    <# Builds the strict C11 motion suite with the first available compiler. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'motion_tests.exe'
    $compilerCommand = Get-Command 'gcc' -ErrorAction SilentlyContinue

    if ($null -eq $compilerCommand)
    {
        $compilerCommand = Get-Command 'clang' -ErrorAction SilentlyContinue
    }
    if ($null -eq $compilerCommand)
    {
        throw 'Motion host tests require gcc or clang on PATH.'
    }
    if (-not (Test-Path -LiteralPath $buildDirectory))
    {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    Push-Location $projectRoot
    try
    {
        & $compilerCommand.Source `
            '-std=c11' '-Wall' '-Wextra' '-Werror' `
            '-IApp\Config' '-IApp\Motion' `
            'Tests\host\motion_test_main.c' `
            'App\Config\arm_config.c' `
            'App\Motion\joint_motion.c' `
            '-lm' '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Motion compilation failed with exit code $LASTEXITCODE."
        }
        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Motion tests failed with exit code $LASTEXITCODE."
        }
    }
    finally
    {
        Pop-Location
    }
}

Invoke-MotionHostTests
