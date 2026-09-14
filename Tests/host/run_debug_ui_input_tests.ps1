<#
.SYNOPSIS
Builds and runs the hardware-independent ADC five-way input behavior suite.
.DESCRIPTION
Uses GCC/Clang with strict C99 warnings. A unique executable is retained in
the OS temporary directory; no project build artifacts or deletion occur.
.PARAMETER Compiler
Optional full GCC/Clang path when the compiler is absent from PATH.
#>

param(
    [string]$Compiler = '',
    [string]$CStandard = 'c99'
)

$ErrorActionPreference = 'Stop'

function Invoke-DebugUiInputHostTests {
    <# Builds and runs the deterministic input suite, preserving native failures. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $compilerCommand = $null
    $compilerPath = $Compiler
    $originalPath = $env:PATH
    $testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) `
        ('debug_ui_input_tests_' + [Guid]::NewGuid().ToString('N') + '.exe')

    if ([string]::IsNullOrWhiteSpace($compilerPath))
    {
        $compilerCommand = Get-Command 'gcc' -ErrorAction SilentlyContinue
        if ($null -eq $compilerCommand)
        {
            $compilerCommand = Get-Command 'clang' -ErrorAction SilentlyContinue
        }
        if ($null -eq $compilerCommand)
        {
            throw 'Input tests require GCC/Clang on PATH or -Compiler with an absolute path.'
        }
        $compilerPath = $compilerCommand.Source
    }
    $compilerPath = (Resolve-Path -LiteralPath $compilerPath).Path
    if ($CStandard -notin @('c99', 'c11'))
    {
        throw 'CStandard must be c99 or c11.'
    }

    Push-Location $projectRoot
    try
    {
        $env:PATH = (Split-Path -Parent $compilerPath) + ';' + $originalPath
        & $compilerPath "-std=$CStandard" '-Wall' '-Wextra' '-Werror' `
            '-Wconversion' '-Wsign-conversion' '-Wshadow' '-pedantic' '-O2' `
            '-IApp\DebugUi' 'Tests\host\debug_ui_input_test_main.c' `
            'App\DebugUi\debug_ui_input.c' '-o' $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Input compilation failed with exit code $LASTEXITCODE."
        }
        & $testExecutable
        if ($LASTEXITCODE -ne 0)
        {
            throw "Input behavior tests failed with exit code $LASTEXITCODE."
        }
        Write-Output "INPUT_TESTS_OK compiler=$compilerPath standard=$CStandard executable=$testExecutable"
    }
    finally
    {
        $env:PATH = $originalPath
        Pop-Location
    }
}

Invoke-DebugUiInputHostTests
