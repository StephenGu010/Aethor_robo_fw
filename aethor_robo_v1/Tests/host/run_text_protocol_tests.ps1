<#
.SYNOPSIS
Builds and runs the hardware-independent aethor-text-v1 codec tests.

.DESCRIPTION
Compiles the bounded text parser and its focused C11 contract tests with all
warnings treated as errors. Build-only artifacts stay under Tests/host/build.
#>

$ErrorActionPreference = 'Stop'

function Get-TextProtocolTestCompiler {
    <# Returns the first supported host C compiler available on PATH. #>
    $gccCommand = Get-Command 'gcc' -ErrorAction SilentlyContinue
    if ($null -ne $gccCommand) {
        return $gccCommand
    }

    $clangCommand = Get-Command 'clang' -ErrorAction SilentlyContinue
    if ($null -ne $clangCommand) {
        return $clangCommand
    }

    throw 'Text protocol host tests require gcc or clang on PATH.'
}

function Invoke-TextProtocolHostTests {
    <# Builds the text parser and runs its strict host contract suite. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build'
    $testExecutable = Join-Path $buildDirectory 'text_protocol_tests.exe'
    $compilerCommand = Get-TextProtocolTestCompiler

    if (-not (Test-Path -LiteralPath $buildDirectory)) {
        New-Item -ItemType Directory -Path $buildDirectory | Out-Null
    }

    Push-Location $projectRoot
    try {
        & $compilerCommand.Source `
            '-std=c11' `
            '-Wall' `
            '-Wextra' `
            '-Werror' `
            '-IApp\Protocol' `
            'Tests\host\text_protocol_test_main.c' `
            'App\Protocol\text_protocol.c' `
            '-o' $testExecutable
        if ($LASTEXITCODE -ne 0) {
            throw "Text protocol compilation failed with exit code $LASTEXITCODE."
        }

        & $testExecutable
        if ($LASTEXITCODE -ne 0) {
            throw "Text protocol tests failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }
}

Invoke-TextProtocolHostTests
