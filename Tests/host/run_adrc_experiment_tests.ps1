<# .SYNOPSIS
Compile and run deterministic offline ADRC supervisor fault injection without hardware access.
#>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [switch]$RedBaseline
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$executable = Join-Path $buildDirectory 'adrc_experiment_tests.exe'
$arguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-O1', '-g', '-IApp/Adrc')
if ($RedBaseline) {
    $arguments += @('-DADRC_TEST_RED_BASELINE', '-Wno-unused-variable')
} else {
    $arguments += 'App/Adrc/adrc_experiment.c'
}
$arguments += @('Tests/host/adrc_experiment_test_main.c', '-lm', '-o', $executable)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
Push-Location $projectRoot
try {
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0) { throw 'ADRC supervisor compilation failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'ADRC supervisor assertion failed.' }
} finally {
    Pop-Location
}
