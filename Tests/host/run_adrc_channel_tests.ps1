<# .SYNOPSIS
Compile and run the dedicated ADRC CAN channel against deterministic HAL fakes.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
$executable = Join-Path $buildDirectory 'adrc_channel_tests.exe'
$savedSearchPath = $env:PATH
Push-Location $projectRoot
try {
    $env:PATH = (Split-Path $Compiler) + ';' + $savedSearchPath
    & $Compiler -std=c99 -Wall -Wextra -Werror -pedantic -ITests/host/adrc_fakes -IApp/Motor -IApp/Platform `
        Tests/host/adrc_channel_test_main.c App/Platform/stm32_adrc_channel.c `
        App/Motor/s3519_codec.c App/Motor/can_frame.c -lm -o $executable
    if ($LASTEXITCODE -ne 0) { throw 'ADRC channel compilation failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'ADRC channel test failed.' }
} finally { Pop-Location; $env:PATH = $savedSearchPath }
