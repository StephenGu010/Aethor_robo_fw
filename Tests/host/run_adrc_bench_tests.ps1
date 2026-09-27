<# .SYNOPSIS
Compiles and runs the fixed-memory ADRC integration contracts entirely offline.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
$testExecutable = Join-Path $buildDirectory 'adrc_bench_tests.exe'
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
Push-Location $projectRoot
try {
    & $Compiler -std=c99 -Wall -Wextra -Werror -pedantic `
        -IApp/Adrc -IApp/Arm -IApp/Config -IApp/Motion -IApp/Motor -IApp/Protocol -IApp/Telemetry `
        Tests/host/adrc_bench_test_main.c App/Adrc/adrc_bench.c App/Adrc/adrc_experiment.c `
        App/Adrc/adrc_protocol.c App/Protocol/ascii_protocol.c App/Protocol/text_protocol.c `
        App/Protocol/protocol_engine.c App/Config/arm_config.c App/Config/build_info.c `
        App/Motion/joint_motion.c -lm -o $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'ADRC bench compilation failed.' }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'ADRC bench assertions failed.' }
} finally { Pop-Location }
