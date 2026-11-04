<#
.SYNOPSIS
Exhaustively checks actual S3519 position decoding and representable command round trips without hardware.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$outputDirectory = Join-Path $projectRoot 'output\adrc\host\wide_position_codec'
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

function Invoke-WidePositionCodecChecks {
    <# Build and execute each optimization level while preserving the native nonzero exit status. #>
    foreach ($optimization in @('O0','O2')) {
        $executable = Join-Path $outputDirectory ('wide_position_codec_' + $optimization + '.exe')
        & $Compiler -std=c99 -Wall -Wextra -Werror -Wpedantic -fno-fast-math ('-' + $optimization) `
            '-IApp/Motor' 'Tests/host/adrc_wide_position_codec_test_main.c' `
            'App/Motor/s3519_codec.c' 'App/Motor/can_frame.c' -lm -o $executable
        if ($LASTEXITCODE -ne 0) { throw "Actual wide position codec compilation failed: $optimization" }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw "Actual wide position codec assertions failed: $optimization" }
    }
}

Push-Location $projectRoot
try { Invoke-WidePositionCodecChecks }
finally { Pop-Location }
