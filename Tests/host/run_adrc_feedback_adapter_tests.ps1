<# .SYNOPSIS Compiles deterministic causal feedback adapter tests without hardware access. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build'
if (-not (Test-Path -LiteralPath $buildDirectory)) { New-Item -ItemType Directory -Path $buildDirectory | Out-Null }
$executablePath = Join-Path $buildDirectory 'adrc_feedback_adapter_tests.exe'
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
Push-Location $projectRoot
try {
    & $Compiler -std=c99 -Wall -Wextra -Werror -pedantic '-IApp/Adrc' `
        Tests/host/adrc_feedback_adapter_test_main.c App/Adrc/adrc_feedback_adapter.c -lm -o $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'ADRC feedback adapter compilation failed.' }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'ADRC feedback adapter tests failed.' }
}
finally { Pop-Location }
