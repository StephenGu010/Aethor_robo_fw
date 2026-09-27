<#
.SYNOPSIS
Builds and runs HAL-independent LCD/ADRC ownership tests.

.DESCRIPTION
Uses strict host C warnings and stores the ignored executable under output/adrc/host.
#>
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $projectRoot 'output\adrc\host'
$testExecutable = Join-Path $buildDirectory 'adrc_lcd_ownership_tests.exe'
$compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe'
if (-not (Test-Path -LiteralPath $compiler)) { throw 'The host GCC compiler is missing.' }
New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
Push-Location $projectRoot
try {
    & $compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
        '-IApp/Adrc' 'Tests/host/adrc_lcd_ownership_test_main.c' `
        'App/Adrc/adrc_lcd_ownership.c' '-o' $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Ownership test compilation failed.' }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Ownership test assertions failed.' }
}
finally { Pop-Location }
