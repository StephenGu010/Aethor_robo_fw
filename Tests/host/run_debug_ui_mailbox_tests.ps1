<# .SYNOPSIS Strict host tests of the bounded local UI mailbox; no hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build'
$compilerPath = $Compiler
$env:PATH = (Split-Path $compilerPath) + ';' + $env:PATH
if (-not (Test-Path -LiteralPath $buildDirectory)) {
    New-Item -ItemType Directory -Path $buildDirectory | Out-Null
}
Push-Location $projectRoot
try {
    $executablePath = Join-Path $buildDirectory 'debug_ui_mailbox_tests.exe'
    & $compilerPath -std=c11 -Wall -Wextra -Werror '-IApp/DebugUi' `
        'Tests/host/debug_ui_mailbox_test_main.c' 'App/DebugUi/debug_ui_mailbox.c' `
        -o $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Debug UI mailbox compilation failed.' }
    & $executablePath
    if ($LASTEXITCODE -ne 0) { throw 'Debug UI mailbox tests failed.' }
}
finally { Pop-Location }
