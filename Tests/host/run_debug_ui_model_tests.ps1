<# .SYNOPSIS
Build and run the pure-C model tests with synthetic bench profiles.
No hardware, global execution policy or installed software is changed.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-DebugUiModelTests {
    <# Compile all model behavior tests with strict warnings, then execute. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $projectRoot 'Tests\ui\build\model'
    New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
    $executable = Join-Path $buildDirectory 'debug_ui_model_tests.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-g' `
        '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        ('-I' + (Join-Path $projectRoot 'App\DebugUi')) `
        (Join-Path $PSScriptRoot 'debug_ui_model_test_main.c') `
        (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') '-lm' '-o' $executable
    if ($LASTEXITCODE -ne 0) { throw 'Model compilation failed.' }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw 'Model behavioral test failed.' }
    $readonlyExecutable = Join-Path $buildDirectory 'debug_ui_readonly_tests.exe'
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-DAETHOR_DEBUG_UI_ENABLE=1' `
        ('-I' + (Join-Path $projectRoot 'App\DebugUi')) `
        (Join-Path $projectRoot 'Tests\ui\debug_ui_readonly_test.c') `
        (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') '-lm' '-o' $readonlyExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Read-only model compilation failed.' }
    & $readonlyExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Read-only compile gate test failed.' }
}
Invoke-DebugUiModelTests
