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
        '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1' `
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
    $staticLockedExecutable = Join-Path $buildDirectory 'debug_ui_static_locked_tests.exe'
    & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-DAETHOR_DEBUG_UI_ENABLE=1' `
        '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
        ('-I' + (Join-Path $projectRoot 'App\DebugUi')) ('-I' + $PSScriptRoot) `
        (Join-Path $projectRoot 'Tests\ui\debug_ui_static_locked_test.c') `
        (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') '-lm' '-o' $staticLockedExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Default-locked static model compilation failed.' }
    & $staticLockedExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Default-locked static model test failed.' }
    foreach ($singleGate in @(0, 1)) {
        $singleExecutable = Join-Path $buildDirectory ('single_static_ui_' + $singleGate + '.exe')
        & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-pedantic' '-g' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' `
            '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1' `
            ('-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=' + $singleGate) `
            ('-I' + (Join-Path $projectRoot 'App\DebugUi')) ('-I' + $PSScriptRoot) `
            (Join-Path $projectRoot 'Tests\ui\single_static_model_test.c') `
            (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') '-lm' '-o' $singleExecutable
        if ($LASTEXITCODE -ne 0) { throw 'Single static UI compilation failed.' }
        & $singleExecutable
        if ($LASTEXITCODE -ne 0) { throw 'Single static UI behavior failed.' }
    }
}
Invoke-DebugUiModelTests
