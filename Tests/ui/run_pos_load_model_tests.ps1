<# .SYNOPSIS Compile RAM-only production POS load model tests with locked and enabled gates. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-PosLoadModelTests {
    <# Strict host compilation exercises UI confirmation and STOP without linking hardware. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\pos_load'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    foreach ($featureGate in @(0, 1)) {
        $executable = Join-Path $buildDirectory ('pos_load_model_' + $featureGate + '.exe')
        & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            ('-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=' + $featureGate) `
            ('-I' + (Join-Path $projectRoot 'App\DebugUi')) ('-I' + (Join-Path $projectRoot 'Tests\host')) `
            (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') `
            (Join-Path $PSScriptRoot 'pos_load_model_test.c') '-lm' '-o' $executable
        if ($LASTEXITCODE -ne 0) { throw 'POS load model compilation failed.' }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw 'POS load model assertions failed.' }
    }
}
Invoke-PosLoadModelTests
