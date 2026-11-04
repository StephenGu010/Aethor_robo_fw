<# .SYNOPSIS Runs display-only metadata/navigation/STOP model regressions without hardware. #>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-LoadTrialPageTests {
    <# Compile strict C99 for both feature gates; metadata never grants control permission. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\load_trial_pages'
    [void](New-Item -ItemType Directory -Force -Path $buildDirectory)
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    foreach ($singleGate in @(0, 1)) {
        $executable = Join-Path $buildDirectory ('load_trial_pages_' + $singleGate + '.exe')
        & $Compiler '-std=c99' '-Wall' '-Wextra' '-Werror' '-pedantic' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            ('-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=' + $singleGate) `
            ('-I' + (Join-Path $projectRoot 'App\DebugUi')) ('-I' + (Join-Path $projectRoot 'Tests\host')) `
            (Join-Path $projectRoot 'App\DebugUi\debug_ui_model.c') `
            (Join-Path $PSScriptRoot 'load_trial_pages_model_test.c') '-lm' '-o' $executable
        if ($LASTEXITCODE -ne 0) { throw 'Load trial page model compilation failed.' }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw 'Load trial page model assertion failed.' }
    }
}
Invoke-LoadTrialPageTests
