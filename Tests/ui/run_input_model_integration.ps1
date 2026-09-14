<# .SYNOPSIS
Run real input+model traces for interrupted confirmation and overflow STOP.
#>
param([string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'

function Invoke-InputModelIntegration {
    <# Use both production state machines, and synthetic ADC/snapshot fixtures. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\integration'
    New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
    $executable = Join-Path $buildDirectory 'input_model_integration.exe'
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    Push-Location $projectRoot
    try {
        & $Compiler '-std=c11' '-Wall' '-Wextra' '-Werror' '-g' '-IApp/DebugUi' '-ITests/host' `
            '-DAETHOR_DEBUG_UI_ENABLE=1' '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1' '-DAETHOR_DEBUG_UI_ALLOW_MIT=1' `
            'App/DebugUi/debug_ui_input.c' 'App/DebugUi/debug_ui_model.c' `
            'Tests/ui/input_model_integration_test.c' '-lm' '-o' $executable
        if ($LASTEXITCODE -ne 0) { throw 'Input/model integration build failed.' }
        & $executable
        if ($LASTEXITCODE -ne 0) { throw 'Input/model integration assertion failed.' }
    }
    finally { Pop-Location }
}
Invoke-InputModelIntegration
