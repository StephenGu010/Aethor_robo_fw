<# .SYNOPSIS
Run persistent UI review regressions with real LVGL and production task/model.
All RTOS, ADC, SPI and App endpoint fixtures are simulated host-only services.
#>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [ValidateSet('all', 'model', 'task')][string]$Suite = 'all',
    [string]$Case = 'all'
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\review'
$imageDirectory = Join-Path $buildDirectory 'images'
New-Item -ItemType Directory -Force -Path $imageDirectory | Out-Null
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$defines = @('-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1', '-DAETHOR_DEBUG_UI_ALLOW_MIT=1')
$includes = @('-IUi', '-IMiddlewares/Third_Party/LVGL', '-ITests/ui/stubs', '-IApp', '-IApp/DebugUi',
    '-IApp/Arm', '-IApp/Config', '-IApp/Motor', '-IApp/Motion', '-IApp/Protocol', '-IApp/Telemetry', '-IApp/Platform')

function Invoke-Regression {
    <# Compile one explicit source set and fail on any console assertion failure. #>
    param([string]$Name, [string[]]$Sources, [string[]]$RunArguments)
    $executable = Join-Path $buildDirectory ($Name + '.exe')
    $response = Join-Path $buildDirectory ($Name + '.rsp')
    $arguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-O1', '-g', '-DLV_CONF_INCLUDE_SIMPLE')
    $arguments += $defines + $includes + $Sources + @('-lm', '-o', $executable)
    [System.IO.File]::WriteAllLines($response, @($arguments | ForEach-Object { '"' + $_.Replace('\', '/') + '"' }), [System.Text.UTF8Encoding]::new($false))
    & $Compiler ('@' + $response)
    if ($LASTEXITCODE -ne 0) { throw "Regression compile failed: $Name" }
    & $executable @RunArguments
    if ($LASTEXITCODE -ne 0) { throw "Regression assertion failed: $Name" }
}

Push-Location $projectRoot
try {
    if ($Suite -eq 'all' -or $Suite -eq 'model') {
        Invoke-Regression 'model_review' @('App/DebugUi/debug_ui_model.c', 'Tests/ui/model_review_regression.c') @($Case)
    }
    if ($Suite -eq 'all' -or $Suite -eq 'task') {
        $sources = @(Get-Content 'Ui/lvgl_sources.txt' | Where-Object { $_ -and -not $_.StartsWith('#') })
        $sources += @('Ui/debug_ui_view.c', 'Ui/debug_ui_graphics_guard.c', 'Ui/lv_port_disp.c',
            'Ui/lv_port_indev.c', 'Ui/fonts/ui_font_16.c', 'App/DebugUi/debug_ui_model.c',
            'App/DebugUi/debug_ui_input.c', 'App/DebugUi/debug_ui_mailbox.c', 'App/Config/build_info.c',
            'Tests/ui/task_display_regression.c')
        Invoke-Regression 'task_display' $sources @($imageDirectory, $Case)
    }
}
finally { Pop-Location }
