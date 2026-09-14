<# .SYNOPSIS
Compile and render the production UI using LVGL v8.3.11 and a fake SPI display.
Every translation unit is explicitly selected; demos/examples/assets are absent.
#>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
)
$ErrorActionPreference = 'Stop'

function Invoke-LvglRenderSmoke {
    <# Build selected real LVGL sources, render PPM and validate/convert them. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $buildDirectory = Join-Path $PSScriptRoot 'build\render'
    $imageDirectory = Join-Path $PSScriptRoot 'artifacts'
    New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null
    New-Item -ItemType Directory -Force -Path $imageDirectory | Out-Null
    $env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
    $sourcePaths = @(Get-Content -LiteralPath (Join-Path $projectRoot 'Ui\lvgl_sources.txt') |
        Where-Object { $_ -and -not $_.StartsWith('#') })
    $sourcePaths += @('Ui/debug_ui_view.c', 'Ui/debug_ui_graphics_guard.c', 'Ui/lv_port_disp.c',
        'Ui/lv_port_indev.c', 'Ui/fonts/ui_font_16.c', 'App/DebugUi/debug_ui_model.c',
        'Tests/ui/ui_render_smoke.c')
    $arguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-O1', '-g',
        '-DLV_CONF_INCLUDE_SIMPLE', '-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1',
        '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-IUi', '-IMiddlewares/Third_Party/LVGL', '-IApp/DebugUi')
    $arguments += $sourcePaths
    $arguments += @('-lm', '-o', 'Tests/ui/build/render/ui_render_smoke.exe')
    $responsePath = Join-Path $buildDirectory 'compile.rsp'
    [System.IO.File]::WriteAllLines($responsePath, $arguments, [System.Text.UTF8Encoding]::new($false))
    Push-Location $projectRoot
    try {
        & $Compiler ('@' + $responsePath)
        if ($LASTEXITCODE -ne 0) { throw 'LVGL render build failed.' }
        & (Join-Path $buildDirectory 'ui_render_smoke.exe') $imageDirectory (Join-Path $projectRoot 'Ui\fonts\charset.txt')
        if ($LASTEXITCODE -ne 0) { throw 'LVGL render/glyph/layout test failed.' }
        & $Python (Join-Path $PSScriptRoot 'verify_render_artifacts.py') $imageDirectory
        if ($LASTEXITCODE -ne 0) { throw 'Render artifact verification failed.' }
    }
    finally { Pop-Location }
}
Invoke-LvglRenderSmoke
