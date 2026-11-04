<# .SYNOPSIS Renders only the new static observation cases using maintained actual LVGL and flush helpers. #>
param(
    [string]$Compiler = 'D:\application\WinLibs\mingw64\bin\gcc.exe',
    [string]$Python = 'C:\Users\Aethor_ca\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe',
    [string]$OutputDirectory = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$buildDirectory = Join-Path $PSScriptRoot 'build\static_peak_reason'
$imageDirectory = Join-Path $projectRoot 'output\repair_20261023\static_peak_reason_ui_fix\render'
if ($OutputDirectory) { $imageDirectory = [System.IO.Path]::GetFullPath($OutputDirectory) }
[void](New-Item -ItemType Directory -Force -Path $buildDirectory)
[void](New-Item -ItemType Directory -Force -Path $imageDirectory)
$env:PATH = (Split-Path $Compiler) + ';' + $env:PATH
$sourcePaths = @(Get-Content -LiteralPath (Join-Path $projectRoot 'Ui\lvgl_sources.txt') |
    Where-Object { $_ -and -not $_.StartsWith('#') })
$sourcePaths += @('Ui/debug_ui_view.c', 'Ui/debug_ui_graphics_guard.c', 'Ui/lv_port_disp.c',
    'Ui/lv_port_indev.c', 'Ui/fonts/ui_font_16.c', 'App/DebugUi/debug_ui_model.c',
    'Tests/ui/static_peak_reason_render.c')
$arguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter', '-O1', '-g',
    '-DLV_CONF_INCLUDE_SIMPLE', '-DAETHOR_DEBUG_UI_ENABLE=1', '-DAETHOR_DEBUG_UI_ALLOW_MOTION=1',
    '-DAETHOR_DEBUG_UI_ALLOW_MIT=1', '-DAETHOR_DEBUG_UI_ALLOW_STATIC_LOAD=1', '-DAETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD=1',
    '-DAETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST=1', '-IUi', '-IMiddlewares/Third_Party/LVGL', '-IApp/DebugUi')
$arguments += $sourcePaths
$executable = Join-Path $buildDirectory 'static_peak_reason_render.exe'
$arguments += @('-lm', '-o', 'Tests/ui/build/static_peak_reason/static_peak_reason_render.exe')
$responsePath = Join-Path $buildDirectory 'compile.rsp'
[System.IO.File]::WriteAllLines($responsePath, $arguments, [System.Text.UTF8Encoding]::new($false))
Push-Location $projectRoot
try {
    & $Compiler ('@' + $responsePath)
    if ($LASTEXITCODE -ne 0) { throw 'Static peak LVGL compilation failed.' }
    & $executable $imageDirectory (Join-Path $projectRoot 'Ui\fonts\charset.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Static peak LVGL geometry/glyph/semantic regression failed.' }
    & $Python (Join-Path $PSScriptRoot 'verify_static_peak_render.py') $imageDirectory
    if ($LASTEXITCODE -ne 0) { throw 'Static peak raster verification failed.' }
}
finally { Pop-Location }
