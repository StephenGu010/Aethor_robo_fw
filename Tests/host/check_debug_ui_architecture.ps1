<#
.SYNOPSIS
Checks the optional debug UI task, dependency and DMA memory boundaries.
.DESCRIPTION
Complements the existing Phase 0 checks. It does not certify hardware timing.
#>
$ErrorActionPreference = 'Stop'

function Assert-DebugUiContract {
    <# Records a contract failure so one run reports every missing integration. #>
    param([bool]$Condition, [string]$Message,
          [System.Collections.Generic.List[string]]$Failures)
    if (-not $Condition) { $Failures.Add($Message) }
}

function Invoke-DebugUiArchitectureCheck {
    <# Verifies the production source boundaries without running board code. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $failures = [System.Collections.Generic.List[string]]::new()
    $freertos = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\freertos.c') -Raw
    $uiTaskBlock = [regex]::Match($freertos,
        '(?s)#if AETHOR_DEBUG_UI_ENABLE\s+osThreadStaticDef\(DebugUiTask,.*?#endif')
    Assert-DebugUiContract $uiTaskBlock.Success 'DebugUiTask creation must be feature gated.' $failures
    Assert-DebugUiContract ($uiTaskBlock.Value -match 'osPriorityLow,\s*0,\s*2048,') `
        'DebugUiTask must use low priority and 2048 static stack words.' $failures
    Assert-DebugUiContract ($freertos -match 'aethor_app_debug_ui_process\s*\(') `
        'ProtocolTask must service local requests.' $failures
    Assert-DebugUiContract ($freertos -match 'protocolLinesProcessed\s*<\s*4U') `
        'USB input must have a bounded batch so local STOP cannot starve.' $failures
    $controlBody = [regex]::Match($freertos,
        '(?s)void StartArmControlTask\([^)]*\)\s*\{.*?(?=/\* USER CODE BEGIN Header_StartCanRxTask)')
    Assert-DebugUiContract ($controlBody.Success -and
        $controlBody.Value -notmatch '\blv_\w+\s*\(|lcd_st7789_|HAL_SPI_|HAL_ADC_') `
        'Control task must not render or access LCD/ADC hardware.' $failures
    $uiRoot = Join-Path $projectRoot 'Ui'
    if (Test-Path -LiteralPath $uiRoot) {
        foreach ($uiFile in (Get-ChildItem -LiteralPath $uiRoot -Recurse -File |
            Where-Object { $_.Extension -in @('.c', '.inc') })) {
            $source = Get-Content -LiteralPath $uiFile.FullName -Raw
            Assert-DebugUiContract ($source -notmatch '\b(?:s3519_|motor_runtime_|HAL_FDCAN_|protocol_engine_)\w*\s*\(') `
                ('UI bypasses the application command owner: ' + $uiFile.Name) $failures
            Assert-DebugUiContract ($source -notmatch 'aethor_app_process_protocol_line\s*\(') `
                ('UI must not impersonate a USB text client: ' + $uiFile.Name) $failures
        }
    } else { $failures.Add('Ui directory is missing.') }
    $scatterPath = Join-Path $projectRoot 'MDK-ARM\aethor_memory.sct'
    if (Test-Path -LiteralPath $scatterPath) {
        $scatter = Get-Content -LiteralPath $scatterPath -Raw
        Assert-DebugUiContract ($scatter -match 'RW_LCD_DMA\s+0x24000000\s+0x00008000') `
            'DMA buffer must have its reserved 32 KiB AXI region.' $failures
        Assert-DebugUiContract ($scatter -match '\(\.lcd_dma\)' -and $scatter -match '\(\.lvgl_pool\)') `
            'LCD buffers and LVGL pool need explicit link selectors.' $failures
    } else { $failures.Add('Maintained UI scatter file is missing.') }
    $projectXml = [xml](Get-Content -LiteralPath (Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx') -Raw)
    foreach ($targetName in @('CtrBoard-H7_FDCAN', 'LCD-ReadOnly', 'LCD-POS', 'LCD-MIT')) {
        $target = @($projectXml.Project.Targets.Target | Where-Object TargetName -eq $targetName)
        Assert-DebugUiContract ($target.Count -eq 1) ('Missing unique Keil target: ' + $targetName) $failures
    }
    if ($failures.Count -ne 0) { throw ($failures -join [Environment]::NewLine) }
    Write-Output 'DEBUG_UI_ARCHITECTURE_PASSED'
}

Invoke-DebugUiArchitectureCheck
