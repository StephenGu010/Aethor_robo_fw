<#
.SYNOPSIS
Checks the PRD Phase 0 firmware architecture contracts.

.DESCRIPTION
Verifies allocation, dependency, CubeMX, task-entry, and Keil project rules
without compiling or changing project files.
#>

$ErrorActionPreference = 'Stop'

function Add-ArchitectureFailure {
    <# Records one failed architecture rule with its evidence. #>
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyCollection()]
        [System.Collections.Generic.List[string]]$FailureList,

        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    $FailureList.Add($Message)
}

function Assert-TextContains {
    <# Records a failure when the supplied text does not match a required pattern. #>
    param(
        [Parameter(Mandatory = $true)]
        [AllowEmptyCollection()]
        [System.Collections.Generic.List[string]]$FailureList,

        [Parameter(Mandatory = $true)]
        [string]$Text,

        [Parameter(Mandatory = $true)]
        [string]$Pattern,

        [Parameter(Mandatory = $true)]
        [string]$Message
    )

    if ($Text -notmatch $Pattern)
    {
        Add-ArchitectureFailure -FailureList $FailureList -Message $Message
    }
}

function Invoke-Phase0ArchitectureCheck {
    <# Runs all Phase 0 architecture checks and returns a process-friendly result. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $failureList = [System.Collections.Generic.List[string]]::new()
    $applicationRoot = Join-Path $projectRoot 'App'
    $applicationFiles = Get-ChildItem -LiteralPath $applicationRoot -Recurse -File -Include '*.c', '*.h'
    $applicationPathPrefixLength = $applicationRoot.TrimEnd('\').Length + 1
    $allocationPattern = '\b(malloc|calloc|realloc|free|pvPortMalloc|vPortFree)\s*\('
    $platformIncludePattern = '^\s*#\s*include\s*[<"](?:stm32h7xx|cmsis_os|FreeRTOS|fdcan|usb_device|usbd_|usart)'
    $motorCommandPattern = '\b(?:enable_motor|disable_motor|motor_enable|motor_disable|dm_motor_|s3519_)\w*\s*\('

    foreach ($applicationFile in $applicationFiles)
    {
        $applicationText = Get-Content -LiteralPath $applicationFile.FullName -Raw
        if ($applicationText -match $allocationPattern)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "Dynamic allocation call found in $($applicationFile.FullName)."
        }

        $relativeApplicationPath = $applicationFile.FullName.Substring($applicationPathPrefixLength)
        if (($relativeApplicationPath -notlike 'Motor\*') -and
            ($applicationText -match $motorCommandPattern))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "Motor command escaped App/Motor in $($applicationFile.FullName)."
        }

        if (($relativeApplicationPath -notlike 'Platform\*') -and
            ($applicationText -match "(?m)$platformIncludePattern"))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "Platform dependency crossed into App/$relativeApplicationPath."
        }
    }

    $iocText = Get-Content -LiteralPath (Join-Path $projectRoot 'CtrBoard-H7_FDCAN.ioc') -Raw
    Assert-TextContains -FailureList $failureList -Text $iocText `
        -Pattern 'FREERTOS\.Tasks01=defaultTask,0,512,StartDefaultTask,Default,NULL,Static,defaultTaskBuffer,defaultTaskControlBlock' `
        -Message 'CubeMX defaultTask is not configured for static allocation.'
    Assert-TextContains -FailureList $failureList -Text $iocText `
        -Pattern 'PA15\(JTDI\)\.GPIO_Label=USER_KEY' `
        -Message 'CubeMX USER_KEY label is missing from PA15.'

    $mainText = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\main.c') -Raw
    $freertosText = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\freertos.c') -Raw
    Assert-TextContains -FailureList $failureList -Text $mainText `
        -Pattern 'aethor_app_init\s*\(' `
        -Message 'main.c does not initialize the Phase 0 application facade.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'osThreadStaticDef\s*\(' `
        -Message 'freertos.c does not create defaultTask statically.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_service\s*\(' `
        -Message 'freertos.c does not service the Phase 0 application facade.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'pdMS_TO_TICKS\s*\(\s*4U\s*\)' `
        -Message 'defaultTask period is not 4 ms.'
    if (($mainText -match 'aethor_application') -or ($freertosText -match 'aethor_application'))
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'Legacy aethor_application entry point is still referenced.'
    }

    $keilProjectPath = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
    $keilProjectXml = [xml](Get-Content -LiteralPath $keilProjectPath -Raw)
    $keilFileNames = @($keilProjectXml.SelectNodes('//FileName') | ForEach-Object { $_.'#text' })
    $requiredKeilSources = @(
        'aethor_app.c',
        'arm_config.c',
        'build_info.c',
        'arm_controller.c',
        'diagnostics.c',
        'ascii_protocol.c',
        'can_frame.c',
        'can_tx_scheduler.c',
        'motor_bank.c',
        'motor_discovery.c',
        's3519_codec.c',
        'can_rx_inbox.c',
        'usb_cdc_stream.c',
        'stm32_platform.c',
        'usbd_core.c',
        'usbd_ctlreq.c',
        'usbd_ioreq.c',
        'usbd_cdc.c'
    )
    foreach ($requiredKeilSource in $requiredKeilSources)
    {
        $sourceCount = @($keilFileNames | Where-Object { $_ -eq $requiredKeilSource }).Count
        if ($sourceCount -ne 1)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "$requiredKeilSource occurs $sourceCount times in the Keil target; expected 1."
        }
    }

    foreach ($legacyKeilSource in @('aethor_application.c', 'dual_motor_controller.c', 'joint_controller.c', 'bsp_fdcan.c', 'usb_cdc_transport.c'))
    {
        if ($keilFileNames -contains $legacyKeilSource)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "$legacyKeilSource must not be compiled in PRD Phase 0."
        }
    }

    $usbInterfaceText = Get-Content -LiteralPath (Join-Path $projectRoot 'USB_DEVICE\App\usbd_cdc_if.c') -Raw
    Assert-TextContains -FailureList $failureList -Text $usbInterfaceText `
        -Pattern 'stm32_platform_usb_receive_isr\s*\(' `
        -Message 'USB CDC receive callback is not connected to the new platform stream.'
    Assert-TextContains -FailureList $failureList -Text $usbInterfaceText `
        -Pattern 'stm32_platform_usb_tx_complete_isr\s*\(' `
        -Message 'USB CDC transmit-complete callback is not connected to the in-flight buffer lifetime.'

    if ($failureList.Count -ne 0)
    {
        foreach ($failureMessage in $failureList)
        {
            Write-Host "[FAIL] $failureMessage"
        }
        exit 1
    }

    Write-Host '[PASS] App sources contain no dynamic allocation calls.'
    Write-Host '[PASS] App business layers do not include platform headers.'
    Write-Host '[PASS] Executable motor frame generation is confined to App/Motor.'
    Write-Host '[PASS] CubeMX retains the static default task and USER_KEY label.'
    Write-Host '[PASS] main.c and freertos.c use the Phase 0 application entry.'
    Write-Host '[PASS] Keil compiles one copy of each required source and no legacy controller.'
    Write-Host '[PASS] Phase 0 architecture contracts are satisfied.'
}

Invoke-Phase0ArchitectureCheck
