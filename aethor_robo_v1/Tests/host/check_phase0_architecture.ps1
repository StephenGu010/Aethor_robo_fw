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
    $requiredStaticTasks = @(
        'ArmControlTask',
        'CanRxTask',
        'ProtocolTask',
        'UsbTxTask',
        'TelemetryTask',
        'DiagnosticsTask'
    )
    foreach ($requiredStaticTask in $requiredStaticTasks)
    {
        Assert-TextContains -FailureList $failureList -Text $iocText `
            -Pattern "FREERTOS\.Tasks01=[^\r\n]*\b$requiredStaticTask,[^;]+,Static,[^;]+" `
            -Message "CubeMX does not preserve static task $requiredStaticTask."
    }
    Assert-TextContains -FailureList $failureList -Text $iocText `
        -Pattern 'PA15\(JTDI\)\.GPIO_Label=USER_KEY' `
        -Message 'CubeMX USER_KEY label is missing from PA15.'

    $mainText = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\main.c') -Raw
    $freertosText = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\freertos.c') -Raw
    Assert-TextContains -FailureList $failureList -Text $mainText `
        -Pattern 'aethor_app_init\s*\(' `
        -Message 'main.c does not initialize the Phase 0 application facade.'
    foreach ($requiredStaticTask in $requiredStaticTasks)
    {
        Assert-TextContains -FailureList $failureList -Text $freertosText `
            -Pattern "osThreadStaticDef\s*\(\s*$requiredStaticTask\s*," `
            -Message "freertos.c does not create $requiredStaticTask statically."
    }
    if (([regex]::Matches($freertosText, 'osThreadStaticDef\s*\(')).Count -ne $requiredStaticTasks.Count)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'freertos.c must create exactly six static application tasks.'
    }
    $requiredTaskStackWords = [ordered]@{
        ArmControlTask = 768
        ProtocolTask = 1280
        TelemetryTask = 1024
    }
    foreach ($taskStackRequirement in $requiredTaskStackWords.GetEnumerator())
    {
        $taskName = $taskStackRequirement.Key
        $stackWords = $taskStackRequirement.Value
        Assert-TextContains -FailureList $failureList -Text $freertosText `
            -Pattern "osThreadStaticDef\s*\(\s*$taskName\s*,[^\r\n]*,\s*$stackWords\s*," `
            -Message "freertos.c does not reserve $stackWords stack words for $taskName."
        Assert-TextContains -FailureList $failureList -Text $iocText `
            -Pattern "FREERTOS\.Tasks01=[^\r\n]*\b$taskName,[^,]+,$stackWords," `
            -Message "CubeMX does not preserve $stackWords stack words for $taskName."
    }
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_service\s*\(' `
        -Message 'freertos.c does not service the Phase 0 application facade.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_next_can_frame\s*\(' `
        -Message 'ArmControlTask does not produce bounded discovery traffic.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'stm32_platform_can_submit\s*\(' `
        -Message 'ArmControlTask does not submit application CAN traffic.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'stm32_platform_can_pop_received\s*\(' `
        -Message 'CanRxTask does not drain the bounded ISR inbox.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_receive_can_frame\s*\(' `
        -Message 'CanRxTask does not route received frames to the motor runtime.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'osThreadStaticDef\s*\(\s*ArmControlTask\s*,[^\r\n]*osPriorityRealtime' `
        -Message 'ArmControlTask must remain higher priority than CanRxTask.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'osThreadStaticDef\s*\(\s*CanRxTask\s*,[^\r\n]*osPriorityHigh' `
        -Message 'CanRxTask priority evidence changed; re-evaluate MotorRuntime ownership.'
    # CanRxTask and the higher-priority ArmControlTask share MotorRuntime. Keep
    # only the in-memory receive decode inside a task critical section; obtain
    # time and perform platform/blocking work outside it.
    $canRxTaskMatch = [regex]::Match(
        $freertosText,
        'void\s+StartCanRxTask\s*\([^)]*\)[\s\S]*?(?=/\* USER CODE BEGIN Header_StartProtocolTask \*/)')
    if (-not $canRxTaskMatch.Success)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'CanRxTask body could not be isolated for concurrency checks.'
    }
    else
    {
        Assert-TextContains -FailureList $failureList -Text $canRxTaskMatch.Value `
            -Pattern 'timestampUs\s*=\s*AethorMonotonicTimestampUs\s*\(\s*\)\s*;\s*taskENTER_CRITICAL\s*\(\s*\)\s*;\s*\(void\)aethor_app_receive_can_frame\s*\(\s*&receivedFrame\s*,\s*timestampUs\s*\)\s*;\s*taskEXIT_CRITICAL\s*\(\s*\)' `
            -Message 'CanRxTask does not serialize the in-memory MotorRuntime receive update.'
        $canRxCriticalMatch = [regex]::Match(
            $canRxTaskMatch.Value,
            'taskENTER_CRITICAL\s*\(\s*\)\s*;(?<Body>[\s\S]*?)taskEXIT_CRITICAL\s*\(\s*\)')
        if ($canRxCriticalMatch.Success -and
            ($canRxCriticalMatch.Groups['Body'].Value -match 'AethorMonotonicTimestampUs|stm32_platform_|\bHAL_|\b(?:ul|x|v)Task|taskYIELD|portMAX_DELAY'))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'CanRxTask critical section contains timestamp, platform I/O, or blocking work.'
        }
    }
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'stm32_platform_usb_next_line\s*\(' `
        -Message 'ProtocolTask does not drain complete USB lines in task context.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_process_protocol_line\s*\(' `
        -Message 'ProtocolTask does not dispatch parsed requests to the protocol engine.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'pdMS_TO_TICKS\s*\(\s*4U\s*\)' `
        -Message 'ArmControlTask period is not 4 ms.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'pdMS_TO_TICKS\s*\(\s*10U\s*\)' `
        -Message 'TelemetryTask does not service the 100 Hz maximum cadence.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'pdMS_TO_TICKS\s*\(\s*100U\s*\)' `
        -Message 'DiagnosticsTask period is not 100 ms.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'ulTaskNotifyTake\s*\(' `
        -Message 'RX-driven tasks do not wait on bounded task notifications.'
    # DiagnosticsTask may only publish transport faults; ArmControlTask owns
    # lifecycle mutation and ProtocolTask notification after service.
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_report_transport_fault[\s\S]{0,500}ArmControlTaskHandle[\s\S]{0,200}xTaskNotifyGive\s*\(\s*\(TaskHandle_t\)ArmControlTaskHandle' `
        -Message 'DiagnosticsTask does not wake ArmControlTask after publishing a transport fault.'
    if ($freertosText -match 'aethor_app_report_transport_fault[\s\S]{0,500}xTaskNotifyGive\s*\(\s*\(TaskHandle_t\)ProtocolTaskHandle')
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'DiagnosticsTask must not notify ProtocolTask directly for a transport fault.'
    }
    if (($mainText -match 'aethor_application') -or ($freertosText -match 'aethor_application'))
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'Legacy aethor_application entry point is still referenced.'
    }
    # Lower-priority Protocol/Telemetry readers use application-injected hooks
    # so the platform and host share the same short scheduling boundary.
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'EnterAethorAppTaskCritical[\s\S]{0,160}taskENTER_CRITICAL\s*\(\s*\)' `
        -Message 'freertos.c does not provide the application task-critical enter hook.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'ExitAethorAppTaskCritical[\s\S]{0,160}taskEXIT_CRITICAL\s*\(\s*\)' `
        -Message 'freertos.c does not provide the application task-critical exit hook.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_set_task_critical_hooks\s*\(\s*EnterAethorAppTaskCritical\s*,\s*ExitAethorAppTaskCritical\s*\)' `
        -Message 'FreeRTOS initialization does not register application query critical hooks.'

    # Cleanup HOLD is allowed only after this one-shot confirmed every selected
    # motor enabled; keep this lifecycle gate explicit in the application layer.
    $aethorAppText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\aethor_app.c') -Raw
    $protocolEngineText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\Protocol\protocol_engine.c') -Raw
    $protocolContextMatch = [regex]::Match(
        $aethorAppText,
        'static\s+void\s+aethor_app_update_protocol_context\s*\([^)]*\)[\s\S]*?(?=/\*\*[\r\n]+ \* @brief Initializes all static Phase 0 application state\.)')
    if (-not $protocolContextMatch.Success)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'Protocol query-context update body could not be isolated.'
    }
    else
    {
        Assert-TextContains -FailureList $failureList -Text $protocolContextMatch.Value `
            -Pattern 'aethor_app_enter_task_critical\s*\(\s*\)[\s\S]{0,240}arm_controller_get_snapshot[\s\S]*protocol_engine_update_query_context[\s\S]{0,160}aethor_app_exit_task_critical\s*\(\s*\)' `
            -Message 'Protocol query snapshots are not published inside paired task-critical hooks.'
        $queryCriticalMatch = [regex]::Match(
            $protocolContextMatch.Value,
            'aethor_app_enter_task_critical\s*\(\s*\)\s*;(?<Body>[\s\S]*?)aethor_app_exit_task_critical\s*\(\s*\)')
        if ($queryCriticalMatch.Success -and
            ($queryCriticalMatch.Groups['Body'].Value -match 'process_text|generate_stream|append_|format_|stm32_platform_|\bHAL_|\b(?:ul|x|v)Task|taskYIELD|portMAX_DELAY'))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'Protocol query critical section contains parsing, formatting, I/O, or blocking work.'
        }
    }
    Assert-TextContains -FailureList $failureList -Text $aethorAppText `
        -Pattern 'aethor_app_process_protocol_line[\s\S]{0,700}aethor_app_update_protocol_context\s*\([^;]+;[\s\S]{0,200}protocol_engine_process_text_line' `
        -Message 'Protocol parsing is not kept after the bounded query snapshot update.'
    Assert-TextContains -FailureList $failureList -Text $aethorAppText `
        -Pattern 'aethor_app_generate_stream_output[\s\S]{0,500}aethor_app_update_protocol_context\s*\([^;]+;[\s\S]{0,200}protocol_engine_generate_stream_output' `
        -Message 'Stream formatting is not kept after the bounded query snapshot update.'
    Assert-TextContains -FailureList $failureList -Text $aethorAppText `
        -Pattern 'aethor_app_get_motor_snapshot[\s\S]{0,500}aethor_app_enter_task_critical[\s\S]{0,240}motor_runtime_get_snapshot[\s\S]{0,240}aethor_app_exit_task_critical' `
        -Message 'Public MotorRuntime snapshot reads do not use the shared task-critical hooks.'
    Assert-TextContains -FailureList $failureList -Text $aethorAppText `
        -Pattern 'application_action\.enabled_by_action_mask\s*&\s*application_action\.cleanup_disable_mask\)\s*==\s*application_action\.cleanup_disable_mask' `
        -Message 'One-shot cleanup HOLD does not require enabled_by_action_mask to cover the cleanup mask.'
    # STOP ownership is part of priority-slot publication, so the consumer may
    # never observe a STOP before its admission gate has become visible.
    Assert-TextContains -FailureList $failureList -Text $protocolEngineText `
        -Pattern 'active_stop_request_id\s*=\s*command->request_id;[\s\S]{0,160}\+\+engine->stop_write_sequence' `
        -Message 'STOP lifecycle ownership is published after its priority command slot.'
    # A normal motion slot is visible only after its admission metadata. This
    # prevents ArmControlTask from consuming a MOVE before its gate is owned.
    Assert-TextContains -FailureList $failureList -Text $protocolEngineText `
        -Pattern 'protocol_engine_publish_normal_command[\s\S]{0,900}commands\[slot_index\]\s*=\s*\*command;[\s\S]{0,500}active_motion_request_id\s*=\s*command->request_id;[\s\S]{0,240}active_motion_accepted_at_us\s*=\s*command->accepted_at_us;[\s\S]{0,240}active_motion_planned_duration_us\s*=\s*command->planned_duration_us;[\s\S]{0,240}protocol_engine_compiler_barrier\s*\(\s*\)\s*;[\s\S]{0,120}\+\+engine->command_write_sequence' `
        -Message 'Normal MOVE ownership metadata is not published before command_write_sequence.'

    $keilProjectPath = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
    $keilProjectXml = [xml](Get-Content -LiteralPath $keilProjectPath -Raw)
    $keilFileNames = @($keilProjectXml.SelectNodes('//FileName') | ForEach-Object { $_.'#text' })
    $requiredKeilSources = @(
        'aethor_app.c',
        'arm_config.c',
        'build_info.c',
        'arm_controller.c',
        'joint_reference.c',
        'joint_motion.c',
        'diagnostics.c',
        'ascii_protocol.c',
        'protocol_engine.c',
        'can_frame.c',
        'can_tx_scheduler.c',
        'motor_bank.c',
        'motor_discovery.c',
        'motor_runtime.c',
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
    Write-Host '[PASS] CubeMX retains six static application tasks and the USER_KEY label.'
    Write-Host '[PASS] main.c and freertos.c use the Phase 0 application entry.'
    Write-Host '[PASS] One-shot cleanup HOLD requires completed ENABLE ownership.'
    Write-Host '[PASS] Keil compiles one copy of each required source and no legacy controller.'
    Write-Host '[PASS] Phase 0 architecture contracts are satisfied.'
}

Invoke-Phase0ArchitectureCheck
