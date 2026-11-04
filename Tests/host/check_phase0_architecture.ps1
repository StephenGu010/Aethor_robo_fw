<#
.SYNOPSIS
Checks the PRD Phase 0 firmware architecture contracts.

.DESCRIPTION
Verifies allocation, dependency, CubeMX, task-entry, exact three-source queued-arrival bridges,
and Keil project rules without compiling or changing project files.
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

function Test-NormalCommandPublicationOrder {
    <# Checks publication order within the named C function, regardless of its size. #>
    param(
        [Parameter(Mandatory = $true)]
        [string]$Text
    )

    $publicationFunction = [regex]::Match($Text,
        '(?ms)^static uint8_t protocol_engine_publish_normal_command\([^)]*\)\s*\{.*?^\}')
    if (-not $publicationFunction.Success) { return $false }
    if (([regex]::Matches($publicationFunction.Value, '\+\+engine->command_write_sequence')).Count -ne 1) {
        return $false
    }

    $publicationPattern = 'commands\[slot_index\]\s*=\s*\*command;[\s\S]*?' +
        'active_motion_request_id\s*=\s*command->request_id;[\s\S]*?' +
        'active_motion_origin\s*=\s*command->origin;[\s\S]*?' +
        'active_motion_epoch\s*=\s*command->session_id;[\s\S]*?' +
        'active_motion_accepted_at_us\s*=\s*command->accepted_at_us;[\s\S]*?' +
        'active_motion_planned_duration_us\s*=\s*command->planned_duration_us;[\s\S]*?' +
        'protocol_engine_compiler_barrier\s*\(\s*\)\s*;[\s\S]*?' +
        '\+\+engine->command_write_sequence'
    return $publicationFunction.Value -match $publicationPattern
}

function Test-StopCommandPublicationOrder {
    <# Verifies STOP payload and complete source identity before its one publish. #>
    param([Parameter(Mandatory = $true)][string]$Text)
    $publicationFunction = [regex]::Match($Text,
        '(?ms)^static uint8_t protocol_engine_enqueue_command\([^)]*\)\s*\{.*?^\}')
    if (-not $publicationFunction.Success) { return $false }
    if (([regex]::Matches($publicationFunction.Value, '\+\+engine->stop_write_sequence')).Count -ne 1) {
        return $false
    }
    $publicationPattern = 'stop_command\s*=\s*\*command;[\s\S]*?' +
        'active_stop_request_id\s*=\s*command->request_id;[\s\S]*?' +
        'active_stop_origin\s*=\s*command->origin;[\s\S]*?' +
        'active_stop_epoch\s*=\s*command->session_id;[\s\S]*?' +
        'protocol_engine_compiler_barrier\s*\(\s*\)\s*;[\s\S]*?' +
        '\+\+engine->stop_write_sequence'
    return $publicationFunction.Value -match $publicationPattern
}

function Test-MotorCommandBoundary {
    <# Allows only the inherited, feature-guarded maintenance codecs with their fixed arguments. #>
    param([string]$Text, [string]$RelativePath)
    $commandPattern = '\b(?:enable_motor|disable_motor|motor_enable|motor_disable|dm_motor_|s3519_)\w*\s*\('
    if ($RelativePath -like 'Motor\*') { return $true }
    if ($RelativePath -ne 'aethor_app.c') { return $Text -notmatch $commandPattern }
    $allowedCalls = @(
        @{ Function = 'aethor_app_mapping_receive'; Call = 's3519_decode_parameter_response\(frame,\s*&response\)' },
        @{ Function = 'aethor_app_mapping_receive'; Call = 's3519_decode_feedback\(frame,\s*&conservative_ranges,\s*&feedback\)' },
        @{ Function = 'aethor_app_mapping_next'; Call = 's3519_pack_parameter_read\(7U,\s*\(S3519Register\)mapping->register_address,\s*frame\)' },
        @{ Function = 'aethor_app_mapping_next'; Call = 's3519_pack_mode_command\(7U,\s*S3519_CONTROL_MODE_POSITION_VELOCITY,\s*S3519_MODE_COMMAND_DISABLE,\s*frame\)' },
        @{ Function = 'aethor_app_pop_emergency_can_frame'; Call = 's3519_pack_mode_command\(7U,\s*S3519_CONTROL_MODE_POSITION_VELOCITY,\s*S3519_MODE_COMMAND_DISABLE,\s*frame\)' }
    )
    $remaining = $Text
    foreach ($allowed in $allowedCalls) {
        $functionMatch = [regex]::Match($remaining, '(?ms)^(?:static\s+)?\w+\s+' + $allowed.Function + '\([^)]*\)\s*\{.*?^\}')
        if (-not $functionMatch.Success) { return $false }
        $calls = [regex]::Matches($functionMatch.Value, $allowed.Call)
        if ($calls.Count -ne 1) { return $false }
        $callOffset = $functionMatch.Index + $calls[0].Index
        $guards = [System.Collections.Generic.List[string]]::new()
        foreach ($directive in [regex]::Matches($remaining.Substring(0, $callOffset), '(?m)^\s*#\s*(if|ifdef|ifndef|elif|else|endif)\b([^\r\n]*)')) {
            $kind = $directive.Groups[1].Value
            if ($kind -eq 'endif') {
                if ($guards.Count -eq 0) { return $false }
                $guards.RemoveAt($guards.Count - 1)
            } elseif ($kind -eq 'else' -or $kind -eq 'elif') {
                if ($guards.Count -eq 0) { return $false }
                $guards[$guards.Count - 1] = ''
            } else { $guards.Add($kind + ' ' + $directive.Groups[2].Value.Trim()) }
        }
        if (-not $guards.Contains('if AETHOR_ADRC_LCD_INTEGRATED')) { return $false }
        $remaining = $remaining.Remove($callOffset, $calls[0].Length)
    }
    return $remaining -notmatch $commandPattern
}

function Get-CanRxContractDefinition {
    <# Finds one complete C definition with legal indentation and requires its exact enclosing feature guard, or top-level ownership. #>
    param([Parameter(Mandatory = $true)][string]$Code,
        [Parameter(Mandatory = $true)][string]$FunctionName,
        [Parameter(Mandatory = $true)][ValidateSet('void', 'static void', 'MotorRuntimeStatus')][string]$ReturnType,
        [AllowEmptyString()][string]$ExpectedGuard = '')
    $returnTypePattern = (($ReturnType -split ' ' | ForEach-Object { [regex]::Escape($_) }) -join '[ \t]+')
    $definitionPattern = '(?ms)^[ \t]*' + $returnTypePattern + '[ \t]+' + [regex]::Escape($FunctionName) +
        '[ \t]*\([^)]*\)\s*\{(?:[^{}]|(?<Nested>\{)|(?<-Nested>\}))*(?(Nested)(?!))\}'
    $definitions = [regex]::Matches($Code, $definitionPattern)
    if ($definitions.Count -ne 1) { return $null }
    $guards = [System.Collections.Generic.List[string]]::new()
    foreach ($directive in [regex]::Matches($Code.Substring(0, $definitions[0].Index),
        '(?m)^[ \t]*#[ \t]*(if|ifdef|ifndef|elif|else|endif)\b([^\r\n]*)')) {
        $kind = $directive.Groups[1].Value
        if ($kind -eq 'endif') {
            if ($guards.Count -eq 0) { return $null }
            $guards.RemoveAt($guards.Count - 1)
        } elseif ($kind -eq 'else' -or $kind -eq 'elif') {
            if ($guards.Count -eq 0) { return $null }
            $guards[$guards.Count - 1] = ''
        } else {
            $guards.Add([regex]::Replace($kind + $directive.Groups[2].Value, '\s+', ''))
        }
    }
    if ($ExpectedGuard.Length -eq 0) {
        if ($guards.Count -ne 0) { return $null }
    } elseif ($guards.Count -ne 1 -or $guards[0] -cne [regex]::Replace($ExpectedGuard, '\s+', '')) {
        return $null
    }
    return $definitions[0]
}

function Test-CanRxCriticalBoundary {
    <# Whitelists exact real receive bridges from three explicit sources and rejects source-local macro shadowing, without preprocessing unrelated C. #>
    param([Parameter(Mandatory = $true)][string]$Text,
        [Parameter(Mandatory = $true)][string]$FacadeText,
        [Parameter(Mandatory = $true)][string]$StaticTransportText)
    # Join continued lines, then replace actual comments with spaces; quoted comment markers cannot hide directives.
    $commentOrLiteral = '(?s)"(?:\\.|[^"\\])*"|''(?:\\.|[^''\\])*''|/\*.*?\*/|//[^\r\n]*'
    $retainLiteral = { param($token) if ($token.Value.StartsWith('/*') -or $token.Value.StartsWith('//')) { ' ' } else { $token.Value } }
    $coreCode = [regex]::Replace([regex]::Replace($Text, '\\(?:\r\n|\n|\r)', ''), $commentOrLiteral, $retainLiteral)
    $facadeCode = [regex]::Replace([regex]::Replace($FacadeText, '\\(?:\r\n|\n|\r)', ''), $commentOrLiteral, $retainLiteral)
    $staticTransportCode = [regex]::Replace([regex]::Replace($StaticTransportText, '\\(?:\r\n|\n|\r)', ''), $commentOrLiteral, $retainLiteral)
    $controlledSymbols = @('StartCanRxTask', 'ReceiveSingleStaticFrame', 'CollectSingleStaticReceipt', 'CollectCapabilityReceipt',
        'AethorMonotonicTimestampUs', 'HAL_GetTick', 'aethor_app_receive_can_frame', 'aethor_app_receive_can_frame_observed',
        'arrivalLowerBoundUs', 'receivedObservedUs', 'arrivalLowerUs', 'observedUs', 'arrival_lower_us', 'observed_us',
        'aethor_app_debug_ui_receive_preflight_frame', 'taskENTER_CRITICAL', 'taskEXIT_CRITICAL',
        'stm32_platform_can_pop_received_at', 'stm32_platform_can_single_static_rx_identity',
        'stm32_platform_can_collect_single_static_receipt', 'aethor_app_debug_ui_preflight_transmitted',
        'aethor_app_enter_task_critical', 'aethor_app_exit_task_critical',
        'AETHOR_DEBUG_UI_ALLOW_MOTION', 'AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD', 'AETHOR_ADRC_BENCH', 'AETHOR_ADRC_LCD_INTEGRATED',
        'AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD', 'AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST', 'aethor_app_receive_static_frame', 'aethor_app_static_transmitted',
        'stm32_platform_can_static_rx_identity', 'stm32_platform_can_collect_static', 'DebugUiStaticTxIdentity',
        'DebugUiIdentity', 'orderedIdentity', 'identity', 'ordered', 'lower_us', 'sof', 'sofValid', 'sof_valid',
        'epoch', 'request_id', 'sequence', 'phase', 'motor_id', 'DEBUG_UI_STATIC_TX_PREFLIGHT', 'DEBUG_UI_STATIC_TX_POS_DISABLE',
        'pos_disable_identity', 'pos_disable_submitted', 'pos_disable_confirmed', 'pos_disable_failed',
        'pos_disable_rx_generation', 'pos_disable_ordered_generation',
        'DEBUG_UI_ORIGIN_USB', 'DEBUG_UI_ORIGIN_LOCAL_UI', 'ARM_JOINT_COUNT', 'DEBUG_UI_MOTOR_COUNT',
        'STATIC_TRACE_RAW_RX', 'STATIC_TRACE_ACCEPTED_RX', 'application_action', 'application_debug_ui',
        'application_motor_runtime', 'aethor_app_static_trace_record_identity', 'aethor_app_static_trace_record',
        'debug_ui_static_tx_identity_equal', 'succeeded', 'trace_identity',
        'single_static_tx_confirmed', 'single_static_tx_submitted', 'single_static_tx_failed',
        'single_static_rx_generation', 'single_static_ordered_generation', 'feedback_generation',
        'static_identity', 'static_transport_identity', 'state', 'AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT',
        'origin', 'session_id', 'command', 'local_operation', 'motor_mask',
        'MOTOR_RUNTIME_STATUS_OK', 'DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT',
        'bank', 'motors', 'joint_index', 'selected_index', 'generation_before', 'before', 'after',
        'frame', 'requestId', 'ordered_identity', 'legacy', 'status', 'NULL', 'CanFrame', 'MotorRuntimeStatus',
        'uint8_t', 'uint16_t', 'uint32_t', 'uint64_t', 'void', 'const', 'static',
        'if', 'else', 'endif', 'for', 'continue', 'return')
    $controlledMacro = '(?m)^[ \t]*#[ \t]*(?:define|undef)[ \t]+(?:' +
        (($controlledSymbols | ForEach-Object { [regex]::Escape($_) }) -join '|') + ')\b'
    foreach ($explicitCode in @($coreCode, $facadeCode, $staticTransportCode)) {
        if ([regex]::IsMatch($explicitCode, $controlledMacro)) { return $false }
    }
    $receiveFunction = Get-CanRxContractDefinition -Code $coreCode -FunctionName 'StartCanRxTask' -ReturnType 'void'
    if ($null -eq $receiveFunction) { return $false }
    $locks = [regex]::Matches($receiveFunction.Value, 'taskENTER_CRITICAL\s*\(\s*\)')
    $unlocks = [regex]::Matches($receiveFunction.Value, 'taskEXIT_CRITICAL\s*\(\s*\)')
    if ($locks.Count -ne 1 -or $unlocks.Count -ne 1) { return $false }
    $critical = [regex]::Match($receiveFunction.Value, 'taskENTER_CRITICAL\s*\(\s*\)\s*;(?<Body>[\s\S]*?)taskEXIT_CRITICAL\s*\(\s*\)')
    $legacy = '(void)aethor_app_receive_can_frame(&receivedFrame,arrivalLowerBoundUs);'
    $observed = '#ifAETHOR_ADRC_BENCH||AETHOR_ADRC_LCD_INTEGRATED{uint64_treceivedObservedUs=AethorMonotonicTimestampUs();' +
        'CollectCapabilityReceipt(receivedObservedUs);(void)aethor_app_receive_can_frame_observed(&receivedFrame,arrivalLowerBoundUs,receivedObservedUs);}' +
        '#else' + $legacy + '#endif'
    $single = '#ifAETHOR_DEBUG_UI_ALLOW_MOTION&&AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD{' +
        'uint64_treceivedObservedUs=AethorMonotonicTimestampUs();#ifAETHOR_ADRC_BENCH||AETHOR_ADRC_LCD_INTEGRATED' +
        'CollectCapabilityReceipt(receivedObservedUs);#endifReceiveSingleStaticFrame(&receivedFrame,arrivalLowerBoundUs,receivedObservedUs);}' +
        '#else' + $observed + '#endif'
    $fullStatic = $single.Replace('#ifAETHOR_DEBUG_UI_ALLOW_MOTION&&AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD{',
        '#ifAETHOR_DEBUG_UI_ALLOW_MOTION&&(AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD||AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD||AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST){')
    if (-not $critical.Success) { return $false }
    $criticalCode = [regex]::Replace($critical.Groups['Body'].Value, '\s+', '')
    if ($criticalCode -cne $legacy -and $criticalCode -cne $observed -and $criticalCode -cne $single -and $criticalCode -cne $fullStatic) { return $false }
    foreach ($ownedCall in @('aethor_app_receive_can_frame(?:_observed)?', 'aethor_app_debug_ui_receive_preflight_frame',
        'aethor_app_receive_static_frame', 'aethor_app_static_transmitted', 'stm32_platform_can_static_rx_identity',
        'stm32_platform_can_collect_static', 'ReceiveSingleStaticFrame', 'AethorMonotonicTimestampUs',
        'CollectCapabilityReceipt', 'CollectSingleStaticReceipt')) {
        $callPattern = '\b' + $ownedCall + '\s*\('
        if ([regex]::Matches($receiveFunction.Value, $callPattern).Count -ne
            [regex]::Matches($critical.Value, $callPattern).Count) { return $false }
    }
    if ($criticalCode -ceq $single -or $criticalCode -ceq $fullStatic) {
        # This is an exact bounded allowlist, not acceptance of an arbitrary wrapper name.
        $expectedCoreBridge = @'
#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
static void CollectSingleStaticReceipt(void)
{
  uint32_t epoch, requestId;
  uint8_t succeeded;
  if (stm32_platform_can_collect_single_static_receipt(&epoch, &requestId, &succeeded))
  { aethor_app_debug_ui_preflight_transmitted(epoch, requestId, succeeded); }
}
static void ReceiveSingleStaticFrame(const CanFrame *frame, uint64_t arrivalLowerUs, uint64_t observedUs)
{
  DebugUiIdentity orderedIdentity = {DEBUG_UI_ORIGIN_USB, 0U, 0U};
  CollectSingleStaticReceipt();
  if (stm32_platform_can_single_static_rx_identity(frame, &orderedIdentity.epoch, &orderedIdentity.request_id))
  { orderedIdentity.origin = DEBUG_UI_ORIGIN_LOCAL_UI; }
  (void)aethor_app_debug_ui_receive_preflight_frame(frame, arrivalLowerUs, observedUs, &orderedIdentity);
}
#endif
'@
        $expectedFullCoreBridge = @'
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
static void CollectSingleStaticReceipt(void)
{
  DebugUiStaticTxIdentity identity;
  uint8_t succeeded;
  uint16_t sof;
  if (stm32_platform_can_collect_static(&identity, &succeeded, &sof))
  { aethor_app_static_transmitted(&identity, succeeded, sof, AethorMonotonicTimestampUs()); }
}
static void ReceiveSingleStaticFrame(const CanFrame *frame, uint64_t arrivalLowerUs, uint64_t observedUs)
{
  DebugUiStaticTxIdentity orderedIdentity;
  uint16_t sof = 0U;
  uint8_t ordered, sofValid = 0U;
  CollectSingleStaticReceipt();
  ordered = stm32_platform_can_static_rx_identity(frame, &orderedIdentity, &sof, &sofValid);
  (void)aethor_app_receive_static_frame(frame, arrivalLowerUs, observedUs, ordered ? &orderedIdentity : NULL, sof, sofValid);
}
#endif
'@
        $expectedFacadeBridge = @'
MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame(const CanFrame *frame,
    uint64_t arrival_lower_us, uint64_t observed_us, const DebugUiIdentity *ordered_identity)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    uint8_t joint_index, selected_index = 0U;
    uint32_t generation_before;
    MotorRuntimeStatus status;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        if (application_action.command.motor_mask & (1U << joint_index)) selected_index = joint_index;
    generation_before = application_motor_runtime.bank.motors[selected_index].feedback_generation;
    status = aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us);
    if (status == MOTOR_RUNTIME_STATUS_OK && ordered_identity &&
        ordered_identity->origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
        application_action.command.session_id == ordered_identity->epoch &&
        application_action.command.request_id == ordered_identity->request_id &&
        application_action.single_static_tx_submitted && !application_action.single_static_tx_failed &&
        application_motor_runtime.bank.motors[selected_index].feedback_generation != generation_before)
        application_action.single_static_ordered_generation = application_motor_runtime.bank.motors[selected_index].feedback_generation;
    return status;
#else
    (void)ordered_identity;
    return aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us);
#endif
}
'@
        $normalizedCore = [regex]::Replace($coreCode, '\s+', '')
        $expectedBridge = if ($criticalCode -ceq $fullStatic) { $expectedFullCoreBridge } else { $expectedCoreBridge }
        # The complete receipt function permits exactly one fresh clock at the actual TX callback, never at the RX lower bound.
        if (-not $normalizedCore.Contains([regex]::Replace($expectedBridge, '\s+', ''))) { return $false }
        $expectedFeatureGuard = if ($criticalCode -ceq $fullStatic) {
            'if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)'
        } else { 'if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD' }
        foreach ($functionName in @('CollectSingleStaticReceipt', 'ReceiveSingleStaticFrame')) {
            $coreDefinition = Get-CanRxContractDefinition -Code $coreCode -FunctionName $functionName -ReturnType 'static void' -ExpectedGuard $expectedFeatureGuard
            if ($null -eq $coreDefinition) { return $false }
        }
        $facadeDefinition = Get-CanRxContractDefinition -Code $facadeCode -FunctionName 'aethor_app_debug_ui_receive_preflight_frame' -ReturnType 'MotorRuntimeStatus'
        if ($null -eq $facadeDefinition -or [regex]::Replace($facadeDefinition.Value, '\s+', '') -cne
            [regex]::Replace($expectedFacadeBridge, '\s+', '')) { return $false }
        if ($criticalCode -ceq $fullStatic) {
            # Bind the explicit third source to the one executable, top-level include used by the real facade.
            $includePattern = '(?m)^[ \t]*#[ \t]*include[ \t]+"DebugUi/debug_ui_static_transport_app\.inc"[ \t]*\r?$'
            $staticIncludes = [regex]::Matches($facadeCode, $includePattern)
            if ($staticIncludes.Count -ne 1) { return $false }
            $includeGuardDepth = 0
            foreach ($directive in [regex]::Matches($facadeCode.Substring(0, $staticIncludes[0].Index), '(?m)^\s*#\s*(if|ifdef|ifndef|endif)\b')) {
                if ($directive.Groups[1].Value -eq 'endif') { --$includeGuardDepth } else { ++$includeGuardDepth }
                if ($includeGuardDepth -lt 0) { return $false }
            }
            if ($includeGuardDepth -ne 0) { return $false }
            $expectedStaticFacadeBridge = @'
MotorRuntimeStatus aethor_app_receive_static_frame(const CanFrame *frame, uint64_t lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *identity, uint16_t sof, uint8_t sof_valid)
{
#if !AETHOR_ADRC_BENCH
    uint32_t before[ARM_JOINT_COUNT];
    uint8_t joint_index;
    MotorRuntimeStatus status;
    DebugUiIdentity legacy = { DEBUG_UI_ORIGIN_USB, 0U, 0U };
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    const DebugUiStaticTxIdentity *trace_identity = application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ? identity : NULL;
#endif
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        before[joint_index] = application_motor_runtime.bank.motors[joint_index].feedback_generation;
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    aethor_app_static_trace_record_identity(STATIC_TRACE_RAW_RX, 0U, frame, lower_us, observed_us, sof, sof_valid, 0U, trace_identity);
#else
    aethor_app_static_trace_record(STATIC_TRACE_RAW_RX, 0U, frame, lower_us, observed_us, sof, sof_valid, 0U);
#endif
    if (identity && identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT &&
        debug_ui_static_tx_identity_equal(identity, &application_debug_ui.static_transport_identity)) {
        legacy.origin = DEBUG_UI_ORIGIN_LOCAL_UI; legacy.epoch = identity->epoch; legacy.request_id = identity->request_id;
    }
    status = aethor_app_debug_ui_receive_preflight_frame(frame, lower_us, observed_us, &legacy);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        uint32_t after = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        if (after == before[joint_index]) continue;
        if (identity && identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE &&
            identity->motor_id == joint_index + 1U &&
            debug_ui_static_tx_identity_equal(identity, &application_debug_ui.pos_disable_identity) &&
            application_debug_ui.pos_disable_submitted && application_debug_ui.pos_disable_confirmed &&
            !application_debug_ui.pos_disable_failed && after != application_debug_ui.pos_disable_rx_generation)
            application_debug_ui.pos_disable_ordered_generation = after;
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        aethor_app_static_trace_record_identity(STATIC_TRACE_ACCEPTED_RX, (uint8_t)(joint_index + 1U), frame,
            lower_us, observed_us, sof, sof_valid, 0U, trace_identity);
#else
        aethor_app_static_trace_record(STATIC_TRACE_ACCEPTED_RX, (uint8_t)(joint_index + 1U), frame,
            lower_us, observed_us, sof, sof_valid, 0U);
#endif
        if (identity && identity->motor_id == joint_index + 1U &&
            debug_ui_static_tx_identity_equal(identity, &application_action.static_identity) &&
            application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
            application_action.single_static_tx_confirmed && !application_action.single_static_tx_failed &&
            after != application_action.single_static_rx_generation)
            application_action.single_static_ordered_generation = after;
    }
    return status;
#else
    (void)identity; (void)sof; (void)sof_valid;
    return aethor_app_receive_can_frame_observed(frame, lower_us, observed_us);
#endif
}
'@
            $staticDefinition = Get-CanRxContractDefinition -Code $staticTransportCode -FunctionName 'aethor_app_receive_static_frame' -ReturnType 'MotorRuntimeStatus'
            if ($null -eq $staticDefinition -or $facadeCode -match '(?m)^[ \t]*MotorRuntimeStatus[ \t]+aethor_app_receive_static_frame[ \t]*\(' -or
                [regex]::Replace($staticDefinition.Value, '\s+', '') -cne
                [regex]::Replace($expectedStaticFacadeBridge, '\s+', '')) { return $false }
        }
    }
    return $true
}

function Test-CanRxArrivalContract {
    <# Requires one queued arrival publication and immutable forwarding through every enabled decode path; all three actual sources are explicit inputs. #>
    param([Parameter(Mandatory = $true)][string]$Text,
        [Parameter(Mandatory = $true)][string]$FacadeText,
        [Parameter(Mandatory = $true)][string]$StaticTransportText)
    $commentOrLiteral = '(?s)"(?:\\.|[^"\\])*"|''(?:\\.|[^''\\])*''|/\*.*?\*/|//[^\r\n]*'
    $retainLiteral = { param($token) if ($token.Value.StartsWith('/*') -or $token.Value.StartsWith('//')) { ' ' } else { $token.Value } }
    $coreCode = [regex]::Replace([regex]::Replace($Text, '\\(?:\r\n|\n|\r)', ''), $commentOrLiteral, $retainLiteral)
    $receiveFunction = [regex]::Match($coreCode,
        '(?ms)^void StartCanRxTask\([^)]*\)\s*\{.*?^\}')
    if (-not $receiveFunction.Success) { return $false }
    $body = $receiveFunction.Value
    $popPattern = 'stm32_platform_can_pop_received_at\s*\(\s*&receivedFrame\s*,\s*&arrivalLowerBoundUs\s*\)'
    $queuedArrivalInitialized = $body -match 'uint64_t arrivalLowerBoundUs\s*=\s*0ULL\s*;'
    $queuedArrivalAssignments = [regex]::Matches($body, '\barrivalLowerBoundUs\s*=(?!=)').Count
    return ([regex]::Matches($body, $popPattern).Count -eq 1) -and $queuedArrivalInitialized -and
        ($queuedArrivalAssignments -eq 1) -and
        (Test-CanRxCriticalBoundary -Text $Text -FacadeText $FacadeText -StaticTransportText $StaticTransportText)
}

function Invoke-Phase0ArchitectureCheck {
    <# Runs all Phase 0 architecture checks and returns a process-friendly result. #>
    $projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
    $failureList = [System.Collections.Generic.List[string]]::new()
    $applicationRoot = Join-Path $projectRoot 'App'
    $applicationFiles = Get-ChildItem -LiteralPath $applicationRoot -Recurse -File -Include '*.c', '*.h', '*.inc'
    $applicationPathPrefixLength = $applicationRoot.TrimEnd('\').Length + 1
    $allocationPattern = '\b(malloc|calloc|realloc|free|pvPortMalloc|vPortFree)\s*\('
    $platformIncludePattern = '^\s*#\s*include\s*[<"](?:stm32h7xx|cmsis_os|FreeRTOS|fdcan|usb_device|usbd_|usart)'

    foreach ($applicationFile in $applicationFiles)
    {
        $applicationText = Get-Content -LiteralPath $applicationFile.FullName -Raw
        if ($applicationText -match $allocationPattern)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "Dynamic allocation call found in $($applicationFile.FullName)."
        }

        $relativeApplicationPath = $applicationFile.FullName.Substring($applicationPathPrefixLength)
        if (-not (Test-MotorCommandBoundary -Text $applicationText -RelativePath $relativeApplicationPath))
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
    $aethorAppText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\aethor_app.c') -Raw
    $staticTransportText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\DebugUi\debug_ui_static_transport_app.inc') -Raw
    Assert-TextContains -FailureList $failureList -Text $mainText `
        -Pattern 'aethor_app_init\s*\(' `
        -Message 'main.c does not initialize the Phase 0 application facade.'
    foreach ($requiredStaticTask in $requiredStaticTasks)
    {
        Assert-TextContains -FailureList $failureList -Text $freertosText `
            -Pattern "osThreadStaticDef\s*\(\s*$requiredStaticTask\s*," `
            -Message "freertos.c does not create $requiredStaticTask statically."
    }
    # The optional UI is hand-maintained in the CubeMX USER section. Check the
    # exact feature guard before excluding that one task from the six-task base.
    $optionalUiTaskBlock = [regex]::Match($freertosText,
        '(?s)#if AETHOR_DEBUG_UI_ENABLE\s+osThreadStaticDef\(DebugUiTask,.*?#endif')
    $baseTaskText = $freertosText
    if ($optionalUiTaskBlock.Success)
    {
        $baseTaskText = $baseTaskText.Replace($optionalUiTaskBlock.Value, '')
        if (([regex]::Matches($optionalUiTaskBlock.Value, 'osThreadStaticDef\s*\(')).Count -ne 1)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'Optional UI guard contains more than one task.'
        }
    }
    if (([regex]::Matches($baseTaskText, 'osThreadStaticDef\s*\(')).Count -ne $requiredStaticTasks.Count)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'freertos.c must retain six base tasks and only one feature-gated UI task.'
    }
    # Stack expressions share one private compile-time definition. Inspect code
    # without comments so an obsolete declaration in a comment cannot satisfy it.
    $freertosCode = [regex]::Replace($freertosText, '(?s)/\*.*?\*/|//[^\r\n]*', '')
    $taskStackConfiguration = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\aethor_task_stacks.h') -Raw
    $taskStackCode = [regex]::Replace($taskStackConfiguration, '(?s)/\*.*?\*/|//[^\r\n]*', '')
    Assert-TextContains -FailureList $failureList -Text $freertosCode `
        -Pattern '(?m)^#include "aethor_task_stacks\.h"' `
        -Message 'freertos.c must use the private task stack configuration.'
    Assert-TextContains -FailureList $failureList -Text $taskStackCode `
        -Pattern '(?m)^#include "debug_ui_config\.h"' `
        -Message 'Task stack selection must use the real debug UI compile gate.'
    Assert-TextContains -FailureList $failureList -Text $taskStackCode `
        -Pattern '#if AETHOR_DEBUG_UI_ALLOW_MOTION && \(AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD \|\| AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD \|\| AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST\)\s+#define AETHOR_CAN_RX_TASK_STACK_WORDS \(640U\)\s+#else\s+#define AETHOR_CAN_RX_TASK_STACK_WORDS \(384U\)\s+#endif' `
        -Message 'Only the motion-enabled static-load receive path may increase CAN RX to 640 words; other builds must retain 384 words.'
    Assert-TextContains -FailureList $failureList -Text $taskStackCode `
        -Pattern '#if AETHOR_ADRC_LCD_INTEGRATED \|\| AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD \|\| AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD\s+#define AETHOR_PROTOCOL_TASK_STACK_WORDS \(2048U\)\s+#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS \(512U\)\s+#else\s+#define AETHOR_PROTOCOL_TASK_STACK_WORDS \(1280U\)\s+#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS \(384U\)\s+#endif' `
        -Message 'Only integrated LCD or static-load builds may increase Protocol/Diagnostics stacks; other builds must retain 1280/384 words.'
    foreach ($stackMacro in @('AETHOR_CAN_RX_TASK_STACK_WORDS', 'AETHOR_PROTOCOL_TASK_STACK_WORDS', 'AETHOR_DIAGNOSTICS_TASK_STACK_WORDS'))
    {
        if (([regex]::Matches($taskStackCode, '(?m)^#define\s+' + $stackMacro + '\b')).Count -ne 2)
        {
            Add-ArchitectureFailure -FailureList $failureList -Message "$stackMacro must have exactly the two guarded definitions."
        }
    }
    $requiredTaskStackWords = [ordered]@{
        ArmControlTask = 1280
        CanRxTask = 384
        ProtocolTask = 1280
        UsbTxTask = 384
        TelemetryTask = 1024
        DiagnosticsTask = 384
    }
    $taskStackMacros = @{
        CanRxTask = 'AETHOR_CAN_RX_TASK_STACK_WORDS'
        ProtocolTask = 'AETHOR_PROTOCOL_TASK_STACK_WORDS'
        DiagnosticsTask = 'AETHOR_DIAGNOSTICS_TASK_STACK_WORDS'
    }
    $taskPriorities = @{
        ArmControlTask = 'osPriorityRealtime'; CanRxTask = 'osPriorityHigh'
        ProtocolTask = 'osPriorityAboveNormal'; UsbTxTask = 'osPriorityNormal'
        TelemetryTask = 'osPriorityBelowNormal'; DiagnosticsTask = 'osPriorityLow'
    }
    foreach ($taskStackRequirement in $requiredTaskStackWords.GetEnumerator())
    {
        $taskName = $taskStackRequirement.Key
        $stackWords = $taskStackRequirement.Value
        $stackArgument = [string]$stackWords
        if ($taskStackMacros.ContainsKey($taskName)) { $stackArgument = $taskStackMacros[$taskName] }
        $taskBaseName = $taskName.Substring(0, 1).ToLowerInvariant() + $taskName.Substring(1)
        $taskBufferName = $taskBaseName + 'Buffer'
        $taskControlName = $taskBaseName + 'ControlBlock'
        $taskPriority = $taskPriorities[$taskName]
        Assert-TextContains -FailureList $failureList -Text $freertosCode `
            -Pattern "(?m)^uint32_t\s+$taskBufferName\s*\[\s*$stackArgument\s*\]\s*;" `
            -Message "$taskName buffer must use its verified stack capacity."
        Assert-TextContains -FailureList $failureList -Text $freertosCode `
            -Pattern "osThreadStaticDef\s*\(\s*$taskName\s*,\s*Start$taskName\s*,\s*$taskPriority\s*,\s*0\s*,\s*$stackArgument\s*,\s*$taskBufferName\s*,\s*&$taskControlName\s*\)" `
            -Message "$taskName creation must match its buffer capacity, entry and priority."
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
        -Pattern 'stm32_platform_can_pop_received_at\s*\(' `
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
    # only the exact bounded receipt/SOF-metadata and decode allowlist inside the
    # shared task critical section; unlisted timestamp/I/O/blocking work fails.
    $canRxTaskMatch = [regex]::Match(
        $freertosText,
        'void\s+StartCanRxTask\s*\([^)]*\)\s*\{[\s\S]*?(?=/\* USER CODE BEGIN Header_StartProtocolTask \*/)')
    if (-not $canRxTaskMatch.Success)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'CanRxTask body could not be isolated for concurrency checks.'
    }
    else
    {
        if (-not (Test-CanRxArrivalContract -Text $freertosText -FacadeText $aethorAppText -StaticTransportText $staticTransportText)) {
            Add-ArchitectureFailure -FailureList $failureList `
                -Message 'CanRxTask must serialize decoding with the queued arrival lower bound, not dequeue time.'
        }
        if (-not (Test-CanRxCriticalBoundary -Text $freertosText -FacadeText $aethorAppText -StaticTransportText $staticTransportText))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'CanRxTask critical section differs from the bounded queued-arrival/observed decode contract.'
        }
    }
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'stm32_platform_usb_next_line\s*\(' `
        -Message 'ProtocolTask does not drain complete USB lines in task context.'
    Assert-TextContains -FailureList $failureList -Text $freertosText `
        -Pattern 'aethor_app_process_protocol_line\s*\(' `
        -Message 'ProtocolTask does not dispatch parsed requests to the protocol engine.'
    # ProtocolTask is the sole ProtocolEngine formatter. TelemetryTask only
    # supplies a 10 ms wakeup so USB commands are applied before stream output.
    $protocolTaskMatch = [regex]::Match(
        $freertosText,
        'void\s+StartProtocolTask\s*\([^)]*\)\s*\{[\s\S]*?(?=/\* USER CODE BEGIN Header_StartUsbTxTask \*/)')
    $telemetryTaskMatch = [regex]::Match(
        $freertosText,
        'void\s+StartTelemetryTask\s*\([^)]*\)\s*\{[\s\S]*?(?=/\* USER CODE BEGIN Header_StartDiagnosticsTask \*/)')
    if (-not $protocolTaskMatch.Success)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'ProtocolTask body could not be isolated for owner checks.'
    }
    else
    {
        Assert-TextContains -FailureList $failureList -Text $protocolTaskMatch.Value `
            -Pattern 'aethor_app_pop_protocol_result_output[\s\S]*stm32_platform_usb_next_line[\s\S]*while\s*\(\s*\(+lineStatus[\s\S]*AethorMonotonicTimestampUs\s*\(\s*\)[\s\S]{0,240}aethor_app_generate_stream_output[\s\S]{0,240}QueueProtocolOutputBatch' `
            -Message 'ProtocolTask does not drain results and USB lines before telemetry formatting.'
    }
    if (-not $telemetryTaskMatch.Success)
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'TelemetryTask body could not be isolated for owner checks.'
    }
    else
    {
        Assert-TextContains -FailureList $failureList -Text $telemetryTaskMatch.Value `
            -Pattern 'xTaskNotifyGive\s*\(\s*\(TaskHandle_t\)ProtocolTaskHandle\s*\)' `
            -Message 'TelemetryTask does not wake the sole ProtocolTask owner.'
        if ($telemetryTaskMatch.Value -match 'aethor_app_generate_stream_output|QueueProtocolOutputBatch|AethorMonotonicTimestampUs')
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'TelemetryTask still reads or formats ProtocolEngine state directly.'
        }
    }
    if ($protocolTaskMatch.Success)
    {
        $allQueueOwnerReferences = [regex]::Matches(
            $freertosText,
            'QueueProtocolOutputBatch\s*\(').Count
        $protocolQueueReferences = [regex]::Matches(
            $protocolTaskMatch.Value,
            'QueueProtocolOutputBatch\s*\(').Count
        if ($allQueueOwnerReferences -ne ($protocolQueueReferences + 2))
        {
            Add-ArchitectureFailure -FailureList $failureList -Message 'A task other than ProtocolTask queues formatted protocol output.'
        }
    }
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
    $protocolEngineText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\Protocol\protocol_engine.c') -Raw
    $motorRuntimeHeaderText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\Motor\motor_runtime.h') -Raw
    $motorRuntimeText = Get-Content -LiteralPath (Join-Path $projectRoot 'App\Motor\motor_runtime.c') -Raw
    if ($motorRuntimeHeaderText -match 'motor_runtime_remove_parameter_response')
    {
        Add-ArchitectureFailure -FailureList $failureList -Message 'Internal parameter-set mutation escaped through the public MotorRuntime header.'
    }
    Assert-TextContains -FailureList $failureList -Text $motorRuntimeText `
        -Pattern 'static\s+uint8_t\s+motor_runtime_remove_parameter_response[\s\S]{0,320}entry_index\s*>=\s*set->count[\s\S]{0,120}return\s+0U' `
        -Message 'Internal parameter removal is not static and defensively bounded.'
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
    if (-not (Test-StopCommandPublicationOrder -Text $protocolEngineText)) {
        Add-ArchitectureFailure -FailureList $failureList `
            -Message 'STOP payload/origin/epoch ownership must precede priority-slot publication.'
    }
    # A normal motion slot is visible only after its admission metadata. This
    # prevents ArmControlTask from consuming a MOVE before its gate is owned.
    if (-not (Test-NormalCommandPublicationOrder -Text $protocolEngineText)) {
        Add-ArchitectureFailure -FailureList $failureList `
            -Message 'Normal MOVE ownership metadata is not published before command_write_sequence.'
    }

    $keilProjectPath = Join-Path $projectRoot 'MDK-ARM\CtrBoard-H7_FDCAN.uvprojx'
    $keilProjectXml = [xml](Get-Content -LiteralPath $keilProjectPath -Raw)
    $baseKeilTargets = @($keilProjectXml.Project.Targets.Target |
        Where-Object { $_.TargetName -eq 'CtrBoard-H7_FDCAN' })
    if ($baseKeilTargets.Count -ne 1)
    {
        throw 'The original CtrBoard-H7_FDCAN Keil target must remain unique.'
    }
    $keilFileNames = @($baseKeilTargets[0].SelectNodes('.//FileName') | ForEach-Object { $_.'#text' })
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
    Write-Host '[PASS] Motor codecs remain in App/Motor except the fixed guarded ADRC maintenance calls.'
    Write-Host '[PASS] Six base tasks, optional guarded UI task, and USER_KEY remain valid.'
    Write-Host '[PASS] main.c and freertos.c use the Phase 0 application entry.'
    Write-Host '[PASS] One-shot cleanup HOLD requires completed ENABLE ownership.'
    Write-Host '[PASS] Keil compiles one copy of each required source and no legacy controller.'
    Write-Host '[PASS] Phase 0 architecture contracts are satisfied.'
}

Invoke-Phase0ArchitectureCheck
