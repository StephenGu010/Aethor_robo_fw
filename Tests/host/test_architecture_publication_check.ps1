<#
.SYNOPSIS
Verifies publication and exact queued-arrival/critical-section contracts against three explicit real sources and non-noop mutations.
#>
$ErrorActionPreference = 'Stop'

function New-CanRxMutationCase {
    <# Mutates exactly one named real function and carries all three complete sources explicitly; missing anchors fail the test itself. #>
    param([string]$Name, [string]$CoreText, [string]$FacadeText,
        [Parameter(Mandatory = $true)][string]$StaticTransportText,
        [ValidateSet('Task','Bridge','Receipt','Facade','StaticFacade')][string]$Target,
        [string]$Old, [AllowEmptyString()][string]$Replacement)
    $patterns = @{
        Task = '(?ms)^void StartCanRxTask\([^)]*\)\s*\{.*?^\}'
        Bridge = '(?ms)^static void ReceiveSingleStaticFrame\([^)]*\)\s*\{.*?^\}'
        Receipt = '(?ms)^static void CollectSingleStaticReceipt\([^)]*\)\s*\{.*?^\}'
        Facade = '(?ms)^MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame\([^)]*\)\s*\{.*?^\}'
        StaticFacade = '(?ms)^MotorRuntimeStatus aethor_app_receive_static_frame\([^)]*\)\s*\{.*?^\}'
    }
    $source = if ($Target -eq 'Facade') { $FacadeText } elseif ($Target -eq 'StaticFacade') { $StaticTransportText } else { $CoreText }
    $functions = [regex]::Matches($source, $patterns[$Target])
    if ($functions.Count -ne 1 -or -not $functions[0].Value.Contains($Old)) { throw ('CAN mutation anchor missing/ambiguous: ' + $Name) }
    $changed = $functions[0].Value.Replace($Old, $Replacement)
    if ($changed -ceq $functions[0].Value) { throw ('CAN mutation is a no-op: ' + $Name) }
    $changedSource = $source.Remove($functions[0].Index, $functions[0].Length).Insert($functions[0].Index, $changed)
    $case = @{ Name = $Name; Text = $CoreText; FacadeText = $FacadeText; StaticTransportText = $StaticTransportText; Expected = $false;
        BaseText = $CoreText; BaseFacadeText = $FacadeText; BaseStaticTransportText = $StaticTransportText }
    if ($Target -eq 'Facade') { $case.FacadeText = $changedSource }
    elseif ($Target -eq 'StaticFacade') { $case.StaticTransportText = $changedSource }
    else { $case.Text = $changedSource }
    return $case
}

function Invoke-PublicationCheckerRegression {
    <# Loads only the checker helper and verifies growth and ordering mutations. #>
    $checkerPath = Join-Path $PSScriptRoot 'check_phase0_architecture.ps1'
    $parseTokens = $null
    $parseErrors = $null
    $checkerAst = [System.Management.Automation.Language.Parser]::ParseFile(
        $checkerPath, [ref]$parseTokens, [ref]$parseErrors)
    $helper = $checkerAst.Find({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Test-NormalCommandPublicationOrder'
    }, $true)
    if ($null -eq $helper) { throw 'Publication checker helper is missing.' }
    . ([scriptblock]::Create($helper.Extent.Text))
    $sourcePath = Join-Path $PSScriptRoot '..\..\App\Protocol\protocol_engine.c'
    $sourceText = Get-Content -LiteralPath $sourcePath -Raw
    $functionMatch = [regex]::Match($sourceText,
        '(?ms)^static uint8_t protocol_engine_publish_normal_command\([^)]*\)\s*\{.*?^\}')
    if (-not $functionMatch.Success) { throw 'Publication function was not found.' }
    $functionText = $functionMatch.Value
    $cases = @(
        @{ Name = 'current source'; Text = $sourceText; Expected = $true },
        @{ Name = 'long admission logic'; Text = $functionText.Replace('uint8_t used_count', ('/*' + ('x' * 3000) + '*/' + "`n    uint8_t used_count")); Expected = $true },
        @{ Name = 'early publication'; Text = $functionText.Replace('++engine->command_write_sequence;', '').Replace('engine->commands[slot_index] = *command;', '++engine->command_write_sequence; engine->commands[slot_index] = *command;'); Expected = $false },
        @{ Name = 'missing barrier'; Text = $functionText.Replace('protocol_engine_compiler_barrier();', ''); Expected = $false },
        @{ Name = 'missing origin'; Text = $functionText.Replace('engine->active_motion_origin = command->origin;', ''); Expected = $false },
        @{ Name = 'missing epoch'; Text = $functionText.Replace('engine->active_motion_epoch = command->session_id;', ''); Expected = $false },
        @{ Name = 'duplicate sequence publication'; Text = $functionText.Replace('++engine->command_write_sequence;', '++engine->command_write_sequence; ++engine->command_write_sequence;'); Expected = $false },
        @{ Name = 'metadata in different function'; Text = ($functionText.Replace('engine->active_motion_request_id = command->request_id;', '') + "`n" + $functionText.Replace('protocol_engine_publish_normal_command', 'unrelated_function')); Expected = $false }
    )
    foreach ($testCase in $cases) {
        if ((Test-NormalCommandPublicationOrder -Text $testCase.Text) -ne $testCase.Expected) {
            throw ('Publication regression failed: ' + $testCase.Name)
        }
    }
    $stopHelper = $checkerAst.Find({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Test-StopCommandPublicationOrder'
    }, $true)
    if ($null -eq $stopHelper) { throw 'STOP publication helper is missing.' }
    . ([scriptblock]::Create($stopHelper.Extent.Text))
    $stopCases = @(
        @{ Name = 'current STOP'; Text = $sourceText; Expected = $true },
        @{ Name = 'missing STOP origin'; Text = $sourceText.Replace('engine->active_stop_origin = command->origin;', ''); Expected = $false },
        @{ Name = 'missing STOP epoch'; Text = $sourceText.Replace('engine->active_stop_epoch = command->session_id;', ''); Expected = $false },
        @{ Name = 'early STOP'; Text = $sourceText.Replace('++engine->stop_write_sequence;', '').Replace('engine->stop_command = *command;', '++engine->stop_write_sequence; engine->stop_command = *command;'); Expected = $false },
        @{ Name = 'duplicate STOP'; Text = $sourceText.Replace('++engine->stop_write_sequence;', '++engine->stop_write_sequence; ++engine->stop_write_sequence;'); Expected = $false }
    )
    foreach ($testCase in $stopCases) {
        if ((Test-StopCommandPublicationOrder -Text $testCase.Text) -ne $testCase.Expected) {
            throw ('STOP publication regression failed: ' + $testCase.Name)
        }
    }
    $arrivalHelper = $checkerAst.Find({
        param($node)
        $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and
        $node.Name -eq 'Test-CanRxArrivalContract'
    }, $true)
    if ($null -eq $arrivalHelper) { throw 'CAN arrival checker helper is missing.' }
    foreach ($helperName in @('Get-CanRxContractDefinition', 'Test-CanRxCriticalBoundary', 'Test-MotorCommandBoundary')) {
        $boundaryHelper = $checkerAst.Find({
            param($node)
            $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq $helperName
        }, $true)
        if ($null -eq $boundaryHelper) { throw ('Boundary checker helper is missing: ' + $helperName) }
        . ([scriptblock]::Create($boundaryHelper.Extent.Text))
    }
    . ([scriptblock]::Create($arrivalHelper.Extent.Text))
    $taskText = Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\..\Core\Src\freertos.c') -Raw
    $facadeText = Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\..\App\aethor_app.c') -Raw
    $staticTransportText = Get-Content -LiteralPath (Join-Path $PSScriptRoot '..\..\App\DebugUi\debug_ui_static_transport_app.inc') -Raw
    $critical = [regex]::Match(([regex]::Match($taskText, '(?ms)^void StartCanRxTask\([^)]*\)\s*\{.*?^\}')).Value,
        'taskENTER_CRITICAL\(\);(?<Body>[\s\S]*?)taskEXIT_CRITICAL\(\);')
    if (-not $critical.Success) { throw 'Real receive critical section is missing.' }
    $legacyCall = '(void)aethor_app_receive_can_frame(&receivedFrame, arrivalLowerBoundUs);'
    $observedBlock = @'
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
      {
        uint64_t receivedObservedUs = AethorMonotonicTimestampUs();
        CollectCapabilityReceipt(receivedObservedUs);
        (void)aethor_app_receive_can_frame_observed(&receivedFrame, arrivalLowerBoundUs, receivedObservedUs);
      }
#else
      (void)aethor_app_receive_can_frame(&receivedFrame, arrivalLowerBoundUs);
#endif
'@
    $legacyText = $taskText.Replace($critical.Groups['Body'].Value, "`n" + $legacyCall + "`n")
    $observedText = $taskText.Replace($critical.Groups['Body'].Value, "`n" + $observedBlock + "`n")
    if ($legacyText -ceq $taskText -or $observedText -ceq $taskText) { throw 'Legacy/observed positive fixtures did not replace the real critical block.' }
    $staticGuard = '#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)'
    $singleGuard = '#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD'
    $coreBridge = [regex]::Match($taskText, '(?ms)^' + [regex]::Escape($staticGuard) + '\s*.*?^static void CollectSingleStaticReceipt\([^)]*\)\s*\{.*?^\}.*?^static void ReceiveSingleStaticFrame\([^)]*\)\s*\{.*?^\}[^#]*^#endif')
    if (-not $coreBridge.Success) { throw 'Actual full-identity Core bridge is missing.' }
    $singleCoreBridge = @'
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
    $singleText = $taskText.Replace($coreBridge.Value, $singleCoreBridge).Replace($staticGuard, $singleGuard)
    if ($singleText -ceq $taskText) { throw 'Single-preflight positive fixture did not replace the full-identity bridge.' }
    $arrivalCases = [System.Collections.Generic.List[object]]::new()
    foreach ($positive in @(
        @{ Name = 'queued arrival current bridge'; Text = $taskText },
        @{ Name = 'queued arrival legacy'; Text = $legacyText },
        @{ Name = 'queued arrival observed'; Text = $observedText },
        @{ Name = 'queued arrival single preflight'; Text = $singleText },
        @{ Name = 'comments cannot alter ownership counts'; Text = $taskText.Replace('CollectSingleStaticReceipt();', '/* arrivalLowerBoundUs = 1; taskEXIT_CRITICAL(); */ CollectSingleStaticReceipt();') })) {
        $positive.FacadeText = $facadeText; $positive.StaticTransportText = $staticTransportText
        $positive.Expected = $true; $arrivalCases.Add($positive)
    }
    # Preserve every original arrival mutation on both the old observed shape and the real new branch.
    $originalMutations = @(
        @('legacy pop','stm32_platform_can_pop_received_at','stm32_platform_can_pop_received'),
        @('wrong decode time','&receivedFrame, arrivalLowerBoundUs','&receivedFrame, timestampUs'),
        @('observed path uses dequeue time','&receivedFrame, arrivalLowerBoundUs, receivedObservedUs','&receivedFrame, receivedObservedUs, receivedObservedUs'),
        @('legacy path uses dequeue time','&receivedFrame, arrivalLowerBoundUs);','&receivedFrame, receivedObservedUs);'),
        @('missing observed timestamp','uint64_t receivedObservedUs = AethorMonotonicTimestampUs();',''),
        @('overwrite queued arrival','CollectCapabilityReceipt(receivedObservedUs);','arrivalLowerBoundUs = receivedObservedUs; CollectCapabilityReceipt(receivedObservedUs);'),
        @('dequeue timestamp','uint64_t arrivalLowerBoundUs = 0ULL;','uint64_t arrivalLowerBoundUs = AethorMonotonicTimestampUs();'),
        @('missing receive lock','taskENTER_CRITICAL();',''), @('missing receive unlock','taskEXIT_CRITICAL();',''),
        @('blocking receive work','CollectCapabilityReceipt(receivedObservedUs);','CollectCapabilityReceipt(receivedObservedUs); vTaskDelay(1U);'),
        @('HAL receive work','CollectCapabilityReceipt(receivedObservedUs);','CollectCapabilityReceipt(receivedObservedUs); HAL_Delay(1U);'),
        @('platform receive work','CollectCapabilityReceipt(receivedObservedUs);','CollectCapabilityReceipt(receivedObservedUs); stm32_platform_can_poll();'),
        @('second timestamp','CollectCapabilityReceipt(receivedObservedUs);','CollectCapabilityReceipt(receivedObservedUs); AethorMonotonicTimestampUs();'),
        @('weakened observed feature guard','#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED','#if 1'),
        @('missing capability receipt','CollectCapabilityReceipt(receivedObservedUs);',''),
        @('extra unguarded decode','++processedFrameCount;','(void)aethor_app_receive_can_frame(&receivedFrame, receivedObservedUs); ++processedFrameCount;')
    )
    foreach ($variant in @(@{ Name = 'current'; Text = $taskText }, @{ Name = 'observed'; Text = $observedText }, @{ Name = 'single'; Text = $singleText })) {
        foreach ($mutation in $originalMutations) {
            $arrivalCases.Add((New-CanRxMutationCase -Name ($variant.Name + ' ' + $mutation[0]) -CoreText $variant.Text -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Task -Old $mutation[1] -Replacement $mutation[2]))
        }
    }
    foreach ($mutation in @(
        @('legacy pop','stm32_platform_can_pop_received_at','stm32_platform_can_pop_received'),
        @('wrong decode time','&receivedFrame, arrivalLowerBoundUs','&receivedFrame, timestampUs'),
        @('dequeue timestamp','uint64_t arrivalLowerBoundUs = 0ULL;','uint64_t arrivalLowerBoundUs = AethorMonotonicTimestampUs();'),
        @('missing lock','taskENTER_CRITICAL();',''), @('missing unlock','taskEXIT_CRITICAL();',''),
        @('blocking work',$legacyCall,($legacyCall + ' vTaskDelay(1U);')),
        @('HAL work',$legacyCall,($legacyCall + ' HAL_Delay(1U);')),
        @('platform work',$legacyCall,($legacyCall + ' stm32_platform_can_poll();')),
        @('second timestamp',$legacyCall,($legacyCall + ' AethorMonotonicTimestampUs();')),
        @('overwrite lower',$legacyCall,('arrivalLowerBoundUs = 123U; ' + $legacyCall)),
        @('commented decode',$legacyCall,('/* ' + $legacyCall + ' */')))) {
        $arrivalCases.Add((New-CanRxMutationCase -Name ('legacy ' + $mutation[0]) -CoreText $legacyText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Task -Old $mutation[1] -Replacement $mutation[2]))
    }
    foreach ($mutation in @(
        @('lower replaced by observation','frame, arrivalLowerUs, observedUs','frame, observedUs, observedUs'),
        @('observation replaced by lower','frame, arrivalLowerUs, observedUs','frame, arrivalLowerUs, arrivalLowerUs'),
        @('overwrite lower','CollectSingleStaticReceipt();','arrivalLowerUs = observedUs; CollectSingleStaticReceipt();'),
        @('missing receipt','CollectSingleStaticReceipt();',''),
        @('receipt after decode','CollectSingleStaticReceipt();',''),
        @('missing SOF identity','stm32_platform_can_single_static_rx_identity','unrelated_identity'),
        @('blocking work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); vTaskDelay(1U);'),
        @('HAL work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); HAL_Delay(1U);'),
        @('platform work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); stm32_platform_can_poll();'),
        @('new clock','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); AethorMonotonicTimestampUs();'),
        @('commented receipt','CollectSingleStaticReceipt();','/* CollectSingleStaticReceipt(); */'),
        @('commented decode','(void)aethor_app_debug_ui_receive_preflight_frame(frame, arrivalLowerUs, observedUs, &orderedIdentity);','/* (void)aethor_app_debug_ui_receive_preflight_frame(frame, arrivalLowerUs, observedUs, &orderedIdentity); */'))) {
        $case = New-CanRxMutationCase -Name ('single bridge ' + $mutation[0]) -CoreText $singleText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Bridge -Old $mutation[1] -Replacement $mutation[2]
        if ($mutation[0] -eq 'receipt after decode') {
            $case.Text = $case.Text.Replace('(void)aethor_app_debug_ui_receive_preflight_frame(frame, arrivalLowerUs, observedUs, &orderedIdentity);', '(void)aethor_app_debug_ui_receive_preflight_frame(frame, arrivalLowerUs, observedUs, &orderedIdentity); CollectSingleStaticReceipt();')
        }
        $arrivalCases.Add($case)
    }
    foreach ($mutation in @(
        @('lower replaced by observation','frame, arrivalLowerUs, observedUs','frame, observedUs, observedUs'),
        @('observation replaced by lower','frame, arrivalLowerUs, observedUs','frame, arrivalLowerUs, arrivalLowerUs'),
        @('overwrite lower','CollectSingleStaticReceipt();','arrivalLowerUs = observedUs; CollectSingleStaticReceipt();'),
        @('missing receipt','CollectSingleStaticReceipt();',''),
        @('receipt after decode','CollectSingleStaticReceipt();',''),
        @('missing SOF identity','stm32_platform_can_static_rx_identity','unrelated_identity'),
        @('missing full token','&orderedIdentity, &sof, &sofValid','NULL, &sof, &sofValid'),
        @('wrong SOF validity','&orderedIdentity, &sof, &sofValid','&orderedIdentity, &sof, &ordered'),
        @('missing ordered condition','ordered ? &orderedIdentity : NULL','&orderedIdentity'),
        @('wrong SOF decode','NULL, sof, sofValid','NULL, 0U, sofValid'),
        @('wrong SOF validity decode','NULL, sof, sofValid','NULL, sof, 1U'),
        @('blocking work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); vTaskDelay(1U);'),
        @('HAL work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); HAL_Delay(1U);'),
        @('platform work','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); stm32_platform_can_poll();'),
        @('new clock','CollectSingleStaticReceipt();','CollectSingleStaticReceipt(); AethorMonotonicTimestampUs();'),
        @('commented receipt','CollectSingleStaticReceipt();','/* CollectSingleStaticReceipt(); */'),
        @('commented decode','(void)aethor_app_receive_static_frame(frame, arrivalLowerUs, observedUs, ordered ? &orderedIdentity : NULL, sof, sofValid);','/* (void)aethor_app_receive_static_frame(frame, arrivalLowerUs, observedUs, ordered ? &orderedIdentity : NULL, sof, sofValid); */'))) {
        $case = New-CanRxMutationCase -Name ('full bridge ' + $mutation[0]) -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Bridge -Old $mutation[1] -Replacement $mutation[2]
        if ($mutation[0] -eq 'receipt after decode') {
            $case.Text = $case.Text.Replace('(void)aethor_app_receive_static_frame(frame, arrivalLowerUs, observedUs, ordered ? &orderedIdentity : NULL, sof, sofValid);', '(void)aethor_app_receive_static_frame(frame, arrivalLowerUs, observedUs, ordered ? &orderedIdentity : NULL, sof, sofValid); CollectSingleStaticReceipt();')
        }
        $arrivalCases.Add($case)
    }
    foreach ($variant in @(@{ Name = 'full'; Text = $taskText; Guard = $staticGuard }, @{ Name = 'single'; Text = $singleText; Guard = $singleGuard })) {
        $arrivalCases.Add((New-CanRxMutationCase -Name ($variant.Name + ' feature gate weakened') -CoreText $variant.Text -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Task -Old $variant.Guard -Replacement '#if 1'))
        $arrivalCases.Add((New-CanRxMutationCase -Name ($variant.Name + ' extra bridge outside lock') -CoreText $variant.Text -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Task -Old '++processedFrameCount;' -Replacement 'ReceiveSingleStaticFrame(&receivedFrame, arrivalLowerBoundUs, receivedObservedUs); ++processedFrameCount;'))
    }
    $arrivalCases.Add((New-CanRxMutationCase -Name 'single receipt inserts clock' -CoreText $singleText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Receipt -Old 'uint32_t epoch, requestId;' -Replacement 'uint32_t epoch, requestId; AethorMonotonicTimestampUs();'))
    foreach ($mutation in @(
        @('inserts second clock','DebugUiStaticTxIdentity identity;','DebugUiStaticTxIdentity identity; AethorMonotonicTimestampUs();'),
        @('missing actual TX observation','AethorMonotonicTimestampUs()','0U'),
        @('wrong completion identity','&identity, succeeded, sof','NULL, succeeded, sof'),
        @('wrong completion result','&identity, succeeded, sof','&identity, 1U, sof'),
        @('wrong completion SOF','&identity, succeeded, sof','&identity, succeeded, 0U'),
        @('legacy receipt substituted','stm32_platform_can_collect_static','stm32_platform_can_collect_single_static_receipt'),
        @('blocking work','uint16_t sof;','uint16_t sof; vTaskDelay(1U);'),
        @('platform work','uint16_t sof;','uint16_t sof; stm32_platform_can_poll();'))) {
        $arrivalCases.Add((New-CanRxMutationCase -Name ('full receipt ' + $mutation[0]) -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Receipt -Old $mutation[1] -Replacement $mutation[2]))
    }
    foreach ($mutation in @(
        @('wrong lower','frame, arrival_lower_us, observed_us','frame, observed_us, observed_us'),
        @('wrong observation','frame, arrival_lower_us, observed_us','frame, arrival_lower_us, arrival_lower_us'),
        @('overwrite lower','uint32_t generation_before;','uint32_t generation_before; arrival_lower_us = observed_us;'),
        @('new clock','uint32_t generation_before;','uint32_t generation_before; HAL_GetTick();'),
        @('blocking work','uint32_t generation_before;','uint32_t generation_before; vTaskDelay(1U);'),
        @('feature gate weakened','#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD','#if 1'),
        @('commented decode','aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us)','/* aethor_app_receive_can_frame_observed(frame, arrival_lower_us, observed_us) */'))) {
        $arrivalCases.Add((New-CanRxMutationCase -Name ('facade ' + $mutation[0]) -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target Facade -Old $mutation[1] -Replacement $mutation[2]))
    }
    foreach ($mutation in @(
        @('wrong lower','frame, lower_us, observed_us','frame, observed_us, observed_us'),
        @('wrong observation','frame, lower_us, observed_us','frame, lower_us, lower_us'),
        @('overwrite lower','uint32_t before[ARM_JOINT_COUNT];','uint32_t before[ARM_JOINT_COUNT]; lower_us = observed_us;'),
        @('new clock','uint32_t before[ARM_JOINT_COUNT];','uint32_t before[ARM_JOINT_COUNT]; AethorMonotonicTimestampUs();'),
        @('blocking work','uint32_t before[ARM_JOINT_COUNT];','uint32_t before[ARM_JOINT_COUNT]; vTaskDelay(1U);'),
        @('HAL work','uint32_t before[ARM_JOINT_COUNT];','uint32_t before[ARM_JOINT_COUNT]; HAL_Delay(1U);'),
        @('platform work','uint32_t before[ARM_JOINT_COUNT];','uint32_t before[ARM_JOINT_COUNT]; stm32_platform_can_poll();'),
        @('feature gate weakened','#if !AETHOR_ADRC_BENCH','#if 1'),
        @('static gate weakened','#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)','#if 1'),
        @('missing raw trace','aethor_app_static_trace_record_identity(STATIC_TRACE_RAW_RX','unrelated_raw_trace(STATIC_TRACE_RAW_RX'),
        @('missing accepted trace','aethor_app_static_trace_record_identity(STATIC_TRACE_ACCEPTED_RX','unrelated_accepted_trace(STATIC_TRACE_ACCEPTED_RX'),
        @('lost SOF metadata','observed_us, sof, sof_valid','observed_us, 0U, 0U'),
        @('wrong preflight identity','identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT','identity->phase != DEBUG_UI_STATIC_TX_PREFLIGHT'),
        @('wrong axis identity','identity->motor_id == joint_index + 1U','identity->motor_id == joint_index'),
        @('missing phase identity','debug_ui_static_tx_identity_equal(identity, &application_action.static_identity)','1U'),
        @('missing actual TX requirement','application_action.single_static_tx_confirmed && !application_action.single_static_tx_failed','1U'),
        @('cached generation accepted','after != application_action.single_static_rx_generation','after == application_action.single_static_rx_generation'),
        @('commented decode','aethor_app_debug_ui_receive_preflight_frame(frame, lower_us, observed_us, &legacy)','/* aethor_app_debug_ui_receive_preflight_frame(frame, lower_us, observed_us, &legacy) */'))) {
        $arrivalCases.Add((New-CanRxMutationCase -Name ('static facade ' + $mutation[0]) -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target StaticFacade -Old $mutation[1] -Replacement $mutation[2]))
    }
    # A valid-looking definition inside a comment cannot replace executable bridge/facade code.
    foreach ($definition in @(
        @{ Name = 'whole Core bridge commented'; Target = 'Bridge'; Source = $taskText; Pattern = '(?ms)^static void ReceiveSingleStaticFrame\([^)]*\)\s*\{.*?^\}' },
        @{ Name = 'whole facade wrapper commented'; Target = 'Facade'; Source = $facadeText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame\([^)]*\)\s*\{.*?^\}' },
        @{ Name = 'whole static facade commented'; Target = 'StaticFacade'; Source = $staticTransportText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_receive_static_frame\([^)]*\)\s*\{.*?^\}' })) {
        $definitionText = [regex]::Match($definition.Source, $definition.Pattern).Value
        if (-not $definitionText) { throw ('Comment mutation definition missing: ' + $definition.Name) }
        $arrivalCases.Add((New-CanRxMutationCase -Name $definition.Name -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target $definition.Target -Old $definitionText -Replacement ('/*' + $definitionText + '*/')))
    }
    # A dead exact definition must never vouch for an indented executable replacement.
    foreach ($definition in @(
        @{ Name = 'static facade'; Target = 'StaticFacade'; Source = $staticTransportText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_receive_static_frame\([^)]*\)\s*\{.*?^\}';
           BadBody = 'return aethor_app_receive_can_frame_observed(frame, observed_us, observed_us);' },
        @{ Name = 'legacy facade'; Target = 'Facade'; Source = $facadeText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame\([^)]*\)\s*\{.*?^\}';
           BadBody = 'return aethor_app_receive_can_frame_observed(frame, observed_us, observed_us);' })) {
        $definitionText = [regex]::Match($definition.Source, $definition.Pattern).Value
        if (-not $definitionText) { throw ('Definition ownership mutation anchor missing: ' + $definition.Name) }
        $signature = $definitionText.Substring(0, $definitionText.IndexOf('{')).TrimEnd()
        $disabledDefinition = "#if 0`n" + $definitionText + "`n#endif"
        $replacementDefinition = '  ' + $signature + "`n  {`n    " + $definition.BadBody + "`n  }"
        foreach ($mutation in @(
            @{ Name = 'disabled definition'; Replacement = $disabledDefinition },
            @{ Name = 'disabled exact plus indented replacement'; Replacement = ($disabledDefinition + "`n" + $replacementDefinition) },
            @{ Name = 'indented duplicate replacement'; Replacement = ($definitionText + "`n" + $replacementDefinition) })) {
            $case = New-CanRxMutationCase -Name ($definition.Name + ' ' + $mutation.Name) -CoreText $taskText -FacadeText $facadeText -StaticTransportText $staticTransportText -Target $definition.Target -Old $definitionText -Replacement $mutation.Replacement
            $case.CriticalExpected = $false
            $arrivalCases.Add($case)
        }
        $indentedDefinition = '  ' + $definitionText.Replace("`n", "`n  ")
        $positive = @{ Name = ($definition.Name + ' legal leading whitespace'); Text = $taskText; FacadeText = $facadeText; StaticTransportText = $staticTransportText; Expected = $true }
        if ($definition.Target -eq 'Facade') { $positive.FacadeText = $facadeText.Replace($definitionText, $indentedDefinition) }
        else { $positive.StaticTransportText = $staticTransportText.Replace($definitionText, $indentedDefinition) }
        if ($definition.Source -ceq $definition.Source.Replace($definitionText, $indentedDefinition)) { throw 'Indented definition positive fixture is a no-op.' }
        $arrivalCases.Add($positive)
    }
    foreach ($replacement in @(
        ("#if 0`n" + $coreBridge.Value + "`n#endif"),
        ("#if 0`n" + $coreBridge.Value + "`n#endif`n  static void ReceiveSingleStaticFrame(const CanFrame *frame, uint64_t arrivalLowerUs, uint64_t observedUs)`n  {`n    (void)aethor_app_receive_can_frame_observed(frame, observedUs, observedUs);`n  }"))) {
        $arrivalCases.Add(@{ Name = ('disabled Core bridge ownership ' + $arrivalCases.Count); Expected = $false; CriticalExpected = $false;
            Text = $taskText.Replace($coreBridge.Value, $replacement); FacadeText = $facadeText; StaticTransportText = $staticTransportText;
            BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText })
    }
    $coreGuard = [regex]::Match($taskText, '(?s)' + [regex]::Escape($staticGuard) + '\s*/\*\* @brief Forward only the dedicated LCD DISABLE receipt;.*?\*/\s*static void CollectSingleStaticReceipt')
    if (-not $coreGuard.Success) { throw 'Actual Core receipt/bridge feature guard is missing.' }
    $arrivalCases.Add(@{ Name = 'Core helper feature guard weakened'; Expected = $false; FacadeText = $facadeText; StaticTransportText = $staticTransportText;
        BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText;
        Text = $taskText.Replace($coreGuard.Value, $coreGuard.Value.Replace($staticGuard, '#if 1')) })
    $arrivalCases.Add(@{ Name = 'single Core helper feature guard weakened'; Expected = $false; FacadeText = $facadeText; StaticTransportText = $staticTransportText;
        BaseText = $singleText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText;
        Text = $singleText.Replace($singleCoreBridge, $singleCoreBridge.Replace($singleGuard, '#if 1')) })
    $includeDirective = '#include "DebugUi/debug_ui_static_transport_app.inc"'
    foreach ($mutation in @(
        @('missing static include',''),
        @('wrong static include','#include "DebugUi/unrelated_static_transport_app.inc"'),
        @('commented static include',('/* ' + $includeDirective + ' */')),
        @('duplicate static include',($includeDirective + "`n" + $includeDirective)),
        @('disabled static include',("#if 0`n" + $includeDirective + "`n#endif")))) {
        if (-not $facadeText.Contains($includeDirective)) { throw 'Static facade include mutation anchor missing.' }
        $arrivalCases.Add(@{ Name = $mutation[0]; Expected = $false; Text = $taskText; StaticTransportText = $staticTransportText;
            FacadeText = $facadeText.Replace($includeDirective, $mutation[1]);
            BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText })
    }
    # Directives outside the literal task/helper bodies can shadow their calls; mutate each explicit source independently.
    $macroCases = [System.Collections.Generic.List[object]]::new()
    $controlledSymbols = @('ReceiveSingleStaticFrame', 'StartCanRxTask', 'CollectSingleStaticReceipt', 'CollectCapabilityReceipt',
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
    # Check every identifier in the exact contract bodies, including members, enums and local aliases, against the macro protection list.
    $contractBodyText = $singleCoreBridge
    foreach ($sourceDefinition in @(
        @{ Source = $taskText; Pattern = '(?ms)^static void CollectSingleStaticReceipt\([^)]*\)\s*\{.*?^\}' },
        @{ Source = $taskText; Pattern = '(?ms)^static void ReceiveSingleStaticFrame\([^)]*\)\s*\{.*?^\}' },
        @{ Source = $facadeText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame\([^)]*\)\s*\{.*?^\}' },
        @{ Source = $staticTransportText; Pattern = '(?ms)^MotorRuntimeStatus aethor_app_receive_static_frame\([^)]*\)\s*\{.*?^\}' })) {
        $contractDefinition = [regex]::Match($sourceDefinition.Source, $sourceDefinition.Pattern)
        if (-not $contractDefinition.Success) { throw 'Exact-contract identifier coverage source is missing.' }
        $contractBodyText += "`n" + $contractDefinition.Value
    }
    $contractBodyText = [regex]::Replace($contractBodyText, '(?s)"(?:\\.|[^"\\])*"|''(?:\\.|[^''\\])*''|/\*.*?\*/|//[^\r\n]*', ' ')
    $contractIdentifiers = @([regex]::Matches($contractBodyText, '\b[A-Za-z_]\w*\b') | ForEach-Object Value | Sort-Object -Unique)
    foreach ($contractIdentifier in $contractIdentifiers) {
        if ($controlledSymbols -cnotcontains $contractIdentifier) { throw ('Exact-contract identifier lacks macro mutation coverage: ' + $contractIdentifier) }
    }
    $macroCases.Add(@{ Name = 'StaticTransport actual TX completion replaced by HAL submission'; Text = $taskText; FacadeText = $facadeText;
        StaticTransportText = "#define single_static_tx_confirmed single_static_tx_submitted`n" + $staticTransportText + "`n#undef single_static_tx_confirmed`n";
        Expected = $false; CriticalExpected = $false; BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText })
    foreach ($sourceSide in @('Core','Facade','StaticTransport')) {
        foreach ($symbol in $controlledSymbols) {
            foreach ($directive in @('define','undef')) {
                $insert = '#' + $directive + ' ' + $symbol + $(if ($directive -eq 'define') { ' architecture_fixture_replacement' } else { '' }) + "`n"
                $case = @{ Name = ($sourceSide + ' ' + $directive + ' ' + $symbol); Text = $taskText; FacadeText = $facadeText; StaticTransportText = $staticTransportText;
                    Expected = $false; CriticalExpected = $false; BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText }
                if ($sourceSide -eq 'Core') { $case.Text = $taskText.Replace('void StartCanRxTask(', $insert + 'void StartCanRxTask(') }
                elseif ($sourceSide -eq 'Facade') { $case.FacadeText = $facadeText.Replace('MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame(', $insert + 'MotorRuntimeStatus aethor_app_debug_ui_receive_preflight_frame(') }
                else { $case.StaticTransportText = $staticTransportText.Replace('MotorRuntimeStatus aethor_app_receive_static_frame(', $insert + 'MotorRuntimeStatus aethor_app_receive_static_frame(') }
                $macroCases.Add($case)
            }
        }
        foreach ($insert in @(
            '#define ReceiveSingleStaticFrame(frame, lower, observed) do { HAL_Delay(1U); (void)aethor_app_debug_ui_receive_preflight_frame(frame, observed, observed, 0); } while (0)',
            '# define/**/ReceiveSingleStaticFrame(frame, lower, observed) replacement',
            ('static const char *macro_literal_start = "/*";' + "`n" + '#define ReceiveSingleStaticFrame replacement' + "`n" + 'static const char *macro_literal_end = "*/";'),
            ('#def' + '\' + "`n" + 'ine ReceiveSingleStaticFrame replacement'),
            ('#define ReceiveSingleSta' + '\' + "`r`n" + 'ticFrame replacement'),
            ('#undef ReceiveSingleSta' + '\' + "`n" + 'ticFrame'))) {
            $case = @{ Name = ($sourceSide + ' lexical macro ' + $macroCases.Count); Text = $taskText; FacadeText = $facadeText; StaticTransportText = $staticTransportText;
                Expected = $false; CriticalExpected = $false; BaseText = $taskText; BaseFacadeText = $facadeText; BaseStaticTransportText = $staticTransportText }
            if ($sourceSide -eq 'Core') { $case.Text = $insert + "`n" + $taskText }
            elseif ($sourceSide -eq 'Facade') { $case.FacadeText = $insert + "`n" + $facadeText }
            else { $case.StaticTransportText = $insert + "`n" + $staticTransportText }
            $macroCases.Add($case)
        }
    }
    foreach ($case in $macroCases) { $arrivalCases.Add($case) }
    $arrivalCases.Add(@{ Name = 'unrelated and commented macros stay allowed'; Expected = $true;
        Text = "#define architecture_unrelated_fixture 1`n/* #undef ReceiveSingleStaticFrame */`n" + $taskText;
        FacadeText = "// #define aethor_app_receive_can_frame_observed replacement`n" + $facadeText;
        StaticTransportText = "/* #undef aethor_app_receive_static_frame */`n" + $staticTransportText })
    foreach ($testCase in $arrivalCases) {
        if (-not $testCase.Expected -and $testCase.Text -ceq $testCase.BaseText -and $testCase.FacadeText -ceq $testCase.BaseFacadeText -and
            $testCase.StaticTransportText -ceq $testCase.BaseStaticTransportText) {
            throw ('CAN arrival regression mutation was a no-op: ' + $testCase.Name)
        }
        $arrivalAccepted = Test-CanRxArrivalContract -Text $testCase.Text -FacadeText $testCase.FacadeText -StaticTransportText $testCase.StaticTransportText
        $criticalAccepted = if ($testCase.ContainsKey('CriticalExpected')) {
            Test-CanRxCriticalBoundary -Text $testCase.Text -FacadeText $testCase.FacadeText -StaticTransportText $testCase.StaticTransportText
        } else { $null }
        if ($arrivalAccepted -ne $testCase.Expected) {
            throw ('CAN arrival regression failed: ' + $testCase.Name + '; arrival=' + $arrivalAccepted + '; critical=' + $criticalAccepted)
        }
        if ($testCase.ContainsKey('CriticalExpected') -and $criticalAccepted -ne $testCase.CriticalExpected) {
            throw ('CAN critical macro regression failed: ' + $testCase.Name)
        }
    }
    Write-Output ('CAN_RX_CONTROLLED_MACRO_REGRESSIONS_PASSED (' + $macroCases.Count + ' cases; three_explicit_sources=1 define_undef=1 lexical_splicing_comments=1 arrival_and_critical_reject=1)')
    Write-Output ('CAN_RX_BRIDGE_REGRESSIONS_PASSED (' + $arrivalCases.Count + ' cases; explicit_static_include=1 nonnoop=1 comments_cannot_satisfy_code=1 legacy_observed_single_full_paths=1)')
    $motorCases = @(
        @{ Name = 'fixed guarded facade codecs'; Text = $facadeText; Path = 'aethor_app.c'; Expected = $true },
        @{ Name = 'codec in motor layer'; Text = 's3519_pack_mode_command(1U, mode, command, frame);'; Path = 'Motor\fixture.c'; Expected = $true },
        @{ Name = 'codec outside motor layer'; Text = 's3519_decode_feedback(frame, ranges, feedback);'; Path = 'Arm\fixture.c'; Expected = $false },
        @{ Name = 'current static transport include'; Text = $staticTransportText; Path = 'DebugUi\debug_ui_static_transport_app.inc'; Expected = $true },
        @{ Name = 'new codec in static transport include'; Text = ($staticTransportText + "`ns3519_pack_mode_command(1U, mode, command, frame);"); Path = 'DebugUi\debug_ui_static_transport_app.inc'; Expected = $false },
        @{ Name = 'new facade codec'; Text = ($facadeText + "`ns3519_pack_mit_command();"); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'new facade actuator'; Text = ($facadeText + "`nmotor_enable();"); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance ENABLE'; Text = $facadeText.Replace('S3519_MODE_COMMAND_DISABLE, frame)', 'S3519_MODE_COMMAND_ENABLE, frame)'); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance MIT mode'; Text = $facadeText.Replace('s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY', 's3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT'); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance wrong motor'; Text = $facadeText.Replace('s3519_pack_parameter_read(7U,', 's3519_pack_parameter_read(6U,'); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance wrong register'; Text = $facadeText.Replace('s3519_pack_parameter_read(7U, (S3519Register)mapping->register_address,', 's3519_pack_parameter_read(7U, S3519_REGISTER_CONTROL_MODE,'); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance wrong function'; Text = $facadeText.Replace('aethor_app_mapping_next(', 'unrelated_mapping_next('); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance missing feature guard'; Text = $facadeText.Replace('#if AETHOR_ADRC_LCD_INTEGRATED', '#if 1'); Path = 'aethor_app.c'; Expected = $false },
        @{ Name = 'maintenance duplicate codec'; Text = $facadeText.Replace('s3519_decode_parameter_response(frame, &response)', 's3519_decode_parameter_response(frame, &response) + s3519_decode_parameter_response(frame, &response)'); Path = 'aethor_app.c'; Expected = $false }
    )
    foreach ($testCase in $motorCases) {
        if ((Test-MotorCommandBoundary -Text $testCase.Text -RelativePath $testCase.Path) -ne $testCase.Expected) {
            throw ('Motor boundary regression failed: ' + $testCase.Name)
        }
    }
    Write-Output ('PUBLICATION_CHECKER_REGRESSIONS_PASSED (' + ($cases.Count + $stopCases.Count + $arrivalCases.Count + $motorCases.Count) + ' cases)')
}

Invoke-PublicationCheckerRegression
