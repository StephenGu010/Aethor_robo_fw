/** @file persistence_joint_platform_test.c
 * @brief Real App/Core/Platform joint harness with synthetic HAL registers and RTOS time only.
 * No board, serial port, flash, EEPROM, physical USB edge or ARM image is exercised.
 */
#include <setjmp.h>
#include <stdlib.h>
#define main existing_integrated_fixture_main
#include "adrc_lcd_integration_test_main.c"
#undef main
#include "../../App/Platform/stm32_platform.c"
#include "stm32_adrc_channel.h"

/** @brief Independently controlled external observations; none of these fields is fabricated App evidence. */
typedef struct {
    FDCAN_GlobalTypeDef registers;
    FDCAN_TxHeaderTypeDef header;
    FDCAN_RxHeaderTypeDef rx_header;
    FDCAN_TxEventFifoTypeDef event;
    uint8_t payload[8], rx_payload[8];
    uint32_t tick, link_word, buffer, tx_calls, abort_calls, critical_depth, final_injection, adrc_calls;
    uint32_t device_pmax_raw, device_mode, save_calls;
    float device_position_rad, device_uoff;
    uint16_t counter;
    uint8_t rx_available, event_available, skip_pending, keep_bto, hal_failure, trace_items;
} JointFixture;
static JointFixture joint;
FDCAN_HandleTypeDef hfdcan1;
uint32_t single_fake_irq_mask;
/** @brief Minimal DWT metadata only for unchanged Core diagnostic arithmetic. */
typedef struct { uint32_t CYCCNT; } JointDwt;
static JointDwt joint_dwt;
static JointDwt *DWT = &joint_dwt;
static const uint32_t SystemCoreClock = 400000000U;
static uint32_t controlExecutionMaxUs, controlPeriodMaxUs, controlPreviousCycle;
static uint8_t controlCycleValid;
static void *ProtocolTaskHandle;
static jmp_buf control_return;
/** @brief RTOS tick units match the maintained four millisecond loop boundary. */
typedef uint32_t TickType_t;

/** @brief Preserve the production adapter's saved interrupt mask exactly. */
void single_fake_restore_irq(uint32_t value) { single_fake_irq_mask = value; }
/** @brief Accept metadata initialization without producing CAN traffic. */
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, const FDCAN_FilterTypeDef *filter)
{ (void)handle; (void)filter; return HAL_OK; }
/** @brief Preserve the actual platform's global filter setup. */
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{ (void)handle; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
/** @brief Preserve the actual platform's watermark setup. */
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t count)
{ (void)handle; (void)fifo; (void)count; return HAL_OK; }
/** @brief Start only initializes synthetic transport metadata. */
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
/** @brief Stop never clears pending hardware debt. */
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
/** @brief No task notification is invented by HAL configuration. */
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t buffer)
{ (void)handle; (void)mask; (void)buffer; return HAL_OK; }
/** @brief HAL admission records exact bytes; pending and successful transmission remain separate observations. */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
    const FDCAN_TxHeaderTypeDef *header, const uint8_t *payload)
{
    (void)handle; ++joint.tx_calls; joint.header = *header;
    memset(joint.payload, 0, sizeof(joint.payload)); memcpy(joint.payload, payload, header->DataLength >> 16U);
    if (header->Identifier == 0x7ffU && payload[2] == 0xaaU) ++joint.save_calls;
    if (!joint.skip_pending) joint.registers.TXBRP |= 0x100U;
    if (!joint.keep_bto) joint.registers.TXBTO &= ~0x100U;
    joint.registers.TXBCF &= ~0x100U;
    return joint.hal_failure ? HAL_ERROR : HAL_OK;
}
/** @brief Full FIFO capacity is distinct from whether an accepted request has transmitted. */
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle) { (void)handle; return 8U; }
/** @brief Raw backlog is visible to the real platform before inbox decoding. */
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{ (void)handle; return fifo == FDCAN_RX_FIFO0 ? joint.rx_available : 0U; }
/** @brief Pin exact hardware identity or deliberately report an unknown one. */
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle) { (void)handle; return joint.buffer; }
/** @brief Abort requests do not automatically clear TXBRP or imply cancellation completion. */
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t mask)
{ (void)handle; assert(mask); ++joint.abort_calls; return HAL_OK; }
/** @brief Return immutable synthetic wire metadata once to the production ISR/inbox. */
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo,
    FDCAN_RxHeaderTypeDef *header, uint8_t *payload)
{
    (void)handle; (void)fifo; if (!joint.rx_available) return HAL_ERROR;
    *header = joint.rx_header; memcpy(payload, joint.rx_payload, 8U); joint.rx_available = 0U; return HAL_OK;
}
/** @brief Preserve the production strict-slot timestamp prescaler. */
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *handle, uint32_t prescaler)
{ (void)handle; assert(prescaler == FDCAN_TIMESTAMP_PRESC_16); return HAL_OK; }
/** @brief Preserve the shared internal RX/TX timestamp domain. */
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *handle, uint32_t operation)
{ (void)handle; assert(operation == FDCAN_TIMESTAMP_INTERNAL); return HAL_OK; }
/** @brief Fine SOF counter is independent of the board's one millisecond clock. */
uint16_t HAL_FDCAN_GetTimestampCounter(const FDCAN_HandleTypeDef *handle) { (void)handle; return joint.counter; }
/** @brief Supply a separately scheduled TX event rather than deriving one from HAL_OK. */
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *handle, FDCAN_TxEventFifoTypeDef *event)
{
    (void)handle; if (!joint.event_available) return HAL_ERROR;
    *event = joint.event; joint.event_available = 0U; joint.registers.TXEFS &= ~FDCAN_TXEFS_EFFL; return HAL_OK;
}
/** @brief Production FDCAN nominal bitrate arithmetic remains executable. */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t peripheral) { assert(peripheral == RCC_PERIPHCLK_FDCAN); return 80000000U; }
/** @brief Preserve one millisecond quantization exactly. */
uint32_t HAL_GetTick(void) { return joint.tick; }
/** @brief USB transmit is outside this test and emits no data. */
uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length) { (void)data; (void)length; return USBD_OK; }
/** @brief Core task critical sections remain distinct from Platform IRQ masking. */
static void EnterTaskCritical(void) { ++joint.critical_depth; }
/** @brief Reject unmatched task critical exits. */
static void ExitTaskCritical(void) { assert(joint.critical_depth); --joint.critical_depth; }
#define taskENTER_CRITICAL() EnterTaskCritical()
#define taskEXIT_CRITICAL() ExitTaskCritical()
#define pdMS_TO_TICKS(value) (value)
#define TaskHandle_t void *
/** @brief Core observation time is the same quantized board clock used by Platform. */
static uint64_t AethorMonotonicTimestampUs(void) { return (uint64_t)joint.tick * 1000ULL; }
/** @brief Synthetic full USB event-word mutation occurs at the real final Core callback. */
uint32_t CDC_PersistenceLinkState_HS(void)
{
    if (single_fake_irq_mask && joint.final_injection) {
        unsigned injection = joint.final_injection; joint.final_injection = 0U;
        if (injection == 1U) joint.link_word ^= 1U;
        if (injection == 2U) joint.link_word ^= 0x80000000UL;
        if (injection == 3U) aethor_app_persistence_preempt(AethorMonotonicTimestampUs());
        if (injection == 4U) ++application_debug_ui.epoch;
        if (injection == 5U) application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
    }
    return joint.link_word;
}
/** @brief Empty dedicated ADRC channel provides no completion proof. */
AdrcCanReceipt stm32_adrc_channel_collect(void) { AdrcCanReceipt receipt = {0}; return receipt; }
/** @brief An unexpected ADRC transport admission is counted and refused. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float torque)
{ (void)frame; (void)kind; (void)torque; ++joint.adrc_calls; return 0U; }
/** @brief No scheduler delay occurs in the unused guard branch. */
static void vTaskDelay(unsigned ticks) { (void)ticks; }
/** @brief Supply the same quantized tick to the unchanged guard branch. */
static TickType_t xTaskGetTickCount(void) { return joint.tick; }
/** @brief End exactly one complete maintained Core control-loop iteration. */
static void vTaskDelayUntil(TickType_t *last_wake, unsigned ticks)
{ assert(last_wake && ticks == 4U && !joint.critical_depth); longjmp(control_return, 1); }
/** @brief Notifications are RTOS metadata only and never hardware completion. */
static unsigned xTaskNotifyGive(void *handle) { (void)handle; return 1U; }
#include "persistence_core_extracted.inc"

/** @brief Execute one actual full Core loop with bounded external scheduling. */
static void Control(void) { if (!setjmp(control_return)) RunExtractedControlTask(); }
/** @brief Advance coarse and fine observations without running any production code implicitly. */
static void Advance(unsigned milliseconds) { joint.tick += milliseconds; joint.counter += (uint16_t)(milliseconds * 80U); }
/** @brief Initialize actual discovery/profile/disabled state, actual Platform, and one private readonly owner. */
static void Fixture(void)
{
    ProtocolOutputBatch output;
    MotorDiscoveryResult *discovery;
    memset(&joint, 0, sizeof(joint)); memset(&hfdcan1, 0, sizeof(hfdcan1));
    memset(&application_persistence, 0, sizeof(application_persistence));
    memset(&application_persistence_transaction, 0, sizeof(application_persistence_transaction));
    memset(&application_persistence_save_journal, 0, sizeof(application_persistence_save_journal));
    single_fake_irq_mask = 0U; joint.tick = 110U; joint.counter = 100U; joint.buffer = 0x100U;
    joint.link_word = 0x80000001UL; hfdcan1.Instance = &joint.registers;
    joint.device_pmax_raw = 0x41480000U; joint.device_mode = 1U;
    joint.device_position_rad = 0.0F; joint.device_uoff = 1.25F;
    assert(motor_persistence_rid(37U) == 0x32U && motor_persistence_rid(43U) == 0x50U);
    hfdcan1.Init.StdFiltersNbr = 7U; hfdcan1.Init.TxFifoQueueElmtsNbr = 8U; hfdcan1.Init.TxEventsNbr = 8U;
    hfdcan1.Init.NominalPrescaler = 1U; hfdcan1.Init.NominalTimeSeg1 = 59U; hfdcan1.Init.NominalTimeSeg2 = 20U;
    aethor_app_init(1000U, 1234U); aethor_app_set_task_critical_hooks(EnterTaskCritical, ExitTaskCritical);
    assert(request("0 hello", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    fixture_disabled_motor7(109000U); discovery = &application_motor_runtime.discovery.results[6];
    discovery->ranges.torque_max_nm = 10.0F; discovery->software_version = 808597046U; discovery->sub_version = 49U;
    discovery->hardware_version = 1U;
    assert(stm32_platform_init(application_motor_runtime.configuration) == STM32_PLATFORM_STATUS_OK);
    aethor_app_integrated_set_can_idle(1U, 104000U); aethor_app_integrated_set_can_idle(1U, 110000U);
    aethor_app_persistence_observe_link(joint.link_word, 110000U);
    assert(aethor_app_persistence_begin_readonly(91U, 110000U));
    assert(aethor_app_persistence_locked());
}
/** @brief Model physical transmission completion and its matching original SOF event separately. */
static void Complete(void)
{
    joint.registers.TXBRP &= ~0x100U; joint.registers.TXBTO |= 0x100U;
    if (joint.header.TxEventFifoControl == FDCAN_STORE_TX_EVENTS) {
        memset(&joint.event, 0, sizeof(joint.event)); joint.event.Identifier = joint.header.Identifier;
        joint.event.DataLength = joint.header.DataLength >> 16U; joint.event.MessageMarker = joint.header.MessageMarker;
        joint.event.EventType = FDCAN_TX_EVENT; joint.event.TxTimestamp = joint.counter;
        joint.event_available = 1U; joint.registers.TXEFS = 1U;
    }
}
/** @brief Push immutable wire frame through real HAL ISR, real inbox/tag capture and real Core RX bridge. */
static void Receive(const CanFrame *frame, uint16_t sof)
{
    CanFrame decoded; uint64_t lower;
    memset(&joint.rx_header, 0, sizeof(joint.rx_header)); joint.rx_header.Identifier = frame->identifier;
    joint.rx_header.DataLength = (uint32_t)frame->length << 16U; joint.rx_header.RxTimestamp = sof;
    memcpy(joint.rx_payload, frame->data, 8U); joint.rx_available = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&decoded, &lower) == CAN_RX_INBOX_STATUS_OK);
    assert(!single_fake_irq_mask); EnterTaskCritical();
    ReceiveSingleStaticFrame(&decoded, lower, AethorMonotonicTimestampUs()); ExitTaskCritical();
}
/** @brief Complete one independent strict FD and qualify only a later SOF and later coarse lower bound. */
static void HealthExpected(uint8_t expected_ready)
{
    CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    uint32_t count = joint.tx_calls;
    Control(); assert(joint.tx_calls == count + 1U);
    assert(joint.header.Identifier == 7U && joint.header.DataLength == FDCAN_DLC_BYTES_8);
    for (unsigned byte = 0U; byte < 7U; ++byte) assert(joint.payload[byte] == 0xffU);
    assert(joint.payload[7] == 0xfdU && joint.header.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
    assert(application_persistence.disable_submitted && !application_persistence.disable_actual_tx);
    Complete(); Advance(1U); stm32_platform_can_observe_rx_boundary(AethorMonotonicTimestampUs());
    Receive(&feedback, (uint16_t)(joint.event.TxTimestamp + 1U));
    assert(application_persistence.disable_actual_tx);
    if (!expected_ready) {
        printf("JOINT_POSITION_FD_RESULT expected_ready=0 ready=%u health_proven=%u fault=%u p_m=%.6f u_off=%.6f\n",
            application_persistence.snapshot_ready, application_persistence.health_proven,
            application_persistence.fault, (double)joint.device_position_rad, (double)joint.device_uoff);
        assert(!application_persistence.health_proven && !application_persistence.snapshot_ready &&
            !application_persistence.continuity_checked && application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
        return;
    }
    assert(application_persistence.health_proven);
    assert(application_persistence.health.observed_us > application_persistence.disable_submitted_us);
    assert(debug_ui_static_tx_identity_equal(&application_persistence.health_identity, &application_persistence.disable_identity));
}
/** @brief Require healthy strict FD for the ordinary joint transport scenarios. */
static void Health(void) { HealthExpected(1U); }
/** @brief Encode a register's deterministic fixed-profile raw baseline, without manufacturing transport evidence. */
static uint32_t Register(unsigned index)
{
    float value = 1.0F; uint32_t raw;
    if (index == 7U) return 23U;
    if (index == 8U) return 7U;
    if (index == 10U) return joint.device_mode;
    if (index == 14U) return 808597046U;
    if (index == 36U) return 49U;
    if (index == 21U) return joint.device_pmax_raw;
    if (index == 22U) value = 200.0F;
    if (index == 23U) value = 10.0F;
    /* Vendor RID0x32 is u_off; only RID0x50 is the motor's native position p_m. */
    if (index == 37U) value = joint.device_uoff;
    if (index == 43U) value = joint.device_position_rad;
    if (motor_persistence_is_unsigned(index)) return 1U;
    memcpy(&raw, &value, sizeof(raw)); return raw;
}
/** @brief Build the exact vendor register reply for the current immutable requested RID. */
static CanFrame Reply(unsigned index)
{
    CanFrame frame = {23U, 8U, {7U, 0U, 0x33U, 0U}}; uint32_t raw = Register(index);
    frame.data[3] = motor_persistence_rid(index);
    for (unsigned byte = 0U; byte < 4U; ++byte) frame.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
    return frame;
}
/** @brief Submit actual App transfer via actual Core helper and dedicated Platform reservation. */
static MotorPersistenceIdentity SubmitRead(unsigned index)
{
    MotorPersistenceIdentity identity; uint32_t count = joint.tx_calls; Advance(1U); Control();
    assert(!application_persistence.fault && joint.tx_calls == count + 1U);
    assert(joint.header.Identifier == 0x7ffU && joint.header.DataLength == FDCAN_DLC_BYTES_4);
    assert(joint.payload[0] == 7U && joint.payload[1] == 0U && joint.payload[2] == 0x33U);
    assert(joint.payload[3] == motor_persistence_rid(index) && !platform_capability_tracker.active);
    assert(platform_persistence.active && application_persistence.parameter_submitted);
    identity = application_persistence.parameter_identity;
    assert(motor_persistence_identity_equal(&identity, &platform_persistence.identity));
    assert(platform_persistence.submitted_us == application_persistence_transaction.submitted_us);
    assert(application_persistence_transaction.boundary_us == AethorMonotonicTimestampUs());
    return identity;
}
/** @brief Real capability collector cannot steal any overlapping maintenance RID receipt. */
static void AssertNoCapabilityReceipt(void)
{
    CanFrame frame; uint8_t succeeded;
    assert(!stm32_platform_can_collect_capability_receipt(&frame, &succeeded));
}
/** @brief Drive all45 exact reads and preserve original identities and coarse observation bounds. */
static void TestReadonly(void)
{
    Fixture(); Health();
    for (unsigned index = 0U; index < 45U; ++index) {
        CanFrame reply = Reply(index); MotorPersistenceIdentity identity;
        if (AethorMonotonicTimestampUs() - application_persistence.health.observed_us >= 49000U) Health();
        identity = SubmitRead(index); AssertNoCapabilityReceipt();
        assert(joint.registers.TXBRP == 0x100U); Complete();
        EnterTaskCritical(); CollectPersistenceReceipt(AethorMonotonicTimestampUs()); ExitTaskCritical();
        AssertNoCapabilityReceipt(); assert(!platform_persistence.active);
        /* The board permits a parameter RX lower bound equal to its submit boundary. */
        Receive(&reply, joint.counter);
        assert(application_persistence_transaction.read_index == index + 1U && !application_persistence.fault);
        const MotorPersistenceItem *item = &application_persistence_transaction.current.items[index];
        assert(item->valid && item->raw == Register(index));
        assert(motor_persistence_identity_equal(&item->identity, &identity));
        assert(item->submitted_us == item->boundary_us && item->rx_lower_us == item->submitted_us);
        printf("JOINT_ITEM index=%u rid=%u boot=%lu request=%lu owner=%lu sequence=%lu phase=%u created=%llu submitted=%llu boundary=%llu tx_observed=%llu rx_lower=%llu received=%llu raw=%08lx\n",
            index, motor_persistence_rid(index), (unsigned long)identity.boot_epoch, (unsigned long)identity.request_id,
            (unsigned long)identity.owner_generation, (unsigned long)identity.sequence, identity.phase,
            (unsigned long long)item->created_us, (unsigned long long)item->submitted_us, (unsigned long long)item->boundary_us,
            (unsigned long long)item->send_completed_observed_us, (unsigned long long)item->rx_lower_us,
            (unsigned long long)item->received_us, (unsigned long)item->raw);
    }
    Advance(1U); Health(); assert(application_persistence.snapshot_ready && application_persistence.continuity_checked);
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_BASELINE_READY);
    assert(!application_persistence_transaction.save_used && !application_persistence_transaction.save_uncertain);
    puts("JOINT_READONLY45_PASS actual_app=1 actual_platform=1 verbatim_core=1 hal_only_synthetic=1");
}
/** @brief RX can precede TX observation but cannot complete before the actual pinned TXBTO proof. */
static void TestDeferred(void)
{
    Fixture(); Health(); (void)SubmitRead(0U); CanFrame reply = Reply(0U);
    Receive(&reply, joint.counter); assert(application_persistence_transaction.deferred);
    assert(!application_persistence_transaction.read_index && !application_persistence_transaction.actual_tx);
    Complete(); CollectPersistenceReceipt(AethorMonotonicTimestampUs());
    assert(application_persistence_transaction.read_index == 1U && !application_persistence.fault);
    puts("JOINT_RX_BEFORE_TX_OBSERVATION_DEFERRED_PASS");
}
/** @brief Delayed pending, stale BTO, cancellation and unknown hardware slots retain real transport debt. */
static void TestDebt(void)
{
    for (unsigned scenario = 0U; scenario < 5U; ++scenario) {
        Fixture(); Health();
        if (scenario == 0U) { joint.keep_bto = 1U; joint.registers.TXBTO = 0x100U; }
        if (scenario == 1U || scenario == 2U) joint.skip_pending = 1U;
        if (scenario == 3U) joint.buffer = 0U;
        (void)SubmitRead(0U);
        if (scenario == 4U) aethor_app_persistence_preempt(AethorMonotonicTimestampUs());
        if (scenario == 2U) aethor_app_persistence_preempt(AethorMonotonicTimestampUs());
        CollectPersistenceReceipt(AethorMonotonicTimestampUs());
        assert(platform_persistence.active && application_persistence.parameter_outstanding);
        assert(!application_persistence_transaction.read_index);
        if (scenario == 1U || scenario == 2U) {
            assert(!joint.registers.TXBRP); joint.registers.TXBRP = 0x100U;
            CollectPersistenceReceipt(AethorMonotonicTimestampUs());
            assert(platform_persistence.active);
        }
        if (scenario == 2U || scenario == 4U) assert(joint.abort_calls);
        if (scenario == 0U) { joint.registers.TXBRP = 0U; CollectPersistenceReceipt(AethorMonotonicTimestampUs());
            assert(application_persistence.fault && !platform_persistence.active); }
        else if (scenario == 3U) { Complete(); CollectPersistenceReceipt(AethorMonotonicTimestampUs()); assert(platform_persistence.active); }
        else { Complete(); CollectPersistenceReceipt(AethorMonotonicTimestampUs());
            assert(!platform_persistence.active && !application_persistence.parameter_outstanding);
            assert((application_persistence.fault != MOTOR_PERSISTENCE_ERROR_NONE) == (scenario == 2U || scenario == 4U)); }
        printf("JOINT_DEBT_PASS scenario=%u aborts=%lu\n", scenario, (unsigned long)joint.abort_calls);
    }
}
/** @brief Final callback rejects synthetic USB generation/configured, STOP, local epoch and owner changes without HAL. */
static void TestFinal(void)
{
    for (unsigned stage = 0U; stage < 2U; ++stage) for (unsigned injection = 1U; injection <= 5U; ++injection) {
        Fixture(); if (stage) Health(); uint32_t count = joint.tx_calls;
        Advance(1U); joint.final_injection = injection; Control();
        assert(!joint.final_injection && joint.tx_calls == count && application_persistence.fault);
        joint.link_word = 0x80000001UL; application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
        aethor_app_persistence_observe_link(joint.link_word, AethorMonotonicTimestampUs());
        AethorPersistenceTransfer transfer; assert(!aethor_app_persistence_next_transfer(AethorMonotonicTimestampUs(), &transfer));
        assert(aethor_app_persistence_locked() && !application_persistence_transaction.save_used);
        printf("JOINT_FINAL_GATE_PASS stage=%u synthetic_event=%u hal_delta=0\n", stage, injection);
    }
}
/** @brief App lock blocks queued controls and actual Core preserves emergency STOP until pending debt clears. */
static void TestMutex(void)
{
    Fixture(); Health(); (void)SubmitRead(0U);
    application_action.control_group_ready = 1U; application_adrc_bridge.frame_pending = 1U;
    uint32_t count = joint.tx_calls, generation = aethor_app_persistence_admission_generation();
    Control(); assert(joint.tx_calls == count && !joint.adrc_calls);
    assert(application_action.control_group_ready && application_adrc_bridge.frame_pending);
    assert(!aethor_app_persistence_allow_ordinary(generation));
    ProtocolOutputBatch output;
    (void)request("1 bench stop 1,3", AethorMonotonicTimestampUs(), &output);
    assert(application_debug_ui.mailbox.stop_intent_mask == 0x45U);
    Control(); assert(joint.tx_calls == count && joint.abort_calls && platform_persistence.active);
    Complete(); Control(); assert(!platform_persistence.active && joint.tx_calls > count);
    assert(joint.header.Identifier == 7U && joint.payload[7] == 0xfdU);
    assert(application_persistence.fault && aethor_app_persistence_locked());
    assert(!application_persistence_transaction.save_used);
    puts("JOINT_MUTEX_AND_EMERGENCY_DEBT_PASS");
}
/** @brief Raw backlog, missing actual TX and invalid FD ordering cannot create fresh maintenance health. */
static void TestHealthReject(void)
{
    for (unsigned scenario = 0U; scenario < 6U; ++scenario) {
        CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
        Fixture();
        if (!scenario) joint.rx_available = 1U;
        Control();
        if (!scenario) { assert(!joint.tx_calls && !application_persistence.health_proven); }
        else {
            assert(joint.tx_calls == 1U); Complete();
            if (scenario == 1U) joint.registers.TXBTO = 0U;
            if (scenario == 2U) ++joint.event.MessageMarker;
            if (scenario != 3U) { Advance(1U); stm32_platform_can_observe_rx_boundary(AethorMonotonicTimestampUs()); }
            uint16_t sof = (uint16_t)(joint.event.TxTimestamp + (scenario == 4U ? 0U : 1U));
            if (scenario == 5U) { joint.registers.TXBRP = 0x100U; joint.registers.TXBTO = 0U; }
            Receive(&feedback, sof);
            assert(!application_persistence.health_proven && !application_persistence_transaction.read_index);
        }
        assert(!application_persistence_transaction.save_used);
        printf("JOINT_HEALTH_REJECT_PASS scenario=%u\n", scenario);
    }
}
/** @brief Preserve actual ISR/inbox arrival bounds through delayed Core decode and inspect real Platform ordering.
 * Nonzero identity_variant deliberately corrupts one captured tag field as a hostile metadata input; it is never a positive proof.
 */
static uint64_t ReceiveHealthBoundary(const CanFrame *frame, uint16_t sof, unsigned decode_delay_ms,
    unsigned identity_variant, uint8_t expected_ordered)
{
    CanFrame decoded; uint64_t lower;
    DebugUiStaticTxIdentity identity;
    uint16_t returned_sof = 0U;
    uint8_t sof_valid = 0U, ordered, slot = platform_can_rx_inbox.write_index;
    memset(&joint.rx_header, 0, sizeof(joint.rx_header)); joint.rx_header.Identifier = frame->identifier;
    joint.rx_header.DataLength = (uint32_t)frame->length << 16U; joint.rx_header.RxTimestamp = sof;
    memcpy(joint.rx_payload, frame->data, 8U); joint.rx_available = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    uint64_t captured_lower = platform_can_rx_inbox.arrival_lower_bound_us[slot];
    DebugUiStaticTxIdentity *captured_identity = &platform_single_static_rx[slot].identity;
    if (identity_variant == 1U) ++captured_identity->epoch;
    if (identity_variant == 2U) ++captured_identity->request_id;
    if (identity_variant == 3U) ++captured_identity->sequence;
    if (identity_variant == 4U) ++captured_identity->motor_id;
    if (identity_variant == 5U) ++captured_identity->phase;
    assert(identity_variant <= 5U);
    Advance(decode_delay_ms);
    assert(stm32_platform_can_pop_received_at(&decoded, &lower) == CAN_RX_INBOX_STATUS_OK);
    assert(lower == captured_lower && !single_fake_irq_mask);
    EnterTaskCritical(); ReceiveSingleStaticFrame(&decoded, lower, AethorMonotonicTimestampUs());
    ordered = stm32_platform_can_static_rx_identity(&decoded, &identity, &returned_sof, &sof_valid);
    ExitTaskCritical();
    printf("JOINT_HEALTH_BOUNDARY lower=%llu submitted=%llu decode=%llu tx_sof=%u rx_sof=%u counter=%u ordered=%u sof_valid=%u actual_tx=%u health_proven=%u fault=%u\n",
        (unsigned long long)lower, (unsigned long long)application_persistence.disable_submitted_us,
        (unsigned long long)AethorMonotonicTimestampUs(), application_persistence.disable_sof,
        returned_sof, joint.counter, ordered, sof_valid, application_persistence.disable_actual_tx,
        application_persistence.health_proven, application_persistence.fault);
    assert(ordered == expected_ordered && sof_valid && returned_sof == sof);
    if (ordered) assert(debug_ui_static_tx_identity_equal(&identity, &application_persistence.disable_identity));
    assert(!application_persistence.parameter_submitted && !platform_persistence.active && !joint.save_calls);
    return lower;
}
/** @brief Require prompt +9 SOF feedback, legal wrap and delayed decode without advancing the coarse arrival bound. */
static void TestHealthSameTickPositive(void)
{
    const CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    for (unsigned scenario = 0U; scenario < 3U; ++scenario) {
        Fixture(); joint.counter = scenario == 1U ? 65532U : 46814U;
        uint64_t previous_health = application_persistence.health.observed_us;
        Control(); assert(joint.tx_calls == 1U && application_persistence.disable_submitted &&
            !application_persistence.disable_actual_tx);
        Complete(); joint.counter = (uint16_t)(joint.event.TxTimestamp + 9U);
        stm32_platform_can_observe_rx_boundary(AethorMonotonicTimestampUs());
        uint64_t lower = ReceiveHealthBoundary(&feedback, joint.counter, scenario == 2U ? 20U : 0U, 0U, 1U);
        assert(lower == application_persistence.disable_submitted_us && lower > previous_health);
        if (!application_persistence.health_proven) {
            puts("JOINT_SAME_TICK_REQUIRED_HEALTH_MISSING actualTX_ordered_identity_SOF9_valid=1"); exit(4);
        }
        assert(application_persistence.disable_actual_tx && application_persistence.health.observed_us == lower && !application_persistence.disable_wait_rx &&
            !application_persistence.fault && !application_persistence_transaction.read_index &&
            !application_persistence_transaction.save_used && !application_persistence_transaction.save_uncertain);
        assert(debug_ui_static_tx_identity_equal(&application_persistence.health_identity,
            &application_persistence.disable_identity));
        assert(joint.tx_calls == 1U && aethor_app_persistence_locked());
        printf("JOINT_SAME_TICK_HEALTH_PASS scenario=%u delta=9 preserved_lower=1 delay_ms=%u\n",
            scenario, scenario == 2U ? 20U : 0U);
    }
}
/** @brief Keep coarse history, wire order, actual TX, TEF and every captured identity field fail closed at the new boundary. */
static void TestHealthSameTickReject(void)
{
    const CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    const char *names[] = {"lower_before_submit", "lower_equals_previous_health", "same_sof", "old_sof",
        "future_sof", "half_wrap", "no_actual_tx", "no_tef", "wrong_marker", "wrong_can_id",
        "wrong_dlc", "lost_tef", "wrong_epoch", "wrong_request", "wrong_sequence", "wrong_motor", "wrong_phase", "unknown_lower"};
    for (unsigned scenario = 0U; scenario < sizeof(names) / sizeof(names[0]); ++scenario) {
        Fixture();
        /* Synthetic prior history and malformed bounds exercise defensive App gates, never valid health evidence. */
        if (scenario == 0U) Advance(1U);
        if (scenario == 1U) application_persistence.health.observed_us = AethorMonotonicTimestampUs();
        uint64_t previous_health = application_persistence.health.observed_us;
        Control(); assert(joint.tx_calls == 1U);
        uint16_t transmitted_sof = joint.counter;
        if (scenario != 6U) Complete();
        joint.counter = (uint16_t)(transmitted_sof + 9U);
        uint16_t received_sof = joint.counter;
        if (scenario == 0U) platform_rx_lower_bound_us = application_persistence.disable_submitted_us - 1000ULL;
        if (scenario == 2U) received_sof = transmitted_sof;
        if (scenario == 3U) received_sof = (uint16_t)(transmitted_sof - 1U);
        if (scenario == 4U) received_sof = (uint16_t)(joint.counter + 1U);
        if (scenario == 5U) joint.counter = received_sof = (uint16_t)(transmitted_sof + 32768U);
        if (scenario == 7U) { joint.event_available = 0U; joint.registers.TXEFS = 0U; }
        if (scenario == 8U) ++joint.event.MessageMarker;
        if (scenario == 9U) ++joint.event.Identifier;
        if (scenario == 10U) joint.event.DataLength = 7U;
        if (scenario == 11U) joint.registers.IR |= FDCAN_IR_TEFL;
        if (scenario == 17U) platform_rx_lower_bound_us = 0ULL;
        unsigned identity_variant = scenario >= 12U && scenario <= 16U ? scenario - 11U : 0U;
        uint8_t expected_ordered = (uint8_t)(scenario <= 1U || scenario == 17U);
        uint64_t lower = ReceiveHealthBoundary(&feedback, received_sof, 0U, identity_variant, expected_ordered);
        assert(!application_persistence.health_proven && application_persistence.health.observed_us == previous_health);
        assert(!application_persistence_transaction.read_index && !application_persistence_transaction.save_used &&
            !application_persistence_transaction.save_uncertain && aethor_app_persistence_locked());
        if (scenario == 0U) assert(lower < application_persistence.disable_submitted_us && lower > previous_health &&
            application_persistence.disable_actual_tx);
        if (scenario == 1U) assert(lower == application_persistence.disable_submitted_us && lower == previous_health);
        if (scenario >= 2U && scenario <= 16U) assert(lower == application_persistence.disable_submitted_us);
        printf("JOINT_SAME_TICK_REJECT_PASS case=%s\n", names[scenario]);
    }
}
/** @brief Expired frame/health and total budgets plus clock rollback remain irrevocable before a delayed same-tick RX. */
static void TestHealthSameTickDeadlines(void)
{
    const CanFrame feedback = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    const char *names[] = {"frame_100ms", "total_45s", "clock_rollback"};
    for (unsigned scenario = 0U; scenario < 3U; ++scenario) {
        Fixture(); Control(); Complete(); joint.counter = (uint16_t)(joint.event.TxTimestamp + 9U);
        uint64_t total_deadline = application_persistence_transaction.total_deadline_us;
        uint64_t phase_deadline = application_persistence_transaction.phase_deadline_us;
        if (scenario == 0U) Advance(100U);
        if (scenario == 1U) joint.tick = (uint32_t)(total_deadline / 1000ULL);
        if (scenario == 2U) --joint.tick;
        (void)ReceiveHealthBoundary(&feedback, (uint16_t)(joint.event.TxTimestamp + 9U), 0U, 0U, (uint8_t)(scenario == 0U));
        MotorPersistenceError expected = scenario == 1U ? MOTOR_PERSISTENCE_ERROR_TIMEOUT :
            (scenario == 2U ? MOTOR_PERSISTENCE_ERROR_CLOCK : MOTOR_PERSISTENCE_ERROR_HEALTH);
        assert(application_persistence.fault == expected && !application_persistence.health_proven &&
            application_persistence_transaction.state == MOTOR_PERSISTENCE_ABORTED);
        assert(application_persistence_transaction.total_deadline_us == total_deadline &&
            application_persistence_transaction.phase_deadline_us == phase_deadline);
        joint.tick += 1U; aethor_app_persistence_service(AethorMonotonicTimestampUs());
        assert(application_persistence.fault == expected && joint.tx_calls == 1U &&
            !application_persistence_transaction.save_used && !application_persistence_transaction.save_uncertain);
        printf("JOINT_SAME_TICK_DEADLINE_REJECT_PASS case=%s\n", names[scenario]);
    }
}
/** @brief Preserve the slow positive and all original health negatives alongside the new same-tick contract. */
static void TestHealthSameTick(void)
{
    Fixture(); Health(); puts("JOINT_ORIGINAL_SLOW_HEALTH_PASS");
    TestHealthSameTickPositive(); TestHealthSameTickReject(); TestHealthSameTickDeadlines();
}
/** @brief Finish one real parameter submission using exact ACK and independently scheduled HAL transmission proof. */
static void FinishPair(const CanFrame *reply, uint8_t early_reply)
{
    MotorPersistenceIdentity identity = application_persistence.parameter_identity;
    if (early_reply) {
        Receive(reply, joint.counter);
        assert(application_persistence_transaction.deferred && application_persistence.parameter_outstanding);
    }
    Complete(); CollectPersistenceReceipt(AethorMonotonicTimestampUs());
    AssertNoCapabilityReceipt();
    if (!early_reply) Receive(reply, joint.counter);
    assert(!application_persistence.fault && !application_persistence.parameter_outstanding && !platform_persistence.active);
    assert(motor_persistence_identity_equal(&identity, &application_persistence_transaction.identity));
}
/** @brief Complete all45 of the current real phase and require an independent post-snapshot strict FD. */
static void ReadPhaseUnpublished(uint8_t expected_phase, uint8_t early_reply)
{
    for (unsigned index = 0U; index < 45U; ++index) {
        if (!application_persistence.health_proven ||
            AethorMonotonicTimestampUs() - application_persistence.health.observed_us >= 49000U) { Advance(1U); Health(); }
        MotorPersistenceIdentity identity = SubmitRead(index); CanFrame reply = Reply(index);
        assert(identity.phase == expected_phase); FinishPair(&reply, early_reply);
        const MotorPersistenceItem *item = &application_persistence_transaction.current.items[index];
        assert(item->valid && item->raw == Register(index));
        assert(motor_persistence_identity_equal(&item->identity, &identity));
    }
    assert(!application_persistence.snapshot_ready);
}
/** @brief Publish one true all45 phase only after independent FD and export unchanged original evidence. */
static void ReadPhase(uint8_t expected_phase, uint8_t early_reply)
{
    ReadPhaseUnpublished(expected_phase, early_reply); Advance(1U); Health();
    assert(application_persistence.snapshot_ready && application_persistence.continuity_checked);
    for (unsigned index = 0U; index < 45U; ++index) {
        AethorPersistenceItemView view; MotorPersistenceItem original = application_persistence_transaction.current.items[index];
        uint64_t original_clock = application_persistence.last_us;
        assert(aethor_app_persistence_get_item(91U, expected_phase, index, &view));
        assert(view.rid == motor_persistence_rid(index));
        assert(motor_persistence_identity_equal(&view.item.identity, &original.identity));
        assert(view.item.raw == original.raw && view.item.valid == original.valid);
        assert(view.item.created_us == original.created_us && view.item.submitted_us == original.submitted_us);
        assert(view.item.boundary_us == original.boundary_us && view.item.send_completed_observed_us == original.send_completed_observed_us);
        assert(view.item.rx_lower_us == original.rx_lower_us && view.item.received_us == original.received_us);
        assert(application_persistence.last_us == original_clock);
        if (joint.trace_items) printf("JOINT_TYPED_ITEM phase=%u rid=%u boot=%lu request=%lu owner=%lu sequence=%lu created=%llu submitted=%llu boundary=%llu tx_observed=%llu rx_lower=%llu received=%llu raw=%08lx\n",
            expected_phase, view.rid, (unsigned long)view.item.identity.boot_epoch, (unsigned long)view.item.identity.request_id,
            (unsigned long)view.item.identity.owner_generation, (unsigned long)view.item.identity.sequence,
            (unsigned long long)view.item.created_us, (unsigned long long)view.item.submitted_us,
            (unsigned long long)view.item.boundary_us, (unsigned long long)view.item.send_completed_observed_us,
            (unsigned long long)view.item.rx_lower_us, (unsigned long long)view.item.received_us, (unsigned long)view.item.raw);
    }
}
/** @brief Transport exactly the two fixed volatile target writes, refreshing health after each proven mapping change. */
static void Prepare(uint8_t early_reply)
{
    assert(aethor_app_persistence_prepare(91U, AethorMonotonicTimestampUs()));
    while (application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        if (!application_persistence.health_proven) { Advance(1U); Health(); }
        unsigned step = application_persistence_transaction.write_step;
        uint8_t rid = step == 0U ? 21U : 10U; uint32_t raw = step == 0U ? 0x44000000U : 2U;
        uint32_t count = joint.tx_calls; Advance(1U); Control();
        assert(joint.tx_calls == count + 1U && platform_persistence.active);
        assert(joint.header.Identifier == 0x7ffU && joint.header.DataLength == FDCAN_DLC_BYTES_8);
        assert(joint.payload[0] == 7U && joint.payload[1] == 0U && joint.payload[2] == 0x55U && joint.payload[3] == rid);
        for (unsigned byte = 0U; byte < 4U; ++byte) assert(joint.payload[byte + 4U] == (uint8_t)(raw >> (byte * 8U)));
        if (step == 0U) joint.device_pmax_raw = raw; else joint.device_mode = raw;
        CanFrame reply = {23U, 8U, {7U, 0U, 0x55U, 0U}}; reply.data[3] = rid;
        for (unsigned byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (byte * 8U));
        FinishPair(&reply, early_reply); assert(!application_persistence.health_proven);
        assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == 12.5F);
        assert(application_motor_runtime.discovery.results[6].observed_control_mode == 1U);
    }
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARE_VERIFY);
    ReadPhase(MOTOR_PERSISTENCE_PREPARE_VERIFY, early_reply);
    assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARED);
}
/** @brief Exercise typed prepare, verify, postverify FD, unique AA, full30ms settle, readback and unchanged original journal. */
static void TestTyped(void)
{
    for (unsigned early_reply = 0U; early_reply < 2U; ++early_reply) {
        AethorPersistenceSaveJournal journal, later; AethorPersistenceStatus status;
        Fixture(); joint.trace_items = 1U; Health(); ReadPhase(MOTOR_PERSISTENCE_BASELINE, (uint8_t)early_reply); Prepare((uint8_t)early_reply);
        assert(aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
        assert(!aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        ReadPhase(MOTOR_PERSISTENCE_SAVE_VERIFY, (uint8_t)early_reply);
        assert(application_persistence.health.observed_us > application_persistence_transaction.verified_us);
        assert(aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        assert(!aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        Advance(1U); Control();
        assert(joint.header.Identifier == 0x7ffU && joint.header.DataLength == FDCAN_DLC_BYTES_4);
        assert(joint.payload[0] == 7U && joint.payload[1] == 0U && joint.payload[2] == 0xaaU && joint.payload[3] == 1U);
        assert(joint.save_calls == 1U && application_persistence_transaction.save_used && application_persistence_transaction.save_uncertain);
        CanFrame acknowledgement = {23U, 4U, {7U, 0U, 0xaaU, 1U}}; FinishPair(&acknowledgement, (uint8_t)early_reply);
        assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_SETTLE);
        assert(aethor_app_persistence_get_save_journal(91U, &journal) && journal.valid);
        assert(journal.tx.identifier == 0x7ffU && journal.tx.length == 4U && !memcmp(journal.tx.data, joint.payload, 4U));
        assert(journal.ack.identifier == 23U && journal.ack.length == 4U && !memcmp(journal.ack.data, acknowledgement.data, 4U));
        assert(journal.settle_until_us >= journal.receipt.received_us + 30000ULL);
        uint64_t settle_start = AethorMonotonicTimestampUs(); uint32_t count = joint.tx_calls;
        Advance(29U); Control(); assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_SETTLE && joint.tx_calls == count);
        Advance(1U); assert(AethorMonotonicTimestampUs() >= settle_start + 30000ULL);
        aethor_app_persistence_service(AethorMonotonicTimestampUs());
        assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_READBACK);
        ReadPhase(MOTOR_PERSISTENCE_READBACK, (uint8_t)early_reply);
        assert(application_persistence_transaction.current.items[0].created_us >= journal.settle_until_us);
        assert(aethor_app_persistence_get_save_journal(91U, &later));
        assert(!memcmp(&journal, &later, sizeof(journal)));
        assert(aethor_app_persistence_get_status(&status) && status.state == MOTOR_PERSISTENCE_COMPLETE);
        assert(status.save_used && !status.save_uncertain && status.locked && !status.fault && joint.save_calls == 1U);
        assert(!aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
        assert(!aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        printf("JOINT_SAVE_JOURNAL boot=%lu request=%lu owner=%lu sequence=%lu phase=%u created=%llu submitted=%llu boundary=%llu tx_observed=%llu rx_lower=%llu received=%llu settle_until=%llu\n",
            (unsigned long)journal.receipt.identity.boot_epoch, (unsigned long)journal.receipt.identity.request_id,
            (unsigned long)journal.receipt.identity.owner_generation, (unsigned long)journal.receipt.identity.sequence,
            journal.receipt.identity.phase, (unsigned long long)journal.receipt.created_us,
            (unsigned long long)journal.receipt.submitted_us, (unsigned long long)journal.receipt.boundary_us,
            (unsigned long long)journal.receipt.send_completed_observed_us, (unsigned long long)journal.receipt.rx_lower_us,
            (unsigned long long)journal.receipt.received_us, (unsigned long long)journal.settle_until_us);
        printf("JOINT_TYPED_SAVE_PASS early_reply=%u unique_aa=%lu settle_us=%llu original_journal_preserved=1\n",
            early_reply, (unsigned long)joint.save_calls, (unsigned long long)(journal.settle_until_us - journal.receipt.received_us));
    }
}
/** @brief Revoke every real final AA callback variant after complete typed verification; no bad-to-good event may retry Save. */
static void TestTypedReject(void)
{
    for (unsigned injection = 1U; injection <= 5U; ++injection) {
        Fixture(); Health(); ReadPhase(MOTOR_PERSISTENCE_BASELINE, 0U); Prepare(0U);
        assert(aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
        ReadPhase(MOTOR_PERSISTENCE_SAVE_VERIFY, 0U);
        assert(aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        uint32_t count = joint.tx_calls; joint.final_injection = injection; Advance(1U); Control();
        assert(joint.tx_calls == count && !joint.save_calls && application_persistence.fault);
        assert(!application_persistence_transaction.save_used && !application_persistence_transaction.save_uncertain);
        joint.link_word = 0x80000001UL; application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
        Control(); assert(joint.tx_calls == count && !joint.save_calls && aethor_app_persistence_locked());
        assert(!aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
        printf("JOINT_TYPED_NO_SAVE_AFTER_REVOCATION_PASS synthetic_event=%u\n", injection);
    }
    /* Losing the mapping after a real 55 submission forbids even a new FD until an exact ACK plus actual TX. */
    Fixture(); Health(); ReadPhase(MOTOR_PERSISTENCE_BASELINE, 0U);
    assert(aethor_app_persistence_prepare(91U, AethorMonotonicTimestampUs()));
    Advance(1U); Control(); assert(!application_persistence.pmax_known && platform_persistence.active);
    uint32_t count = joint.tx_calls; Complete(); CollectPersistenceReceipt(AethorMonotonicTimestampUs());
    CanFrame stale = Reply(21U); Receive(&stale, joint.counter);
    assert(!application_persistence.fault && application_persistence_transaction.pending &&
        !application_persistence.pmax_known);
    Advance(1U); Control(); assert(joint.tx_calls == count && !joint.save_calls);
    /* A matching write opcode with the old PMAX value still faults instead of confirming mapping. */
    stale.data[2] = 0x55U; Receive(&stale, joint.counter); assert(application_persistence.fault);
    Advance(1U); Control(); assert(joint.tx_calls == count && !joint.save_calls);
    assert(!aethor_app_persistence_prepare(91U, AethorMonotonicTimestampUs()));
    assert(!aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
    puts("JOINT_TYPED_UNKNOWN_MAPPING_WRONG_ACK_NO_SAVE_PASS");
}
/** @brief Exercise actual local and global STOP scopes through real protocol admission while transport is locked. */
static void TestStopScope(void)
{
    for (unsigned scenario = 0U; scenario < 3U; ++scenario) {
        ProtocolOutputBatch output; Fixture();
        if (scenario == 2U) application_debug_ui.mailbox.stop_intent_mask = 0x7fU;
        (void)request(scenario == 1U ? "1 stop" : "1 bench stop 1,3", AethorMonotonicTimestampUs(), &output);
        assert(application_debug_ui.mailbox.stop_intent_mask == (scenario ? 0x7fU : 0x45U));
        assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_STOP && !joint.save_calls);
    }
    puts("JOINT_STOP_LOCAL_UNION_AND_GLOBAL7F_PASS");
}
/** @brief A distinct and changing u_off cannot be substituted for actual p_m in any publication or Save gate. */
static void TestUoffIndependent(void)
{
    Fixture(); Health(); ReadPhase(MOTOR_PERSISTENCE_BASELINE, 0U);
    assert(joint.device_uoff != joint.device_position_rad && application_persistence.snapshot_ready);
    joint.device_uoff = 2.5F; Prepare(0U);
    assert(application_persistence.snapshot_ready && application_persistence_transaction.state == MOTOR_PERSISTENCE_PREPARED);
    assert(aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
    joint.device_uoff = 3.75F; ReadPhase(MOTOR_PERSISTENCE_SAVE_VERIFY, 0U);
    assert(application_persistence_transaction.current.items[37].raw == Register(37U));
    assert(application_persistence_transaction.current.items[43].raw == Register(43U));
    assert(aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
    assert(!joint.save_calls && !application_persistence.fault);
    puts("JOINT_UOFF_DIFFERS_NATIVE_BASELINE_PREPARE_SAVEVERIFY_PASS actual_p_m_rid=0x50 u_off_rid=0x32");
}
/** @brief Only true p_m changes in a real45 response stream; native FD and u_off remain stable and distinct. */
static void TestPositionJump(uint8_t save_verify)
{
    Fixture(); joint.device_uoff = 0.0001F; Health();
    if (save_verify) {
        ReadPhase(MOTOR_PERSISTENCE_BASELINE, 0U); Prepare(0U);
        assert(aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
    }
    uint8_t phase = save_verify ? MOTOR_PERSISTENCE_SAVE_VERIFY : MOTOR_PERSISTENCE_BASELINE;
    uint64_t total_deadline = application_persistence_transaction.total_deadline_us;
    joint.device_position_rad = 1.0F;
    ReadPhaseUnpublished(phase, 0U);
    uint64_t phase_deadline = application_persistence_transaction.phase_deadline_us;
    assert(application_persistence_transaction.current.items[43].valid &&
        application_persistence_transaction.current.items[43].identity.phase == phase);
    assert(application_persistence_transaction.current.items[43].raw == Register(43U));
    assert(application_persistence_transaction.current.items[37].raw == Register(37U));
    assert(joint.device_position_rad != joint.device_uoff && joint.device_uoff != 0.0F);
    Advance(1U); HealthExpected(0U);
    assert(application_persistence_transaction.total_deadline_us == total_deadline &&
        application_persistence_transaction.phase_deadline_us == phase_deadline);
    assert(!aethor_app_persistence_prepare(91U, AethorMonotonicTimestampUs()));
    assert(!aethor_app_persistence_request_save(91U, AethorMonotonicTimestampUs()));
    assert(!aethor_app_persistence_commit_save(91U, AethorMonotonicTimestampUs()));
    uint32_t count = joint.tx_calls; Control();
    assert(joint.tx_calls == count && !joint.save_calls && !application_persistence_transaction.save_used);
    assert(!application_persistence_transaction.save_uncertain && aethor_app_persistence_locked());
    printf("JOINT_ACTUAL_PM_JUMP_REJECT_PASS phase=%u rid=0x50 observed_through_real45=1 AA=0 retry=0\n", phase);
}
/** @brief Run independent process-local scenarios and report only verified joint offline scope. */
int main(int argc, char **argv)
{
    assert(!setvbuf(stdout, NULL, _IONBF, 0U));
    const char *selected = argc > 1 ? argv[1] : "all";
    if (!strcmp(selected, "readonly") || !strcmp(selected, "all")) TestReadonly();
    if (!strcmp(selected, "deferred") || !strcmp(selected, "all")) TestDeferred();
    if (!strcmp(selected, "debt") || !strcmp(selected, "all")) TestDebt();
    if (!strcmp(selected, "final") || !strcmp(selected, "all")) TestFinal();
    if (!strcmp(selected, "mutex") || !strcmp(selected, "all")) TestMutex();
    if (!strcmp(selected, "health_reject") || !strcmp(selected, "all")) TestHealthReject();
    if (!strcmp(selected, "health_same_tick") || !strcmp(selected, "all")) TestHealthSameTick();
    if (!strcmp(selected, "typed") || !strcmp(selected, "all")) TestTyped();
    if (!strcmp(selected, "typed_reject") || !strcmp(selected, "all")) TestTypedReject();
    if (!strcmp(selected, "stop_scope") || !strcmp(selected, "all")) TestStopScope();
    if (!strcmp(selected, "pm_uoff") || !strcmp(selected, "all")) TestUoffIndependent();
    if (!strcmp(selected, "pm_baseline") || !strcmp(selected, "all")) TestPositionJump(0U);
    if (!strcmp(selected, "pm_saveverify") || !strcmp(selected, "all")) TestPositionJump(1U);
    puts("PERSISTENCE_JOINT_PLATFORM_PASS no_physical_hardware=1 no_eeprom_qualification=1"); return 0;
}
