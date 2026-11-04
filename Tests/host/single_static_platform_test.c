/** @file single_static_platform_test.c
 * @brief Actual facade/platform/Core-bridge ordering against hardware metadata stubs. No serial, probe, board or flash.
 */
#define main existing_single_platform_app_fixture_main
#include "debug_ui_app_test_main.c"
#undef main
#include "../../App/Platform/stm32_platform.c"

static FDCAN_GlobalTypeDef fake_registers;
FDCAN_HandleTypeDef hfdcan1;
uint32_t single_fake_irq_mask;
static uint32_t fake_tick, fake_kernel_hz, fake_tx_count, fake_config_calls, fake_abort_calls;
static uint32_t fake_tx_free_level = 7U;
static uint8_t fake_rx1_available;
static uint16_t fake_counter;
static uint8_t fake_rx_available, fake_event_available, fake_abort_clears, fake_timestamp_failure;
static FDCAN_RxHeaderTypeDef fake_rx_header;
static uint8_t fake_rx_payload[8];
static FDCAN_TxHeaderTypeDef fake_last_tx;
static FDCAN_TxEventFifoTypeDef fake_event;
static uint8_t fake_expect_pop_capture;
static uint8_t fake_startup_tx, fake_hal_failure;
static uint8_t fake_strict_delay_pending, fake_strict_keep_bto, fake_strict_keep_bcf;
static uint32_t fake_strict_latest_buffer = 0x100U;
static uint16_t fake_expected_pop_timestamp;

/** @brief The sidecar and frame must both be copied while the adapter still holds the IRQ mask. */
void single_fake_restore_irq(uint32_t value)
{
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    if (fake_expect_pop_capture) {
        assert(single_fake_irq_mask == 1U && platform_single_static_last_rx.timestamp == fake_expected_pop_timestamp);
        assert(platform_single_static_last_frame.length == 8U && platform_single_static_last_frame.data[0] == 7U);
        fake_expect_pop_capture = 0U;
    }
#else
    (void)fake_expect_pop_capture; (void)fake_expected_pop_timestamp;
#endif
    single_fake_irq_mask = value;
}

/** @brief Successful metadata-only HAL setup leaves all actual traffic under test control. */
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *handle, const FDCAN_FilterTypeDef *filter) { (void)handle; (void)filter; return HAL_OK; }
/** @brief Preserve the existing global filter call. */
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle, uint32_t a, uint32_t b, uint32_t c, uint32_t d) { (void)handle; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
/** @brief Preserve the existing FIFO watermark call. */
HAL_StatusTypeDef HAL_FDCAN_ConfigFifoWatermark(FDCAN_HandleTypeDef *handle, uint32_t fifo, uint32_t value) { (void)handle; (void)fifo; (void)value; return HAL_OK; }
/** @brief Record no implicit traffic on start. */
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
/** @brief Stop stub never clears an unresolved hardware buffer. */
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle) { (void)handle; return HAL_OK; }
/** @brief Preserve notification configuration without scheduling tasks. */
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle, uint32_t mask, uint32_t buffer) { (void)handle; (void)mask; (void)buffer; return HAL_OK; }
/** @brief HAL submission proves only a new pending buffer; success and event appear separately. */
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle, const FDCAN_TxHeaderTypeDef *header, const uint8_t *payload)
{ (void)handle; if (!fake_startup_tx) assert(payload[7] == 0xFDU); if (fake_hal_failure) return HAL_ERROR; fake_last_tx = *header; ++fake_tx_count;
  if (!fake_strict_delay_pending) fake_registers.TXBRP |= 0x100U;
  if (!fake_strict_keep_bto) fake_registers.TXBTO &= ~0x100U;
  if (!fake_strict_keep_bcf) fake_registers.TXBCF &= ~0x100U;
  return HAL_OK; }
/** @brief There is hardware FIFO capacity unless the pinned entry itself remains pending. */
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *handle) { (void)handle; return fake_tx_free_level; }
/** @brief Expose the raw hardware backlog before ISR decoding. */
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *handle, uint32_t fifo) { (void)handle; return fifo == FDCAN_RX_FIFO0 ? fake_rx_available : fake_rx1_available; }
/** @brief Pin the exact FIFO buffer returned by the latest submission. */
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle) { (void)handle; return fake_strict_latest_buffer; }
/** @brief An explicit abort may remain pending; the production tracker must retain that uncertainty. */
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t mask)
{ (void)handle; ++fake_abort_calls; if (fake_abort_clears) { fake_registers.TXBRP &= ~mask; fake_registers.TXBCF |= mask; } return HAL_OK; }
/** @brief Return immutable SOF metadata regardless of delayed ISR entry. */
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *payload)
{ (void)handle; (void)fifo; if (!fake_rx_available) return HAL_ERROR; *header = fake_rx_header; memcpy(payload, fake_rx_payload, 8U); fake_rx_available = 0U; return HAL_OK; }
/** @brief Timestamp setup is opt-in and can independently fail closed. */
HAL_StatusTypeDef HAL_FDCAN_ConfigTimestampCounter(FDCAN_HandleTypeDef *handle, uint32_t prescaler)
{ (void)handle; assert(prescaler == FDCAN_TIMESTAMP_PRESC_16); ++fake_config_calls; return fake_timestamp_failure ? HAL_ERROR : HAL_OK; }
/** @brief Only the internal shared RX/TX timestamp domain is accepted. */
HAL_StatusTypeDef HAL_FDCAN_EnableTimestampCounter(FDCAN_HandleTypeDef *handle, uint32_t operation) { (void)handle; assert(operation == FDCAN_TIMESTAMP_INTERNAL); return HAL_OK; }
/** @brief Hardware time is distinct from the HAL millisecond observation clock. */
uint16_t HAL_FDCAN_GetTimestampCounter(const FDCAN_HandleTypeDef *handle) { (void)handle; return fake_counter; }
/** @brief Pop one successful TX event with its original SOF timestamp and marker. */
HAL_StatusTypeDef HAL_FDCAN_GetTxEvent(FDCAN_HandleTypeDef *handle, FDCAN_TxEventFifoTypeDef *event)
{ (void)handle; if (!fake_event_available) return HAL_ERROR; *event = fake_event; fake_event_available = 0U; fake_registers.TXEFS &= ~FDCAN_TXEFS_EFFL; return HAL_OK; }
/** @brief Actual configured kernel frequency feeds the half-wrap proof. */
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t peripheral) { assert(peripheral == RCC_PERIPHCLK_FDCAN); return fake_kernel_hz; }
/** @brief Milliseconds bound action lifetime but never determine RX/TX fine ordering. */
uint32_t HAL_GetTick(void) { return fake_tick; }
/** @brief USB transmission remains an unused stub. */
uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length) { (void)data; (void)length; return USBD_OK; }

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Synthetic monotonic clock for the maintained Core receipt observer. */
static uint64_t AethorMonotonicTimestampUs(void) { return (uint64_t)fake_tick * 1000ULL; }
#include "core_receive_bridge.inc"
/** @brief Initialize real application discovery/profile, platform and one reserved local preflight transaction. */
static DebugUiIdentity platform_reserve(uint16_t anchor, unsigned unavailable)
{
    DebugUiMotorProfile profile;
    DebugUiSnapshot snapshot;
    DebugUiRequest request = {0};
    DebugUiAdmission admission;
    DebugUiIdentity identity;
    CanFrame frame;
    CanTxPriority priority;
    test_fixture();
    profile = application_debug_ui.profiles[0];
    application_motor_runtime.discovery.results[6] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    application_motor_runtime.discovery.results[6].observed_master_id = application_motor_runtime.configuration->joints[6].master_id;
    application_motor_runtime.discovery.results[6].observed_esc_id = 7U;
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    assert(aethor_app_debug_ui_set_motor_profile(7U, &profile));
    memset(&fake_registers, 0, sizeof(fake_registers)); memset(&hfdcan1, 0, sizeof(hfdcan1));
    hfdcan1.Instance = &fake_registers; hfdcan1.Init.StdFiltersNbr = 7U; hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
    hfdcan1.Init.TxEventsNbr = 8U; hfdcan1.Init.NominalPrescaler = 1U; hfdcan1.Init.NominalTimeSeg1 = 59U; hfdcan1.Init.NominalTimeSeg2 = 20U;
    fake_tick = 3U; fake_counter = anchor; fake_kernel_hz = 80000000U;
    fake_tx_count = fake_abort_calls = 0U; fake_rx_available = fake_event_available = 0U; fake_timestamp_failure = fake_hal_failure = fake_startup_tx = 0U;
    fake_abort_clears = single_fake_irq_mask = 0U;
    fake_strict_delay_pending = fake_strict_keep_bto = fake_strict_keep_bcf = 0U; fake_strict_latest_buffer = 0x100U;
    if (unavailable == 1U) fake_timestamp_failure = 1U;
    if (unavailable == 2U) hfdcan1.Init.TxEventsNbr = 0U;
    assert(stm32_platform_init(application_motor_runtime.configuration) == STM32_PLATFORM_STATUS_OK);
    test_health(2000ULL, true);
    assert(aethor_app_debug_ui_get_snapshot(2000ULL, &snapshot));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI; request.identity.epoch = snapshot.epoch; request.identity.request_id = 1U;
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT; request.target_motor_id = 7U;
    request.requested_mode = DEBUG_UI_MODE_MIT; request.created_at_us = 2000ULL;
    request.reference_generation = snapshot.motors[6].reference_generation;
    request.single_static_evidence_generation = snapshot.motors[6].single_static_evidence_generation;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(2000ULL);
    assert(aethor_app_debug_ui_poll_admission(&admission) && admission.accepted);
    (void)aethor_app_service(2000ULL);
    assert(aethor_app_next_can_frame(3000ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_debug_ui_preflight_frame_identity(&frame, &identity));
    if (unavailable == 3U) fake_kernel_hz = 160000000U;
    if (unavailable == 4U) fake_kernel_hz = 0U;
    if (unavailable == 5U) hfdcan1.Init.NominalPrescaler = 0U;
    if (unavailable == 6U) hfdcan1.Init.NominalPrescaler = 0x20000001U;
    assert((stm32_platform_can_submit_single_static_preflight(&frame, identity.epoch, identity.request_id,
        aethor_app_debug_ui_preflight_timeout_us(), aethor_app_debug_ui_preflight_submitted) != 0U) == (unavailable == 0U));
    assert(fake_tx_count == 0U);
    return identity;
}
/** @brief Submit the reserved request once through the actual HAL adapter. */
static DebugUiIdentity platform_prepare(uint16_t anchor)
{
    DebugUiIdentity identity = platform_reserve(anchor, 0U);
    assert(stm32_platform_can_service_tx(7U) == STM32_PLATFORM_STATUS_OK);
    assert(fake_tx_count == 1U && fake_last_tx.TxEventFifoControl == FDCAN_STORE_TX_EVENTS && fake_last_tx.MessageMarker);
    return identity;
}
/** @brief Model the controller's successful wire transmission and immutable TX event separately from task polling. */
static void platform_complete_tx(uint16_t transmitted)
{
    memset(&fake_event, 0, sizeof(fake_event));
    fake_event.Identifier = fake_last_tx.Identifier; fake_event.IdType = FDCAN_STANDARD_ID;
    fake_event.TxFrameType = FDCAN_DATA_FRAME; fake_event.DataLength = 8U;
    fake_event.FDFormat = FDCAN_CLASSIC_CAN; fake_event.BitRateSwitch = FDCAN_BRS_OFF;
    fake_event.MessageMarker = fake_last_tx.MessageMarker; fake_event.EventType = FDCAN_TX_EVENT;
    fake_event.TxTimestamp = transmitted; fake_event_available = 1U; fake_registers.TXEFS = 1U;
    fake_registers.TXBRP &= ~0x100U; fake_registers.TXBTO |= 0x100U;
}
/** @brief Queue raw hardware feedback with SOF predating or following TX, without assigning decode time to it. */
static void platform_raw_feedback(uint16_t received)
{
    CanFrame encoded;
    assert(s3519_pack_mit(7U, &application_motor_runtime.discovery.results[6].ranges,
        0.26F, 0.0F, 0.0F, 0.0F, 0.0F, &encoded) == S3519_CODEC_STATUS_OK);
    memset(&fake_rx_header, 0, sizeof(fake_rx_header));
    fake_rx_header.Identifier = application_motor_runtime.configuration->joints[6].master_id;
    fake_rx_header.IdType = FDCAN_STANDARD_ID; fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_8; fake_rx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fake_rx_header.BitRateSwitch = FDCAN_BRS_OFF; fake_rx_header.RxTimestamp = received;
    fake_rx_payload[0] = 7U; fake_rx_payload[1] = encoded.data[0]; fake_rx_payload[2] = encoded.data[1]; fake_rx_payload[3] = encoded.data[2];
    fake_rx_payload[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    fake_rx_payload[5] = encoded.data[7]; fake_rx_payload[6] = 30U; fake_rx_payload[7] = 31U;
    fake_rx_available = 1U;
}
/** @brief Execute the maintained ISR queue and exact extracted Core receive bridge. */
static void platform_decode(void)
{
    CanFrame frame;
    uint64_t lower;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
    assert(single_fake_irq_mask == 0U);
    ReceiveSingleStaticFrame(&frame, lower, (uint64_t)fake_tick * 1000ULL);
}
/** @brief Pending accepted RX, delayed inbox/raw FIFO RX and same-bin RX never qualify; fast post-wire and wrap RX do. */
static void platform_wire_order_cases(void)
{
    unsigned condition;
    for (condition = 0U; condition < 16U; ++condition) {
        DebugUiCompletion completion;
        uint64_t timestamp_us;
        uint8_t success = (uint8_t)(condition == 3U || condition == 4U);
        uint16_t anchor = condition == 4U ? 65520U : 100U;
        (void)platform_prepare(anchor);
        if (condition <= 2U) {
            platform_raw_feedback((uint16_t)(anchor + 2U)); fake_counter = (uint16_t)(anchor + 3U);
            if (condition == 0U) platform_decode();
            if (condition == 1U) HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        }
        platform_complete_tx((uint16_t)(anchor + 10U)); fake_counter = (uint16_t)(anchor + 32U);
        if (condition >= 3U) platform_raw_feedback((uint16_t)(anchor + (condition == 5U ? 10U : 20U)));
        if (condition == 6U) ++fake_event.MessageMarker;
        if (condition == 7U) fake_registers.IR |= FDCAN_IR_TEFL;
        if (condition == 8U) ++fake_event.Identifier;
        if (condition == 9U) fake_event.DataLength = FDCAN_DLC_BYTES_8;
        if (condition == 10U) fake_event.EventType = 0x00800000U;
        if (condition == 11U) fake_event.TxTimestamp = (uint16_t)(anchor + 40U);
        if (condition == 12U) fake_rx_header.RxTimestamp = (uint16_t)(anchor + 40U);
        if (condition == 13U) fake_counter = (uint16_t)(anchor + 32768U);
        if (condition == 14U) fake_rx_header.RxTimestamp = (uint16_t)(anchor - 1U);
        if (condition == 15U) fake_tick = 503U;
        if (condition == 0U) CollectSingleStaticReceipt();
        else if (condition == 1U) {
            CanFrame frame; uint64_t lower;
            assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
            ReceiveSingleStaticFrame(&frame, lower, 4000ULL);
        } else platform_decode();
        for (timestamp_us = 4000ULL; timestamp_us <= 506000ULL && application_action.state != AETHOR_APP_ACTION_IDLE; timestamp_us += 4000ULL) {
            fake_tick = (uint32_t)(timestamp_us / 1000ULL); test_health(timestamp_us, true);
            (void)aethor_app_service(timestamp_us); aethor_app_debug_ui_process(timestamp_us);
        }
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert((completion.code == DEBUG_UI_COMPLETED) == success);
        assert(completion.disabled_confirmed == success && completion.motor_mask == 0x40U);
        assert(fake_tx_count == 1U && !application_debug_ui.single_static_lcd_owned[6]);
    }
    puts("SINGLE_STATIC_PLATFORM_WIRE_ORDER_PASS cases=16 pending_accepted_rejected=1 delayed_inbox_and_raw_backlog_rejected=1 fast_post_tx_before_poll=1 wrap=1 same_bin_marker_id_dlc_event_loss_future_half_wrap_deadline_rejected=1 frames_each=1");
}
/** @brief Old events are drained at actual HAL submission; reservation polling must not consume or poison them. */
static void platform_old_event_and_marker_wrap(void)
{
    CanFrame frame; uint64_t lower;
    uint32_t epoch, request_id; uint8_t succeeded;
    (void)platform_reserve(100U, 0U);
    memset(&fake_event, 0, sizeof(fake_event));
    fake_event.MessageMarker = 77U; fake_event.TxTimestamp = 90U;
    fake_event_available = 1U; fake_registers.TXEFS = 1U;
    assert(!stm32_platform_can_collect_single_static_receipt(&epoch, &request_id, &succeeded));
    assert(fake_event_available && !platform_single_static.wire_invalid && !fake_tx_count);
    platform_single_static_next_marker = 255U;
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
    assert(!fake_event_available && fake_tx_count == 1U && !platform_single_static.wire_invalid);
    platform_complete_tx(110U); fake_counter = 132U; platform_raw_feedback(120U);
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
    ReceiveSingleStaticFrame(&frame, lower, 4000ULL);
    assert(application_action.single_static_tx_confirmed && application_action.single_static_ordered_generation);
    /* Exercise the actual next reservation, whose marker crosses 255 to nonzero 1. */
    assert(!platform_single_static.active);
    frame = platform_single_static.frame;
    assert(stm32_platform_can_submit_single_static_preflight(&frame, 88U, 99U,
        aethor_app_debug_ui_preflight_timeout_us(), aethor_app_debug_ui_preflight_submitted));
    assert(platform_single_static.wire.marker == 1U);
    puts("SINGLE_STATIC_PLATFORM_OLD_EVENT_PASS drained_at_hal_submit=1 no_early_poll=1 marker_wrap_nonzero=1");
}
/** @brief Reject unavailable or ambiguous hardware timing without emitting preflight or affecting ordinary traffic. */
static void platform_window_unavailable(void)
{
    unsigned unavailable;
    assert(debug_ui_single_static_wire_window(80000000U, 80U, 500000ULL));
    assert(!debug_ui_single_static_wire_window(83886080U, 80U, 500000ULL));
    for (unavailable = 1U; unavailable <= 6U; ++unavailable) {
        CanFrame frame; uint8_t payload[8] = {255U,255U,255U,255U,255U,255U,255U,253U};
        (void)platform_reserve(100U, unavailable);
        assert(!platform_can_tx_scheduler.count && !fake_tx_count && !platform_single_static.active);
        assert(can_frame_init(&frame, 7U, payload, 8U) == CAN_FRAME_STATUS_OK);
        assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &frame) == CAN_TX_SCHEDULER_STATUS_OK);
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        assert(fake_tx_count == 1U && fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS && !fake_last_tx.MessageMarker);
    }
    puts("SINGLE_STATIC_PLATFORM_WINDOW_PASS invalid_cases=6 original_timeout_us=500000 exact_half_wrap_rejected=1 ordinary_frames_unchanged=1");
}
/** @brief Qualification belongs to one accepted selected sample; unrelated/rejected/new unproved feedback cannot borrow it. */
static void platform_sample_qualification(void)
{
    uint32_t selected_generation, ordered_generation;
    MotorFeedbackSnapshot snapshot;
    CanFrame frame; uint64_t lower;
    (void)platform_prepare(100U); platform_complete_tx(110U); fake_counter = 132U;
    selected_generation = application_motor_runtime.bank.motors[6].feedback_generation;
    platform_raw_feedback(120U);
    fake_rx_payload[0] = 1U; fake_rx_header.Identifier = application_motor_runtime.configuration->joints[0].master_id;
    platform_decode();
    assert(application_motor_runtime.bank.motors[6].feedback_generation == selected_generation &&
        !application_action.single_static_ordered_generation);
    platform_raw_feedback(121U); fake_rx_payload[0] = 0U;
    platform_decode();
    assert(application_motor_runtime.bank.motors[6].feedback_generation == selected_generation &&
        !application_action.single_static_ordered_generation);
    application_motor_runtime.bank.motors[6].feedback_generation = 0xFFFFFFFFU;
    platform_raw_feedback(122U); platform_decode();
    ordered_generation = application_action.single_static_ordered_generation;
    assert(ordered_generation == 1U && application_motor_runtime.bank.motors[6].feedback_generation == 1U);
    fake_tick = 4U;
    stm32_platform_can_observe_rx_boundary(4000ULL);
    platform_raw_feedback(102U);
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
    ReceiveSingleStaticFrame(&frame, lower, 4000ULL);
    assert(application_action.single_static_ordered_generation == ordered_generation &&
        application_motor_runtime.bank.motors[6].feedback_generation != ordered_generation);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_single_static_preflight_validate(&application_action.command, &snapshot, 4000ULL, true)
        == DEBUG_UI_REASON_STALE_FEEDBACK);
    assert(fake_tx_count == 1U);
    puts("SINGLE_STATIC_PLATFORM_SAMPLE_PASS other_id_and_rejected_do_not_qualify=1 counter_wrap_nonzero=1 later_unproved_sample_revokes_qualification=1");
}
/** @brief Ring-slot metadata is copied with its frame while IRQs are masked, including index wrap and a pre-masked caller. */
static void platform_sidecar_slot_lifetime(void)
{
    unsigned slot;
    (void)platform_prepare(100U);
    for (slot = 0U; slot < CAN_RX_INBOX_CAPACITY + 4U; ++slot) {
        CanFrame frame; uint64_t lower;
        platform_raw_feedback((uint16_t)(105U + slot));
        HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
        single_fake_irq_mask = (slot & 1U);
        fake_expected_pop_timestamp = (uint16_t)(105U + slot); fake_expect_pop_capture = 1U;
        assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
        assert(!fake_expect_pop_capture);
        assert(single_fake_irq_mask == (slot & 1U));
        assert(platform_single_static_last_rx.timestamp == (uint16_t)(105U + slot) &&
            platform_single_static_last_rx.epoch == platform_single_static.epoch &&
            platform_single_static_last_rx.request_id == platform_single_static.request_id);
        assert(!memcmp(&frame, &platform_single_static_last_frame, sizeof(frame)));
    }
    single_fake_irq_mask = 0U;
    puts("SINGLE_STATIC_PLATFORM_SIDECAR_PASS ring_wrap=1 frame_and_sof_copied_before_irq_restore=1 prior_irq_mask_preserved=1");
}
/** @brief Explicit STOP requests abort once but retain unknown pending DISABLE debt until cancellation is actually observed. */
static void platform_cancel_pending_stop(void)
{
    CanFrame stop_frame;
    DebugUiRequest stop_request;
    CanTxPriority priority;
    uint8_t abort_status;
    (void)platform_prepare(100U);
    stop_request = test_request(DEBUG_UI_OPERATION_STOP, 4000ULL);
    stop_request.identity.request_id = 2U; stop_request.target_motor_id = 7U;
    assert(aethor_app_debug_ui_submit(&stop_request) == DEBUG_UI_REASON_NONE);
    test_health(4000ULL, true); aethor_app_debug_ui_process(4000ULL); (void)aethor_app_service(4000ULL);
    assert(aethor_app_next_can_frame(4000ULL, &stop_frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY && stop_frame.data[7] == 0xFDU);
    assert(stm32_platform_can_submit(priority, &stop_frame) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(fake_abort_calls == 1U && platform_single_static.cancellation_requested && fake_tx_count == 1U);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    CollectSingleStaticReceipt();
    assert(platform_single_static.active && application_debug_ui.single_static_transport_mask == 0x40U);
    fake_tick = 600U; test_health(600000ULL, true); (void)aethor_app_service(600000ULL);
    assert(platform_single_static.active && application_debug_ui.single_static_transport_mask == 0x40U);
    fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
    CollectSingleStaticReceipt();
    abort_status = platform_single_static.terminal_succeeded;
    assert(!platform_single_static.active && !abort_status && !application_debug_ui.single_static_transport_mask);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 2U);
    assert(fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS && !fake_last_tx.MessageMarker && fake_abort_calls == 1U);
    platform_raw_feedback(120U); fake_counter = 132U; platform_decode();
    assert(!application_action.single_static_ordered_generation);
    puts("SINGLE_STATIC_PLATFORM_STOP_PASS software_stop_preempts=1 abort_once=1 pending_debt_survives_timeout=1 confirmed_cancel_releases_stop_frame=1 no_old_handoff=1");
}
#endif

#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
/** @brief Transport-only callback records the exact generic phase identity; application guards have separate facade tests. */
static uint8_t platform_generic_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    assert(identity && identity->epoch == 71U && identity->request_id == 73U && identity->sequence);
    assert(identity->motor_id == 7U && identity->phase >= DEBUG_UI_STATIC_TX_PRELOAD && boundary_us == 3000ULL);
    return 1U;
}
/** @brief Reset deterministic HAL metadata only; no command, port, probe or board is accessed. */
static void platform_generic_fixture(void)
{
    test_fixture();
    memset(&fake_registers, 0, sizeof(fake_registers)); memset(&hfdcan1, 0, sizeof(hfdcan1));
    hfdcan1.Instance = &fake_registers; hfdcan1.Init.StdFiltersNbr = 7U; hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
    hfdcan1.Init.TxEventsNbr = 8U; hfdcan1.Init.NominalPrescaler = 1U;
    hfdcan1.Init.NominalTimeSeg1 = 59U; hfdcan1.Init.NominalTimeSeg2 = 20U;
    fake_tick = 3U; fake_counter = 100U; fake_kernel_hz = 80000000U;
    fake_tx_count = fake_abort_calls = 0U; fake_rx_available = fake_event_available = 0U;
    fake_abort_clears = fake_timestamp_failure = fake_hal_failure = fake_expect_pop_capture = single_fake_irq_mask = 0U;
    fake_startup_tx = 1U;
    fake_strict_delay_pending = fake_strict_keep_bto = fake_strict_keep_bcf = 0U; fake_strict_latest_buffer = 0x100U;
    assert(stm32_platform_init(application_motor_runtime.configuration) == STM32_PLATFORM_STATUS_OK);
}
/** @brief Every generic phase has real TX proof, queued STOP cancellation, pending debt and HAL failure coverage. */
static void platform_generic_phase_cases(void)
{
    uint8_t phase;
    unsigned scenario;
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_TAKEOVER; ++phase)
        if (phase != DEBUG_UI_STATIC_TX_POS_DISABLE &&
            (phase != DEBUG_UI_STATIC_TX_TAKEOVER || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD))
        for (scenario = 0U; scenario < 4U; ++scenario) {
            DebugUiStaticTxIdentity identity = {71U, 73U, (uint32_t)(phase * 10U + scenario + 1U), 7U, phase}, received;
            CanFrame frame, emergency;
            uint8_t succeeded;
            uint16_t sof;
            platform_generic_fixture();
            memset(&frame, 0, sizeof(frame)); frame.identifier = 7U; frame.length = 8U;
            frame.data[0] = 0x9BU; frame.data[7] = 0x65U;
            if (phase == DEBUG_UI_STATIC_TX_ENABLE) {
                memset(frame.data, 0xFFU, 8U); frame.data[7] = 0xFCU;
            }
            assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT, S3519_MODE_COMMAND_DISABLE, &emergency) == S3519_CODEC_STATUS_OK);
            assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
            assert(!stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
            if (scenario == 0U) {
                assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
                fake_registers.TXBRP = 0U; fake_registers.TXBTO = 0x100U;
                fake_event.Identifier = 7U; fake_event.IdType = FDCAN_STANDARD_ID;
                fake_event.TxFrameType = FDCAN_DATA_FRAME; fake_event.FDFormat = FDCAN_CLASSIC_CAN;
                fake_event.BitRateSwitch = FDCAN_BRS_OFF; fake_event.DataLength = 8U;
                fake_event.EventType = FDCAN_TX_EVENT; fake_event.MessageMarker = fake_last_tx.MessageMarker;
                fake_event.TxTimestamp = 110U; fake_event_available = 1U;
                fake_registers.TXEFS = FDCAN_TXEFS_EFFL; fake_counter = 120U;
                assert(stm32_platform_can_collect_static(&received, &succeeded, &sof));
                assert(succeeded && sof == 110U && debug_ui_static_tx_identity_equal(&identity, &received));
                assert(!stm32_platform_can_collect_static(&received, &succeeded, &sof));
            } else if (scenario == 1U) {
                /* Emergency before HAL arm removes the queued strict bytes permanently. */
                assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
                assert(stm32_platform_can_collect_static(&received, &succeeded, &sof) && !succeeded);
                assert(stm32_platform_can_service_tx(2U) == STM32_PLATFORM_STATUS_OK);
                assert(fake_tx_count == 1U && fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS);
                assert(platform_can_tx_scheduler.count == 0U && !platform_single_static.active);
            } else if (scenario == 2U) {
                assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
                assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
                fake_tick = 600U;
                assert(!stm32_platform_can_collect_static(&received, &succeeded, &sof));
                assert(platform_single_static.active && fake_abort_calls == 1U);
                assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
                fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
                assert(stm32_platform_can_collect_static(&received, &succeeded, &sof) && !succeeded);
                assert(debug_ui_static_tx_identity_equal(&identity, &received));
            } else {
                fake_hal_failure = 1U;
                assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_CAN_TX_ERROR);
                assert(stm32_platform_can_collect_static(&received, &succeeded, &sof) && !succeeded);
                assert(!fake_registers.TXBRP && !platform_single_static.active);
            }
        }
    fake_startup_tx = fake_hal_failure = 0U;
    puts("STATIC_STARTUP_PLATFORM_PASS phases=3 cases=12 actualtx_identity_queued_stop_pending_debt_hal_error=1");
}
#endif

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Real HAL acceptance before TX scan visibility retains ownership through STOP and delayed cancellation. */
static void platform_strict_scan_delay(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 8U; ++scenario) {
        DebugUiStaticTxIdentity identity = {71U, 73U, scenario + 1U, 7U, DEBUG_UI_STATIC_TX_PRELOAD}, receipt;
        CanFrame frame, emergency; uint8_t succeeded; uint16_t sof;
        platform_generic_fixture();
        memset(&frame, 0, sizeof(frame)); frame.identifier = 7U; frame.length = 8U; frame.data[7] = 0x65U;
        assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT, S3519_MODE_COMMAND_DISABLE, &emergency) == S3519_CODEC_STATUS_OK);
        fake_strict_delay_pending = 1U;
        if (scenario == 4U) fake_strict_latest_buffer = 0U;
        if (scenario == 5U) fake_strict_latest_buffer = 0x300U;
        if (scenario == 6U) { fake_strict_keep_bto = 1U; fake_registers.TXBTO = 0x100U; }
        if (scenario == 7U) { fake_strict_keep_bcf = 1U; fake_registers.TXBCF = 0x100U; }
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        if (scenario == 4U || scenario == 5U) {
            assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
            assert(debug_ui_static_tx_identity_equal(&receipt, &identity));
        } else assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(platform_single_static.active && !fake_abort_calls);
        assert(!stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
        assert(!stm32_platform_can_is_idle());
        if (scenario == 6U || scenario == 7U) {
            /* Old BTO/BCF alone cannot release; a completely matching fresh event proves termination but cannot grant success. */
            platform_complete_tx(110U); fake_counter = 132U;
            assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
            assert(!platform_single_static.active); continue;
        }
        if (scenario == 1U) assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
        if (scenario == 2U) fake_tick = 503U;
        if (scenario == 3U) platform_can_rx_inbox.bus_off_latched = 1U;
        if (scenario >= 1U && scenario <= 3U) {
            assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
            assert(debug_ui_static_tx_identity_equal(&receipt, &identity));
        }
        assert(platform_single_static.active && !fake_abort_calls);
        assert(debug_ui_static_tx_identity_equal(&platform_single_static.identity, &identity));
        assert(!memcmp(&platform_single_static.frame, &frame, sizeof(frame)));
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        if (scenario != 0U) {
            CanFrame controls[ARM_JOINT_COUNT]; unsigned joint;
            for (joint = 0U; joint < ARM_JOINT_COUNT; ++joint) { controls[joint] = frame; controls[joint].identifier = joint + 1U; }
            assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &frame) == CAN_TX_SCHEDULER_STATUS_FULL);
            assert(stm32_platform_can_submit_control_group(controls, ARM_JOINT_COUNT) == CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED);
        }
        assert(stm32_platform_can_service_tx(1U) == (scenario == 3U ? STM32_PLATFORM_STATUS_CAN_BUS_OFF : STM32_PLATFORM_STATUS_OK));
        assert(fake_tx_count == 1U);
        fake_registers.TXBRP = 0x100U;
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(fake_abort_calls == (scenario != 0U));
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(fake_abort_calls == (scenario != 0U));
        fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
        if (scenario == 0U) {
            fake_registers.TXBCF = 0U; platform_complete_tx(110U); fake_counter = 132U;
            assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded && sof == 110U);
        } else {
            assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
            if (scenario == 4U || scenario == 5U) {
                assert(platform_single_static.active); /* Unknown accepted buffer needs independent fresh terminal proof. */
                platform_complete_tx(110U); fake_counter = 132U;
                assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
            }
        }
        assert(!platform_single_static.active);
        if (scenario == 1U) {
            fake_strict_delay_pending = 0U;
            assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 2U);
            assert(fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS && fake_last_tx.Identifier == 7U);
        }
    }
    fake_strict_delay_pending = fake_strict_keep_bto = fake_strict_keep_bcf = 0U; fake_strict_latest_buffer = 0x100U;
    puts("STATIC_STRICT_SCAN_DELAY_PASS cases=8 accepted_zero_retains=1 first_pending_stop_timeout_busoff=3 abort_once=1 unknown_latest=2 old_status_only_not_terminal=2 clean_pending_BTO_SOF_success=1");
}

/** @brief Wrong marker/ID/DLC/timestamp/event-loss never turns unobserved pending into a released or successful slot. */
static void platform_strict_event_without_pending(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 12U; ++scenario) {
        DebugUiStaticTxIdentity identity = {71U, 73U, scenario + 1U, 7U, DEBUG_UI_STATIC_TX_PRELOAD}, receipt;
        CanFrame frame; uint8_t succeeded; uint16_t sof;
        platform_generic_fixture(); memset(&frame, 0, sizeof(frame)); frame.identifier = 7U; frame.length = 8U;
        fake_strict_delay_pending = 1U;
        if (scenario >= 6U) fake_strict_latest_buffer = 0U;
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        platform_complete_tx(110U); fake_counter = 132U;
        if (scenario % 6U == 1U) ++fake_event.MessageMarker;
        if (scenario % 6U == 2U) ++fake_event.Identifier;
        if (scenario % 6U == 3U) fake_event.DataLength = 4U;
        if (scenario % 6U == 4U) fake_event.TxTimestamp = 140U;
        if (scenario % 6U == 5U) fake_registers.IR = FDCAN_IR_TEFL;
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
        assert(platform_single_static.active == (scenario % 6U != 0U));
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert((stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted) == 0U) == (scenario % 6U != 0U));
    }
    fake_strict_delay_pending = 0U; fake_strict_latest_buffer = 0x100U;
    puts("STATIC_STRICT_EVENT_WITHOUT_PENDING_PASS known_and_unknown_buffer=2 fresh_match_failed_terminal=2 invalid_metadata_or_loss_keeps_debt=10 no_pending_success=0");
}

/** @brief The generic production preflight bridge also requires motor/phase/nonzero sequence, beyond legacy epoch/request. */
static void platform_full_preflight_identity(void)
{
    DebugUiStaticTxIdentity identity, wrong;
    CanFrame frame;
    uint64_t lower;
    (void)platform_prepare(100U);
    identity = platform_single_static.identity;
    platform_complete_tx(110U); fake_counter = 140U;
    wrong = identity; ++wrong.sequence;
    aethor_app_static_transmitted(&wrong, 1U, 110U, 3000U);
    assert(!application_action.single_static_tx_confirmed && application_debug_ui.single_static_transport_mask);
    wrong = identity; wrong.motor_id = 1U;
    aethor_app_static_transmitted(&wrong, 1U, 110U, 3000U);
    assert(!application_action.single_static_tx_confirmed);
    CollectSingleStaticReceipt();
    assert(application_action.single_static_tx_confirmed);
    platform_raw_feedback(120U);
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower) == CAN_RX_INBOX_STATUS_OK);
    wrong = identity; ++wrong.sequence;
    assert(aethor_app_receive_static_frame(&frame, lower, 3000U, &wrong, 120U, 1U) == MOTOR_RUNTIME_STATUS_OK);
    assert(!application_action.single_static_ordered_generation);
    /* The wrong-token sample was still accepted by the bank. Give the next actual frame
     * a newer real arrival lower bound; never weaken duplicate-sample rejection. */
    fake_tick = 4U;
    stm32_platform_can_observe_rx_boundary(4000U);
    platform_raw_feedback(130U); platform_decode();
    assert(application_action.single_static_ordered_generation);
    puts("STATIC_PREFLIGHT_FULL_IDENTITY_PASS wrong_sequence_wrong_axis_receipt_and_rx_failclosed=1 real_core_proof_accepts=1");
}

/** @brief Start a real admitted ID7 static command and its ordinary disabled preflight on the actual HAL adapter. */
static void platform_app_startup_fixture(ProtocolCommand *command, MotorFeedbackSnapshot *snapshot, float torque_ff_nm)
{
    DebugUiMotorProfile profile;
    DebugUiSingleStaticProfile long_profile = {1U, 1000U, 60000U, 0.0F};
    DebugUiRequest request;
    DebugUiSnapshot view;
    MotorJointFeedback feedback;
    CanFrame frame;
    CanTxPriority priority;
    platform_generic_fixture();
    long_profile.torque_ff_nm = torque_ff_nm;
    profile = application_debug_ui.profiles[0];
    application_motor_runtime.discovery.results[6] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.results[6].observed_esc_id = 7U;
    application_motor_runtime.discovery.results[6].observed_master_id = application_motor_runtime.configuration->joints[6].master_id;
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    application_motor_runtime.discovery.verified_joint_mask = 0x40U;
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    assert(aethor_app_debug_ui_set_motor_profile(7U, &profile));
    memset(&feedback, 0, sizeof(feedback)); feedback.timestamp_us = 1000U;
    feedback.position_rad = 0.26F; feedback.driver_state = S3519_DRIVER_STATE_DISABLED;
    feedback.mos_temperature_c = 30.0F; feedback.rotor_temperature_c = 31.0F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[6].master_id, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
    assert(aethor_app_debug_ui_set_single_static_profile(7U, &long_profile));
    assert(aethor_app_debug_ui_get_snapshot(1000U, &view));
    request = test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000U);
    request.target_motor_id = 7U; request.reference_generation = view.motors[6].reference_generation;
    assert(test_admission(request, 1000U).accepted);
    {
        DebugUiCompletion completion;
        assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_COMPLETED);
    }
    assert(aethor_app_debug_ui_get_snapshot(1001U, &view));
    request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, 1001U);
    request.target_motor_id = 7U; request.hold_duration_ms = 1000U;
    request.torque_ff_nm = torque_ff_nm;
    request.delta_rad = request.speed_rad_s = request.kp = request.kd = 0.0F;
    request.reference_generation = view.motors[6].reference_generation;
    request.single_static_evidence_generation = view.motors[6].single_static_evidence_generation;
    request.static_positions_rad[6] = view.motors[6].position_rad;
    request.static_references[6] = view.motors[6].reference_generation;
    {
        DebugUiAdmission admission = test_admission(request, 1001U);
        if (!admission.accepted) fprintf(stderr, "STARTUP_FULL_CHAIN_ADMISSION reason=%u reference=%lu evidence=%lu configured=%u\n",
            (unsigned)admission.reason, (unsigned long)request.reference_generation,
            (unsigned long)request.single_static_evidence_generation,
            (unsigned)application_debug_ui.single_static_profiles[6].configured);
        assert(admission.accepted);
    }
    assert(protocol_engine_pop_command(&application_protocol_engine, command));
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_start_one_shot_move(command, snapshot, 1002U));
    assert(aethor_app_next_can_frame(3000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 7U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(stm32_platform_can_submit(priority, &frame) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
    fake_registers.TXBRP = 0U; fake_registers.TXBTO = 0x100U;
    fake_tick = 4U; fake_counter = 132U;
    stm32_platform_can_observe_rx_boundary(4000U);
    platform_raw_feedback(120U); platform_decode();
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(snapshot, 4001U));
    assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
}
/** @brief One marked frame traverses the real platform and Core receipt/RX bridge, never an app-only fake receipt. */
static void platform_takeover_transfer(uint64_t timestamp_us, uint8_t phase, MotorFeedbackSnapshot *snapshot, uint8_t receive_feedback)
{
    CanFrame frame;
    CanTxPriority priority;
    DebugUiStaticTxIdentity identity;
    fake_tick = (uint32_t)((timestamp_us + 999ULL) / 1000ULL);
    test_health((uint64_t)fake_tick * 1000ULL, true);
    assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_static_frame_identity(&frame, &identity) && identity.phase == phase);
    if (phase == DEBUG_UI_STATIC_TX_PRELOAD || phase == DEBUG_UI_STATIC_TX_FIRST_HOLD)
        assert(!memcmp(frame.data, application_action.static_frozen_frames[6].data, 8U));
    assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, aethor_app_static_submitted));
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
    assert(application_action.single_static_tx_submitted);
    assert(!application_action.single_static_tx_confirmed);
    platform_complete_tx((uint16_t)(fake_counter + 10U)); fake_counter = (uint16_t)(fake_counter + 30U);
    CollectSingleStaticReceipt();
    if (phase == DEBUG_UI_STATIC_TX_TAKEOVER) {
        uint16_t encoded_torque = (uint16_t)(((frame.data[6] & 15U) << 8U) | frame.data[7]);
        float torque_limit = application_motor_runtime.discovery.results[6].ranges.torque_max_nm;
        assert(fabsf(application_debug_ui.static_load.transmitted_torque_ff_nm -
            ((float)encoded_torque / 4095.0F * (2.0F * torque_limit) - torque_limit)) < 0.000002F);
    }
    if (phase == DEBUG_UI_STATIC_TX_ENABLE || !receive_feedback) return;
    ++fake_tick;
    stm32_platform_can_observe_rx_boundary((uint64_t)fake_tick * 1000ULL);
    platform_raw_feedback((uint16_t)(fake_event.TxTimestamp + 10U));
    if (phase != DEBUG_UI_STATIC_TX_PRELOAD) fake_rx_payload[0] |= (uint8_t)(S3519_DRIVER_STATE_ENABLED << 4U);
    platform_decode();
    assert(application_action.single_static_tx_confirmed && application_action.single_static_ordered_generation);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, (uint64_t)fake_tick * 1000ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Finish the unchanged zero-FF startup through real HAL/Core before exercising takeover refreshes. */
static uint64_t platform_takeover_begin(MotorFeedbackSnapshot *snapshot, float target)
{
    ProtocolCommand command;
    uint8_t phase;
    platform_app_startup_fixture(&command, snapshot, target);
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
        platform_takeover_transfer((uint64_t)(fake_tick + 1U) * 1000ULL, phase, snapshot, 1U);
        if (phase != DEBUG_UI_STATIC_TX_ENABLE)
            assert(!aethor_app_advance_one_shot_setup(snapshot, (uint64_t)fake_tick * 1000ULL + 1ULL));
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    assert(application_debug_ui.static_load.takeover_active);
    assert(!application_debug_ui.static_load.requested_torque_ff_nm);
    return application_action.static_takeover_started_us;
}

/** @brief Poll real App safety and scheduling using the latest actual decoded feedback, without supplying a receipt. */
static void platform_takeover_poll(MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    fake_tick = (uint32_t)(timestamp_us / 1000ULL);
    test_health(timestamp_us, true);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(snapshot, timestamp_us));
}

/** @brief Signed takeover sends the current 19-step value every 4 ms, each with fresh immutable wire proof. */
static void platform_app_takeover_integration(void)
{
    unsigned sign;
    for (sign = 0U; sign < 4U; ++sign) {
        MotorFeedbackSnapshot snapshot;
        DebugUiStaticTxIdentity previous_identity;
        uint8_t previous_step = 0U;
        unsigned refresh_count = 0U, increment_count = 0U, zero_count = 0U;
        uint64_t started, timestamp_us, fixed_deadline;
        float target = (sign & 1U) ? -1.05F : 1.05F;
        started = platform_takeover_begin(&snapshot, target);
        fixed_deadline = started + 1000000ULL;
        previous_identity = application_action.static_identity;
        for (timestamp_us = started + 4000ULL; timestamp_us < fixed_deadline; timestamp_us += 4000ULL) {
            uint8_t expected_step = (uint8_t)((timestamp_us - started) / 50000ULL);
            platform_takeover_poll(&snapshot, timestamp_us);
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(application_action.frames.count == 1U && application_action.frame_read_index == 0U);
            assert(application_action.deadline_us == fixed_deadline);
            assert(application_action.static_takeover_step == expected_step);
            assert(application_action.static_identity.epoch == previous_identity.epoch &&
                application_action.static_identity.request_id == previous_identity.request_id &&
                application_action.static_identity.motor_id == 7U &&
                application_action.static_identity.phase == DEBUG_UI_STATIC_TX_TAKEOVER &&
                application_action.static_identity.sequence == previous_identity.sequence + 1U);
            assert(!application_action.single_static_tx_confirmed && !application_action.single_static_ordered_generation);
            assert(fabsf(application_debug_ui.static_load.requested_torque_ff_nm -
                target * (float)expected_step / 19.0F) < 0.000001F);
            if (application_action.static_takeover_step != previous_step) {
                assert(application_action.static_takeover_step == previous_step + 1U);
                ++increment_count;
                previous_step = application_action.static_takeover_step;
            }
            if (!expected_step) ++zero_count;
            previous_identity = application_action.static_identity;
            platform_takeover_transfer(timestamp_us, DEBUG_UI_STATIC_TX_TAKEOVER, &snapshot,
                (uint8_t)(sign != 2U || timestamp_us != started + 996000ULL));
            ++refresh_count;
            assert(!platform_single_static.active && !application_debug_ui.single_static_transport_mask);
            assert(debug_ui_static_tx_identity_equal(&previous_identity, &platform_single_static.identity));
            assert(fabsf(application_debug_ui.static_load.transmitted_torque_ff_nm -
                application_debug_ui.static_load.requested_torque_ff_nm) <= 20.0F / 4095.0F);
            assert(!application_debug_ui.static_load.takeover_complete && application_debug_ui.static_load.elapsed_ms == 0U);
        }
        assert(refresh_count == 249U && increment_count == 19U && zero_count == 12U);
        assert(fake_tx_count == 253U);
        timestamp_us = started + (sign == 1U ? 1002500ULL : sign == 3U ? 1060000ULL : 1000000ULL);
        platform_takeover_poll(&snapshot, timestamp_us);
        if (sign >= 2U) {
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
            assert(!application_debug_ui.static_load.takeover_complete);
            assert(application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
            continue;
        }
        assert(previous_step == 19U && application_debug_ui.static_load.takeover_complete);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        assert(application_action.deadline_us == started + 2000000ULL);
        assert(application_debug_ui.static_load.started_at_us == started + 1000000ULL);
        assert(application_debug_ui.static_load.takeover_elapsed_ms == 1000U);
        assert(fabsf(application_debug_ui.static_load.transmitted_torque_ff_nm - target) <= 20.0F / 4095.0F);
    }
    puts("STATIC_TAKEOVER_PLATFORM_PASS signs=2 startup_zero=1 actual_tx_core_rx=1 increments=19 refresh4ms=249 zero_refreshes=12 immutable_id_each=1 late_poll_anchored=1 latest_final_rx_required=1 task_stall_rejected=1 fixed_1s_window=1");
}

/** @brief Unresolved queue/HAL/RX, STOP/cancel, expiry and revoked ownership cannot overwrite or extend a refresh. */
static void platform_takeover_refresh_failures(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 13U; ++scenario) {
        MotorFeedbackSnapshot snapshot;
        CanFrame frame, emergency, frozen;
        CanTxPriority priority;
        DebugUiStaticTxIdentity identity, old_identity;
        uint64_t started = platform_takeover_begin(&snapshot, 1.05F);
        uint64_t timestamp_us, due_us = scenario >= 8U && scenario <= 10U ? 996000ULL : 4000ULL;
        uint32_t tx_before;
        old_identity = application_action.static_identity;
        for (timestamp_us = started + 4000ULL; timestamp_us < started + due_us; timestamp_us += 4000ULL) {
            platform_takeover_poll(&snapshot, timestamp_us);
            platform_takeover_transfer(timestamp_us, DEBUG_UI_STATIC_TX_TAKEOVER, &snapshot, 1U);
        }
        timestamp_us = started + due_us;
        if (scenario == 11U) timestamp_us = started + 10000ULL;
        tx_before = fake_tx_count;
        platform_takeover_poll(&snapshot, timestamp_us);
        if (scenario != 11U) {
            identity = application_action.static_identity;
            frozen = application_action.frames.frames[0];
            if (scenario != 9U) {
                assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
                assert(aethor_app_static_frame_identity(&frame, &identity));
                assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, aethor_app_static_submitted));
                if (scenario == 3U) fake_hal_failure = 1U;
                if (scenario == 5U) application_debug_ui.mailbox.stop_intent_mask = 0x40U;
                if (scenario == 7U) application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
                if (scenario != 0U) {
                    fake_tick = (uint32_t)((timestamp_us + 999ULL) / 1000ULL);
                    if (scenario == 6U) fake_tick += 6U;
                    assert(stm32_platform_can_service_tx(1U) ==
                        (scenario == 3U || scenario == 5U || scenario == 6U || scenario == 7U ?
                         STM32_PLATFORM_STATUS_CAN_TX_ERROR : STM32_PLATFORM_STATUS_OK));
                    if (scenario == 2U || scenario == 4U || scenario == 8U || scenario == 10U) {
                        platform_complete_tx((uint16_t)(fake_counter + 10U));
                        fake_counter = (uint16_t)(fake_counter + 30U);
                    }
                    CollectSingleStaticReceipt();
                }
                if (scenario == 12U) {
                    assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT,
                        S3519_MODE_COMMAND_DISABLE, &emergency) == S3519_CODEC_STATUS_OK);
                    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
                    CollectSingleStaticReceipt();
                    assert(platform_single_static.active && !application_action.single_static_tx_confirmed);
                }
                if (scenario == 4U) {
                    platform_raw_feedback((uint16_t)(fake_event.TxTimestamp + 10U));
                    fake_rx_payload[0] |= (uint8_t)(S3519_DRIVER_STATE_ENABLED << 4U);
                    assert(can_frame_init(&frame, fake_rx_header.Identifier, fake_rx_payload, 8U) == CAN_FRAME_STATUS_OK);
                    assert(aethor_app_receive_static_frame(&frame, timestamp_us + 1000ULL, timestamp_us + 1000ULL,
                        &old_identity, 0U, 1U) == MOTOR_RUNTIME_STATUS_OK);
                    assert(!application_action.single_static_ordered_generation);
                }
                if (scenario == 10U) {
                    fake_tick = (uint32_t)((started + 1001000ULL) / 1000ULL);
                    stm32_platform_can_observe_rx_boundary((uint64_t)fake_tick * 1000ULL);
                    platform_raw_feedback((uint16_t)(fake_event.TxTimestamp + 10U));
                    fake_rx_payload[0] |= (uint8_t)(S3519_DRIVER_STATE_ENABLED << 4U);
                    platform_decode();
                }
            }
            if (scenario != 6U) {
                timestamp_us = scenario == 10U ? started + 1002500ULL : started + due_us + 4000ULL;
                platform_takeover_poll(&snapshot, timestamp_us);
            } else {
                timestamp_us = started + due_us + 8000ULL;
                platform_takeover_poll(&snapshot, timestamp_us);
            }
            assert(!memcmp(&frozen, &platform_single_static.frame, sizeof(frozen)) || scenario == 9U);
            assert(application_action.static_identity.sequence == identity.sequence);
            assert(!aethor_app_static_submitted(&identity, timestamp_us + 1ULL));
        }
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        assert(!application_debug_ui.static_load.takeover_complete && !application_debug_ui.static_load.takeover_active);
        assert(application_action.frames.count == 1U && application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(fake_tx_count <= tx_before + 1U);
        if (scenario == 0U || scenario == 3U || scenario == 5U || scenario == 6U ||
            scenario == 7U || scenario == 9U || scenario == 11U) assert(fake_tx_count == tx_before);
        fake_hal_failure = 0U;
        assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT, S3519_MODE_COMMAND_DISABLE, &emergency) == S3519_CODEC_STATUS_OK);
        if (scenario != 12U)
            assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
        if (fake_registers.TXBRP) {
            assert(platform_single_static.active && application_debug_ui.single_static_transport_mask);
            assert(fake_abort_calls == 1U);
            assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == tx_before + 1U);
            fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
        }
        CollectSingleStaticReceipt();
        assert(!platform_single_static.active && !application_debug_ui.single_static_transport_mask);
    }
    puts("STATIC_TAKEOVER_REFRESH_FAILURE_PASS cases=13 queued_pending_no_rx_old_identity_hal_error_stop_cancel_revoked_late_hal_latest_deadline_late_rx_stall=1 no_overwrite_no_catchup_direct_disable=1 pending_debt_retained=1");
}

/** @brief Early failure reaches the real App while Platform alone retains immutable physical debt and blocks emergency HAL. */
static void platform_takeover_scan_delay_failure(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 2U; ++scenario) {
        MotorFeedbackSnapshot snapshot;
        CanFrame frame, frozen, emergency; CanTxPriority priority;
        DebugUiStaticTxIdentity identity;
        uint64_t started = platform_takeover_begin(&snapshot, 1.05F), timestamp_us = started + 4000ULL;
        uint32_t tx_before = fake_tx_count;
        platform_takeover_poll(&snapshot, timestamp_us);
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity)); frozen = frame;
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, aethor_app_static_submitted));
        fake_tick = (uint32_t)((timestamp_us + 999ULL) / 1000ULL);
        test_health((uint64_t)fake_tick * 1000ULL, true);
        fake_strict_delay_pending = 1U; if (scenario == 0U) fake_strict_latest_buffer = 0U;
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        if (scenario == 1U) platform_can_rx_inbox.bus_off_latched = 1U;
        CollectSingleStaticReceipt();
        assert(application_action.single_static_tx_failed && !application_action.single_static_tx_confirmed);
        assert(!application_debug_ui.single_static_transport_mask && platform_single_static.active && !stm32_platform_can_is_idle());
        assert(debug_ui_static_tx_identity_equal(&platform_single_static.identity, &identity));
        assert(!memcmp(&platform_single_static.frame, &frozen, sizeof(frozen)));
        platform_takeover_poll(&snapshot, timestamp_us + 4000ULL);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        assert(!application_debug_ui.static_load.takeover_complete);
        assert(aethor_app_next_can_frame(timestamp_us + 4000ULL, &emergency, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY && emergency.data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(stm32_platform_can_submit(priority, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
        assert(stm32_platform_can_service_tx(1U) == (scenario == 1U ? STM32_PLATFORM_STATUS_CAN_BUS_OFF : STM32_PLATFORM_STATUS_OK));
        assert(fake_tx_count == tx_before + 1U && !fake_abort_calls);
        platform_can_rx_inbox.bus_off_latched = 0U; /* Synthetic bus recovery does not clear accepted debt. */
        fake_registers.TXBRP = 0x100U;
        CollectSingleStaticReceipt(); CollectSingleStaticReceipt();
        assert(platform_single_static.active && fake_abort_calls == 1U && fake_tx_count == tx_before + 1U);
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == tx_before + 1U);
        fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
        if (scenario == 0U) {
            CollectSingleStaticReceipt(); assert(platform_single_static.active);
            platform_complete_tx((uint16_t)(fake_counter + 10U)); fake_counter = (uint16_t)(fake_counter + 30U);
        }
        CollectSingleStaticReceipt();
        assert(!platform_single_static.active && !application_debug_ui.single_static_transport_mask);
        fake_strict_delay_pending = 0U; fake_strict_latest_buffer = 0x100U;
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == tx_before + 2U);
        assert(fake_last_tx.MessageMarker == 0U && fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS);
    }
    puts("STATIC_TAKEOVER_SCAN_DELAY_APP_PASS cases=2 early_failure_disable_intent=1 App_mask_zero_not_transport_clear=1 immutable_debt=1 pending_before_after_blocks_HAL=1 abort_once=1");
}

/** @brief An unsafe accepted sample cannot be erased by healthy uncorrelated RX before the next safety poll. */
static void platform_takeover_unsafe_rx_recovery(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 42U; ++scenario) {
        MotorFeedbackSnapshot snapshot;
        CanFrame healthy_frame, unsafe_frame;
        DebugUiStaticTxIdentity identity;
        uint64_t attack_elapsed_us = (scenario < 14U || scenario >= 28U) ? 952000ULL : 52000ULL;
        uint64_t started = platform_takeover_begin(&snapshot, 1.05F), timestamp_us;
        for (timestamp_us = started + 4000ULL; timestamp_us <= started + attack_elapsed_us; timestamp_us += 4000ULL) {
            platform_takeover_poll(&snapshot, timestamp_us);
            platform_takeover_transfer(timestamp_us, DEBUG_UI_STATIC_TX_TAKEOVER, &snapshot,
                (uint8_t)(timestamp_us != started + attack_elapsed_us || scenario >= 28U));
        }
        if (scenario >= 28U) assert(application_action.static_takeover_proven_us != 0ULL);
        identity = application_action.static_identity;
        platform_raw_feedback((uint16_t)(fake_event.TxTimestamp + 10U));
        fake_rx_payload[0] |= (uint8_t)(S3519_DRIVER_STATE_ENABLED << 4U);
        assert(can_frame_init(&healthy_frame, fake_rx_header.Identifier, fake_rx_payload, 8U) == CAN_FRAME_STATUS_OK);
        unsafe_frame = healthy_frame;
        switch (scenario % 7U) {
            case 0U: unsafe_frame.data[0] = 7U; break;
            case 1U: unsafe_frame.data[4] |= 15U; unsafe_frame.data[5] = 255U; break;
            case 2U: unsafe_frame.data[6] = 56U; break;
            case 3U: unsafe_frame.data[7] = 56U; break;
            case 4U: unsafe_frame.data[3] = 255U; unsafe_frame.data[4] |= 240U; break;
            case 5U: unsafe_frame.data[1] = unsafe_frame.data[2] = 255U; break;
            default: unsafe_frame.data[0] = 0x87U; break;
        }
        timestamp_us = started + attack_elapsed_us + 2000ULL;
        test_health(timestamp_us, true);
        (void)aethor_app_receive_static_frame(&unsafe_frame, timestamp_us, timestamp_us,
            (scenario % 14U < 7U) ? &identity : NULL, 0U, 1U);
        (void)aethor_app_receive_static_frame(&healthy_frame, timestamp_us + 1000ULL, timestamp_us + 1000ULL, NULL, 0U, 0U);
        timestamp_us = started + attack_elapsed_us + 4000ULL;
        test_health(timestamp_us, true);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT)
            (void)aethor_app_advance_one_shot_setup(&snapshot, timestamp_us);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        assert(!application_debug_ui.static_load.takeover_complete);
        assert(application_action.frames.count == 1U && application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
    }
    puts("STATIC_TAKEOVER_UNSAFE_RX_RECOVERY_PASS cases=42 first_final_unproven_final_proven=3 correlated_and_unrelated=2 disabled_torque_mos_rotor_speed_pose_fault=7 sticky_direct_disable=1");
}

/** @brief Real App guards, marked HAL completion and maintained Core bridge complete the entire startup exactly once. */
static void platform_app_startup_integration(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot snapshot;
    CanFrame frame, frozen;
    CanTxPriority priority;
    DebugUiStaticTxIdentity identity;
    uint8_t stage;
    unsigned failure_case;
    platform_app_startup_fixture(&command, &snapshot, 0.0F);
    frozen = application_action.static_frozen_frames[6];
    for (stage = DEBUG_UI_STATIC_TX_PRELOAD; stage <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++stage) {
        ++fake_tick; test_health((uint64_t)fake_tick * 1000ULL, true);
        assert(application_action.static_identity.phase == stage);
        assert(aethor_app_next_can_frame((uint64_t)fake_tick * 1000ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity) && identity.phase == stage && identity.motor_id == 7U);
        if (stage != DEBUG_UI_STATIC_TX_ENABLE) assert(!memcmp(frame.data, frozen.data, 8U));
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, aethor_app_static_submitted));
        aethor_app_static_can_queued(&frame, 1U, 1U, (uint64_t)fake_tick * 1000ULL);
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        assert(application_action.single_static_tx_submitted);
        platform_complete_tx((uint16_t)(fake_counter + 10U)); fake_counter = (uint16_t)(fake_counter + 30U);
        if (stage == DEBUG_UI_STATIC_TX_ENABLE) {
            CollectSingleStaticReceipt();
            assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_FIRST_HOLD);
            assert(!memcmp(application_action.frames.frames[0].data, frozen.data, 8U));
        } else {
            ++fake_tick;
            stm32_platform_can_observe_rx_boundary((uint64_t)fake_tick * 1000ULL);
            platform_raw_feedback((uint16_t)(fake_event.TxTimestamp + 10U));
            if (stage == DEBUG_UI_STATIC_TX_FIRST_HOLD) fake_rx_payload[0] |= (uint8_t)(S3519_DRIVER_STATE_ENABLED << 4U);
            platform_decode();
            assert(application_action.single_static_tx_confirmed && application_action.single_static_ordered_generation);
            assert(motor_runtime_get_snapshot(&application_motor_runtime, (uint64_t)fake_tick * 1000ULL,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
            assert(!aethor_app_advance_one_shot_setup(&snapshot, (uint64_t)fake_tick * 1000ULL + 1ULL));
        }
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    assert(application_action.static_preloaded_mask == 0x40U && application_action.static_first_hold_mask == 0x40U);
    assert(application_action.target_position_rad[6] == command.values[6] && fake_tx_count == 4U);
    for (failure_case = 0U; failure_case < 2U; ++failure_case) {
        CanFrame emergency;
        platform_app_startup_fixture(&command, &snapshot, 0.0F);
        ++fake_tick; test_health((uint64_t)fake_tick * 1000ULL, true);
        assert(aethor_app_next_can_frame((uint64_t)fake_tick * 1000ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity));
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, aethor_app_static_submitted));
        if (!failure_case) {
            fake_hal_failure = 1U;
            assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_CAN_TX_ERROR);
        } else {
            assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_MIT, S3519_MODE_COMMAND_DISABLE, &emergency) == S3519_CODEC_STATUS_OK);
            assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &emergency) == CAN_TX_SCHEDULER_STATUS_OK);
        }
        CollectSingleStaticReceipt();
        assert(application_action.single_static_tx_failed && !application_action.single_static_tx_confirmed);
        assert(!aethor_app_advance_one_shot_setup(&snapshot, (uint64_t)fake_tick * 1000ULL + 1ULL));
        assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT && !application_action.enabled_by_action_mask);
        assert(fake_tx_count == 1U);
    }
    fake_startup_tx = fake_hal_failure = 0U;
    puts("STATIC_STARTUP_APP_PLATFORM_CORE_PASS id=7 preload_enable_firsthold_hold=1 frozen_bytes=1 actual_pending_sof_core_rx=1 hal_error_and_queued_stop_failclosed=2");
}
#endif

#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
/** @brief A selected DISABLE withdraws queued/pending MIT and POS controls, retaining other axes and pending debt. */
static void platform_static_control_withdrawal(void)
{
    unsigned motor_id, mode, pending;
    for (motor_id = 1U; motor_id <= 7U; ++motor_id) {
        for (mode = 0U; mode < 2U; ++mode) {
            for (pending = 0U; pending < 2U; ++pending) {
                CanFrame control, other, disable;
                CanTxPriority priority;
                uint8_t payload[8] = {0};
                uint32_t issued;
                test_fixture();
                memset(&fake_registers, 0, sizeof(fake_registers));
                memset(&hfdcan1, 0, sizeof(hfdcan1));
                hfdcan1.Instance = &fake_registers;
                hfdcan1.Init.StdFiltersNbr = 7U;
                hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
                fake_tick = 3U; fake_kernel_hz = 80000000U;
                fake_tx_count = fake_abort_calls = 0U;
                fake_hal_failure = fake_abort_clears = fake_timestamp_failure = 0U;
                fake_startup_tx = 1U;
                assert(stm32_platform_init(application_motor_runtime.configuration) == STM32_PLATFORM_STATUS_OK);
                assert(can_frame_init(&control, motor_id + (mode ? 0x100U : 0U), payload, 8U) == CAN_FRAME_STATUS_OK);
                assert(can_frame_init(&other, (motor_id % 7U) + 1U + (mode ? 0x100U : 0U), payload, 8U) == CAN_FRAME_STATUS_OK);
                assert(s3519_pack_mode_command((uint8_t)motor_id, S3519_CONTROL_MODE_MIT,
                    S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
                assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &control) == CAN_TX_SCHEDULER_STATUS_OK);
                if (pending) {
                    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
                    assert(fake_tx_count == 1U && fake_registers.TXBRP == 0x100U);
                    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &control) == CAN_TX_SCHEDULER_STATUS_OK);
                }
                assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &other) == CAN_TX_SCHEDULER_STATUS_OK);
                assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &disable) == CAN_TX_SCHEDULER_STATUS_OK);
                assert(platform_can_tx_scheduler.count == 2U);
                if (pending) {
                    assert(fake_abort_calls == 1U);
                    issued = fake_tx_count;
                    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
                    assert(fake_tx_count == issued); /* Abort request alone is not cancellation proof. */
                    fake_registers.TXBRP = 0U;
                    fake_registers.TXBCF = 0x100U;
                }
                assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
                assert(fake_last_tx.Identifier == motor_id);
                assert(can_tx_scheduler_pop(&platform_can_tx_scheduler, &other, &priority) == CAN_TX_SCHEDULER_STATUS_OK);
                assert(other.identifier == (motor_id % 7U) + 1U + (mode ? 0x100U : 0U));
                assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
                assert(platform_can_tx_scheduler.count == 0U);
            }
        }
    }
    puts("STATIC_CONTROL_WITHDRAWAL_PASS ids=7 modes=2 queued_and_hal_pending=2 other_axis_retained=1");
}
#endif

/** @brief Execute gate0 ordinary-path compatibility or gate1 exact hardware ordering through actual production adapters. */
#ifndef SINGLE_STATIC_PLATFORM_ENTRY
#define SINGLE_STATIC_PLATFORM_ENTRY main
#endif
/** @brief Real strict-probe regression: independent TX debt remains unsafe even when the FIFO is completely free. */
static void strict_release_probe_tests(void)
{
    memset(&fake_registers, 0, sizeof(fake_registers)); memset(&hfdcan1, 0, sizeof(hfdcan1));
    hfdcan1.Instance = &fake_registers; hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
    platform_diagnostics.can_started = 1U; fake_tx_free_level = 8U;
    assert(stm32_platform_remote_release_is_idle());
    fake_registers.TXBRP = 1U;
    assert(stm32_platform_can_is_idle() && !stm32_platform_remote_release_is_idle());
    fake_registers.TXBRP = 0U; fake_registers.TXBCR = 1U;
    assert(!stm32_platform_remote_release_is_idle());
    fake_registers.TXBCR = 0U; fake_rx_available = 1U;
    assert(!stm32_platform_remote_release_is_idle());
    fake_rx_available = 0U; fake_rx1_available = 1U;
    assert(!stm32_platform_remote_release_is_idle());
    fake_rx1_available = 0U; platform_can_rx_inbox.write_index = 1U;
    assert(!stm32_platform_remote_release_is_idle());
    platform_can_rx_inbox.write_index = 0U;
    assert(stm32_platform_remote_release_is_idle());
    fake_tx_free_level = 7U; platform_diagnostics.can_started = 0U;
    puts("REMOTE_RELEASE_PLATFORM_PROBE_PASS independent_tx_cancel_rx_fifo_and_inbox_debt_closed");
}

int SINGLE_STATIC_PLATFORM_ENTRY(void)
{
    strict_release_probe_tests();
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    platform_strict_scan_delay();
    platform_strict_event_without_pending();
    platform_wire_order_cases();
    platform_old_event_and_marker_wrap();
    platform_window_unavailable();
    platform_sample_qualification();
    platform_sidecar_slot_lifetime();
    platform_cancel_pending_stop();
#elif !AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
    ArmConfig configuration;
    CanFrame frame;
    uint8_t payload[8] = {255U,255U,255U,255U,255U,255U,255U,253U};
    memset(&fake_registers, 0, sizeof(fake_registers)); memset(&hfdcan1, 0, sizeof(hfdcan1));
    hfdcan1.Instance = &fake_registers; hfdcan1.Init.StdFiltersNbr = 7U; hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
    test_fixture();
    configuration = *application_motor_runtime.configuration;
    assert(stm32_platform_init(&configuration) == STM32_PLATFORM_STATUS_OK && fake_config_calls == 0U);
    assert(can_frame_init(&frame, 7U, payload, 8U) == CAN_FRAME_STATUS_OK);
    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &frame) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
    assert(fake_tx_count == 1U && fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS && fake_last_tx.MessageMarker == 0U);
    puts("SINGLE_STATIC_PLATFORM_GATE0_PASS timestamp_setup_absent=1 ordinary_frame_unchanged=1");
#endif
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    platform_static_control_withdrawal();
    platform_generic_phase_cases();
#endif
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    platform_full_preflight_identity();
    platform_app_startup_integration();
    platform_app_takeover_integration();
    platform_takeover_refresh_failures();
    platform_takeover_unsafe_rx_recovery();
    platform_takeover_scan_delay_failure();
#endif
    return 0;
}
