/** @file pos_load_platform_test.c
 * @brief POS-only production platform transport and cancellation against deterministic HAL metadata stubs.
 */
#define SINGLE_STATIC_PLATFORM_ENTRY pos_load_existing_platform_suite
#include "single_static_platform_test.c"
#if AETHOR_DEBUG_UI_ALLOW_MIT || AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#error "POS platform evidence must run with every MIT/static authorization gate disabled."
#endif
/** @brief Publish the selected HAL request's completion metadata. */
static void pos_platform_complete_tx(uint16_t timestamp);

/** @brief Model an expired application submission guard without suppressing emergency DISABLE traffic. */
static uint8_t pos_platform_reject_submission(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    assert(identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE && boundary_us);
    return 0U;
}

/** @brief Accept only a complete terminal identity for a real HAL submission of the selected axis. */
static uint8_t pos_platform_selected_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    assert(identity && identity->epoch && identity->request_id && identity->sequence && identity->motor_id == 1U);
    assert(identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE && boundary_us == 3000ULL);
    return 1U;
}

/** @brief The existing ID1 codec's offset DISABLE is valid, while another axis's offset is rejected. */
static void pos_platform_terminal_offset_id(void)
{
    DebugUiStaticTxIdentity identity = {71U, 73U, 2U, 1U, DEBUG_UI_STATIC_TX_POS_DISABLE}, receipt;
    CanFrame disable;
    uint8_t succeeded;
    uint16_t sof;
    platform_generic_fixture();
    assert(s3519_pack_mode_command(1U, S3519_CONTROL_MODE_POSITION_VELOCITY,
        S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
    assert(disable.identifier == 0x101U);
    disable.identifier = 0x102U;
    assert(!stm32_platform_can_submit_static(&disable, &identity, 500000ULL, pos_platform_selected_submitted));
    disable.identifier = 0x101U;
    assert(stm32_platform_can_submit_static(&disable, &identity, 500000ULL, pos_platform_selected_submitted));
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    assert(fake_last_tx.Identifier == 0x101U && fake_last_tx.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
    pos_platform_complete_tx(110U); fake_counter = 130U;
    assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded);
    assert(debug_ui_static_tx_identity_equal(&identity, &receipt));
    puts("POS_LOAD_PLATFORM_OFFSET_DISABLE_PASS real_id1_codec_offset=1 wrong_axis_offset_rejected=1 actual_tx_identity=1");
}

/** @brief Publish one immutable hardware TX completion for the last real HAL submission. */
static void pos_platform_complete_tx(uint16_t timestamp)
{
    memset(&fake_event, 0, sizeof(fake_event));
    fake_event.Identifier = fake_last_tx.Identifier; fake_event.IdType = FDCAN_STANDARD_ID;
    fake_event.TxFrameType = FDCAN_DATA_FRAME; fake_event.FDFormat = FDCAN_CLASSIC_CAN;
    fake_event.BitRateSwitch = FDCAN_BRS_OFF; fake_event.DataLength = 8U;
    fake_event.EventType = FDCAN_TX_EVENT; fake_event.MessageMarker = fake_last_tx.MessageMarker;
    fake_event.TxTimestamp = timestamp; fake_event_available = 1U;
    fake_registers.TXEFS = FDCAN_TXEFS_EFFL;
    fake_registers.TXBRP = 0U; fake_registers.TXBTO = 0x100U;
}

/** @brief Decode raw RX metadata and return only the production adapter's full ordered identity match. */
static uint8_t pos_platform_rx_matches(uint16_t timestamp, const DebugUiStaticTxIdentity *expected)
{
    CanFrame frame;
    DebugUiStaticTxIdentity received;
    uint64_t lower_bound;
    uint16_t sof;
    uint8_t valid;
    memset(&fake_rx_header, 0, sizeof(fake_rx_header));
    fake_rx_header.Identifier = application_motor_runtime.configuration->joints[6].master_id;
    fake_rx_header.IdType = FDCAN_STANDARD_ID; fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_8; fake_rx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fake_rx_header.BitRateSwitch = FDCAN_BRS_OFF; fake_rx_header.RxTimestamp = timestamp;
    memset(fake_rx_payload, 0, sizeof(fake_rx_payload)); fake_rx_payload[0] = 7U;
    fake_rx_available = 1U;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower_bound) == CAN_RX_INBOX_STATUS_OK);
    return (uint8_t)(stm32_platform_can_static_rx_identity(&frame, &received, &sof, &valid) &&
        valid && sof == timestamp && debug_ui_static_tx_identity_equal(expected, &received));
}

/** @brief Terminal DISABLE cannot borrow queue acceptance, old ENABLE completion, or pre-DISABLE feedback. */
static void pos_platform_terminal_disable(void)
{
    unsigned old_transmitted;
    for (old_transmitted = 0U; old_transmitted < 2U; ++old_transmitted) {
        DebugUiStaticTxIdentity old = {71U, 73U, 1U, 7U, DEBUG_UI_STATIC_TX_ENABLE};
        DebugUiStaticTxIdentity terminal = {71U, 73U, 2U, 7U, DEBUG_UI_STATIC_TX_POS_DISABLE}, receipt;
        CanFrame enable, disable, control;
        uint8_t succeeded;
        uint16_t sof;
        platform_generic_fixture();
        assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_ENABLE, &enable) == S3519_CODEC_STATUS_OK);
        assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
        assert(s3519_pack_position_velocity(7U, 0.2F, 0.02F, &control) == S3519_CODEC_STATUS_OK);
        assert(disable.identifier == 7U);
        assert(stm32_platform_can_submit_static(&enable, &old, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &control) == CAN_TX_SCHEDULER_STATUS_OK);
        assert(stm32_platform_can_submit_static(&disable, &terminal, 500000ULL, platform_generic_submitted));
        assert(fake_abort_calls == 1U && platform_single_static.cancellation_requested);
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(!pos_platform_rx_matches(105U, &terminal));
        assert(stm32_platform_can_service_tx(3U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        if (old_transmitted) pos_platform_complete_tx(110U);
        else { fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U; }
        fake_counter = 120U;
        assert(stm32_platform_can_service_tx(3U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
        assert(debug_ui_static_tx_identity_equal(&old, &receipt));
        assert(!stm32_platform_can_submit_static(&enable, &old, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(3U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 2U);
        assert(fake_last_tx.Identifier == 7U && fake_last_tx.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(!pos_platform_rx_matches(125U, &terminal));
        pos_platform_complete_tx(130U); fake_counter = 150U;
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded && sof == 130U);
        assert(debug_ui_static_tx_identity_equal(&terminal, &receipt));
        assert(!pos_platform_rx_matches(125U, &terminal));
        assert(!pos_platform_rx_matches(130U, &terminal));
        assert(pos_platform_rx_matches(140U, &terminal));
        assert(platform_can_tx_scheduler.count == 0U && fake_tx_count == 2U);
    }
    puts("POS_LOAD_PLATFORM_TERMINAL_PASS delayed_abort=2 old_receipt_retained=2 queue_and_pre_disable_rx_rejected=1 actual_disable_sof_ordered_rx=1");
}

/** @brief Invalid proof conditions still issue the exact emergency DISABLE and never report cleanup confirmation. */
static void pos_platform_terminal_failed_proof(void)
{
    unsigned failure;
    for (failure = 0U; failure < 7U; ++failure) {
        DebugUiStaticTxIdentity identity = {71U, 73U, 2U, 7U, DEBUG_UI_STATIC_TX_POS_DISABLE}, receipt;
        CanFrame disable;
        uint8_t succeeded;
        uint16_t sof;
        platform_generic_fixture();
        assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
            S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
        if (failure == 0U) platform_single_static_timestamp_ready = 0U;
        if (failure == 1U) hfdcan1.Init.NominalPrescaler = 0U;
        if (failure == 2U) fake_rx_available = 1U;
        if (failure == 3U) fake_registers.IR = FDCAN_IR_TEFL;
        assert(stm32_platform_can_submit_static(&disable, &identity,
            failure == 4U ? 600000ULL : 500000ULL,
            failure == 5U ? pos_platform_reject_submission : platform_generic_submitted));
        if (failure == 6U) fake_tick = 503U;
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        assert(fake_last_tx.Identifier == 7U && !memcmp(platform_single_static.frame.data, disable.data, 8U));
        assert(!stm32_platform_can_collect_static(&receipt, &succeeded, &sof));
        assert(!fake_abort_calls && fake_registers.TXBRP == 0x100U);
        pos_platform_complete_tx(110U); fake_counter = 130U;
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
        assert(debug_ui_static_tx_identity_equal(&identity, &receipt));
        fake_rx_available = 0U;
        assert(!pos_platform_rx_matches(120U, &identity));
    }
    puts("POS_LOAD_PLATFORM_TERMINAL_FAILCLOSED_PASS proof_failures=7 emergency_disable_still_submitted=7 no_abort_of_safety_disable=1 cleanup_never_confirmed=1");
}

/** @brief An unsent strict ENABLE is withdrawn, while a malformed phase5 token cannot reserve or cancel anything. */
static void pos_platform_terminal_queued_enable(void)
{
    DebugUiStaticTxIdentity old = {71U, 73U, 1U, 7U, DEBUG_UI_STATIC_TX_ENABLE};
    DebugUiStaticTxIdentity terminal = {71U, 73U, 2U, 7U, DEBUG_UI_STATIC_TX_POS_DISABLE}, receipt;
    CanFrame enable, disable;
    uint8_t succeeded;
    uint16_t sof;
    platform_generic_fixture();
    assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
        S3519_MODE_COMMAND_ENABLE, &enable) == S3519_CODEC_STATUS_OK);
    assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
        S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
    assert(stm32_platform_can_submit_static(&enable, &old, 500000ULL, platform_generic_submitted));
    disable.identifier = 0x106U;
    assert(!stm32_platform_can_submit_static(&disable, &terminal, 500000ULL, platform_generic_submitted));
    assert(!platform_single_static.cancellation_requested && platform_can_tx_scheduler.count == 1U);
    disable.identifier = 7U; disable.data[0] = 0U;
    assert(!stm32_platform_can_submit_static(&disable, &terminal, 500000ULL, platform_generic_submitted));
    disable.data[0] = 0xFFU;
    assert(stm32_platform_can_submit_static(&disable, &terminal, 500000ULL, platform_generic_submitted));
    assert(platform_can_tx_scheduler.count == 0U);
    assert(stm32_platform_can_service_tx(2U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 0U);
    assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && !succeeded);
    assert(debug_ui_static_tx_identity_equal(&old, &receipt));
    assert(stm32_platform_can_service_tx(2U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    assert(fake_last_tx.Identifier == 7U && !memcmp(platform_single_static.frame.data, disable.data, 8U));
    puts("POS_LOAD_PLATFORM_TERMINAL_QUEUED_PASS unsent_enable_withdrawn=1 malformed_disable_rejected=2 receipt_before_terminal_promotion=1");
}

/** @brief Place one correctly encoded disabled reply in the fake hardware FIFO before any ISR or boundary observation. */
static void pos_platform_prepare_reply(uint8_t joint_index, uint16_t received_sof)
{
    CanFrame encoded;
    uint8_t motor_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
    assert(s3519_pack_mit(motor_id, &application_motor_runtime.discovery.results[joint_index].ranges,
        0.2F, 0.0F, 0.0F, 0.0F, 0.0F, &encoded) == S3519_CODEC_STATUS_OK);
    memset(&fake_rx_header, 0, sizeof(fake_rx_header));
    fake_rx_header.Identifier = application_motor_runtime.configuration->joints[joint_index].master_id;
    fake_rx_header.IdType = FDCAN_STANDARD_ID; fake_rx_header.RxFrameType = FDCAN_DATA_FRAME;
    fake_rx_header.DataLength = FDCAN_DLC_BYTES_8; fake_rx_header.FDFormat = FDCAN_CLASSIC_CAN;
    fake_rx_header.BitRateSwitch = FDCAN_BRS_OFF; fake_rx_header.RxTimestamp = received_sof;
    fake_rx_payload[0] = motor_id; fake_rx_payload[1] = encoded.data[0]; fake_rx_payload[2] = encoded.data[1];
    fake_rx_payload[3] = encoded.data[2];
    fake_rx_payload[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    fake_rx_payload[5] = encoded.data[7]; fake_rx_payload[6] = 30U; fake_rx_payload[7] = 31U;
    fake_rx_available = 1U;
}

/** @brief Preserve the production ISR inbox timestamp and App decode while exposing the actual conservative lower bound. */
static uint64_t pos_platform_decode_reply(uint64_t observed_us, MotorRuntimeStatus expected_status)
{
    CanFrame frame;
    DebugUiStaticTxIdentity identity;
    uint64_t lower_bound;
    uint16_t sof;
    uint8_t valid, ordered;
    HAL_FDCAN_RxFifo0Callback(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
    assert(stm32_platform_can_pop_received_at(&frame, &lower_bound) == CAN_RX_INBOX_STATUS_OK);
    ordered = stm32_platform_can_static_rx_identity(&frame, &identity, &sof, &valid);
    {
        MotorRuntimeStatus status = aethor_app_receive_static_frame(&frame, lower_bound, observed_us,
            ordered ? &identity : NULL, sof, valid);
        if (status != expected_status)
            fprintf(stderr, "POS_RX_STATUS actual=%u expected=%u lower=%llu observed=%llu\n",
                (unsigned)status, (unsigned)expected_status, (unsigned long long)lower_bound,
                (unsigned long long)observed_us);
        assert(status == expected_status);
    }
    return lower_bound;
}

/** @brief Route one real encoded ID1 disabled reply through HAL ISR, platform identity and the production App decoder. */
static void pos_platform_app_receive(uint16_t received_sof, uint64_t observed_us)
{
    pos_platform_prepare_reply(0U, received_sof);
    (void)pos_platform_decode_reply(observed_us, MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Admit the actual ID7 POS request through the mailbox and let the production service enter initial preflight. */
static void pos_platform_preflight_fixture(void)
{
    DebugUiRequest request;
    DebugUiSnapshot view;
    MotorJointFeedback feedback;
    platform_generic_fixture();
    application_motor_runtime.discovery.results[6] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.results[6].observed_esc_id = 7U;
    application_motor_runtime.discovery.results[6].observed_master_id = application_motor_runtime.configuration->joints[6].master_id;
    application_motor_runtime.discovery.verified_joint_mask |= 0x40U;
    assert(aethor_app_debug_ui_set_motor_profile(7U, &application_debug_ui.profiles[0]));
    feedback = application_motor_runtime.bank.motors[0].feedback;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[6].master_id, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
    request = test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000ULL);
    assert(aethor_app_debug_ui_get_snapshot(1000ULL, &view));
    request.target_motor_id = 7U; request.reference_generation = view.motors[6].reference_generation;
    assert(test_admission(request, 1000ULL).accepted);
    {
        DebugUiCompletion completion;
        assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_COMPLETED);
    }
    test_health(2000ULL, true);
    request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 2000ULL);
    assert(aethor_app_debug_ui_get_snapshot(2000ULL, &view));
    request.target_motor_id = 7U; request.reference_generation = view.motors[6].reference_generation;
    request.kp = request.kd = 0.0F; request.delta_rad = 0.001F; request.speed_rad_s = 0.002F;
    request.hold_duration_ms = 5000U;
    {
        DebugUiAdmission admission = test_admission(request, 2000ULL);
        if (!admission.accepted) fprintf(stderr, "POS_PREFLIGHT_ADMISSION reason=%u motor=7 reference=%lu\n",
            (unsigned)admission.reason, (unsigned long)request.reference_generation);
        assert(admission.accepted);
    }
    fake_tick = 3U; test_health(3000ULL, true);
    stm32_platform_can_observe_rx_boundary(3000ULL);
    assert(!aethor_app_service(3000ULL));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
    assert(application_action.phase_feedback_baseline_us[6] == 3000ULL);
}

/** @brief Exercise initial POS DISABLE through real App, scheduler, HAL, ISR and decoder under quiet and delayed boundary observations. */
static void pos_platform_initial_preflight(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 6U; ++scenario) {
        CanFrame disable;
        CanTxPriority priority;
        DebugUiStaticTxIdentity identity;
        MotorFeedbackSnapshot snapshot;
        uint64_t lower_bound, deadline;
        pos_platform_preflight_fixture();
        deadline = application_action.deadline_us;
        assert(deadline == 503000ULL);
        assert(aethor_app_next_can_frame(3000ULL, &disable, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
        assert(!fake_tx_count && application_action.frame_read_index == 0U);
        fake_tick = 4U; test_health(4000ULL, true);
        if (scenario == 1U) fake_registers.TXBRP = 0x80U;
        if (scenario == 2U) pos_platform_prepare_reply(6U, 90U);
        stm32_platform_can_observe_rx_boundary(4000ULL);
        assert(!aethor_app_service(4000ULL));
        assert(aethor_app_next_can_frame(4000ULL, &disable, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(disable.identifier == 7U && disable.data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY && !aethor_app_static_frame_identity(&disable, &identity));
        assert(stm32_platform_can_submit(priority, &disable) == CAN_TX_SCHEDULER_STATUS_OK);
        aethor_app_static_can_queued(&disable, 1U, 0U, 4000ULL);
        assert(application_action.phase_feedback_baseline_us[6] == 3000ULL);
        assert(application_debug_ui.pos_load_disable_after_us == 0ULL);
        if (scenario == 3U) pos_platform_prepare_reply(6U, 90U);
        stm32_platform_can_observe_rx_boundary(4000ULL);
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
        assert(fake_last_tx.Identifier == 7U && fake_last_tx.TxEventFifoControl == FDCAN_NO_TX_EVENTS);
        fake_registers.TXBRP = 0U; fake_registers.TXBTO = 0x100U;
        if (scenario != 2U && scenario != 3U && scenario != 4U) pos_platform_prepare_reply(6U, 120U);
        if (scenario == 2U || scenario == 3U) {
            fake_tick = 5U;
            stm32_platform_can_observe_rx_boundary(5000ULL);
        }
        if (scenario == 5U) stm32_platform_can_observe_rx_boundary(2999ULL);
        lower_bound = scenario == 4U ? 0ULL : pos_platform_decode_reply((uint64_t)fake_tick * 1000ULL,
            scenario == 5U ? MOTOR_RUNTIME_STATUS_STALE_FEEDBACK : MOTOR_RUNTIME_STATUS_OK);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, (uint64_t)fake_tick * 1000ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(&snapshot, (uint64_t)fake_tick * 1000ULL));
        fprintf(stderr, "POS_LOAD_INITIAL_PREFLIGHT_OBSERVED scenario=%u lower=%llu baseline=%llu state=%u\n",
            scenario, (unsigned long long)lower_bound,
            (unsigned long long)application_action.phase_feedback_baseline_us[6], (unsigned)application_action.state);
        if (scenario == 0U || scenario == 3U) {
            assert(lower_bound == 4000ULL);
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
        } else {
            DebugUiCompletion completion;
            assert(lower_bound == (scenario < 4U ? 3000ULL : 0ULL));
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
            assert(application_action.deadline_us == deadline);
            assert(aethor_app_advance_one_shot_setup(&snapshot, deadline));
            aethor_app_debug_ui_route_results();
            assert(aethor_app_debug_ui_poll_completion(&completion));
            assert(completion.code == DEBUG_UI_FAILED && completion.error == PROTOCOL_COMMAND_ERROR_FEEDBACK_TIMEOUT);
            assert(completion.stage == DEBUG_UI_STAGE_VALIDATE && completion.timestamp_us == deadline);
            assert(!completion.disabled_confirmed && fake_tx_count == 1U);
        }
        printf("POS_LOAD_INITIAL_PREFLIGHT_DIAGNOSTIC scenario=%u tx=%u lower=%llu baseline=3000 outcome=%s\n",
            scenario, (unsigned)fake_tx_count, (unsigned long long)lower_bound,
            scenario == 0U || scenario == 3U ? "PRELOAD" : "ORIGINAL_TIMEOUT");
    }
    puts("POS_LOAD_INITIAL_PREFLIGHT_PASS quiet_same_tick_reply=1 creation_tick_deferred=1 pending_tx_old_bound_timeout=1 delayed_fifo_old_bound_timeout=1 delayed_fifo_new_bound_progress=1 missing_reply_timeout=1 regressed_clock_rejected=1");
}

/** @brief A saturated STOP bound during preflight retains emergency scheduling evidence and its independent strict receipt. */
static void pos_platform_preflight_fallback_boundary(void)
{
    CanFrame disable;
    CanTxPriority priority;
    DebugUiStaticTxIdentity identity, receipt;
    uint8_t succeeded;
    uint16_t sof;
    pos_platform_preflight_fixture();
    aethor_app_pos_load_bind_fallback(3500ULL);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
    assert(application_debug_ui.pos_load_fallback_after_us == 3500ULL);
    fake_tick = 4U;
    assert(aethor_app_next_can_frame(4000ULL, &disable, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_static_frame_identity(&disable, &identity) && identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE);
    assert(stm32_platform_can_submit_static(&disable, &identity, 500000ULL, aethor_app_static_submitted));
    aethor_app_static_can_queued(&disable, 1U, 1U, 4000ULL);
    fprintf(stderr, "POS_LOAD_PREFLIGHT_FALLBACK_OBSERVED disable_after=%llu baseline=%llu fallback_after=3500\n",
        (unsigned long long)application_debug_ui.pos_load_disable_after_us,
        (unsigned long long)application_action.phase_feedback_baseline_us[6]);
    assert(application_debug_ui.pos_load_disable_after_us == 4000ULL);
    assert(application_action.phase_feedback_baseline_us[6] == 3000ULL);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    assert(application_debug_ui.pos_disable_submitted && !application_debug_ui.pos_disable_confirmed);
    pos_platform_complete_tx(110U); fake_counter = 130U;
    assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded);
    assert(debug_ui_static_tx_identity_equal(&identity, &receipt));
    aethor_app_static_transmitted(&receipt, succeeded, sof, 4000ULL);
    assert(application_debug_ui.pos_disable_confirmed && !application_debug_ui.pos_load.disabled_confirmed);
    puts("POS_LOAD_PREFLIGHT_FALLBACK_PASS queued_boundary_preserved=1 queued_baseline_unchanged=1 strict_emergency_tx=1 tx_without_rx_not_confirmed=1");
}

/** @brief A real admitted POS trial completes cleanup only through its own marked HAL TX and newer ordered RX. */
static void pos_platform_app_terminal_integration(void)
{
    DebugUiRequest request;
    ProtocolCommand command;
    MotorFeedbackSnapshot snapshot;
    DebugUiStaticTxIdentity identity, receipt;
    CanFrame disable;
    CanTxPriority priority;
    uint8_t succeeded;
    uint16_t sof;
    platform_generic_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 1001U);
    request.kp = request.kd = 0.0F; request.delta_rad = 0.005F; request.hold_duration_ms = 5000U;
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, &command));
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_start_one_shot_move(&command, &snapshot, 1002U));
    assert(!aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U));
    assert(aethor_app_next_can_frame(2210U, &disable, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(disable.identifier == 0x101U && priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(aethor_app_pos_load_terminal_frame_identity(&disable, &identity));
    assert(identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE && identity.motor_id == 1U);
    assert(stm32_platform_can_submit_static(&disable, &identity, 500000ULL, aethor_app_static_submitted));
    aethor_app_static_can_queued(&disable, 1U, 1U, 2210U);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    assert(application_debug_ui.pos_disable_submitted && !application_debug_ui.pos_disable_confirmed);
    fake_counter = 120U;
    pos_platform_app_receive(105U, 3200U);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 3200U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(&snapshot, 3201U));
    assert(!application_debug_ui.pos_disable_ordered_generation);
    pos_platform_complete_tx(110U); fake_counter = 130U;
    assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded);
    assert(debug_ui_static_tx_identity_equal(&identity, &receipt));
    aethor_app_static_transmitted(&receipt, succeeded, sof, 3300U);
    assert(application_debug_ui.pos_disable_confirmed);
    fake_tick = 4U; stm32_platform_can_observe_rx_boundary(4000U);
    pos_platform_app_receive(105U, 4000U);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(&snapshot, 4001U));
    assert(!application_debug_ui.pos_load.disabled_confirmed && !application_debug_ui.pos_disable_ordered_generation);
    fake_tick = 5U; stm32_platform_can_observe_rx_boundary(5000U);
    pos_platform_app_receive(120U, 5000U);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 5000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_debug_ui.pos_disable_ordered_generation);
    assert(aethor_app_advance_one_shot_setup(&snapshot, 5001U));
    aethor_app_debug_ui_route_results();
    assert(application_debug_ui.pos_load.disabled_confirmed && !application_debug_ui.pos_load.active);
    assert(fake_tx_count == 1U);
    puts("POS_LOAD_APP_PLATFORM_TERMINAL_PASS actual_pos_request=1 real_classifier_callback_tx_rx=1 codec_id1_offset=1 pretx_rx_rejected=1 only_ordered_phase5_completes=1");
}

/** @brief Hardware-ordered disabled replies in the HAL submission millisecond must settle normal, failed and STOP cleanup. */
static void pos_platform_terminal_same_tick(void)
{
    unsigned cleanup;
    for (cleanup = 0U; cleanup < 3U; ++cleanup) {
        DebugUiRequest request;
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        DebugUiStaticTxIdentity identity, receipt;
        CanFrame disable;
        CanTxPriority priority;
        uint8_t succeeded;
        uint16_t sof;
        uint64_t lower_bound;
        platform_generic_fixture(); test_acquire();
        request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 1001U);
        request.kp = request.kd = 0.0F;
        request.delta_rad = 0.005F; request.hold_duration_ms = 5000U;
        assert(test_admission(request, 1001U).accepted);
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_move(&command, &snapshot, 1002U));
        if (cleanup == 2U) {
            test_health(2200U, true);
            request = test_request(DEBUG_UI_OPERATION_STOP, 2200U);
            assert(test_admission(request, 2200U).accepted);
            (void)aethor_app_service(2201U);
            assert(application_action.command.local_operation == DEBUG_UI_OPERATION_STOP);
            assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
        } else if (cleanup)
            assert(!aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U));
        else assert(!aethor_app_start_one_shot_disable(&snapshot, 2200U));
        assert(aethor_app_next_can_frame(2210U, &disable, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_pos_load_terminal_frame_identity(&disable, &identity));
        assert(stm32_platform_can_submit_static(&disable, &identity, 500000ULL, aethor_app_static_submitted));
        aethor_app_static_can_queued(&disable, 1U, 1U, 2210U);
        fake_tick = 4U;
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        assert(application_debug_ui.pos_load_disable_after_us == 4000ULL);
        pos_platform_complete_tx(110U); fake_counter = 130U;
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded);
        aethor_app_static_transmitted(&receipt, succeeded, sof, 4000ULL);
        /* A matching TX receipt alone is never enough; other cases cover pre-TX/equal-SOF RX. */
        assert(!(cleanup == 2U ? aethor_app_service_active_action(&snapshot, 4000ULL) :
            aethor_app_advance_one_shot_setup(&snapshot, 4000ULL)));
        assert(!application_debug_ui.pos_disable_ordered_generation);
        pos_platform_prepare_reply(0U, 120U);
        lower_bound = pos_platform_decode_reply(4000ULL, MOTOR_RUNTIME_STATUS_OK);
        assert(lower_bound == 4000ULL);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(aethor_app_pos_disable_proven(&snapshot));
        if (cleanup == 2U) {
            /* Fault injection: a broadened STOP has settled both frame debts, but the other axis
             * still has only pre-boundary disabled feedback. POS wire proof cannot confirm it. */
            uint8_t original_mask = application_action.command.motor_mask;
            uint8_t original_count = application_action.frames.count;
            uint8_t original_read = application_action.frame_read_index;
            MotorFeedbackSnapshot broader = snapshot;
            application_action.command.motor_mask |= 2U;
            application_action.frames.count = application_action.frame_read_index = 2U;
            broader.valid_joint_mask |= 2U;
            broader.joints[1] = broader.joints[0];
            broader.joints[1].timestamp_us = 3999ULL;
            application_action.phase_feedback_baseline_us[1] = 4000ULL;
            assert(!aethor_app_service_active_action(&broader, 4001ULL));
            assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
            application_action.command.motor_mask = original_mask;
            application_action.frames.count = original_count;
            application_action.frame_read_index = original_read;
        }
        printf("POS_TERMINAL_SAME_TICK cleanup=%u lower=%llu baseline=%llu wire_proven=1\n", cleanup,
            (unsigned long long)lower_bound,
            (unsigned long long)application_action.phase_feedback_baseline_us[0]);
        fflush(stdout);
        assert(cleanup == 2U ? aethor_app_service_active_action(&snapshot, 4001ULL) :
            aethor_app_advance_one_shot_setup(&snapshot, 4001ULL));
        aethor_app_debug_ui_route_results();
        assert(application_debug_ui.pos_load.disabled_confirmed);
        assert(!application_debug_ui.pos_load.active && fake_tx_count == 1U);
        if (cleanup == 2U) assert(application_debug_ui.pos_load.stopped && !application_debug_ui.pos_load.complete);
    }
    puts("POS_TERMINAL_SAME_TICK_PASS normal_cleanup_stop=3 real_hal_rx=1 tx_without_rx_rejected=1 broadened_stop_old_other_axis_rejected=1");
}

/** @brief Mode2 preload/enable/first-hold frames preserve exact identity and real hardware TX evidence. */
static void pos_platform_strict_phases(void)
{
    uint8_t phase;
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
        DebugUiStaticTxIdentity identity = {71U, 73U, (uint32_t)phase, 7U, phase}, received;
        CanFrame frame;
        uint8_t succeeded;
        uint16_t sof;
        platform_generic_fixture();
        assert(s3519_pack_position_velocity(7U, 0.2F, 0.02F, &frame) == S3519_CODEC_STATUS_OK);
        if (phase == DEBUG_UI_STATIC_TX_ENABLE)
            assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
                S3519_MODE_COMMAND_ENABLE, &frame) == S3519_CODEC_STATUS_OK);
        assert(frame.identifier == (phase == DEBUG_UI_STATIC_TX_ENABLE ? 7U : 0x107U));
        assert(stm32_platform_can_submit_static(&frame, &identity, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        assert(fake_last_tx.Identifier == frame.identifier && fake_last_tx.TxEventFifoControl == FDCAN_STORE_TX_EVENTS);
        fake_registers.TXBRP = 0U; fake_registers.TXBTO = 0x100U;
        fake_event.Identifier = frame.identifier; fake_event.IdType = FDCAN_STANDARD_ID;
        fake_event.TxFrameType = FDCAN_DATA_FRAME; fake_event.FDFormat = FDCAN_CLASSIC_CAN;
        fake_event.BitRateSwitch = FDCAN_BRS_OFF; fake_event.DataLength = 8U;
        fake_event.EventType = FDCAN_TX_EVENT; fake_event.MessageMarker = fake_last_tx.MessageMarker;
        fake_event.TxTimestamp = 110U; fake_event_available = 1U;
        fake_registers.TXEFS = FDCAN_TXEFS_EFFL; fake_counter = 120U;
        assert(stm32_platform_can_collect_static(&received, &succeeded, &sof));
        assert(succeeded && sof == 110U && debug_ui_static_tx_identity_equal(&identity, &received));
    }
    puts("POS_LOAD_PLATFORM_STRICT_PASS pos_only_gate_no_mit=1 real_mode2_ids=3 hardware_receipts=3");
}

/** @brief Emergency disable withdraws selected queued controls and waits for pending hardware cancellation before sending. */
static void pos_platform_cancel_controls(void)
{
    CanFrame frame, disable;
    platform_generic_fixture();
    assert(s3519_pack_position_velocity(7U, 0.2F, 0.02F, &frame) == S3519_CODEC_STATUS_OK);
    assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
        S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &frame) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    assert(platform_pos_control_buffers[6] == 0x100U);
    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_JOINT_CONTROL, &frame) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, &disable) == CAN_TX_SCHEDULER_STATUS_OK);
    assert(fake_abort_calls == 1U && platform_pos_cancel_pending == 0x100U);
    assert(platform_can_tx_scheduler.count == 1U && platform_can_tx_scheduler.entries[0].frame.data[7] == 0xFDU);
    assert(stm32_platform_can_service_tx(2U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 1U);
    fake_registers.TXBRP = 0U; fake_registers.TXBCF = 0x100U;
    assert(stm32_platform_can_service_tx(2U) == STM32_PLATFORM_STATUS_OK && fake_tx_count == 2U);
    assert(fake_last_tx.Identifier == disable.identifier && platform_can_tx_scheduler.count == 0U);
    puts("POS_LOAD_PLATFORM_CANCEL_PASS queued_selected_control_removed=1 pending_abort_wait=1 disable_after_termination=1");
}

/** @brief Repeated terminal probes re-arm hardware identity; old SOF/sequence cannot satisfy the next probe. */
static void pos_platform_terminal_reprobe(void)
{
    DebugUiStaticTxIdentity identity = {71U, 73U, 2U, 7U, DEBUG_UI_STATIC_TX_POS_DISABLE}, receipt;
    CanFrame disable;
    uint8_t succeeded;
    uint16_t sof;
    unsigned probe;
    platform_generic_fixture();
    assert(s3519_pack_mode_command(7U, S3519_CONTROL_MODE_POSITION_VELOCITY,
        S3519_MODE_COMMAND_DISABLE, &disable) == S3519_CODEC_STATUS_OK);
    for (probe = 0U; probe < 3U; ++probe) {
        uint16_t base = (uint16_t)(100U + 100U * probe);
        identity.sequence = 2U + probe;
        fake_counter = base;
        assert(stm32_platform_can_submit_static(&disable, &identity, 500000ULL, platform_generic_submitted));
        assert(!stm32_platform_can_submit_static(&disable, &identity, 500000ULL, platform_generic_submitted));
        assert(stm32_platform_can_service_tx(1U) == STM32_PLATFORM_STATUS_OK);
        pos_platform_complete_tx((uint16_t)(base + 10U)); fake_counter = (uint16_t)(base + 30U);
        assert(stm32_platform_can_collect_static(&receipt, &succeeded, &sof) && succeeded);
        assert(debug_ui_static_tx_identity_equal(&identity, &receipt));
        assert(!pos_platform_rx_matches((uint16_t)(base + 5U), &identity));
        assert(pos_platform_rx_matches((uint16_t)(base + 20U), &identity));
    }
    assert(fake_tx_count == 3U && platform_can_tx_scheduler.count == 0U);
    puts("POS_LOAD_PLATFORM_REPROBE_PASS sequences=3 pending_duplicate_rejected=1 old_sof_rejected=1 fresh_identity_required=1");
}

/** @brief Execute actual POS-only platform proof without MIT/static compile gates or a physical device. */
int main(void)
{
    pos_platform_terminal_reprobe();
    pos_platform_terminal_same_tick();
    pos_platform_initial_preflight();
    pos_platform_preflight_fallback_boundary();
    platform_generic_phase_cases();
    pos_platform_terminal_offset_id();
    pos_platform_app_terminal_integration();
    pos_platform_terminal_disable();
    pos_platform_terminal_failed_proof();
    pos_platform_terminal_queued_enable();
    pos_platform_strict_phases();
    pos_platform_cancel_controls();
    return 0;
}
