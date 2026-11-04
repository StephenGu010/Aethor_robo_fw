/** @file persistence_app_readonly_test.c
 * @brief Synthetic production App facade Stage A tests; no Core/Platform/HAL or hardware qualification.
 */
#define main existing_integrated_fixture_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Vendor Save acknowledgement must never enter the ordinary feedback decoder, even before maintenance. */
static void readonly_aa_isolation(void)
{
    CanFrame acknowledgement = {23U, 4U, {7U, 0U, 0xaaU, 1U}};
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    assert(aethor_app_receive_can_frame_observed(&acknowledgement, 2000U, 2001U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.rejected_feedback_count == 0U);
    acknowledgement.length = 8U;
    acknowledgement.data[6] = acknowledgement.data[7] = 25U;
    assert(aethor_app_receive_can_frame_observed(&acknowledgement, 2002U, 2003U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.accepted_feedback_count == 1U);
    puts("PERSISTENCE_APP_AA_ISOLATION_PASS");
}

/** @brief Starts a new synthetic firmware process; production has no reset/retry entry. */
static uint64_t readonly_fixture(uint32_t mode, float position_range)
{
    ProtocolOutputBatch output;
    MotorDiscoveryResult *discovery;
    memset(&application_persistence, 0, sizeof(application_persistence));
    memset(&application_persistence_transaction, 0, sizeof(application_persistence_transaction));
    aethor_app_init(1000U, 1234U);
    assert(request("0 hello", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    fixture_disabled_motor7(109000U);
    discovery = &application_motor_runtime.discovery.results[6];
    discovery->observed_control_mode = mode;
    discovery->ranges.position_max_rad = position_range;
    discovery->ranges.torque_max_nm = 10.0F;
    discovery->software_version = 808597046U; discovery->sub_version = 49U;
    discovery->hardware_version = 1U;
    aethor_app_integrated_set_can_idle(1U, 104000U);
    aethor_app_integrated_set_can_idle(1U, 110000U);
    aethor_app_persistence_observe_link(0x80000001UL, 110000U);
    return 110010U;
}

/** @brief Builds a real eight-byte feedback wire sample at the narrow VMAX200 zero quantization bin. */
static CanFrame readonly_feedback(uint8_t state)
{
    CanFrame frame = {23U, 8U, {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 25U}};
    frame.data[0] |= (uint8_t)(state << 4U);
    return frame;
}

/** @brief Establishes health only via public exact FD submission, actual receipt and ordered newer feedback. */
static uint64_t readonly_fd(uint64_t now_us)
{
    AethorPersistenceTransfer transfer;
    CanFrame feedback = readonly_feedback(S3519_DRIVER_STATE_DISABLED);
    uint64_t pure_deadline_us = application_persistence_transaction.phase_deadline_us;
    assert(aethor_app_persistence_next_transfer(now_us, &transfer));
    assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_DISABLE);
    assert(transfer.frame.identifier == (application_persistence.confirmed_mode == 1U ||
        (AETHOR_S3519_BASE_SPECIAL_COMMAND_MASK & 0x40U) ? 7U : 0x107U));
    assert(transfer.frame.length == 8U && transfer.frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(!aethor_app_persistence_next_transfer(now_us + 1U, &transfer));
    assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, now_us + 2U));
    assert(aethor_app_persistence_disable_transmitted(&transfer.disable_identity, 1U, 100U, now_us + 3U));
    assert(aethor_app_persistence_receive(&feedback, now_us + 4U, now_us + 5U,
        &transfer.disable_identity, 101U, 1U));
    assert(application_persistence.health_proven && !application_persistence.disable_wait_rx);
    assert(application_persistence_transaction.phase_deadline_us == pure_deadline_us);
    return now_us + 6U;
}

/** @brief Converts a native synthetic float without aliasing or numeric conversion. */
static uint32_t readonly_float_bits(float value)
{ uint32_t raw; memcpy(&raw, &value, sizeof(raw)); return raw; }

/** @brief Supplies one value from the fixed all45 synthetic read-only register baseline. */
static uint32_t readonly_register(unsigned index)
{
    if (index == 7U) return 23U;
    if (index == 8U) return 7U;
    if (index == 10U) return application_persistence.confirmed_mode;
    if (index == 14U) return 808597046U;
    if (index == 36U) return 49U;
    if (index == 21U) return readonly_float_bits(application_persistence.ranges.position_max_rad);
    if (index == 22U) return readonly_float_bits(200.0F);
    if (index == 23U) return readonly_float_bits(10.0F);
    /* Vendor RID0x50 (decimal80 p_m) matches native0; RID0x32 u_off retains its independent default1. */
    if (motor_persistence_rid(index) == 0x50U) return readonly_float_bits(0.0F);
    return motor_persistence_is_unsigned(index) ? 1U : readonly_float_bits(1.0F);
}

/** @brief Drives all45 real pure frames/identities through the public App port with bounded independent refreshes. */
static void readonly_baseline(uint32_t mode, float position_range)
{
    unsigned index, byte;
    AethorPersistenceTransfer transfer;
    uint64_t now_us = readonly_fixture(mode, position_range);
    uint32_t admission_generation = aethor_app_persistence_admission_generation();
    assert(aethor_app_persistence_allow_ordinary(admission_generation));
    assert(aethor_app_persistence_begin_readonly(91U, now_us));
    assert(!aethor_app_persistence_allow_ordinary(admission_generation));
    now_us = readonly_fd(now_us + 1U);
    for (index = 0U; index < 45U; ++index) {
        CanFrame reply = {23U, 8U, {7U, 0U, 0x33U, 0U}};
        uint32_t raw = readonly_register(index);
        now_us += 2000U;
        if (now_us - application_persistence.health.observed_us >= AETHOR_PERSISTENCE_HEALTH_REFRESH_US)
            now_us = readonly_fd(now_us);
        assert(aethor_app_persistence_next_transfer(now_us, &transfer));
        assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_PARAMETER);
        assert(transfer.frame.identifier == 0x7ffU && transfer.frame.length == 4U);
        assert(transfer.frame.data[2] == 0x33U && transfer.frame.data[3] == motor_persistence_rid(index));
        assert(!aethor_app_persistence_next_transfer(now_us + 1U, &transfer));
        assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, now_us + 2U));
        aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 3U, now_us + 4U);
        reply.data[3] = transfer.frame.data[3];
        for (byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
        assert(aethor_app_receive_can_frame_observed(&reply, now_us + 5U, now_us + 6U) == MOTOR_RUNTIME_STATUS_OK);
        assert(application_persistence_transaction.read_index == index + 1U);
        assert(!application_persistence.fault);
        now_us += 7U;
    }
    /* B1 publishes the same complete baseline only after an independent post-snapshot FD/continuity proof. */
    now_us = readonly_fd(now_us + 1U);
    assert(application_persistence.snapshot_ready && application_persistence.continuity_checked);
    assert(application_persistence_transaction.state == ((mode == 2U && position_range == 512.0F) ?
        MOTOR_PERSISTENCE_COMPLETE : MOTOR_PERSISTENCE_BASELINE_READY));
    assert(application_persistence_transaction.skip_save == (mode == 2U && position_range == 512.0F));
    assert(!application_persistence_transaction.save_used && !application_persistence_transaction.save_uncertain);
    assert(!aethor_app_persistence_next_transfer(now_us + 1U, &transfer));
    assert(aethor_app_persistence_locked() && !aethor_app_persistence_allow_ordinary(admission_generation));
    /* A genuine terminal stays terminal; the wrapper still revokes old context and cannot revive. */
    ++application_debug_ui.epoch;
    aethor_app_persistence_service(now_us + 2U);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    if (mode == 2U && position_range == 512.0F) assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_COMPLETE);
    --application_debug_ui.epoch;
    assert(!aethor_app_persistence_next_transfer(now_us + 3U, &transfer));
    assert(!aethor_app_persistence_begin_readonly(92U, now_us + 4U));
    puts("PERSISTENCE_APP_BASELINE45_SKIP_TARGET_PASS");
}

/** @brief Checks every App admission layer including configure/lock early returns and already queued execution. */
static void readonly_admission_gates(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    ProtocolOutputBatch output;
    DebugUiRequest local;
    ProtocolCommand command;
    MotorFeedbackSnapshot snapshot;
    CanFrame frame, frames[ARM_JOINT_COUNT];
    CanTxPriority priority;
    AdrcLcdOwnershipEvidence evidence;
    uint8_t kind;
    float torque;
    unsigned operation;
    assert(aethor_app_persistence_begin_readonly(93U, now_us));
    assert(request("garbage ==", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "persistence_locked"));
    assert(request("94 adrc wide_position motor=7", ++now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(!application_protocol_engine.command_write_sequence);
    memset(&local, 0, sizeof(local)); local.target_motor_id = 7U;
    for (operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE; operation <= DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK; ++operation) {
        local.operation = (DebugUiOperation)operation;
        assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_DISABLED);
        assert(aethor_app_debug_ui_admit_request(&local, ++now_us, &command) == DEBUG_UI_REASON_DISABLED);
    }
    assert(aethor_app_debug_ui_executor_busy());
    application_action.control_group_ready = 1U;
    assert(!aethor_app_pop_control_group(frames));
    assert(application_action.control_group_ready);
    application_action.control_group_ready = 0U;
    assert(aethor_app_next_can_frame(++now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
    aethor_app_integrated_build_evidence(++now_us, 6U, &evidence);
    assert(!evidence.lcd_idle);
    assert(!aethor_app_integrated_pop_probe_frame(&frame, ++now_us));
    assert(!aethor_app_adrc_pop_frame(&frame, &kind, &torque));
    assert(aethor_app_adrc_set_evidence(NULL, NULL, ++now_us) == ADRC_RESULT_INVALID_STATE);
    memset(&command, 0, sizeof(command)); memset(&snapshot, 0, sizeof(snapshot));
    command.type = PROTOCOL_COMMAND_ENABLE; command.request_id = 95U; command.motor_mask = 0x40U;
    assert(aethor_app_start_lifecycle_action(&command, &snapshot, MOTOR_RUNTIME_STATUS_OK, ++now_us));
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && !application_action.frames.count);
    assert(aethor_app_persistence_locked());
    puts("PERSISTENCE_APP_ALL_ADMISSION_GATES_PASS");
}

/** @brief An already populated ADRC output must stay populated under maintenance, even across a forged owner change. */
static void readonly_adrc_producer_gate(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    AethorPersistenceTransfer transfer;
    CanFrame frame;
    uint8_t kind;
    float torque;
    assert(aethor_app_persistence_begin_readonly(104U, now_us));
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
    application_adrc_bridge.frame_pending = 1U;
    application_adrc_bridge.frame = readonly_feedback(S3519_DRIVER_STATE_DISABLED);
    assert(!aethor_app_adrc_pop_frame(&frame, &kind, &torque));
    assert(application_adrc_bridge.frame_pending && !application_adrc_bridge.awaiting_receipt);
    aethor_app_persistence_service(++now_us);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
    application_adrc_bridge.frame_pending = 0U;
    assert(!aethor_app_persistence_next_transfer(++now_us, &transfer));
    puts("PERSISTENCE_APP_ADRC_POP_RETAINED_GATE_PASS");
}

/** @brief Epoch revocation latches immediately so a same-turn restoration cannot resurrect a reservation. */
static void readonly_epoch_immediate_revocation(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    uint32_t original_epoch = application_debug_ui.epoch;
    assert(aethor_app_persistence_begin_readonly(105U, now_us));
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_REMOTE);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    application_debug_ui.epoch = original_epoch;
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    puts("PERSISTENCE_APP_EPOCH_IMMEDIATE_REVOCATION_PASS");
}

/** @brief Admission rejects real source debt, wrong fixed discovery, weak feedback, or less than 4ms CAN quiet. */
static void readonly_entry_gates(void)
{
    unsigned gate;
    for (gate = 0U; gate < 21U; ++gate) {
        uint64_t now_us = readonly_fixture(1U, 12.5F);
        switch (gate) {
            case 0U: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_BUSY; break;
            case 1U: application_debug_ui.single_static_transport_mask = 0x40U; break;
            case 2U: application_debug_ui.pos_disable_pending = 1U; break;
            case 3U: application_transport_fault_write_sequence = 1U; break;
            case 4U: application_adrc_bridge.awaiting_receipt = 1U; break;
            case 5U: application_motor_runtime.parameter_expectations.count = 1U; break;
            case 6U: application_motor_runtime.bank.motors[6].feedback.driver_state = S3519_DRIVER_STATE_ENABLED; break;
            case 7U: application_motor_runtime.bank.motors[6].feedback.torque_nm = 0.151F; break;
            case 8U: application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 50.0F; break;
            case 9U: application_motor_runtime.discovery.results[6].software_version = 1U; break;
            case 10U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 1024.0F; break;
            case 11U: application_integrated_can_quiet_since_us = now_us - 3999U; break;
            case 12U: application_protocol_engine.result_write_sequence = 1U; break;
            case 13U: application_motor_runtime.position_read.pending = 1U; break;
            case 14U: application_coordinate_pair.state = 1U; break;
            case 15U: application_adrc_bridge.simple_guard_armed = 1U; break;
            case 16U: application_debug_ui.mailbox.stop_intent_mask = 0x01U; break;
            case 17U: application_controller.fault = ARM_FAULT_TRANSPORT; break;
            case 18U: application_adrc_bridge.bench.gateway.channel_state[0] = 1U; break;
            case 19U: application_adrc_bridge.bench.experiment.status.state = ADRC_STATE_RUNNING; break;
            default: now_us += 4001U; break;
        }
        assert(!aethor_app_persistence_begin_readonly(96U, now_us));
        assert(!aethor_app_persistence_locked());
    }
    puts("PERSISTENCE_APP_ENTRY_GATES_PASS");
}

/** @brief Wrong/no actual receipt, unrelated identity, old/same/ambiguous SOF and ordinary samples cannot refresh FD. */
static void readonly_fd_proof(void)
{
    uint64_t now_us = readonly_fixture(2U, 12.5F);
    AethorPersistenceTransfer transfer;
    DebugUiStaticTxIdentity wrong;
    CanFrame feedback = readonly_feedback(S3519_DRIVER_STATE_DISABLED);
    assert(aethor_app_persistence_begin_readonly(97U, now_us));
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, ++now_us));
    wrong = transfer.disable_identity; ++wrong.sequence;
    assert(!aethor_app_persistence_disable_transmitted(&wrong, 1U, 100U, ++now_us));
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &transfer.disable_identity, 101U, 1U));
    now_us += 3U;
    assert(!application_persistence.health_proven);
    assert(aethor_app_persistence_disable_transmitted(&transfer.disable_identity, 1U, 100U, ++now_us));
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &wrong, 101U, 1U));
    now_us += 3U;
    assert(!application_persistence.health_proven);
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &transfer.disable_identity, 100U, 1U));
    now_us += 3U;
    assert(!application_persistence.health_proven);
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &transfer.disable_identity, 99U, 1U));
    now_us += 3U;
    assert(!application_persistence.health_proven);
    assert(aethor_app_receive_can_frame_observed(&feedback, now_us + 1U, now_us + 2U) == MOTOR_RUNTIME_STATUS_OK);
    now_us += 3U;
    assert(!application_persistence.health_proven);
    assert(aethor_app_persistence_receive(&feedback, now_us + 1U, now_us + 2U, &transfer.disable_identity, 101U, 1U));
    now_us += 3U;
    assert(application_persistence.health_proven);
    assert(!aethor_app_persistence_next_transfer(now_us + 100001U, &transfer));
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
    puts("PERSISTENCE_APP_FD_ACTUAL_IDENTITY_SOF_PASS");
}

/** @brief A selected bad sample is sticky before a later good sample, and physical intent preserves earlier scope. */
static void readonly_bad_then_good(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    AethorPersistenceTransfer transfer;
    CanFrame bad = readonly_feedback(S3519_DRIVER_STATE_ENABLED);
    CanFrame good = readonly_feedback(S3519_DRIVER_STATE_DISABLED);
    assert(aethor_app_persistence_begin_readonly(98U, now_us));
    now_us = readonly_fd(now_us + 1U);
    /* The safety RX must be checked before a later benign RX can replace it. */
    assert(aethor_app_persistence_receive(&bad, now_us + 1U, now_us + 2U, NULL, 0U, 0U));
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH);
    assert(application_debug_ui.mailbox.stop_intent_mask & 0x40U);
    application_debug_ui.mailbox.stop_intent_mask |= 0x01U;
    assert(aethor_app_persistence_receive(&good, now_us + 3U, now_us + 4U, NULL, 0U, 0U));
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_HEALTH && !application_persistence.health_proven);
    assert(application_debug_ui.mailbox.stop_intent_mask == 0x41U);
    assert(!aethor_app_persistence_next_transfer(now_us + 5U, &transfer));
    puts("PERSISTENCE_APP_BAD_THEN_GOOD_STICKY_PASS");
}

/** @brief STOP and link restoration revoke permits permanently; late exact receipts retire only original debt. */
static void readonly_preemption_and_debt(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    AethorPersistenceTransfer transfer;
    MotorPersistenceIdentity cancellation, wrong;
    ProtocolOutputBatch output;
    CanFrame emergency;
    uint8_t emergency_scope = 0U;
    unsigned emergency_count = 0U;
    assert(aethor_app_persistence_begin_readonly(99U, now_us));
    now_us = readonly_fd(now_us + 1U);
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, ++now_us));
    (void)request("100 stop", ++now_us, &output);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_STOP);
    assert(!application_persistence_transaction.pending);
    assert(aethor_app_persistence_cancel_identity(&cancellation));
    assert(motor_persistence_identity_equal(&cancellation, &transfer.parameter_identity));
    wrong = cancellation; ++wrong.sequence;
    aethor_app_persistence_parameter_transmitted(&wrong, 1U, now_us, now_us + 1U);
    ++now_us;
    assert(aethor_app_persistence_cancel_identity(&cancellation));
    aethor_app_persistence_parameter_transmitted(&cancellation, 0U, now_us, now_us + 1U);
    assert(!aethor_app_persistence_cancel_identity(&cancellation));
    assert(!aethor_app_persistence_next_transfer(now_us + 2U, &transfer));
    (void)aethor_app_service(now_us + 3U);
    while (aethor_app_pop_emergency_can_frame(&emergency)) {
        uint8_t motor_id = (uint8_t)(emergency.identifier & 0xffU);
        assert(emergency.length == 8U && emergency.data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(motor_id >= 1U && motor_id <= 7U);
        emergency_scope |= (uint8_t)(1U << (motor_id - 1U));
        assert(++emergency_count <= MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES);
    }
    assert(emergency_scope == 0x7fU);
    now_us = readonly_fixture(2U, 512.0F);
    assert(aethor_app_persistence_begin_readonly(101U, now_us));
    aethor_app_persistence_observe_link(0x80000003UL, ++now_us);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    aethor_app_persistence_observe_link(0x80000001UL, ++now_us);
    assert(!aethor_app_persistence_next_transfer(++now_us, &transfer));
    assert(!aethor_app_persistence_begin_readonly(102U, ++now_us));
    puts("PERSISTENCE_APP_STOP_LINK_DEBT_NO_REVIVAL_PASS");
}

/** @brief Failed strict FD receipt retains the lock/debt; unknown and AA-shaped packets cannot corrupt MotorBank. */
static void readonly_quarantine_and_fd_failure(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    AethorPersistenceTransfer transfer;
    CanFrame malformed = {23U, 8U, {7U, 0U, 0xaaU, 2U}};
    uint32_t accepted_before = application_motor_runtime.accepted_feedback_count;
    assert(aethor_app_persistence_begin_readonly(103U, now_us));
    assert(aethor_app_receive_can_frame_observed(&malformed, now_us + 1U, now_us + 2U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.accepted_feedback_count == accepted_before);
    malformed.data[2] = 0x55U; malformed.data[3] = 21U;
    assert(aethor_app_receive_can_frame_observed(&malformed, now_us + 3U, now_us + 4U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.accepted_feedback_count == accepted_before);
    now_us += 5U;
    assert(aethor_app_persistence_next_transfer(now_us, &transfer));
    assert(aethor_app_persistence_disable_submitted(&transfer.disable_identity, ++now_us));
    assert(aethor_app_persistence_disable_transmitted(&transfer.disable_identity, 0U, 0U, ++now_us));
    assert(application_persistence.disable_outstanding && application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TX);
    assert(!aethor_app_persistence_next_transfer(++now_us, &transfer));
    puts("PERSISTENCE_APP_QUARANTINE_STRICT_FAILURE_LOCK_PASS");
}

/** @brief Rejected immutable transfers and backwards clocks cannot produce a second pair or a new transaction. */
static void readonly_rejection_and_clock(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    AethorPersistenceTransfer transfer, second;
    assert(aethor_app_persistence_begin_readonly(106U, now_us));
    now_us = readonly_fd(now_us + 1U);
    assert(aethor_app_persistence_next_transfer(++now_us, &transfer));
    aethor_app_persistence_transfer_rejected(&transfer, ++now_us);
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_TX);
    assert(!aethor_app_persistence_next_transfer(++now_us, &second));
    assert(!application_persistence.parameter_submitted);
    assert(!aethor_app_persistence_begin_readonly(107U, ++now_us));
    now_us = readonly_fixture(2U, 12.5F);
    assert(aethor_app_persistence_begin_readonly(108U, now_us));
    assert(!aethor_app_persistence_next_transfer(now_us - 1U, &second));
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CLOCK);
    now_us = readonly_fixture(1U, 12.5F);
    assert(aethor_app_persistence_begin_readonly(109U, now_us));
    aethor_app_init(now_us + 1U, 999U);
    assert(aethor_app_persistence_locked());
    assert(application_persistence_transaction.identity.request_id == 109U);
    assert(!aethor_app_persistence_next_transfer(now_us + 2U, &second));
    assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT);
    puts("PERSISTENCE_APP_REJECT_CLOCK_NO_RETRY_PASS");
}

/** @brief Bench/local STOP adds only its original targets and maintenance ID7; an existing global scope survives. */
static void readonly_stop_scope(void)
{
    uint64_t now_us = readonly_fixture(1U, 12.5F);
    ProtocolOutputBatch output;
    DebugUiRequest local;
    ProtocolCommand command;
    unsigned scenario;
    assert(aethor_app_persistence_begin_readonly(110U, now_us));
    (void)request("111 bench stop 1,3", ++now_us, &output);
    assert(application_debug_ui.mailbox.stop_intent_mask == 0x45U);
    now_us = readonly_fixture(1U, 12.5F);
    assert(aethor_app_persistence_begin_readonly(112U, now_us));
    (void)request("113 bench disable 7", ++now_us, &output);
    assert(application_debug_ui.mailbox.stop_intent_mask == 0x40U);
    for (scenario = 0U; scenario < 3U; ++scenario) {
        now_us = readonly_fixture(1U, 12.5F);
        assert(aethor_app_persistence_begin_readonly(114U, now_us));
        memset(&local, 0, sizeof(local));
        local.operation = DEBUG_UI_OPERATION_STOP; local.target_motor_id = 2U;
        local.identity.epoch = application_debug_ui.epoch; local.identity.request_id = 115U;
        local.created_at_us = now_us;
        if (scenario == 2U) application_debug_ui.mailbox.stop_intent_mask = 0x7fU;
        if (scenario == 1U) (void)aethor_app_debug_ui_admit_request(&local, ++now_us, &command);
        else (void)aethor_app_debug_ui_submit(&local);
        assert(application_debug_ui.mailbox.stop_intent_mask == (scenario == 2U ? 0x7fU : 0x42U));
    }
    puts("PERSISTENCE_APP_STOP_SCOPE_UNION_PASS");
}

/** @brief Completes one exact baseline pair through either RX-after-TX or deferred-ACK-before-actual-TX ordering. */
static uint64_t readonly_baseline_pair(uint64_t now_us, unsigned index, uint32_t raw, uint8_t early_ack)
{
    AethorPersistenceTransfer transfer;
    CanFrame reply = {23U, 8U, {7U, 0U, 0x33U, 0U}};
    unsigned byte;
    now_us += 100U;
    assert(aethor_app_persistence_next_transfer(now_us, &transfer));
    assert(transfer.kind == AETHOR_PERSISTENCE_TRANSFER_PARAMETER);
    assert(transfer.frame.data[3] == motor_persistence_rid(index));
    assert(aethor_app_persistence_parameter_submitted(&transfer.parameter_identity, now_us + 1U));
    reply.data[3] = transfer.frame.data[3];
    for (byte = 0U; byte < 4U; ++byte) reply.data[byte + 4U] = (uint8_t)(raw >> (8U * byte));
    if (early_ack) {
        assert(aethor_app_receive_can_frame_observed(&reply, now_us + 2U, now_us + 3U) == MOTOR_RUNTIME_STATUS_OK);
        assert(!application_persistence_transaction.current.items[index].valid);
        assert(!application_persistence.fault);
        aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 2U, now_us + 4U);
    } else {
        aethor_app_persistence_parameter_transmitted(&transfer.parameter_identity, 1U, now_us + 2U, now_us + 3U);
        assert(!application_persistence_transaction.current.items[index].valid);
        assert(aethor_app_receive_can_frame_observed(&reply, now_us + 4U, now_us + 5U) == MOTOR_RUNTIME_STATUS_OK);
    }
    assert(application_persistence_transaction.current.items[index].valid);
    return now_us + 6U;
}

/** @brief An actual baseline observation that contradicts frozen decode context must revoke immediately at that RID. */
static void readonly_baseline_context_contradiction(void)
{
    static const unsigned critical_indices[] = {10U, 21U, 7U, 8U, 13U, 14U, 22U, 23U, 36U};
    unsigned critical, direction, early_ack, index;
    for (critical = 0U; critical < sizeof(critical_indices) / sizeof(critical_indices[0]); ++critical) {
        for (direction = 0U; direction < 2U; ++direction) {
            for (early_ack = 0U; early_ack < 2U; ++early_ack) {
                uint32_t mode = direction ? 2U : 1U;
                float pmax = direction ? 512.0F : 12.5F;
                uint64_t now_us = readonly_fixture(mode, pmax);
                AethorPersistenceTransfer blocked;
                unsigned changed = critical_indices[critical];
                assert(aethor_app_persistence_begin_readonly(120U, now_us));
                now_us = readonly_fd(now_us + 1U);
                for (index = 0U; index <= changed; ++index) {
                    uint32_t raw = readonly_register(index);
                    if (index == changed) {
                        if (index == 10U) raw = direction ? 1U : 2U;
                        else if (index == 21U) raw = readonly_float_bits(direction ? 12.5F : 512.0F);
                        else if (index == 22U) raw = readonly_float_bits(201.0F);
                        else if (index == 23U) raw = readonly_float_bits(11.0F);
                        else ++raw;
                    }
                    now_us = readonly_baseline_pair(now_us, index, raw, (uint8_t)early_ack);
                    if (index != changed) assert(!application_persistence.fault);
                }
                printf("BASELINE_CONTEXT_CHECK rid=%u mode=%lu pmax=%.1f early_ack=%u state=%u fault=%u read_index=%u\n",
                    (unsigned)motor_persistence_rid(changed), (unsigned long)mode, (double)pmax, early_ack,
                    (unsigned)application_persistence_transaction.state, (unsigned)application_persistence.fault,
                    (unsigned)application_persistence_transaction.read_index);
                assert(application_persistence.fault == MOTOR_PERSISTENCE_ERROR_CONTEXT ||
                    application_persistence.fault == MOTOR_PERSISTENCE_ERROR_SNAPSHOT);
                assert(application_persistence_transaction.state == MOTOR_PERSISTENCE_ABORTED);
                assert(!application_persistence.health_proven && aethor_app_persistence_locked());
                assert(application_persistence.confirmed_mode == mode && application_persistence.ranges.position_max_rad == pmax);
                assert(application_motor_runtime.discovery.results[6].observed_control_mode == mode);
                assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == pmax);
                assert(!aethor_app_persistence_next_transfer(now_us + 1U, &blocked));
                assert(!application_persistence_transaction.save_used && !application_persistence_transaction.skip_save);
            }
        }
    }
    puts("PERSISTENCE_APP_BASELINE_CONTEXT_CONTRADICTION_PASS cases=36");
}

/** @brief Exercises the real facade with explicitly synthetic Classic CAN observations. */
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0U);
    readonly_aa_isolation();
    readonly_entry_gates();
    readonly_baseline(1U, 12.5F);
    readonly_baseline(2U, 12.5F);
    readonly_baseline(1U, 512.0F);
    readonly_baseline(2U, 512.0F);
    readonly_admission_gates();
    readonly_adrc_producer_gate();
    readonly_epoch_immediate_revocation();
    readonly_fd_proof();
    readonly_bad_then_good();
    readonly_preemption_and_debt();
    readonly_quarantine_and_fd_failure();
    readonly_rejection_and_clock();
    readonly_stop_scope();
    readonly_baseline_context_contradiction();
    assert(sizeof(application_persistence) <= 256U);
    printf("PERSISTENCE_APP_STATIC_STORAGE transaction_bytes=%u wrapper_bytes=%u\n",
        (unsigned)sizeof(application_persistence_transaction), (unsigned)sizeof(application_persistence));
    puts("PERSISTENCE_APP_READONLY_FACADE_SYNTHETIC_PASS");
    return 0;
}
