/** @file pos_load_app_test.c
 * @brief Synthetic production-facade POS loaded-motion tests; no hardware access.
 */
#define main existing_debug_ui_test_main
#include "debug_ui_app_test_main.c"
#undef main

/** @brief The typed contract locks by default and rejects forged, multiple-axis or unbounded commands. */
static void pos_load_typed_gate(void)
{
    ProtocolCommand command;
    memset(&command, 0, sizeof(command));
    command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    command.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    command.motor_mask = 1U;
    command.bench_relative_scope = command.values_in_radians = command.relative_target = 1U;
    command.speeds[0] = 0.02F;
    command.hold_duration_us = 1000000ULL;
#if !AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_DISABLED);
    puts("POS_LOAD_DEFAULT_LOCK_PASS");
#else
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_NONE);
    command.hold_duration_us = 60000001ULL;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    command.hold_duration_us = 1000000ULL;
    command.motor_mask = 3U;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    command.motor_mask = 1U;
    command.origin = DEBUG_UI_ORIGIN_USB;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    puts("POS_LOAD_TYPED_BOUNDS_PASS pos_only_no_mit_gate=1");
#endif
}

#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
#include "static_startup_test_helpers.inc"
static uint8_t pos_test_terminal_receipts_enabled = 1U;
/** @brief Consume every current frame and prove only selected POS controls or explicit selected disable. */
static void pos_load_drain(uint64_t timestamp_us, uint8_t disable)
{
    CanFrame frame;
    CanTxPriority priority;
    unsigned frames = 0U;
    while (application_action.frame_read_index < application_action.frames.count) {
        DebugUiStaticTxIdentity terminal_identity;
        uint8_t strict_terminal = 0U;
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == 0x101U || (disable && frame.identifier == 1U));
        assert(frame.length == 8U);
        if (disable) assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        else {
            float speed;
            memcpy(&speed, frame.data + 4U, sizeof(speed));
            assert(speed > 0.0F && speed <= 0.02F);
        }
        if (disable) strict_terminal = aethor_app_static_frame_identity(&frame, &terminal_identity);
        aethor_app_static_can_queued(&frame, 1U, strict_terminal, timestamp_us);
        if (strict_terminal && pos_test_terminal_receipts_enabled) {
            assert(terminal_identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE);
            assert(aethor_app_static_submitted(&terminal_identity, timestamp_us));
            aethor_app_static_transmitted(&terminal_identity, 1U, 100U, timestamp_us);
        }
        assert(++frames <= 2U);
    }
}

/** @brief Reach the strict POS preload through the real mailbox, protocol ring, disabled preflight and validation. */
static void pos_load_fixture(ProtocolCommand *command, MotorFeedbackSnapshot *snapshot)
{
    DebugUiRequest request;
    pos_test_terminal_receipts_enabled = 1U;
    test_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_POS_LOAD_TEST, 1001U);
    request.kp = request.kd = 0.0F;
    request.delta_rad = 0.005F;
    request.hold_duration_ms = 5000U;
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, command));
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_start_one_shot_move(command, snapshot, 1002U));
    pos_load_drain(1003U, 1U);
    test_feedback(2000U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(snapshot, 2001U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
    assert(application_motor_runtime.discovery.results[0].observed_control_mode == 2U);
}

/** @brief Real encoded TX and ordered RX barriers release exactly PRELOAD, ENABLE, FIRST_HOLD into motion. */
static void pos_load_release_preload(MotorFeedbackSnapshot *snapshot, uint64_t first_us)
{
    unsigned phase;
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
        DebugUiStaticTxIdentity identity;
        CanFrame frame;
        CanTxPriority priority;
        uint64_t timestamp_us = first_us + phase * 10ULL;
        test_health(timestamp_us, true);
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == 0x101U);
        assert(aethor_app_static_frame_identity(&frame, &identity) && identity.phase == phase);
        assert(aethor_app_static_submitted(&identity, timestamp_us + 1ULL));
        aethor_app_static_transmitted(&identity, 1U, 100U, timestamp_us + 1ULL);
        if (phase == DEBUG_UI_STATIC_TX_ENABLE) continue;
        startup_test_feedback(1U, phase == DEBUG_UI_STATIC_TX_PRELOAD ?
            S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED, timestamp_us + 2ULL, &identity);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us + 3ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(snapshot, timestamp_us + 3ULL));
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
    pos_load_drain(first_us + 90ULL, 0U);
}

/** @brief Establish an admitted fixture and release its real ordered startup barriers. */
static void pos_load_start(MotorFeedbackSnapshot *snapshot)
{
    ProtocolCommand command;
    pos_load_fixture(&command, snapshot);
    pos_load_release_preload(snapshot, 2010ULL);
}

/** @brief Feed one new selected sample through production protection/phase handling. */
static void pos_load_step(MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us, float position)
{
    uint8_t failed;
    test_feedback(timestamp_us, position, S3519_DRIVER_STATE_ENABLED);
    test_health(timestamp_us, true);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    (void)aethor_app_check_one_shot_motor_safety(snapshot, timestamp_us, &failed);
    if (!failed) (void)aethor_app_advance_one_shot_setup(snapshot, timestamp_us);
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT ||
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) pos_load_drain(timestamp_us, 0U);
}

/** @brief Establish a 250ms stable endpoint using only fresh samples and the configured POS speed ceiling. */
static uint64_t pos_load_reach_hold(MotorFeedbackSnapshot *snapshot)
{
    uint64_t timestamp_us;
    pos_load_start(snapshot);
    for (timestamp_us = 10000U; timestamp_us <= 270000U; timestamp_us += 10000U)
        pos_load_step(snapshot, timestamp_us, application_action.target_position_rad[0]);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    assert(application_debug_ui.pos_load.stage == DEBUG_UI_STAGE_HOLD);
    return timestamp_us;
}

/** @brief Continuous POS hold observes hysteresis, rejects duplicate stability, and never extends its original expiry. */
static void pos_load_recovery_cases(void)
{
    MotorFeedbackSnapshot snapshot;
    uint64_t timestamp_us = pos_load_reach_hold(&snapshot);
    uint64_t deadline_us = application_action.deadline_us;
    uint32_t sample_count;
    float target = application_action.target_position_rad[0];
    pos_load_step(&snapshot, timestamp_us, target + 0.3F * AETHOR_APP_DEG_TO_RAD);
    assert(application_debug_ui.pos_load.disturbances == 1U && application_debug_ui.pos_load.recovering);
    sample_count = application_debug_ui.pos_load.samples;
    assert(!aethor_app_pos_load_sample(&snapshot, timestamp_us + 1ULL));
    assert(application_debug_ui.pos_load.samples == sample_count);
    for (timestamp_us += 10000U; timestamp_us <= 580000U; timestamp_us += 10000U)
        pos_load_step(&snapshot, timestamp_us, target);
    assert(application_debug_ui.pos_load.recoveries == 1U && application_debug_ui.pos_load.recovery_observed);
    if (application_debug_ui.pos_load.recovering || application_action.deadline_us != deadline_us)
        fprintf(stderr, "POS_RECOVERY_DIAGNOSTIC state=%u recovering=%u now_deadline=%llu initial=%llu reason=%u error=%u\n",
            (unsigned)application_action.state, application_debug_ui.pos_load.recovering,
            (unsigned long long)application_action.deadline_us, (unsigned long long)deadline_us,
            (unsigned)application_action.local_failure_reason, (unsigned)application_action.failure_error);
    assert(!application_debug_ui.pos_load.recovering && application_action.deadline_us == deadline_us);
    for (; timestamp_us <= deadline_us; timestamp_us += 10000U) pos_load_step(&snapshot, timestamp_us, target);
    if (snapshot.joints[0].timestamp_us < deadline_us) pos_load_step(&snapshot, deadline_us, target);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT);
    pos_load_drain(deadline_us + 1ULL, 1U);
    puts("POS_LOAD_CONTINUOUS_RECOVERY_PASS all_mode2_nonzero_speed=1 stable250ms=1 hysteresis=1 deadline_fixed=1");
}

/** @brief Hard protections retain their exact original thresholds and jump directly to disable without HOLD. */
static void pos_load_guard_cases(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 9U; ++scenario) {
        MotorFeedbackSnapshot snapshot;
        uint8_t failed = 0U;
        uint64_t timestamp_us = pos_load_reach_hold(&snapshot);
        snapshot.joints[0].timestamp_us = timestamp_us;
        if (scenario == 0U) snapshot.joints[0].torque_nm = 3.6F;
        if (scenario == 1U) snapshot.joints[0].mos_temperature_c = 56.0F;
        if (scenario == 2U) snapshot.joints[0].rotor_temperature_c = 56.0F;
        if (scenario == 3U) snapshot.joints[0].position_rad += 0.6F * AETHOR_APP_DEG_TO_RAD;
        if (scenario == 4U) snapshot.joints[0].velocity_rad_s = 1.0F;
        if (scenario == 5U) snapshot.joints[0].torque_nm = NAN;
        if (scenario == 6U) ++application_debug_ui.reference_generation[0];
        if (scenario == 7U) application_motor_runtime.discovery.results[0].observed_control_mode = 1U;
        if (scenario == 8U) snapshot.valid_joint_mask = 0U;
        (void)aethor_app_check_one_shot_motor_safety(&snapshot, timestamp_us, &failed);
        assert(failed && application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        pos_load_drain(timestamp_us + 1ULL, 1U);
    }
    puts("POS_LOAD_GUARDS_PASS torque_temp_error_speed_finite_reference_mode_stale=9 direct_disable=1");
}

/** @brief Lost TX receipt, old RX and a mismatched sequence cannot release the original preload barrier. */
static void pos_load_receipt_cases(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 4U; ++scenario) {
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        CanFrame frame;
        CanTxPriority priority;
        DebugUiStaticTxIdentity identity, wrong;
        pos_load_fixture(&command, &snapshot);
        assert(aethor_app_next_can_frame(2010U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity));
        assert(aethor_app_static_submitted(&identity, 2011U));
        wrong = identity; ++wrong.sequence;
        if (scenario == 1U) aethor_app_static_transmitted(&wrong, 1U, 100U, 2012U);
        if (scenario >= 2U) aethor_app_static_transmitted(&identity, 1U, 100U, 2012U);
        if (scenario == 3U) startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 2013U, &wrong);
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 2014U));
        assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
        assert(!application_action.static_preloaded_mask);
    }
    puts("POS_LOAD_RECEIPTS_PASS missing_wrong_tx_old_wrong_sequence_rx=4 no_enable=1");
}

/** @brief Complete bounded disable with a genuinely newer selected ACK and route the immutable terminal report. */
static DebugUiCompletion pos_load_finish_disabled(MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    DebugUiCompletion completion;
    pos_load_drain(timestamp_us, 1U);
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, timestamp_us + 1ULL,
        &application_debug_ui.pos_disable_identity);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us + 1ULL,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_advance_one_shot_setup(snapshot, timestamp_us + 2ULL));
    aethor_app_debug_ui_route_results();
    assert(aethor_app_debug_ui_poll_completion(&completion));
    return completion;
}

/** @brief No observed disturbance never manufactures recovery success; failed recovery is terminal even at total expiry. */
static void pos_load_outcome_cases(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 3U; ++scenario) {
        MotorFeedbackSnapshot snapshot;
        DebugUiCompletion completion;
        uint64_t timestamp_us = pos_load_reach_hold(&snapshot), deadline_us = application_action.deadline_us;
        float target = application_action.target_position_rad[0];
        if (scenario == 2U) {
            application_action.deadline_us = timestamp_us + 100000ULL;
            deadline_us = application_action.deadline_us;
        }
        for (; timestamp_us <= deadline_us; timestamp_us += 10000ULL) {
            pos_load_step(&snapshot, timestamp_us, target + (scenario ? 0.3F * AETHOR_APP_DEG_TO_RAD : 0.0F));
            if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) break;
        }
        assert(application_action.state == (scenario ? AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT :
            AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT));
        completion = pos_load_finish_disabled(&snapshot, timestamp_us + 10ULL);
        assert(completion.disabled_confirmed);
        if (!scenario) {
            assert(completion.code == DEBUG_UI_COMPLETED && application_debug_ui.pos_load.complete);
            assert(!application_debug_ui.pos_load.disturbances && !application_debug_ui.pos_load.recovery_observed);
        } else {
            assert(completion.code == DEBUG_UI_FAILED && !application_debug_ui.pos_load.complete);
            assert(application_debug_ui.pos_load.disturbances == 1U && !application_debug_ui.pos_load.recoveries);
        }
        assert(!application_debug_ui.pos_load.active && application_debug_ui.pos_load.disabled_confirmed);
        {
            DebugUiStaticTxIdentity stale_identity;
            assert(!aethor_app_pos_load_terminal_frame_identity(&application_debug_ui.pos_disable_frame, &stale_identity));
            assert(!application_debug_ui.pos_load_cleanup_required);
        }
    }
    puts("POS_LOAD_OUTCOMES_PASS unobserved_honest=1 recovery2s_failed=1 expiry_recovering_failed=1 fresh_disabled_report=3 legacy_disable_not_intercepted=3");
}

/** @brief STOP always replaces all three startup barriers, movement and hold with direct selected disable. */
static void pos_load_stop_cases(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario) {
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        DebugUiRequest stop;
        uint64_t timestamp_us = 2020U;
        if (scenario < 3U) {
            pos_load_fixture(&command, &snapshot);
            aethor_app_static_prepare_phase((uint8_t)(DEBUG_UI_STATIC_TX_PRELOAD + scenario), 0U, 2010U);
        } else if (scenario == 3U) pos_load_start(&snapshot);
        else timestamp_us = pos_load_reach_hold(&snapshot);
        if (scenario == 3U) timestamp_us = 2200U;
        test_health(timestamp_us, true);
        stop = test_request(DEBUG_UI_OPERATION_STOP, timestamp_us);
        assert(test_admission(stop, timestamp_us).accepted);
        (void)aethor_app_service(timestamp_us + 1ULL);
        assert(application_action.command.local_operation == DEBUG_UI_OPERATION_STOP);
        assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
        assert(!application_action.control_group_ready);
        assert(application_debug_ui.pos_load.stop_identity.request_id == stop.identity.request_id);
        pos_load_drain(timestamp_us + 2ULL, 1U);
    }
    puts("POS_LOAD_STOP_PHASES_PASS preload_enable_firsthold_move_hold=5 direct_pos_disable=1");
}

/** @brief A saturated STOP still emits bounded disable and confirms only a newer actual selected safe sample. */
static void pos_load_saturated_stop(uint8_t same_tick)
{
    MotorFeedbackSnapshot snapshot;
    DebugUiRequest request;
    ProtocolCommandResult result;
    uint64_t timestamp_us = pos_load_reach_hold(&snapshot);
    uint8_t result_generated = 0U;
    unsigned index;
    memset(&result, 0, sizeof(result));
    result.origin = DEBUG_UI_ORIGIN_USB;
    result.type = PROTOCOL_COMMAND_DISABLE;
    result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    for (index = 0U; index < PROTOCOL_ENGINE_RESULT_CAPACITY + AETHOR_APP_DEFERRED_RESULT_CAPACITY; ++index) {
        result.request_id = 100U + index;
        assert(aethor_app_submit_or_defer_result(&result));
    }
    request = test_request(DEBUG_UI_OPERATION_STOP, timestamp_us);
    assert(test_admission(request, timestamp_us).accepted);
    assert(aethor_app_debug_ui_service_stop_intent(timestamp_us + 1ULL, &result_generated));
    assert(application_debug_ui.fallback_stop_mask == 1U && !application_action.control_group_ready);
    test_feedback(timestamp_us + 2ULL, 0.205F, S3519_DRIVER_STATE_DISABLED);
    while (application_debug_ui.fallback_frame_read_index < application_debug_ui.fallback_frames.count) {
        CanFrame frame;
        DebugUiStaticTxIdentity identity;
        assert(aethor_app_pop_emergency_can_frame(&frame));
        assert(frame.identifier == 0x101U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(aethor_app_static_frame_identity(&frame, &identity) && identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE);
        aethor_app_static_can_queued(&frame, 1U, 1U, timestamp_us + 3ULL);
    }
    assert(!application_debug_ui.pos_load.disabled_confirmed);
    assert(aethor_app_debug_ui_service_stop_intent(timestamp_us + 4ULL, &result_generated));
    assert(!application_debug_ui.pos_load.disabled_confirmed);
    test_feedback(timestamp_us + 5ULL, 0.205F, S3519_DRIVER_STATE_DISABLED);
    assert(aethor_app_debug_ui_service_stop_intent(timestamp_us + 6ULL, &result_generated));
    assert(!application_debug_ui.pos_load.disabled_confirmed);
    assert(aethor_app_static_submitted(&application_debug_ui.pos_disable_identity, timestamp_us + 7ULL));
    aethor_app_static_transmitted(&application_debug_ui.pos_disable_identity, 1U, 100U,
        timestamp_us + (same_tick ? 7ULL : 8ULL));
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED,
        timestamp_us + (same_tick ? 7ULL : 9ULL), &application_debug_ui.pos_disable_identity);
    assert(aethor_app_debug_ui_service_stop_intent(timestamp_us + 10ULL, &result_generated));
    assert(application_debug_ui.pos_load.disabled_confirmed && application_debug_ui.pos_load.stopped);
    assert(!application_debug_ui.pos_load.complete && !application_debug_ui.pos_load.active);
    result.origin = application_debug_ui.pos_load.identity.origin;
    result.session_id = application_debug_ui.pos_load.identity.epoch;
    result.request_id = application_debug_ui.pos_load.identity.request_id;
    result.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    result.code = PROTOCOL_COMMAND_RESULT_CANCELLED;
    result.disabled_confirmed = 0U;
    aethor_app_pos_load_finish(&result);
    assert(application_debug_ui.pos_load.disabled_confirmed && application_debug_ui.pos_load.stopped);
    puts("POS_LOAD_SATURATED_STOP_PASS result_fifos_full=1 bounded_mode2_disable=1 pre_emit_ack_rejected=1 newer_ack_only=1 delayed_cancel_preserves_confirmation=1 no_pass=1");
}

/** @brief Startup/movement peaks include the exact finite sample that trips torque/temperature protection. */
static void pos_load_early_guard_peaks(void)
{
    unsigned phase, scenario;
    for (phase = 0U; phase < 4U; ++phase) for (scenario = 0U; scenario < 3U; ++scenario) {
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        uint8_t failed;
        if (phase < 3U) {
            pos_load_fixture(&command, &snapshot);
            aethor_app_static_prepare_phase((uint8_t)(DEBUG_UI_STATIC_TX_PRELOAD + phase), 0U, 2010U);
        } else pos_load_start(&snapshot);
        snapshot.joints[0].timestamp_us = 2200U;
        if (scenario == 0U) snapshot.joints[0].torque_nm = 3.6F;
        if (scenario == 1U) snapshot.joints[0].mos_temperature_c = 56.0F;
        if (scenario == 2U) snapshot.joints[0].rotor_temperature_c = 56.0F;
        (void)aethor_app_check_one_shot_motor_safety(&snapshot, 2200U, &failed);
        assert(failed && application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        assert(application_debug_ui.pos_load.samples);
        if (scenario == 0U) assert(application_debug_ui.pos_load.peak_torque_nm == 3.6F);
        if (scenario == 1U) assert(application_debug_ui.pos_load.peak_mos_c == 56.0F);
        if (scenario == 2U) assert(application_debug_ui.pos_load.peak_rotor_c == 56.0F);
        assert(application_debug_ui.pos_load.peak_error_rad == 0.0F);
    }
    puts("POS_LOAD_EARLY_FAULT_PEAKS_PASS startup_phases_move=4 finite_threshold_sample=12 no_endpoint_tracking_invention=1");
}

/** @brief A disabled sample newer than queue admission still cannot complete cleanup before actual DISABLE TX and ordered RX. */
static void pos_load_terminal_requires_wire_proof(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot snapshot;
    pos_load_fixture(&command, &snapshot);
    (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
    pos_test_terminal_receipts_enabled = 0U;
    pos_load_drain(2210U, 1U);
    test_feedback(2220U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2220U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(&snapshot, 2221U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
    assert(aethor_app_static_submitted(&application_debug_ui.pos_disable_identity, 2230U));
    aethor_app_static_transmitted(&application_debug_ui.pos_disable_identity, 1U, 100U, 2231U);
    test_feedback(2240U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2240U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(&snapshot, 2241U));
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 2250U, &application_debug_ui.pos_disable_identity);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2250U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_advance_one_shot_setup(&snapshot, 2251U));
    pos_test_terminal_receipts_enabled = 1U;
    /* A completed TX with no ordered ACK times out, but cannot intercept a subsequent legacy recovery DISABLE. */
    pos_load_fixture(&command, &snapshot);
    (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
    pos_load_drain(2210U, 1U);
    test_feedback(application_action.deadline_us, 0.2F, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, application_action.deadline_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_advance_one_shot_setup(&snapshot, application_action.deadline_us));
    aethor_app_debug_ui_route_results();
    {
        DebugUiStaticTxIdentity retired_identity;
        assert(!aethor_app_pos_load_terminal_frame_identity(&application_debug_ui.pos_disable_frame, &retired_identity));
        assert(!application_debug_ui.pos_load.disabled_confirmed);
    }
    puts("POS_LOAD_TERMINAL_REQUIRES_WIRE_PROOF_PASS queued_new_disabled_does_not_complete=1 real_tx_plus_unproved_rx_rejected=1 ordered_posttx_ack_only=1");
}
#endif

/* Production-boundary regressions for the captured PMAX512 movement and terminal feedback failure. */
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
#include "pos_load_range_cleanup_cases.inc"
#endif

/** @brief Execute the independent gate and actual facade lifecycle regressions. */
static int pos_load_run_suite(void)
{
    pos_load_typed_gate();
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    pos_load_captured_range_cases();
    pos_load_cleanup_refresh_cases();
    pos_load_cleanup_refresh_limits();
    pos_load_idle_stop_rebind_case();
    pos_load_stop_pending_terminal_debt();
    (void)startup_test_finish;
    pos_load_terminal_requires_wire_proof();
    pos_load_recovery_cases();
    pos_load_guard_cases();
    pos_load_receipt_cases();
    pos_load_outcome_cases();
    pos_load_stop_cases();
    pos_load_saturated_stop(0U);
    pos_load_saturated_stop(1U);
    pos_load_early_guard_peaks();
#endif
    return 0;
}

#ifndef POS_LOAD_FIXTURE_ONLY
/** @brief Standalone runner; integration tests may reuse the same production fixtures. */
int main(void) { return pos_load_run_suite(); }
#endif
