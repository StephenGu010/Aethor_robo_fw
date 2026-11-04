/** @file pos_load_range_cleanup_cases.inc
 * @brief Real-facade regressions for captured PMAX512 travel and bounded terminal feedback acquisition.
 * Included by the host POS suite; all samples, receipts and clocks are synthetic.
 */

/** @brief The captured native start is valid for POS load while ordinary POS remains clamped. */
static void pos_load_captured_range_cases(void)
{
    unsigned direction;
    for (direction = 0U; direction < 2U; ++direction) {
        DebugUiSnapshot view;
        MotorFeedbackSnapshot snapshot;
        MotorJointFeedback feedback = {0};
        DebugUiMotorProfile *profile;
        float start = direction ? -12.726745605F : 12.726745605F;
        aethor_app_init(1U, 77U);
        application_motor_runtime.discovery_active = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        application_motor_runtime.discovery.results[6].verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
        application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
        application_motor_runtime.discovery.results[6].observed_esc_id = 7U;
        application_motor_runtime.discovery.results[6].observed_master_id = 23U;
        application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 1200.0F;
        application_motor_runtime.discovery.results[6].ranges = (S3519Ranges){512.0F, 200.0F, 10.0F};
        application_motor_runtime.discovery.verified_joint_mask = 0x40U;
        feedback.position_rad = start; feedback.timestamp_us = 1000U;
        feedback.driver_state = S3519_DRIVER_STATE_DISABLED;
        feedback.mos_temperature_c = feedback.rotor_temperature_c = 30.0F;
        assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
        test_health(1000U, true);
        assert(aethor_app_debug_ui_get_snapshot(1000U, &view));
        /* Publish the actual captured native ordinary profile independently of compile-time axis selection. */
        profile = &application_debug_ui.profiles[6];
        profile->pos_valid = 1U; profile->position_min_rad = -12.5F; profile->position_max_rad = 12.5F;
        profile->max_delta_rad = 25.0F; profile->max_speed_rad_s = 200.0F;
        feedback.driver_state = S3519_DRIVER_STATE_ENABLED; feedback.timestamp_us = 2000U;
        assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
        application_action.command.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
        application_action.command.bench_relative_scope = 1U; application_action.command.motor_mask = 0x40U;
        application_action.motion_plan.start_position_rad[6] = start;
        application_action.target_position_rad[6] = start + (direction ? 39.876F : -39.876F);
        assert(aethor_app_bench_pos_feedback_safe(&snapshot, 6U, 2000U));
        application_action.command.local_operation = DEBUG_UI_OPERATION_POS_MOVE;
        assert(!aethor_app_bench_pos_feedback_safe(&snapshot, 6U, 2000U));
        assert(application_action.local_failure_reason == DEBUG_UI_REASON_OUT_OF_RANGE);
    }
    puts("POS_LOAD_CAPTURED_RANGE_PASS signed_native_start=2 ordinary_pos_preserved=1 explicit_reason=1");
}

/** @brief Moving disabled feedback requests another bounded DISABLE without extending the action deadline. */
static void pos_load_cleanup_refresh_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot snapshot;
    CanFrame frame;
    CanTxPriority priority;
    DebugUiStaticTxIdentity first, next;
    uint64_t deadline;
    pos_load_fixture(&command, &snapshot);
    (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
    pos_load_drain(2300U, 1U);
    first = application_debug_ui.pos_disable_identity;
    deadline = application_action.deadline_us;
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 2400U, &first);
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = -.4395599365F;
    assert(aethor_app_next_can_frame(53000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(aethor_app_static_frame_identity(&frame, &next));
    assert(next.sequence != first.sequence && next.motor_id == first.motor_id);
    assert(application_action.deadline_us == deadline);
    aethor_app_static_can_queued(&frame, 1U, 1U, 53000U);
    assert(aethor_app_static_submitted(&next, 53000U));
    aethor_app_static_transmitted(&first, 1U, 100U, 53001U);
    assert(!application_debug_ui.pos_disable_confirmed);
    aethor_app_static_transmitted(&next, 1U, 100U, 53001U);
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.0F;
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 54000U, &next);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 54000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_advance_one_shot_setup(&snapshot, 54001U));
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    puts("POS_LOAD_CLEANUP_REFRESH_PASS moving_sample_not_accepted=1 new_identity=1 old_receipt_rejected=1 deadline_fixed=1");
}

/** @brief Silence, pending/failed TX, faults and motion cannot create a pass or an unbounded sender. */
static void pos_load_cleanup_refresh_limits(void)
{
    unsigned scenario, attempt;
    for (scenario = 0U; scenario < 7U; ++scenario) {
        ProtocolCommand command;
        MotorFeedbackSnapshot snapshot;
        CanFrame frame;
        CanTxPriority priority;
        uint64_t deadline;
        pos_load_fixture(&command, &snapshot);
        (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
            PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
        pos_load_drain(2300U, 1U);
        deadline = application_action.deadline_us;
        application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = -.4395599365F;
        if (scenario == 1U) application_debug_ui.pos_disable_confirmed = 0U;
        if (scenario == 2U) application_debug_ui.pos_disable_failed = 1U;
        if (scenario == 3U) application_debug_ui.pos_disable_pending = 1U;
        if (scenario == 4U) application_motor_runtime.bank.motors[0].feedback.fault_flags = 1U;
        if (scenario == 5U) ++application_protocol_engine.stop_write_sequence;
        for (attempt = 1U; attempt <= 10U; ++attempt) {
            uint64_t now = 2300ULL + attempt * 50000ULL;
            MotorRuntimeStatus status = aethor_app_next_can_frame(now, &frame, &priority);
            if ((scenario == 0U || scenario == 6U) && attempt < 8U) {
                DebugUiStaticTxIdentity identity;
                assert(status == MOTOR_RUNTIME_STATUS_FRAME_READY);
                assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE && priority == CAN_TX_PRIORITY_EMERGENCY);
                assert(aethor_app_static_frame_identity(&frame, &identity));
                aethor_app_static_can_queued(&frame, 1U, 1U, now);
                assert(aethor_app_static_submitted(&identity, now));
                aethor_app_static_transmitted(&identity, 1U, 100U, now);
                /* Scenario zero supplies moving samples; scenario six remains completely silent. */
                if (scenario == 0U) startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, now + 1ULL, &identity);
            } else assert(status != MOTOR_RUNTIME_STATUS_FRAME_READY);
            assert(application_action.deadline_us == deadline);
        }
        assert(application_debug_ui.pos_disable_attempt_count <= 8U);
        assert(!application_debug_ui.pos_load.disabled_confirmed);
        application_protocol_engine.stop_read_sequence = application_protocol_engine.stop_write_sequence;
        memset(&snapshot, 0, sizeof(snapshot));
        assert(aethor_app_advance_one_shot_setup(&snapshot, deadline));
        aethor_app_debug_ui_route_results();
        assert(!application_debug_ui.pos_load.disabled_confirmed);
    }
    puts("POS_LOAD_CLEANUP_LIMITS_PASS scenarios=7 maximum8=1 no_deadline_extension=1 no_false_confirmation=1");
}

/** @brief A later explicit same-axis STOP binds a failed retained POS report, never the previous TX proof. */
static void pos_load_idle_stop_rebind_case(void)
{
    ProtocolCommand command, stop = {0};
    MotorFeedbackSnapshot snapshot;
    pos_load_fixture(&command, &snapshot);
    (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
    pos_load_drain(2300U, 1U);
    assert(aethor_app_advance_one_shot_setup(&snapshot, application_action.deadline_us));
    aethor_app_debug_ui_route_results();
    stop.type = PROTOCOL_COMMAND_STOP; stop.local_operation = DEBUG_UI_OPERATION_STOP;
    stop.origin = DEBUG_UI_ORIGIN_LOCAL_UI; stop.session_id = application_debug_ui.epoch;
    stop.request_id = 999U; stop.motor_mask = 1U; stop.bench_relative_scope = 1U; stop.cleanup_after_stop = 1U;
    stop.motor_mask = 3U;
    aethor_app_pos_load_bind_stop(&stop, 600000U);
    assert(!application_debug_ui.pos_load.stop_identity.request_id);
    stop.motor_mask = 1U;
    (void)aethor_app_start_lifecycle_action(&stop, &snapshot, MOTOR_RUNTIME_STATUS_OK, 600000U);
    assert(aethor_app_is_pos_load_stop());
    assert(application_debug_ui.pos_load.stop_identity.request_id == 999U);
    pos_load_drain(600010U, 1U);
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 600020U, &application_debug_ui.pos_disable_identity);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 600020U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_service_active_action(&snapshot, 600021U));
    aethor_app_debug_ui_route_results();
    assert(application_debug_ui.pos_load.disabled_confirmed && application_debug_ui.pos_load.stopped);
    assert(!application_debug_ui.pos_load.complete);
    puts("POS_LOAD_IDLE_STOP_REBIND_PASS retained_failed_report=1 same_axis_only=1");
}

/** @brief STOP remains immediately routable while old terminal TX debt retires, then requires a new proof. */
static void pos_load_stop_pending_terminal_debt(void)
{
    ProtocolCommand command, stop = {0};
    MotorFeedbackSnapshot snapshot;
    DebugUiStaticTxIdentity old, next;
    CanFrame frame;
    CanTxPriority priority;
    pos_load_fixture(&command, &snapshot);
    (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_MOTION,
        PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 1U, 2200U);
    pos_test_terminal_receipts_enabled = 0U;
    pos_load_drain(2300U, 1U);
    old = application_debug_ui.pos_disable_identity;
    assert(aethor_app_static_submitted(&old, 2400U));
    stop.type = PROTOCOL_COMMAND_STOP; stop.local_operation = DEBUG_UI_OPERATION_STOP;
    stop.origin = DEBUG_UI_ORIGIN_LOCAL_UI; stop.session_id = application_debug_ui.epoch;
    stop.request_id = 999U; stop.motor_mask = 1U; stop.bench_relative_scope = 1U; stop.cleanup_after_stop = 1U;
    (void)aethor_app_start_lifecycle_action(&stop, &snapshot, MOTOR_RUNTIME_STATUS_OK, 3000U);
    assert(aethor_app_next_can_frame(3100U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE && priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(!aethor_app_pos_load_terminal_frame_identity(&frame, &next));
    aethor_app_static_can_queued(&frame, 1U, 0U, 3100U);
    aethor_app_static_transmitted(&old, 1U, 100U, 3200U);
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 3300U, &old);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 3300U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_pos_disable_proven(&snapshot));
    assert(aethor_app_next_can_frame(54000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_static_frame_identity(&frame, &next) && next.sequence != old.sequence);
    aethor_app_static_can_queued(&frame, 1U, 1U, 54000U);
    assert(aethor_app_static_submitted(&next, 54000U));
    aethor_app_static_transmitted(&next, 1U, 100U, 54001U);
    startup_test_feedback(1U, S3519_DRIVER_STATE_DISABLED, 55000U, &next);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 55000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_pos_disable_proven(&snapshot));
    pos_test_terminal_receipts_enabled = 1U;
    puts("POS_LOAD_STOP_PENDING_DEBT_PASS immediate_disable=1 old_proof_not_reused=1 retire_then_new_identity=1");
}
