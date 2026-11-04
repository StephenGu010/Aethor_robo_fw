/** @file static_load_app_test.c
 * @brief Real facade tests for default-locked seven-axis HOLD; synthetic feedback only.
 */
#define main existing_debug_ui_test_main
#include "debug_ui_app_test_main.c"
#undef main

#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
#include "static_startup_test_helpers.inc"
/** @brief Publishes every joint through the real motor bank with explicit per-axis identities. */
static void static_feedback(uint64_t timestamp_us, uint8_t state)
{
    uint8_t joint_index;
    for (joint_index = 0U; joint_index < 7U; ++joint_index)
    {
        MotorJointFeedback feedback;
        memset(&feedback, 0, sizeof(feedback));
        feedback.timestamp_us = timestamp_us;
        feedback.position_rad = 0.2F + 0.02F * joint_index;
        feedback.driver_state = state;
        feedback.mos_temperature_c = 30.0F;
        feedback.rotor_temperature_c = 31.0F;
        if (application_motor_runtime.bank.motors[joint_index].feedback.timestamp_us == timestamp_us) { continue; }
        assert(motor_bank_update_feedback(&application_motor_runtime.bank,
            application_motor_runtime.configuration->joints[joint_index].master_id,
            application_motor_runtime.configuration->joints[joint_index].esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
    }
}

/** @brief Synthetic seven-axis evidence is injected only into this host executable. */
static DebugUiRequest static_fixture(ProtocolCommand *command, MotorFeedbackSnapshot *feedback)
{
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    DebugUiMotorProfile profile;
    uint8_t joint_index;
    test_fixture();
    profile = application_debug_ui.profiles[0];
    for (joint_index = 0U; joint_index < 7U; ++joint_index)
    {
        MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        *discovery = application_motor_runtime.discovery.results[0];
        discovery->observed_control_mode = 1U;
        discovery->observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
        discovery->observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
        assert(aethor_app_debug_ui_set_motor_profile((uint8_t)(joint_index + 1U), &profile));
    }
    application_motor_runtime.discovery.verified_joint_mask = 0x7FU;
    static_feedback(1000U, S3519_DRIVER_STATE_DISABLED);
    test_acquire();
    request = test_request(DEBUG_UI_OPERATION_STATIC_HOLD, 1001U);
    request.delta_rad = request.speed_rad_s = request.kp = request.kd = 0.0F;
    assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
    assert(snapshot.static_load_enabled);
    for (joint_index = 0U; joint_index < 7U; ++joint_index)
    {
        request.static_positions_rad[joint_index] = snapshot.motors[joint_index].position_rad;
        request.static_references[joint_index] = snapshot.motors[joint_index].reference_generation;
    }
    aethor_app_debug_ui_build_command(&request, 1001U, command);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U, MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, feedback) == MOTOR_RUNTIME_STATUS_OK);
    return request;
}

/** @brief All seven references, modes, poses, gains and finite measurements are independent gates. */
static void static_admission_cases(void)
{
    ProtocolCommand command, queued;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest request;
    unsigned joint_index, scenario;
    request = static_fixture(&command, &feedback);
    assert(command.motor_mask == 0x7FU && !command.relative_target);
    assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_NONE);
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_NONE);
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, &queued));
    assert(queued.motor_mask == 0x7FU && queued.local_operation == DEBUG_UI_OPERATION_STATIC_HOLD);
    for (joint_index = 0U; joint_index < 7U; ++joint_index)
        for (scenario = 0U; scenario < 7U; ++scenario)
        {
            (void)static_fixture(&command, &feedback);
            if (scenario == 0U) { ++command.static_reference_generations[joint_index]; }
            if (scenario == 1U) { feedback.valid_joint_mask &= (uint8_t)~(1U << joint_index); }
            if (scenario == 2U) { command.values[joint_index] += aethor_app_lcd_position_ratio((uint8_t)joint_index); }
            if (scenario == 3U) { application_debug_ui.profiles[joint_index].default_kp += 0.1F; }
            if (scenario == 4U) { feedback.joints[joint_index].mos_temperature_c = 56.0F; }
            if (scenario == 5U) { feedback.joints[joint_index].torque_nm = NAN; }
            if (scenario == 6U) { application_motor_runtime.discovery.results[joint_index].observed_control_mode = 2U; }
            assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
        }
    (void)static_fixture(&command, &feedback);
    command.origin = DEBUG_UI_ORIGIN_USB;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    (void)static_fixture(&command, &feedback);
    command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
    puts("STATIC_LOAD_ADMISSION_PASS per_axis_cases=49 usb_group_rejected=1");
}

/** @brief Pops a whole real facade batch, proving scope and distinguishing ENABLE/HOLD/DISABLE. */
static void static_take_batch(uint64_t timestamp_us, int final_byte)
{
    uint8_t index;
    for (index = 0U; index < 7U; ++index)
    {
        CanFrame frame;
        CanTxPriority priority;
        assert(aethor_app_next_can_frame(timestamp_us + index, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.identifier == application_motor_runtime.configuration->joints[index].esc_id);
        if (final_byte >= 0) { assert(frame.data[7] == (uint8_t)final_byte); }
    }
}

/** @brief Drives the production preflight disabled ACK, ENABLE batch and newly enabled ACK into HOLD. */
static void static_start(ProtocolCommand *command, MotorFeedbackSnapshot *feedback)
{
    DebugUiRequest request = static_fixture(command, feedback);
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, command));
    memset(&application_action, 0, sizeof(application_action));
    application_action.command = *command;
    aethor_app_static_trace_begin(1002U);
    assert(!aethor_app_begin_one_shot_preflight_disable(feedback, 1002U));
    static_take_batch(1003U, S3519_MODE_COMMAND_DISABLE);
    static_feedback(2000U, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U, MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, feedback) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(feedback, 2001U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    startup_test_finish(feedback, 2002U, 3001U);
    assert(application_action.frames.count == 7U);
}

/** @brief Seven emitted HOLD frames, distinct sampling and a real deadline followed by seven fresh disabled ACKs. */
static void static_lifecycle(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiSnapshot snapshot;
    DebugUiCompletion completion;
    uint64_t tick;
    unsigned index;
    static_start(&command, &feedback);
    static_take_batch(3002U, -1);
    for (index = 0U; index < 7U; ++index)
    { assert(fabsf(application_action.target_position_rad[index] - command.values[index]) < 0.000001F); }
    static_feedback(4000U, S3519_DRIVER_STATE_ENABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000U, MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
    aethor_app_static_load_sample(&feedback, 4000U);
    aethor_app_static_load_sample(&feedback, 4001U);
    for (index = 0U; index < 7U; ++index) { assert(application_debug_ui.static_load.samples[index] == 1U); }
    assert(application_action.deadline_us == 103001U);
    for (tick = 4000U; tick < 103001U; tick += 4000U)
    {
        static_feedback(tick, S3519_DRIVER_STATE_ENABLED);
        test_health(tick, true);
        (void)aethor_app_service(tick);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        static_take_batch(tick + 1U, -1);
    }
    static_feedback(103001U, S3519_DRIVER_STATE_ENABLED);
    test_health(103001U, true);
    (void)aethor_app_service(103001U);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT);
    static_take_batch(103002U, S3519_MODE_COMMAND_DISABLE);
    static_feedback(104000U, S3519_DRIVER_STATE_DISABLED);
    /* One old enabled sample must prevent terminal completion for the entire group. */
    application_motor_runtime.bank.motors[6].feedback.driver_state = S3519_DRIVER_STATE_ENABLED;
    (void)aethor_app_service(104001U);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT);
    assert(!application_debug_ui.static_load.complete);
    static_feedback(105000U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(105001U);
    aethor_app_debug_ui_process(105001U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_COMPLETED && completion.motor_mask == 0x7FU && completion.disabled_confirmed);
    assert(aethor_app_debug_ui_get_snapshot(105001U, &snapshot));
    assert(snapshot.static_load.complete && snapshot.static_load.valid_mask == 0x7FU);
    assert(snapshot.static_load.frames_issued_mask == 0x7FU && !snapshot.static_load.active);
    puts("STATIC_LOAD_LIFECYCLE_PASS preflight=7 enable=7 hold=7 disable=7 missing_ack_blocks=1 duplicate_sample_blocks=1");
}

/** @brief Every axis can abort the whole group, including ENABLE_WAIT drift and a blocked HOLD sender. */
static void static_guard_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint8_t failure;
    unsigned joint_index, scenario;
    for (joint_index = 0U; joint_index < 7U; ++joint_index)
        for (scenario = 0U; scenario < 6U; ++scenario)
        {
            static_start(&command, &feedback);
            if (scenario == 0U) { feedback.joints[joint_index].torque_nm = 3.6F; }
            if (scenario == 1U) { feedback.joints[joint_index].rotor_temperature_c = 56.0F; }
            if (scenario == 2U) { feedback.joints[joint_index].position_rad += aethor_app_lcd_position_ratio((uint8_t)joint_index); }
            if (scenario == 3U) {
                application_action.state = AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT;
                feedback.joints[joint_index].position_rad += aethor_app_lcd_position_ratio((uint8_t)joint_index);
            }
            if (scenario == 4U) { ++application_debug_ui.reference_generation[joint_index]; }
            if (scenario == 5U) { feedback.joints[joint_index].driver_state = S3519_DRIVER_STATE_DISABLED; }
            (void)aethor_app_check_one_shot_motor_safety(&feedback, 3002U, &failure);
            assert(failure && application_action.command.motor_mask == 0x7FU);
            assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        }
    static_start(&command, &feedback);
    (void)aethor_app_check_one_shot_motor_safety(&feedback, 7002U, &failure);
    assert(failure && !application_debug_ui.static_load.complete);
    puts("STATIC_LOAD_GUARDS_PASS cases=42 first_batch_sender_stall_blocks=1");
}

/** @brief Original cancellation and a distinct STOP terminal must retain one correctly confirmed trial report. */
static void static_stop_lifecycle(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest stop;
    ProtocolCommandResult unrelated;
    static_start(&command, &feedback);
    static_take_batch(3002U, -1);
    stop = test_request(DEBUG_UI_OPERATION_STOP, 4000U);
    assert(test_admission(stop, 4000U).accepted);
    static_feedback(4000U, S3519_DRIVER_STATE_ENABLED);
    test_health(4000U, true);
    (void)aethor_app_service(4001U);
    assert(application_debug_ui.static_load.stop_identity.request_id == stop.identity.request_id);
    assert(!application_debug_ui.static_load.disabled_confirmed);
    static_take_batch(4002U, -1);
    (void)aethor_app_service(5000U);
    if (application_action.state != AETHOR_APP_ACTION_IDLE) { static_take_batch(5001U, S3519_MODE_COMMAND_DISABLE); }
    static_feedback(6000U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(6001U);
    aethor_app_debug_ui_process(6001U);
    assert(application_debug_ui.static_load.stopped);
    assert(application_debug_ui.static_load.disabled_confirmed && !application_debug_ui.static_load.complete);
    memset(&unrelated, 0, sizeof(unrelated));
    unrelated.local_operation = DEBUG_UI_OPERATION_STOP;
    unrelated.request_id = stop.identity.request_id + 1U;
    unrelated.session_id = stop.identity.epoch;
    unrelated.motor_mask = 0x7FU;
    aethor_app_static_load_finish(&unrelated);
    assert(application_debug_ui.static_load.disabled_confirmed);
    puts("STATIC_LOAD_STOP_IDENTITY_PASS cancelled_trial_plus_bound_stop=1 unrelated_terminal_ignored=1");
}

/** @brief Saturated STOP keeps causal seven-axis feedback confirmation when old cancellation is delivered later. */
static void static_saturated_stop(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest stop;
    ProtocolCommandResult blocked;
    ProtocolOutputBatch output;
    CanFrame frame;
    unsigned index, emitted = 0U;
    static_start(&command, &feedback);
    static_take_batch(3002U, -1);
    for (index = 0U; index < DEBUG_UI_RESULT_CAPACITY; ++index)
    {
        DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[index];
        if (!transaction->used) { transaction->used = transaction->completion_ready = 1U; }
    }
    memset(&blocked, 0, sizeof(blocked));
    blocked.type = PROTOCOL_COMMAND_DISABLE;
    for (index = 0U; index < PROTOCOL_ENGINE_RESULT_CAPACITY; ++index)
    { blocked.request_id = 500U + index; assert(aethor_app_submit_or_defer_result(&blocked)); }
    stop = test_request(DEBUG_UI_OPERATION_STOP, 4000U);
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
    static_feedback(4000U, S3519_DRIVER_STATE_ENABLED);
    (void)aethor_app_service(4001U);
    assert(application_debug_ui.fallback_stop_mask == 0x7FU);
    while (aethor_app_pop_emergency_can_frame(&frame))
    { assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE); ++emitted; }
    assert(emitted == 7U);
    assert(!application_debug_ui.static_load.disabled_confirmed);
    static_feedback(5000U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(5001U);
    assert(application_debug_ui.static_load.disabled_confirmed && application_debug_ui.static_load.stopped);
    for (index = 0U; index < PROTOCOL_ENGINE_RESULT_CAPACITY; ++index)
    { assert(protocol_engine_pop_result_output(&application_protocol_engine, &output)); (void)aethor_app_flush_deferred_results(); }
    aethor_app_debug_ui_route_results();
    assert(application_debug_ui.static_load.disabled_confirmed && application_debug_ui.static_load.stopped);
    assert(!application_debug_ui.static_load.complete);
    puts("STATIC_LOAD_SATURATED_STOP_PASS mask=127 emitted=7 causal_ack=7 late_cancel_preserves_confirmation=1");
}

#endif

/** @brief Executes only offline synthetic trials, including the default-off production boundary. */
#ifndef STATIC_LOAD_TEST_ENTRY
#define STATIC_LOAD_TEST_ENTRY main
#endif
int STATIC_LOAD_TEST_ENTRY(void)
{
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
    static_admission_cases();
    static_lifecycle();
    static_guard_cases();
    static_stop_lifecycle();
    static_saturated_stop();
#else
    DebugUiRequest request;
    ProtocolCommand command;
    test_fixture();
    test_acquire();
    request = test_request((DebugUiOperation)(DEBUG_UI_OPERATION_STOP + 1U), 1001U);
    assert(aethor_app_debug_ui_admit_request(&request, 1001U, &command) == DEBUG_UI_REASON_DISABLED);
    puts("STATIC_LOAD_DEFAULT_LOCK_PASS");
#endif
    return 0;
}
