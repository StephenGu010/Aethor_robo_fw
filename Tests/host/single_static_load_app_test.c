/** @file single_static_load_app_test.c
 * @brief Synthetic single-axis long static-HOLD integration; never opens hardware.
 */
#define main existing_debug_ui_test_main
#include "debug_ui_app_test_main.c"
#undef main

/** @brief Only the new local single-axis workflow can use the independent sixty-second window. */
static void single_static_typed_bounds(void)
{
    ProtocolCommand command;
    memset(&command, 0, sizeof(command));
    command.type = PROTOCOL_COMMAND_MIT_ACTION;
    command.mit_action = PROTOCOL_MIT_ACTION_HOLD;
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    command.control_mode = ARM_CONTROL_MODE_MIT;
    command.motor_mask = 1U;
    command.values_in_radians = command.bench_relative_scope = 1U;
    command.mit_kp = 1.0F;
    command.mit_kd = 0.2F;
    command.hold_duration_us = 60000000ULL;
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_NONE);
    command.origin = DEBUG_UI_ORIGIN_USB;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    command.hold_duration_us = 60001000ULL;
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    puts("SINGLE_STATIC_TYPED_WINDOW_PASS local_sixty_seconds_only=1");
#else
    assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
    {
        DebugUiRequest request;
        DebugUiAdmission admission;
        DebugUiCompletion completion;
        DebugUiSnapshot snapshot;
        test_fixture(); test_acquire();
        request = test_request(DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, 1001U);
        admission = test_admission(request, 1001U);
        assert(!admission.accepted && admission.reason == DEBUG_UI_REASON_DISABLED);
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(completion.code == DEBUG_UI_FAILED && completion.operation == request.operation);
        assert(completion.motor_mask == 1U);
        assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot) && !snapshot.single_static_load_enabled);
        assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    }
    puts("SINGLE_STATIC_DEFAULT_LOCK_PASS");
#endif
}

#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
#include "static_startup_test_helpers.inc"
/** @brief Publish exactly one motor; the other six deliberately have no valid feedback. */
static void single_feedback(uint8_t motor_id, uint64_t timestamp_us, uint8_t state)
{
    MotorJointFeedback feedback;
    uint8_t joint_index = (uint8_t)(motor_id - 1U);
    if (application_motor_runtime.bank.motors[joint_index].feedback_valid &&
        application_motor_runtime.bank.motors[joint_index].feedback.timestamp_us == timestamp_us) { return; }
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.position_rad = 0.2F + 0.01F * joint_index;
    feedback.driver_state = state;
    feedback.mos_temperature_c = 30.0F;
    feedback.rotor_temperature_c = 31.0F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[joint_index].master_id,
        application_motor_runtime.configuration->joints[joint_index].esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Create one immutable request using the actual selected snapshot and authority epoch. */
static DebugUiRequest single_request(uint8_t motor_id, DebugUiOperation operation, uint64_t timestamp_us)
{
    DebugUiSnapshot snapshot;
    DebugUiRequest request = test_request(operation, timestamp_us);
    assert(aethor_app_debug_ui_get_snapshot(timestamp_us, &snapshot));
    request.target_motor_id = motor_id;
    request.reference_generation = snapshot.motors[motor_id - 1U].reference_generation;
    request.delta_rad = request.speed_rad_s = request.kp = request.kd = 0.0F;
    request.hold_duration_ms = 60000U;
    request.static_positions_rad[motor_id - 1U] = snapshot.motors[motor_id - 1U].position_rad;
    request.static_references[motor_id - 1U] = snapshot.motors[motor_id - 1U].reference_generation;
    return request;
}

/** @brief Explicitly configure only the tested ID before acquiring local control. */
static DebugUiRequest single_fixture(uint8_t motor_id, uint8_t configured,
    ProtocolCommand *command, MotorFeedbackSnapshot *feedback)
{
    DebugUiMotorProfile profile;
    MotorDiscoveryResult discovery;
    DebugUiSingleStaticProfile long_profile = {1U, 1000U, 60000U, 0.0F};
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    uint8_t joint_index;
    test_fixture();
    profile = application_debug_ui.profiles[0];
    discovery = application_motor_runtime.discovery.results[0];
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
    {
        memset(&application_motor_runtime.discovery.results[joint_index], 0, sizeof(discovery));
        application_motor_runtime.bank.motors[joint_index].feedback_valid = 0U;
    }
    joint_index = (uint8_t)(motor_id - 1U);
    discovery.observed_control_mode = 1U;
    discovery.observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
    discovery.observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
    application_motor_runtime.discovery.results[joint_index] = discovery;
    application_motor_runtime.discovery.verified_joint_mask = (uint8_t)(1U << joint_index);
    assert(aethor_app_debug_ui_set_motor_profile(motor_id, &profile));
    single_feedback(motor_id, 1000U, S3519_DRIVER_STATE_DISABLED);
    if (configured) { assert(aethor_app_debug_ui_set_single_static_profile(motor_id, &long_profile)); }
    request = single_request(motor_id, DEBUG_UI_OPERATION_ACQUIRE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_COMPLETED);
    request = single_request(motor_id, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, 1001U);
    aethor_app_debug_ui_build_command(&request, 1001U, command);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, feedback) == MOTOR_RUNTIME_STATUS_OK);
    assert(feedback->valid_joint_mask == (uint8_t)(1U << joint_index));
    return request;
}

/** @brief Explicit ID7 gains survive command conversion and remain reference-bound before ENABLE. */
static void single_gain_binding_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest request = single_fixture(7U, 1U, &command, &feedback);
    application_debug_ui.single_static_lcd_owned[6] = 1U;
    application_debug_ui.single_static_gains[6].kp = 100.0F;
    application_debug_ui.single_static_gains[6].kd = 0.30F;
    request.kp = 100.0F; request.kd = 0.30F;
    aethor_app_debug_ui_build_command(&request, 1001U, &command);
    assert(fabsf(command.mit_kp - 100.0F / aethor_app_lcd_position_ratio(6U)) < 0.000001F);
    assert(command.mit_kd == 0.30F);
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) == DEBUG_UI_REASON_NONE);
    command.mit_kp += 0.1F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    aethor_app_debug_ui_build_command(&request, 1001U, &command);
    command.mit_kd = NAN;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) != DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_build_command(&request, 1001U, &command);
    application_debug_ui.single_static_lcd_owned[6] = 0U;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) != DEBUG_UI_REASON_NONE);
    request = single_fixture(6U, 1U, &command, &feedback);
    request.kp = 100.0F; request.kd = 0.30F;
    aethor_app_debug_ui_build_command(&request, 1001U, &command);
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) != DEBUG_UI_REASON_NONE);
    puts("STATIC_GAIN_BINDING_PASS conversion=1 frozen_pair=1 nan=1 ownership=1 other_axis=1");
}

/** @brief Check independent configuration, per-ID evidence and isolation from old/USB windows. */
static void single_admission_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    uint8_t motor_id;
    for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
    {
        (void)single_fixture(motor_id, 0U, &command, &feedback);
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_UNCALIBRATED);
        request = single_fixture(motor_id, 1U, &command, &feedback);
        assert(command.motor_mask == (uint8_t)(1U << (motor_id - 1U)));
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_NONE);
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) == DEBUG_UI_REASON_NONE);
        assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
        assert(snapshot.single_static_load_enabled && snapshot.motors[motor_id - 1U].single_static_profile.configured);
        command.hold_duration_us = 61000000ULL;
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OUT_OF_RANGE);
        command.hold_duration_us = 999000ULL;
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OUT_OF_RANGE);
        command.hold_duration_us = 60000000ULL;
        command.local_operation = DEBUG_UI_OPERATION_MIT_HOLD;
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) != DEBUG_UI_REASON_NONE);
        command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
        command.origin = DEBUG_UI_ORIGIN_USB;
        assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
        command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        command.motor_mask = 3U;
        assert(protocol_engine_validate_typed_command(&command) != DEBUG_UI_REASON_NONE);
        (void)single_fixture(motor_id, 1U, &command, &feedback);
        ++application_debug_ui.reference_generation[motor_id - 1U];
        assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
        assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
        assert(!snapshot.motors[motor_id - 1U].single_static_profile.configured);
        {
            uint8_t changed_field;
            for (changed_field = 0U; changed_field < 4U; ++changed_field)
            {
                MotorDiscoveryResult *discovery;
                MotorDiscoveryResult original;
                (void)single_fixture(motor_id, 1U, &command, &feedback);
                discovery = &application_motor_runtime.discovery.results[motor_id - 1U];
                original = *discovery;
                if (changed_field == 0U) { ++discovery->observed_esc_id; }
                if (changed_field == 1U) { discovery->ranges.torque_max_nm += 1.0F; }
                if (changed_field == 2U) { ++discovery->software_version; }
                if (changed_field == 3U) { discovery->maximum_speed_rad_s += 1.0F; }
                assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
                *discovery = original;
                assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_UNCALIBRATED);
                assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
                assert(!snapshot.motors[motor_id - 1U].single_static_profile.configured);
            }
        }
        (void)request;
    }
    test_fixture();
    {
        DebugUiSingleStaticProfile invalid = {1U, 1000U, 60001U, 0.0F};
        assert(!aethor_app_debug_ui_set_single_static_profile(1U, &invalid));
        invalid.maximum_ms = 60000U; invalid.minimum_ms = 999U;
        assert(!aethor_app_debug_ui_set_single_static_profile(1U, &invalid));
        invalid.minimum_ms = 60001U;
        assert(!aethor_app_debug_ui_set_single_static_profile(1U, &invalid));
        assert(!aethor_app_debug_ui_set_single_static_profile(0U, &invalid));
        assert(!aethor_app_debug_ui_set_single_static_profile(8U, &invalid));
        assert(!aethor_app_debug_ui_set_single_static_profile(1U, NULL));
    }
    puts("SINGLE_STATIC_ADMISSION_PASS ids=7 independent_profile=1 old_and_usb_limits_preserved=1");
}

/** @brief Admit only ID7 single static absolute flange poses inside verified PMAX. */
static void single_flange_range_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiSingleStaticProfile long_profile = {1U, 1000U, 60000U, 0.0F};
    const float positions[] = {-1185.772F * AETHOR_APP_DEG_TO_RAD,
        -3.14159265358979323846F * 44.8F, 3.14159265358979323846F * 44.8F};
    unsigned index;
    (void)single_fixture(7U, 1U, &command, &feedback);
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
    aethor_app_single_static_install(6U, &long_profile, 0U);
    command.static_reference_generations[6] = application_debug_ui.reference_generation[6];
    for (index = 0U; index < sizeof(positions) / sizeof(positions[0]); ++index) {
        command.values[6] = feedback.joints[6].position_rad = positions[index];
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_NONE);
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) == DEBUG_UI_REASON_NONE);
        {
            MotorEmergencyFrameBatch encoded;
            float zero_velocity[ARM_JOINT_COUNT] = {0.0F};
            assert(motor_runtime_build_mit_subset(&application_motor_runtime, 0x40U, command.values,
                zero_velocity, command.mit_kp, command.mit_kd, 0.0F, &encoded) == MOTOR_RUNTIME_STATUS_OK);
        }
    }
    command.values[6] = feedback.joints[6].position_rad = positions[2] + 0.01F;
    application_action.command = command;
    aethor_app_static_trace_begin(1002ULL);
    assert(fabsf(application_static_trace.header.axes[6].minimum_rad - positions[1]) < 0.0001F);
    assert(fabsf(application_static_trace.header.axes[6].maximum_rad - positions[2]) < 0.0001F);
    assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OUT_OF_RANGE);
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 12.5F;
    aethor_app_single_static_install(6U, &long_profile, 0U);
    command.static_reference_generations[6] = application_debug_ui.reference_generation[6];
    command.values[6] = feedback.joints[6].position_rad = positions[0];
    assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OUT_OF_RANGE);
    (void)single_fixture(1U, 1U, &command, &feedback);
    command.values[0] = feedback.joints[0].position_rad = positions[0];
    assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OUT_OF_RANGE);
    puts("SINGLE_STATIC_FLANGE_RANGE_PASS cached_pose=1 absolute_180=1 pmax_intersection=1 other_axis_isolation=1");
}

/** @brief Accept only the configured signed single-axis FF, preserving seven-axis zero-only requests. */
static void single_feedforward_binding_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    const float targets[] = {1.05F, -1.05F};
    unsigned index;
    for (index = 0U; index < 2U; ++index) {
        (void)single_fixture(7U, 1U, &command, &feedback);
        application_debug_ui.single_static_profiles[6].torque_ff_nm = targets[index];
        command.mit_torque_ff_nm = targets[index];
        assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_NONE);
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1001U, true) == DEBUG_UI_REASON_NONE);
        command.mit_torque_ff_nm = -targets[index];
        assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
        command.local_operation = DEBUG_UI_OPERATION_STATIC_HOLD; command.motor_mask = 0x7FU;
        assert(aethor_app_static_load_validate(&command, &feedback) != DEBUG_UI_REASON_NONE);
    }
    puts("SINGLE_STATIC_FF_BINDING_PASS signs=2 wrong_value_rejected=1 seven_zero_only=1");
}

/** @brief Consume exactly one real facade frame with the selected CAN identifier. */
static void single_take_frame(uint8_t motor_id, uint64_t timestamp_us, int final_byte)
{
    CanFrame frame;
    CanTxPriority priority;
    assert(application_action.frames.count == 1U);
    assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == application_motor_runtime.configuration->joints[motor_id - 1U].esc_id);
    if (final_byte >= 0) { assert(frame.data[7] == (uint8_t)final_byte); }
}

/** @brief Drive production DISABLE preflight, fresh ACK, ENABLE and fresh ACK into HOLD. */
static void single_start(uint8_t motor_id, uint32_t duration_ms,
    ProtocolCommand *command, MotorFeedbackSnapshot *feedback)
{
    DebugUiRequest request = single_fixture(motor_id, 1U, command, feedback);
    request.hold_duration_ms = duration_ms;
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, command));
    memset(&application_action, 0, sizeof(application_action));
    application_action.command = *command;
    aethor_app_static_trace_begin(1002U);
    assert(!aethor_app_begin_one_shot_preflight_disable(feedback, 1002U));
    single_take_frame(motor_id, 1003U, S3519_MODE_COMMAND_DISABLE);
    single_feedback(motor_id, 2000U, S3519_DRIVER_STATE_DISABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, feedback) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(feedback, 2001U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    startup_test_finish(feedback, 2002U, 3001U);
    single_take_frame(motor_id, 3002U, -1);
    assert(application_debug_ui.static_load.frames_issued_mask == command->motor_mask);
}

/** @brief A stalled task, unproved queued frame or stale feedback cannot extend or jump the takeover window. */
static void single_takeover_failure_cases(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 7U; ++scenario) {
        ProtocolCommand command;
        MotorFeedbackSnapshot feedback;
        uint64_t started = 4001ULL, timestamp_us;
        single_start(7U, 1000U, &command, &feedback);
        application_debug_ui.single_static_profiles[6].torque_ff_nm = 1.05F;
        application_action.command.mit_torque_ff_nm = 1.05F;
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT;
        assert(!aethor_app_static_begin_takeover(started));
        assert(application_action.deadline_us == started + 1000000ULL);
        timestamp_us = started + (scenario == 0U ? 10000ULL : 4000ULL);
        test_health(timestamp_us, true);
        single_feedback(7U, timestamp_us, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
        if (scenario == 2U) feedback.valid_joint_mask = 0U;
        if (scenario == 3U) feedback.joints[6].torque_nm = 4.0F;
        if (scenario == 4U) feedback.joints[6].mos_temperature_c = 56.0F;
        if (scenario == 5U) feedback.joints[6].position_rad += aethor_app_lcd_position_ratio(6U) * AETHOR_APP_DEG_TO_RAD;
        if (scenario == 6U) ++application_debug_ui.reference_generation[6];
        assert(!aethor_app_static_advance_takeover(&feedback, timestamp_us));
        if (scenario == 1U) {
            CanFrame queued;
            CanTxPriority priority;
            DebugUiStaticTxIdentity identity;
            assert(aethor_app_next_can_frame(timestamp_us + 1ULL, &queued, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            assert(aethor_app_static_frame_identity(&queued, &identity));
            assert(!application_debug_ui.static_load.transmitted_torque_ff_nm);
            timestamp_us += 4000ULL;
            test_health(timestamp_us, true); single_feedback(7U, timestamp_us, S3519_DRIVER_STATE_ENABLED);
            assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
            assert(!aethor_app_static_advance_takeover(&feedback, timestamp_us));
            assert(!aethor_app_static_submitted(&identity, timestamp_us + 1ULL));
        }
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
        assert(!application_debug_ui.static_load.takeover_complete && !application_debug_ui.static_load.takeover_active);
        assert(application_action.frames.count == 1U && application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
    }
    puts("SINGLE_STATIC_TAKEOVER_FAIL_PASS task_stall=1 queued_not_tx=1 stale=1 torque=1 temperature=1 pose=1 binding=1 direct_disable=1");
}

/** @brief Execute all sixty logical seconds with fresh health, feedback and exact-scope frames. */
static void single_lifecycle(uint8_t motor_id, uint32_t duration_ms)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint64_t timestamp_us, deadline_us;
    uint8_t joint_index = (uint8_t)(motor_id - 1U);
    single_start(motor_id, duration_ms, &command, &feedback);
    deadline_us = 3001ULL + (uint64_t)duration_ms * 1000ULL;
    assert(application_action.deadline_us == deadline_us);
    single_feedback(motor_id, 4000U, S3519_DRIVER_STATE_ENABLED);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 4000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
    aethor_app_static_load_sample(&feedback, 4000U);
    aethor_app_static_load_sample(&feedback, 4001U);
    assert(application_debug_ui.static_load.samples[joint_index] == 1U);
    for (timestamp_us = 4000ULL; timestamp_us < deadline_us; timestamp_us += 4000ULL)
    {
        single_feedback(motor_id, timestamp_us, S3519_DRIVER_STATE_ENABLED);
        test_health(timestamp_us, true);
        (void)aethor_app_service(timestamp_us);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        single_take_frame(motor_id, timestamp_us + 1ULL, -1);
    }
    single_feedback(motor_id, deadline_us, S3519_DRIVER_STATE_ENABLED);
    test_health(deadline_us, true);
    (void)aethor_app_service(deadline_us);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT);
    single_take_frame(motor_id, deadline_us + 1ULL, S3519_MODE_COMMAND_DISABLE);
    (void)aethor_app_service(deadline_us + 2000ULL);
    assert(!application_debug_ui.static_load.complete);
    single_feedback(motor_id, deadline_us + 4000ULL, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(deadline_us + 4001ULL);
    aethor_app_debug_ui_process(deadline_us + 4001ULL);
    assert(application_debug_ui.static_load.complete && application_debug_ui.static_load.disabled_confirmed);
    assert(application_debug_ui.static_load.valid_mask == command.motor_mask);
    assert(application_debug_ui.static_load.elapsed_ms == duration_ms);
    assert(application_debug_ui.static_load.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
}

/** @brief Each selected axis retains all six MIT safety guards and first-batch sender timeout. */
static void single_guard_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint8_t motor_id, scenario, failed;
    for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
    {
        for (scenario = 0U; scenario < 15U; ++scenario)
        {
            MotorJointFeedback *selected_feedback;
            single_start(motor_id, 60000U, &command, &feedback);
            selected_feedback = &feedback.joints[motor_id - 1U];
            if (scenario == 0U) { selected_feedback->torque_nm = 3.501F; }
            if (scenario == 1U) { selected_feedback->mos_temperature_c = 56.0F; }
            if (scenario == 2U) { selected_feedback->rotor_temperature_c = 56.0F; }
            if (scenario == 3U) { selected_feedback->position_rad += 1.0F; }
            if (scenario == 4U) { ++application_debug_ui.reference_generation[motor_id - 1U]; }
            if (scenario == 5U) { selected_feedback->driver_state = S3519_DRIVER_STATE_DISABLED; }
            if (scenario == 6U) { selected_feedback->velocity_rad_s = 100.0F; }
            if (scenario == 7U) { feedback.valid_joint_mask = 0U; }
            if (scenario == 8U) { selected_feedback->fault_flags = 1U; }
            if (scenario == 9U) { selected_feedback->position_rad = NAN; }
            if (scenario == 10U) { selected_feedback->velocity_rad_s = NAN; }
            if (scenario == 11U) { selected_feedback->torque_nm = NAN; }
            if (scenario == 12U) { selected_feedback->mos_temperature_c = NAN; }
            if (scenario == 13U) { selected_feedback->rotor_temperature_c = NAN; }
            if (scenario == 14U) { ++application_motor_runtime.discovery.results[motor_id - 1U].software_version; }
            (void)aethor_app_check_one_shot_motor_safety(&feedback, 3003U, &failed);
            assert(failed && application_action.command.motor_mask == (uint8_t)(1U << (motor_id - 1U)));
            /* A protection trip must not inject another powered HOLD before DISABLE. */
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
            assert(application_action.priority == CAN_TX_PRIORITY_EMERGENCY);
            assert(application_action.frames.count == 1U);
            assert(application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_DISABLE);
            assert(application_action.cleanup_disable_mask == command.motor_mask);
        }
        single_start(motor_id, 60000U, &command, &feedback);
        (void)aethor_app_check_one_shot_motor_safety(&feedback, 7003U, &failed);
        assert(failed);
    }
    puts("SINGLE_STATIC_GUARDS_PASS selected_axis_cases=105 sender_stalls=7");
}

/** @brief Preserve one trial across normal STOP cancellation and reject mismatched terminal scope. */
static void single_stop_lifecycle(uint8_t motor_id)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest stop;
    ProtocolCommandResult unrelated;
    DebugUiStaticLoadReport saved;
    uint8_t scenario;
    single_start(motor_id, 60000U, &command, &feedback);
    saved = application_debug_ui.static_load;
    memset(&unrelated, 0, sizeof(unrelated));
    unrelated.origin = saved.identity.origin;
    unrelated.session_id = saved.identity.epoch;
    unrelated.request_id = saved.identity.request_id;
    unrelated.local_operation = saved.operation;
    unrelated.motor_mask = saved.motor_mask;
    unrelated.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    unrelated.disabled_confirmed = 1U;
    for (scenario = 0U; scenario < 5U; ++scenario)
    {
        ProtocolCommandResult wrong = unrelated;
        if (scenario == 0U) { ++wrong.request_id; }
        if (scenario == 1U) { ++wrong.session_id; }
        if (scenario == 2U) { wrong.origin = DEBUG_UI_ORIGIN_USB; }
        if (scenario == 3U) { wrong.local_operation = DEBUG_UI_OPERATION_STATIC_HOLD; }
        if (scenario == 4U) { wrong.motor_mask = (uint8_t)(1U << (motor_id % 7U)); }
        aethor_app_static_load_finish(&wrong);
        assert(memcmp(&saved, &application_debug_ui.static_load, sizeof(saved)) == 0);
    }
    stop = single_request(motor_id, DEBUG_UI_OPERATION_STOP, 4000U);
    assert(test_admission(stop, 4000U).accepted);
    single_feedback(motor_id, 4000U, S3519_DRIVER_STATE_ENABLED);
    test_health(4000U, true);
    (void)aethor_app_service(4001U);
    assert(application_debug_ui.static_load.stop_identity.request_id == stop.identity.request_id);
    assert(!application_debug_ui.static_load.disabled_confirmed);
    single_take_frame(motor_id, 4002U, -1);
    (void)aethor_app_service(5000U);
    if (application_action.state != AETHOR_APP_ACTION_IDLE)
    { single_take_frame(motor_id, 5001U, S3519_MODE_COMMAND_DISABLE); }
    assert(!application_debug_ui.static_load.disabled_confirmed);
    single_feedback(motor_id, 6000U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(6001U);
    aethor_app_debug_ui_process(6001U);
    assert(application_debug_ui.static_load.stopped && application_debug_ui.static_load.disabled_confirmed);
    assert(!application_debug_ui.static_load.complete);
    unrelated.local_operation = DEBUG_UI_OPERATION_STOP;
    unrelated.request_id = stop.identity.request_id + 1U;
    unrelated.disabled_confirmed = 0U;
    aethor_app_static_load_finish(&unrelated);
    assert(application_debug_ui.static_load.disabled_confirmed);
}

/** @brief A saturated STOP emits only the selected ID and requires its newer disabled ACK. */
static void single_saturated_stop(uint8_t motor_id)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiRequest stop;
    ProtocolCommandResult blocked;
    ProtocolOutputBatch output;
    CanFrame frame;
    unsigned index, emitted = 0U;
    single_start(motor_id, 60000U, &command, &feedback);
    for (index = 0U; index < DEBUG_UI_RESULT_CAPACITY; ++index)
    {
        DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[index];
        if (!transaction->used) { transaction->used = transaction->completion_ready = 1U; }
    }
    memset(&blocked, 0, sizeof(blocked));
    blocked.type = PROTOCOL_COMMAND_DISABLE;
    for (index = 0U; index < PROTOCOL_ENGINE_RESULT_CAPACITY; ++index)
    { blocked.request_id = 500U + index; assert(aethor_app_submit_or_defer_result(&blocked)); }
    stop = single_request(motor_id, DEBUG_UI_OPERATION_STOP, 4000U);
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_STOP_LATCHED);
    single_feedback(motor_id, 4000U, S3519_DRIVER_STATE_ENABLED);
    (void)aethor_app_service(4001U);
    assert(application_debug_ui.fallback_stop_mask == command.motor_mask);
    while (aethor_app_pop_emergency_can_frame(&frame))
    {
        assert(frame.identifier == application_motor_runtime.configuration->joints[motor_id - 1U].esc_id);
        assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE); ++emitted;
    }
    assert(emitted == 1U && !application_debug_ui.static_load.disabled_confirmed);
    (void)aethor_app_service(4002U);
    assert(!application_debug_ui.static_load.disabled_confirmed);
    single_feedback(motor_id, 5000U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(5001U);
    assert(application_debug_ui.static_load.disabled_confirmed && application_debug_ui.static_load.stopped);
    for (index = 0U; index < PROTOCOL_ENGINE_RESULT_CAPACITY; ++index)
    { assert(protocol_engine_pop_result_output(&application_protocol_engine, &output)); (void)aethor_app_flush_deferred_results(); }
    aethor_app_debug_ui_route_results();
    assert(application_debug_ui.static_load.disabled_confirmed && application_debug_ui.static_load.stopped);
    assert(!application_debug_ui.static_load.complete);
}

/** @brief Both independent UI/protocol heartbeat losses stop a long trial on each selected ID. */
static void single_health_cleanup(uint8_t motor_id)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint8_t publisher;
    for (publisher = 0U; publisher < 2U; ++publisher)
    {
        single_start(motor_id, 60000U, &command, &feedback);
        single_feedback(motor_id, 153001U, S3519_DRIVER_STATE_ENABLED);
        if (publisher) { test_health(153001U, false); }
        (void)aethor_app_service(153001U);
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        assert(application_debug_ui.diagnostics.health_stop_count == 1U);
        assert(application_action.cleanup_disable_mask == command.motor_mask);
        assert(!application_debug_ui.static_load.complete);
    }
}

/** @brief Long-profile provisioning requires remote idle without reserved results and revokes old captures. */
static void single_profile_boundaries(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    DebugUiSnapshot before, after;
    DebugUiSingleStaticProfile profile = {1U, 1000U, 60000U, 0.0F};
    (void)single_fixture(7U, 1U, &command, &feedback);
    assert(!aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_REMOTE;
    assert(aethor_app_debug_ui_get_snapshot(1001U, &before));
    assert(aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    assert(aethor_app_debug_ui_get_snapshot(1001U, &after));
    assert(after.motors[6].reference_generation != before.motors[6].reference_generation);
    assert(aethor_app_static_load_validate(&command, &feedback) == DEBUG_UI_REASON_OLD_EPOCH);
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT;
    assert(!aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    application_action.state = AETHOR_APP_ACTION_IDLE;
    application_debug_ui.mailbox.transactions[0].used = 1U;
    assert(!aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    application_debug_ui.mailbox.transactions[0].used = 0U;
    profile.configured = 0U;
    assert(aethor_app_debug_ui_set_single_static_profile(7U, &profile));
    assert(aethor_app_debug_ui_get_snapshot(1001U, &after));
    assert(!after.motors[6].single_static_profile.configured);
    assert(test_critical_depth == 0U);
}
#endif

/** @brief Execute deterministic production-code checks with synthetic data only. */
#ifndef SINGLE_STATIC_TEST_ENTRY
#define SINGLE_STATIC_TEST_ENTRY main
#endif
int SINGLE_STATIC_TEST_ENTRY(void)
{
    single_static_typed_bounds();
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    {
        uint8_t motor_id;
        single_takeover_failure_cases();
        single_gain_binding_cases();
        single_feedforward_binding_cases();
        single_flange_range_cases();
        single_admission_cases();
        for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
        { single_lifecycle(motor_id, 1000U); single_lifecycle(motor_id, 60000U); }
        puts("SINGLE_STATIC_LIFECYCLE_PASS ids=7 windows=1s_and_60s fresh_ack_required=1 duplicate_samples_blocked=1");
        single_guard_cases();
        for (motor_id = 1U; motor_id <= DEBUG_UI_MOTOR_COUNT; ++motor_id)
        { single_stop_lifecycle(motor_id); single_saturated_stop(motor_id); single_health_cleanup(motor_id); }
        single_profile_boundaries();
        puts("SINGLE_STATIC_STOP_PASS ids=7 ordinary_and_saturated=1 scope_identity_rejections=35 newer_selected_ack_required=1");
        puts("SINGLE_STATIC_HEALTH_PROFILE_PASS heartbeat_cases=14 provisioning_requires_remote_idle=1 stale_capture_rejected=1");
    }
#endif
    return 0;
}
