/** @file debug_ui_app_test_main.c
 * @brief Local control integration tests with explicit host-only motor fixtures.
 * Includes the facade to inject hardware-independent phase/fault fixtures; production
 * never receives these synthetic evidence profiles or fake motor feedback.
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../App/aethor_app.c"

static uint32_t runtime_sample_lock_depth;
static uint32_t runtime_sample_lock_entries;

/** @brief Observe the bounded diagnostic publication/read critical sections. */
static void runtime_sample_enter(void)
{
    assert(runtime_sample_lock_depth == 0U);
    ++runtime_sample_lock_depth;
    ++runtime_sample_lock_entries;
}

/** @brief Require paired critical exit before a diagnostic API returns. */
static void runtime_sample_exit(void)
{
    assert(runtime_sample_lock_depth == 1U);
    --runtime_sample_lock_depth;
}

/** @brief CPU task samples retain their capacities in a protected value copy. */
static void test_runtime_stack_publication(void)
{
    RuntimeDiagnosticSample sample;
    DiagnosticCounters counters;
    aethor_app_init(1U, 78U);
    memset(&sample, 0, sizeof(sample));
    runtime_sample_lock_depth = runtime_sample_lock_entries = 0U;
    aethor_app_set_task_critical_hooks(runtime_sample_enter, runtime_sample_exit);
    sample.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].allocated_words = 1280U;
    sample.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].free_words = 467U;
    aethor_app_update_runtime_diagnostics(&sample);
    assert(runtime_sample_lock_entries == 1U && runtime_sample_lock_depth == 0U);
    assert(aethor_app_get_diagnostic_counters(&counters));
    assert(runtime_sample_lock_entries == 2U && runtime_sample_lock_depth == 0U);
    assert(counters.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].free_words == 467U);
    assert(counters.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].allocated_words == 1280U);
    sample.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].free_words = 0U;
    assert(counters.task_stacks[DIAGNOSTIC_TASK_PROTOCOL].free_words == 467U);
    aethor_app_set_task_critical_hooks(NULL, NULL);
}

/** @brief Checks full source identity before ownership is released. */
static void test_engine_identity(void)
{
    ProtocolEngine engine;
    ProtocolCommand command;
    ProtocolCommandResult result;
    protocol_engine_init(&engine, 77U);
    memset(&command, 0, sizeof(command));
    command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
    command.request_id = 42U;
    command.session_id = 11U;
    command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    command.motor_mask = 1U;
    command.bench_relative_scope = 1U;
    command.values_in_radians = 1U;
    command.speeds[0] = 0.02F;
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    (void)result;
    assert(protocol_engine_submit_local_command(&engine, &command) == DEBUG_UI_REASON_DISABLED);
    return;
#endif
    assert(protocol_engine_submit_local_command(&engine, &command) == DEBUG_UI_REASON_NONE);
    memset(&result, 0, sizeof(result));
    result.type = command.type;
    result.request_id = command.request_id;
    result.session_id = command.session_id;
    assert(protocol_engine_submit_command_result(&engine, &result));
    assert(engine.active_motion_request_id == 42U);
    result.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    result.session_id = 10U;
    assert(protocol_engine_submit_command_result(&engine, &result));
    assert(engine.active_motion_request_id == 42U);
    result.session_id = 11U;
    assert(protocol_engine_submit_command_result(&engine, &result));
    assert(engine.active_motion_request_id == 0U);
    command.values[0] = NAN;
    assert(protocol_engine_validate_typed_command(&command) == DEBUG_UI_REASON_INVALID_ARGUMENT);
}

/** @brief A queued init result must retain motor7 verification at completion time. */
static void test_init_terminal_verification_snapshot(void)
{
    ProtocolOutputBatch output;
    unsigned index;
    uint8_t found = 0U;
    aethor_app_init(1U, 77U);
    assert(aethor_app_process_protocol_line("0 hello", 7U, 2U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    application_action.command.type = PROTOCOL_COMMAND_INIT_MOTORS;
    application_action.command.request_id = 701U;
    application_action.command.session_id = application_protocol_engine.session_id;
    application_action.command.origin = DEBUG_UI_ORIGIN_USB;
    application_action.command.bench_relative_scope = 1U;
    application_action.command.motor_mask = 0x40U;
    application_motor_runtime.discovery.results[6].verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    /* The cached query still predates the mode readback, as observed on hardware. */
    assert(aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_COMPLETED, 0U, 1000U));
    /* Later discovery changes must not rewrite a retained terminal's evidence. */
    application_motor_runtime.discovery.results[6].verified_fields_mask = 0U;
    assert(aethor_app_pop_protocol_result_output(&output));
    for (index = 0U; index < output.count; ++index)
        if (strstr(output.messages[index].data, "identity=40 mode=40 ranges=40 version=40") != NULL) found = 1U;
    if (!found) { fputs("INIT_TERMINAL_SNAPSHOT_FAIL\n", stderr); exit(1); }
    puts("INIT_TERMINAL_SNAPSHOT_PASS motor7_only_immutable");
}

/** @brief Read-only register polling yields to USB work and cannot authorize motion. */
static void test_idle_position_register_service(void)
{
    CanFrame frame;
    CanTxPriority priority;
    DebugUiSnapshot snapshot;
    aethor_app_init(1U, 77U);
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.results[6].verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
#if AETHOR_DEBUG_UI_ENABLE && !AETHOR_DEBUG_UI_ALLOW_MOTION && \
    AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[0] == 7U && frame.data[2] == 0x33U && frame.data[3] == 0x50U);
    application_protocol_engine.command_write_sequence++;
    assert(aethor_app_next_can_frame(1000010U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(!application_motor_runtime.position_read.pending);
    application_protocol_engine.command_read_sequence++;
#else
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
#endif
    application_motor_runtime.position_read.joint_index = 6U;
    application_motor_runtime.position_read.values[1] = 12.5F;
    application_motor_runtime.position_read.sample_us[1] = 1000000U;
    application_motor_runtime.position_read.seen_mask = 2U;
    assert(aethor_app_debug_ui_get_snapshot(1000050U, &snapshot));
    assert(snapshot.motors[6].register_seen_mask == 2U);
    assert(snapshot.motors[6].register_position[1] == 12.5F);
    assert(!snapshot.motors[6].feedback_valid);
    assert(!snapshot.motors[6].profile.pos_valid);
}

/** @brief Default profile must not contain fictional bench evidence. */
static void test_default_profile(void)
{
    DebugUiSnapshot snapshot;
    DebugUiMotorProfile profile;
    DebugUiRequest request;
    DebugUiAdmission admission;
    aethor_app_init(1U, 77U);
    assert(aethor_app_debug_ui_get_snapshot(1U, &snapshot));
    assert(snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE);
#if AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE
    assert(aethor_app_debug_ui_get_motor_profile(7U, &profile));
    assert(!profile.pos_valid);
    application_motor_runtime.discovery.results[6].verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    application_motor_runtime.discovery.verified_joint_mask = 0x40U;
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 12.5F;
    application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 30.0F;
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    assert(aethor_app_debug_ui_get_snapshot(2U, &snapshot));
    profile = snapshot.motors[6].profile;
    assert(profile.pos_valid && (profile.mit_valid == AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE));
    assert(fabsf(profile.position_min_rad + 12.5F / 19.2032F) < 0.000001F);
    assert(fabsf(profile.position_max_rad - 12.5F / 19.2032F) < 0.000001F);
    assert(fabsf(profile.max_delta_rad - 25.0F / 19.2032F) < 0.000001F);
    assert(profile.max_speed_rad_s == 10.0F);
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 40.0F;
    assert(aethor_app_debug_ui_get_snapshot(3U, &snapshot));
    assert(snapshot.motors[6].profile.max_speed_rad_s == 30.0F);
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = NAN;
    assert(aethor_app_debug_ui_get_snapshot(4U, &snapshot));
    assert(!snapshot.motors[6].profile.pos_valid);
    assert(!snapshot.active && !snapshot.pending);
#endif
    assert(aethor_app_debug_ui_get_motor_profile(1U, &profile));
    assert(!profile.pos_valid && !profile.mit_valid);
    memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = snapshot.epoch;
    request.identity.request_id = 1U;
    request.operation = DEBUG_UI_OPERATION_ACQUIRE;
    request.target_motor_id = 1U;
    request.created_at_us = 1U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(1U);
    assert(aethor_app_debug_ui_poll_admission(&admission));
    assert(!admission.accepted);
    assert(admission.reason == ((AETHOR_DEBUG_UI_ALLOW_MOTION != 0U)
        ? DEBUG_UI_REASON_UNCALIBRATED : DEBUG_UI_REASON_DISABLED));
    request.identity.request_id = 2U;
    request.operation = DEBUG_UI_OPERATION_STOP;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(1U);
    assert(aethor_app_debug_ui_poll_admission(&admission));
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    assert(!admission.accepted && admission.reason == DEBUG_UI_REASON_DISABLED);
    /* Even saturated/repeated STOP remains read-only: no physical intent may escape. */
    {
        uint32_t request_id;
        CanFrame frame;
        for (request_id = 3U; request_id <= 10U; ++request_id)
        {
            DebugUiReason submit_reason;
            request.identity.request_id = request_id;
            submit_reason = aethor_app_debug_ui_submit(&request);
            assert((submit_reason == DEBUG_UI_REASON_NONE) || (submit_reason == DEBUG_UI_REASON_DISABLED));
            aethor_app_debug_ui_process(request_id);
            (void)aethor_app_service(request_id);
            assert(!aethor_app_pop_emergency_can_frame(&frame));
            assert(aethor_app_debug_ui_get_snapshot(request_id, &snapshot));
            assert(!snapshot.active && !snapshot.stop_pending);
            assert(snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE);
        }
    }
#endif
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION
static uint32_t test_request_id;
static uint32_t test_health_sequence;
static uint32_t test_critical_depth;
static uint32_t test_critical_entries;

/** @brief Tracks paired nested critical sections, including all public shared snapshots. */
static void test_enter_critical(void) { ++test_critical_depth; ++test_critical_entries; }
/** @brief Rejects an unbalanced critical release. */
static void test_exit_critical(void) { assert(test_critical_depth != 0U); --test_critical_depth; }

/** @brief Injects a real MotorBank value publication without physical CAN traffic. */
static void test_feedback(uint64_t timestamp_us, float position_rad, uint8_t driver_state)
{
    MotorJointFeedback feedback;
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.position_rad = position_rad;
    feedback.driver_state = driver_state;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[0].master_id,
        application_motor_runtime.configuration->joints[0].esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Publishes fresh checked UI progress; Protocol acknowledgement remains independent. */
static void test_health(uint64_t timestamp_us, bool acknowledge)
{
    DebugUiHealth health;
    memset(&health, 0, sizeof(health));
    health.ui_timestamp_us = timestamp_us;
    health.ui_sequence = ++test_health_sequence;
    health.input_valid = health.display_valid = health.ui_valid = 1U;
    aethor_app_debug_ui_update_health(&health);
    if (acknowledge) { aethor_app_debug_ui_process(timestamp_us); }
}

/** @brief Creates a host-only evidence fixture, explicitly distinct from deployed defaults. */
static void test_fixture(void)
{
    DebugUiMotorProfile profile;
    MotorDiscoveryResult *discovery;
    aethor_app_init(1U, 77U);
    test_request_id = test_health_sequence = test_critical_depth = test_critical_entries = 0U;
    aethor_app_set_task_critical_hooks(test_enter_critical, test_exit_critical);
    memset(&profile, 0, sizeof(profile));
    profile.pos_valid = profile.mit_valid = 1U;
    profile.position_min_rad = -1.0F;
    profile.position_max_rad = 1.0F;
    profile.max_delta_rad = 0.1F;
    profile.max_speed_rad_s = 0.1F;
    profile.mit_position_min_rad = -1.0F;
    profile.mit_position_max_rad = 1.0F;
    profile.mit_max_delta_rad = 0.1F;
    profile.mit_max_speed_rad_s = 0.1F;
    profile.kp_min = 0.5F; profile.kp_max = 2.0F;
    profile.kd_min = 0.5F; profile.kd_max = 2.0F;
    profile.default_kp = profile.default_kd = 1.0F;
    profile.hold_min_ms = 100U; profile.hold_max_ms = 1000U;
    assert(aethor_app_debug_ui_set_motor_profile(1U, &profile));
    discovery = &application_motor_runtime.discovery.results[0];
    discovery->ranges.position_max_rad = 12.5F;
    discovery->ranges.velocity_max_rad_s = 30.0F;
    discovery->ranges.torque_max_nm = 10.0F;
    discovery->maximum_speed_rad_s = 10.0F;
    discovery->observed_control_mode = 2U;
    discovery->observed_master_id = application_motor_runtime.configuration->joints[0].master_id;
    discovery->observed_esc_id = application_motor_runtime.configuration->joints[0].esc_id;
    discovery->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    application_motor_runtime.discovery.verified_joint_mask = 1U;
    test_feedback(1000U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    test_health(1000U, true);
}

/** @brief Creates a fresh typed request from an actual coherent UI snapshot. */
static DebugUiRequest test_request(DebugUiOperation operation, uint64_t timestamp_us)
{
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    assert(aethor_app_debug_ui_get_snapshot(timestamp_us, &snapshot));
    memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = snapshot.epoch;
    request.identity.request_id = ++test_request_id;
    request.operation = operation;
    request.created_at_us = timestamp_us;
    request.reference_generation = snapshot.motors[0].reference_generation;
    request.target_motor_id = 1U;
    request.delta_rad = 0.05F;
    request.speed_rad_s = 0.02F;
    request.kp = request.kd = 1.0F;
    request.hold_duration_ms = 100U;
    return request;
}

/** @brief Polls exactly one admission for the immutable request. */
static DebugUiAdmission test_admission(DebugUiRequest request, uint64_t timestamp_us)
{
    DebugUiAdmission admission;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(timestamp_us);
    assert(aethor_app_debug_ui_poll_admission(&admission));
    assert(admission.identity.request_id == request.identity.request_id);
    assert(admission.identity.epoch == request.identity.epoch);
    return admission;
}

/** @brief Acquires local authority and explicitly consumes both acknowledgements. */
static void test_acquire(void)
{
    DebugUiCompletion completion;
    DebugUiAdmission admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000U), 1000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_COMPLETED);
}

#if AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE
/** @brief Both full-span POS endpoints pass; excess speed/position and changed limits reject. */
static void test_motor7_protocol_envelope(DebugUiOperation operation)
{
    DebugUiSnapshot snapshot;
    MotorFeedbackSnapshot feedback;
    ProtocolCommand command;
    DebugUiRequest request;
    unsigned direction;
    test_fixture();
    application_motor_runtime.discovery.results[6] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.verified_joint_mask |= 0x40U;
    application_motor_runtime.bank.motors[6] = application_motor_runtime.bank.motors[0];
    application_motor_runtime.bank.motors[6].feedback.position_rad = 19.2032F * AETHOR_APP_DEG_TO_RAD;
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    assert(fabsf(snapshot.motors[6].position_rad - AETHOR_APP_DEG_TO_RAD) < 0.000001F);
#if AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE
    assert(snapshot.motors[6].profile.mit_position_max_rad == snapshot.motors[6].profile.position_max_rad);
    assert(snapshot.motors[6].profile.mit_max_delta_rad == snapshot.motors[6].profile.max_delta_rad);
#endif
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    application_debug_ui.target_motor_id = 7U;
    request = test_request(operation, 1000U);
    request.target_motor_id = 7U;
    request.reference_generation = snapshot.motors[6].reference_generation;
    request.delta_rad = AETHOR_APP_DEG_TO_RAD;
    request.speed_rad_s = 10.0F;
    request.kp = 80.0F; request.kd = 0.2F;
    aethor_app_debug_ui_build_command(&request, 1000U, &command);
    /* LCD output-shaft degrees become rotor radians exactly once in either mode. */
    assert(fabsf(command.values[6] - 19.2032F * AETHOR_APP_DEG_TO_RAD) < 0.000001F);
    assert(command.speeds[6] == request.speed_rad_s);
    /* Output-coordinate stiffness must not be amplified by the position ratio. */
    assert(fabsf(command.mit_kp * command.values[6] - request.kp * request.delta_rad) < 0.00001F);
    assert(command.mit_kd == request.kd);
    assert(fabsf(snapshot.motors[6].position_max_rad - 12.5F / 19.2032F) < 0.000001F);
    memset(&feedback, 0, sizeof(feedback));
    feedback.valid_joint_mask = 0x40U;
    /* Exercise the actual reference builder: equal output displacement and
     * speed must not produce a reduction-ratio-longer MIT trajectory. */
    command.speeds[6] = 10.0F * AETHOR_APP_DEG_TO_RAD;
    memset(&application_action, 0, sizeof(application_action));
    application_action.command = command;
    application_action.target_position_rad[6] = command.values[6];
    application_action.target_speed_rad_s[6] = command.speeds[6];
    assert(!aethor_app_start_one_shot_motion(&feedback, 1000U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
    if (operation == DEBUG_UI_OPERATION_MIT_MOVE)
    {
        const CanFrame *frame;
        float wire_position, wire_velocity;
        uint64_t duration_us = application_action.motion_plan.duration_us;
        assert(duration_us >= 187499U && duration_us <= 187501U);
        assert(aethor_app_build_one_shot_reference(1000U + duration_us / 2U) == MOTOR_RUNTIME_STATUS_OK);
        frame = &application_action.frames.frames[0];
        wire_position = (float)(((unsigned)frame->data[0] << 8) | frame->data[1]) * 25.0F / 65535.0F - 12.5F;
        wire_velocity = (float)(((unsigned)frame->data[2] << 4) | (frame->data[3] >> 4)) * 60.0F / 4095.0F - 30.0F;
        assert(fabsf(wire_position - command.values[6] * 0.5F) < 25.0F / 65535.0F);
        assert(fabsf(wire_velocity - command.speeds[6]) < 60.0F / 4095.0F);
        {
            float wire_kp = (float)(((unsigned)(frame->data[3] & 15U) << 8) | frame->data[4]) * 500.0F / 4095.0F;
            assert(fabsf(wire_kp - 80.0F / 19.2032F) < 500.0F / 4095.0F);
        }
    }
    else
    {
        float wire_position, wire_velocity;
        memcpy(&wire_position, application_action.frames.frames[0].data, sizeof(wire_position));
        memcpy(&wire_velocity, &application_action.frames.frames[0].data[4], sizeof(wire_velocity));
        assert(wire_position == command.values[6]);
        assert(wire_velocity == command.speeds[6]);
        assert(application_action.deadline_us >= 2300999U && application_action.deadline_us <= 2301001U);
    }
    command.speeds[6] = request.speed_rad_s;
    for (direction = 0U; direction < 2U; ++direction)
    {
        float sign = direction == 0U ? 1.0F : -1.0F;
        feedback.joints[6].position_rad = -12.5F * sign;
        command.values[6] = 25.0F * sign;
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000U, true) == DEBUG_UI_REASON_NONE);
        command.values[6] = 25.01F * sign;
        assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000U, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    }
    command.values[6] = -25.0F;
    command.speeds[6] = 10.01F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000U, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    command.speeds[6] = 10.0F;
    feedback.joints[6].position_rad = 0.0F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000U, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 9.0F;
    assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000U, true) == DEBUG_UI_REASON_OLD_EPOCH);
    application_motor_runtime.discovery.results[6].verified_fields_mask = 0U;
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    assert(!snapshot.motors[6].profile.pos_valid);
}
#endif

/** @brief Local request expiry, epoch/reference changes and invalid values all reject. */
static void test_local_rejections(void)
{
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    uint8_t scenario;
    for (scenario = 0U; scenario < 7U; ++scenario)
    {
        test_fixture(); test_acquire();
        request = test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U);
        if (scenario == 0U) { --request.identity.epoch; }
        if (scenario == 1U) { request.delta_rad = NAN; }
        if (scenario == 2U) { request.speed_rad_s = INFINITY; }
        if (scenario == 3U) { request.delta_rad = 0.2F; }
        if (scenario == 4U) { ++request.reference_generation; }
        if (scenario == 5U) { request.operation = DEBUG_UI_OPERATION_DISABLE;
            application_action.state = AETHOR_APP_ACTION_ENABLE_WAIT; }
        admission = test_admission(request, scenario == 6U ? 251001U : 1000U);
        assert(!admission.accepted);
        if (scenario == 0U || scenario == 4U) { assert(admission.reason == DEBUG_UI_REASON_OLD_EPOCH); }
        if (scenario == 5U) { assert(admission.reason == DEBUG_UI_REASON_BUSY); }
        if (scenario == 6U) { assert(admission.reason == DEBUG_UI_REASON_STALE_REQUEST); }
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(completion.code == DEBUG_UI_FAILED);
        assert(application_protocol_engine.command_write_sequence == 0U);
    }
}

/** @brief Stale idle POS intent queues only DISABLE; fresh post-request evidence anchors the target. */
static void test_pos_preflight_fresh_anchor(DebugUiOperation operation)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    DebugUiSnapshot before, after;
    MotorFeedbackSnapshot feedback_snapshot;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 4U; ++scenario)
    {
        if (operation == DEBUG_UI_OPERATION_SET_MODE && scenario == 1U) { continue; }
        test_fixture();
        assert(aethor_app_debug_ui_get_snapshot(1000U, &before));
        test_health(1000000U, true);
        assert(aethor_app_debug_ui_get_snapshot(1000000U, &after));
        assert(before.motors[0].reference_generation == after.motors[0].reference_generation);
        admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000000U), 1000000U);
        assert(admission.accepted);
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(application_action.state == AETHOR_APP_ACTION_IDLE);
        assert(application_protocol_engine.command_write_sequence == 0U);
        request = test_request(operation, 1000000U);
        request.requested_mode = DEBUG_UI_MODE_MIT;
        admission = test_admission(request, 1000000U);
        assert(admission.accepted);
        (void)aethor_app_service(1000010U);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
        /* A same-tick reply must be newer than the baseline: wait before the first DISABLE. */
        assert(aethor_app_next_can_frame(1000010U, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
        assert(application_action.frame_read_index == 0U);
        assert(aethor_app_next_can_frame(1000020U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.data[7] == 0xFDU); /* No ENABLE or position frame precedes a fresh disabled ACK. */
        (void)aethor_app_service(1000030U);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
        test_health(1000040U, true);
        test_feedback(1000020U, scenario == 1U ? (operation == DEBUG_UI_OPERATION_MIT_HOLD ? 1.02F : 0.98F) : 0.4F, S3519_DRIVER_STATE_DISABLED);
        if (scenario == 2U) { application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = 0.051F; }
        if (scenario == 3U) { application_motor_runtime.bank.motors[0].feedback.fault_flags = 1U; }
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1000040U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback_snapshot) == MOTOR_RUNTIME_STATUS_OK);
        (void)aethor_app_advance_one_shot_setup(&feedback_snapshot, 1000040U);
        if (scenario == 0U)
        {
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH);
            if (operation != DEBUG_UI_OPERATION_SET_MODE)
                assert(fabsf(application_action.target_position_rad[0] -
                    (operation == DEBUG_UI_OPERATION_MIT_HOLD ? 0.4F : 0.45F)) < 0.00001F);
        }
        else
        {
            assert(application_action.state == AETHOR_APP_ACTION_IDLE);
            aethor_app_debug_ui_process(1000040U);
            assert(aethor_app_debug_ui_poll_completion(&completion));
            assert(completion.code == DEBUG_UI_FAILED && completion.stage == DEBUG_UI_STAGE_VALIDATE);
            assert(completion.reason == (scenario == 1U ? DEBUG_UI_REASON_OUT_OF_RANGE : DEBUG_UI_REASON_NOT_DISABLED));
        }
    }
}

#if AETHOR_DEBUG_UI_ALLOW_MIT
/** @brief Every local MIT energy/position violation starts cleanup, including nonfinite feedback. */
static void test_local_mit_feedback_guard(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 15U; ++scenario)
    {
        MotorFeedbackSnapshot snapshot;
        DebugUiRequest request;
        uint8_t failed;
        test_fixture(); test_acquire();
        request = test_request(DEBUG_UI_OPERATION_MIT_MOVE, 1000U);
        aethor_app_debug_ui_build_command(&request, 1000U, &application_action.command);
        application_action.state = AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT;
        test_feedback(2000U, 0.2F, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        if (scenario == 1U) { snapshot.joints[0].torque_nm = 3.501F; }
        if (scenario == 2U) { snapshot.joints[0].torque_nm = -3.501F; }
        if (scenario == 3U) { snapshot.joints[0].torque_nm = NAN; }
        if (scenario == 4U) { snapshot.joints[0].velocity_rad_s = 0.35F; }
        if (scenario == 5U) { snapshot.joints[0].position_rad = NAN; }
        if (scenario == 6U) { snapshot.joints[0].position_rad = 1.01F; }
        if (scenario == 7U) { snapshot.joints[0].mos_temperature_c = 56U; }
        if (scenario == 8U) { snapshot.joints[0].rotor_temperature_c = 56U; }
        if (scenario == 9U) { snapshot.joints[0].torque_nm = 0.75F; }
        if (scenario == 10U) { snapshot.joints[0].torque_nm = -0.75F; }
        if (scenario == 11U) { snapshot.joints[0].torque_nm = 3.5F; }
        if (scenario == 12U) { snapshot.joints[0].torque_nm = -3.5F; }
        if (scenario == 13U) { snapshot.joints[0].torque_nm = INFINITY; }
        if (scenario == 14U) { snapshot.joints[0].torque_nm = -INFINITY; }
        (void)aethor_app_check_one_shot_motor_safety(&snapshot, 2000U, &failed);
        assert(failed == ((scenario >= 1U && scenario <= 8U) || scenario >= 13U));
        if (failed) {
            DebugUiReason expected_reason = DEBUG_UI_REASON_INVALID_FEEDBACK;
            if (scenario == 1U || scenario == 2U) expected_reason = DEBUG_UI_REASON_TORQUE_LIMIT;
            if (scenario == 4U) expected_reason = DEBUG_UI_REASON_SPEED_LIMIT;
            if (scenario == 6U) expected_reason = DEBUG_UI_REASON_OUT_OF_RANGE;
            if (scenario == 7U || scenario == 8U) expected_reason = DEBUG_UI_REASON_TEMPERATURE_LIMIT;
            assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
            assert(application_action.local_failure_reason == expected_reason);
            assert(application_debug_ui.last_mit_guard_reason == expected_reason);
            assert(application_debug_ui.last_mit_guard_motor_id == 1U);
            assert(application_debug_ui.last_mit_guard_feedback.timestamp_us == snapshot.joints[0].timestamp_us);
        }
    }
}

/** @brief Mode-1 ownership can expire without trapping release; POS bounds never leak into MIT. */
static void test_mit_ownership_and_independent_bounds(void)
{
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    MotorFeedbackSnapshot snapshot;
    ProtocolCommand command;
    test_fixture();
    application_motor_runtime.discovery.results[0].observed_control_mode = 1U;
    test_health(1000000U, true);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000000U), 1000000U);
    assert(admission.accepted && aethor_app_debug_ui_poll_completion(&completion));
    admission = test_admission(test_request(DEBUG_UI_OPERATION_RELEASE, 1000000U), 1000000U);
    assert(admission.accepted && aethor_app_debug_ui_poll_completion(&completion));
    test_fixture(); test_acquire();
    application_debug_ui.profiles[0].mit_max_delta_rad = 0.01F;
    request = test_request(DEBUG_UI_OPERATION_MIT_MOVE, 1000U);
    aethor_app_debug_ui_build_command(&request, 1000U, &command);
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_validate_start(&command, &snapshot, 1000U, true) == DEBUG_UI_REASON_OUT_OF_RANGE);
    request.operation = DEBUG_UI_OPERATION_POS_MOVE;
    aethor_app_debug_ui_build_command(&request, 1000U, &command);
    assert(aethor_app_debug_ui_validate_start(&command, &snapshot, 1000U, true) == DEBUG_UI_REASON_NONE);
}
#endif

/** @brief Bench POS sends a bounded smooth reference and cannot finish before its trajectory. */
static void test_bench_pos_reference_trajectory(void)
{
    MotorFeedbackSnapshot snapshot;
    float encoded_position, previous_position;
    unsigned direction;
    for (direction = 0U; direction < 2U; ++direction)
    {
        uint64_t timestamp_us, duration_us;
        float sign = direction == 0U ? 1.0F : -1.0F;
        test_fixture();
        memset(&application_action, 0, sizeof(application_action));
        application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
        application_action.command.origin = DEBUG_UI_ORIGIN_USB;
        application_action.command.bench_relative_scope = 1U;
        application_action.command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        application_action.command.motor_mask = 1U;
        application_action.target_position_rad[0] = 0.2F + sign * 0.1F;
        application_action.target_speed_rad_s[0] = 0.02F;
        test_feedback(1001U, 0.2F, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_motion(&snapshot, 1001U));
        memcpy(&encoded_position, application_action.frames.frames[0].data, sizeof(encoded_position));
        assert(fabsf(encoded_position - 0.2F) < 0.000001F);
        duration_us = application_action.motion_plan.duration_us;
        assert(duration_us >= 9374000ULL && duration_us <= 9376000ULL);
        /* Even a feedback sample already at the endpoint cannot skip the planned duration. */
        snapshot.joints[0].position_rad = application_action.target_position_rad[0];
        application_action.frame_read_index = application_action.frames.count;
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 2000U));
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        previous_position = 0.2F;
        for (timestamp_us = 5000U; timestamp_us < 1000U + duration_us; timestamp_us += 4000U)
        {
            snapshot.joints[0].position_rad = previous_position;
            snapshot.joints[0].timestamp_us = timestamp_us;
            application_action.frame_read_index = application_action.frames.count;
            assert(!aethor_app_advance_one_shot_setup(&snapshot, timestamp_us));
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
            memcpy(&encoded_position, application_action.frames.frames[0].data, sizeof(encoded_position));
            assert(sign * (encoded_position - previous_position) >= -0.0000002F);
            assert(fabsf(encoded_position - previous_position) <= 0.02F * 0.004F + 0.0000002F);
            assert(application_action.command.control_mode == ARM_CONTROL_MODE_POSITION_VELOCITY);
            previous_position = encoded_position;
        }
        /* Fresh feedback must not permit a large catch-up jump after a stalled sender. */
        timestamp_us += 50001U;
        snapshot.joints[0].position_rad = previous_position;
        snapshot.joints[0].timestamp_us = timestamp_us;
        application_action.frame_read_index = application_action.frames.count;
        assert(!aethor_app_advance_one_shot_setup(&snapshot, timestamp_us));
        assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        assert(application_action.failed_stage == PROTOCOL_COMMAND_STAGE_MOTION);
        assert(application_action.failure_error == PROTOCOL_COMMAND_ERROR_TIMEOUT);
    }
}

/** @brief Bench POS rejects tracking excursions and invalid or out-of-profile position feedback. */
static void test_bench_pos_feedback_guard(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        MotorFeedbackSnapshot snapshot;
        uint8_t failure_detected = 0U;
        test_fixture();
        memset(&application_action, 0, sizeof(application_action));
        application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
        application_action.command.origin = DEBUG_UI_ORIGIN_USB;
        application_action.command.bench_relative_scope = 1U;
        application_action.command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        application_action.command.motor_mask = 1U;
        application_action.target_position_rad[0] = 0.3F;
        application_action.target_speed_rad_s[0] = 0.02F;
        test_feedback(1001U, 0.2F, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_motion(&snapshot, 1001U));
        snapshot.joints[0].timestamp_us = 2000U;
        snapshot.joints[0].position_rad = 0.201F;
        if (scenario == 1U) { snapshot.joints[0].position_rad = 0.21F; }
        if (scenario == 2U) { snapshot.joints[0].position_rad = 0.19F; }
        if (scenario == 3U) { snapshot.joints[0].position_rad = NAN; }
        if (scenario == 4U)
        {
            application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
            application_debug_ui.profiles[0].position_max_rad = 0.2005F;
        }
        if (scenario == 5U) { snapshot.joints[0].timestamp_us = 2001U; }
        (void)aethor_app_check_one_shot_motor_safety(&snapshot, 2000U, &failure_detected);
        assert(failure_detected == (scenario != 0U));
        if (failure_detected != 0U)
        {
            assert(application_action.failed_stage == PROTOCOL_COMMAND_STAGE_MOTION);
            assert(application_action.failed_motor_number == 1U);
            assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        }
    }
}

/** @brief LCD POS holds a fixed endpoint and tolerates lag, but retains bounded cleanup. */
static void test_local_pos_fixed_endpoint(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 10U; ++scenario)
    {
        MotorFeedbackSnapshot snapshot;
        float encoded_position, encoded_speed;
        uint8_t failed;
        test_fixture();
        memset(&application_action, 0, sizeof(application_action));
        application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
        application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        application_action.command.bench_relative_scope = 1U;
        application_action.command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        application_action.command.motor_mask = 1U;
        application_action.target_position_rad[0] = scenario == 1U ? 0.1F : 0.3F;
        application_action.target_speed_rad_s[0] = 10.0F * AETHOR_APP_DEG_TO_RAD;
        test_feedback(1001U, 0.2F, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_motion(&snapshot, 1001U));
        memcpy(&encoded_position, application_action.frames.frames[0].data, sizeof(float));
        memcpy(&encoded_speed, &application_action.frames.frames[0].data[4], sizeof(float));
        assert(fabsf(encoded_position - application_action.target_position_rad[0]) < 0.000001F);
        assert(fabsf(encoded_speed - application_action.target_speed_rad_s[0]) < 0.000001F);
        snapshot.joints[0].timestamp_us = 41001U;
        snapshot.joints[0].position_rad = scenario == 1U ? 0.17F : 0.23F;
        if (scenario == 2U) { snapshot.joints[0].position_rad = 0.19F; }
        if (scenario == 3U) { snapshot.joints[0].position_rad = 0.31F; }
        if (scenario == 4U) { snapshot.joints[0].position_rad = NAN; }
        if (scenario == 5U) { snapshot.joints[0].timestamp_us = 41002U; }
        if (scenario == 6U) { snapshot.valid_joint_mask = 0U; }
        if (scenario == 7U) { snapshot.joints[0].fault_flags = 1U; }
        if (scenario == 8U) { application_debug_ui.profiles[0].position_max_rad = 0.22F; }
        (void)aethor_app_check_one_shot_motor_safety(&snapshot, 41001U, &failed);
        assert(failed == (scenario >= 2U && scenario <= 8U));
        if (failed) { continue; }
        application_action.frame_read_index = application_action.frames.count;
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 41001U));
        memcpy(&encoded_position, application_action.frames.frames[0].data, sizeof(float));
        assert(fabsf(encoded_position - application_action.target_position_rad[0]) < 0.000001F);
        if (scenario == 9U)
        {
            snapshot.joints[0].timestamp_us = 91002U;
            assert(!aethor_app_advance_one_shot_setup(&snapshot, 91002U));
            assert(application_action.failure_error == PROTOCOL_COMMAND_ERROR_TIMEOUT);
            continue;
        }
        /* Endpoint crossing alone must not complete; stale repeated samples cannot settle. */
        snapshot.joints[0].position_rad = application_action.target_position_rad[0];
        snapshot.joints[0].timestamp_us = 45001U;
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 45001U));
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        {
            uint64_t timestamp_us;
            for (timestamp_us = 49001U; timestamp_us <= 249001U; timestamp_us += 4000U)
            {
                application_action.frame_read_index = application_action.frames.count;
                assert(!aethor_app_advance_one_shot_setup(&snapshot, timestamp_us));
                assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
            }
        }
        snapshot.joints[0].timestamp_us = 253001U;
        assert(!aethor_app_advance_one_shot_setup(&snapshot, 253001U));
        /* A gap in endpoint feedback restarts the settle interval. */
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
        {
            uint64_t timestamp_us;
            for (timestamp_us = 257001U; timestamp_us <= 453001U; timestamp_us += 4000U)
            {
                snapshot.joints[0].timestamp_us = timestamp_us;
                application_action.frame_read_index = application_action.frames.count;
                assert(!aethor_app_advance_one_shot_setup(&snapshot, timestamp_us));
            }
        }
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    }
}

/** @brief Fixed LCD POS stops on no progress or deadline, and resets interrupted settling. */
static void test_local_pos_progress_and_settle(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 3U; ++scenario)
    {
        MotorFeedbackSnapshot snapshot;
        uint64_t timestamp_us;
        test_fixture();
        memset(&application_action, 0, sizeof(application_action));
        application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
        application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
        application_action.command.bench_relative_scope = 1U;
        application_action.command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        application_action.command.motor_mask = 1U;
        application_action.target_position_rad[0] = 0.3F;
        application_action.target_speed_rad_s[0] = 0.02F;
        test_feedback(1001U, 0.2F, S3519_DRIVER_STATE_ENABLED);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_motion(&snapshot, 1001U));
        if (scenario == 1U) { application_action.deadline_us = 101001U; }
        for (timestamp_us = 5001U; timestamp_us <= 2001001U; timestamp_us += 4000U)
        {
            snapshot.joints[0].timestamp_us = timestamp_us;
            if (scenario == 2U)
            {
                snapshot.joints[0].position_rad = timestamp_us == 105001U ? 0.28F : 0.3F;
            }
            application_action.frame_read_index = application_action.frames.count;
            assert(!aethor_app_advance_one_shot_setup(&snapshot, timestamp_us));
            if (scenario == 2U && timestamp_us <= 305001U)
            {
                assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
            }
            if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT) { break; }
        }
        if (scenario == 2U)
        {
            assert(timestamp_us == 309001U);
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
        }
        else
        {
            assert(timestamp_us == (scenario == 0U ? 2001001U : 101001U));
            assert(application_action.failure_error == PROTOCOL_COMMAND_ERROR_TIMEOUT);
        }
    }
}

/** @brief Queue admission still expires at Arm start even with deferred POS feedback. */
static void test_relative_start_and_queued_expiry(void)
{
    DebugUiAdmission admission;
    DebugUiRequest request;
    DebugUiCompletion completion;
    test_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    test_feedback(1020U, 0.3F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1020U);
    assert(fabsf(application_action.target_position_rad[0] - 0.35F) < 0.00001F);
    assert(application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI);
    test_fixture(); test_acquire();
    admission = test_admission(test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U), 1000U);
    assert(admission.accepted);
    test_health(251001U, true);
    test_feedback(251001U, 0.3F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(251001U);
    aethor_app_debug_ui_process(251001U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.reason == DEBUG_UI_REASON_STALE_REQUEST);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
}

/** @brief USB hello and same-ID STOP cannot cancel or complete the wrong source. */
static void test_usb_local_arbitration(void)
{
    ProtocolOutputBatch output;
    DebugUiAdmission admission;
    DebugUiRequest request;
    DebugUiCompletion completion;
    char line[64];
    test_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_protocol_engine.active_motion_request_id == request.identity.request_id);
    assert(aethor_app_process_protocol_line("89 bench move 1 position=1 speed=1",
        strlen("89 bench move 1 position=1 speed=1"), 1002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "busy") != NULL);
    (void)snprintf(line, sizeof(line), "%lu bench stop 1", (unsigned long)request.identity.request_id);
    assert(aethor_app_process_protocol_line(line, strlen(line), 1003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(1004U);
    assert(application_action.command.type == PROTOCOL_COMMAND_STOP);
    assert(application_action.command.origin == DEBUG_UI_ORIGIN_USB);
    assert(application_action.command.cleanup_after_stop);
    aethor_app_debug_ui_process(1004U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_CANCELLED);
    assert(completion.identity.origin == DEBUG_UI_ORIGIN_LOCAL_UI);
    assert(application_protocol_engine.active_stop_request_id == request.identity.request_id);
    application_action.frame_read_index = application_action.frames.count;
    (void)aethor_app_service(1005U);
    assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
    test_feedback(1006U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1006U);
    assert(aethor_app_pop_protocol_result_output(&output));
    assert(strstr(output.messages[0].data, "stopped") != NULL);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(!aethor_app_debug_ui_poll_completion(&completion));
}

/** @brief Local STOP preempts remote motion even with a full ordinary request mailbox. */
static void test_local_stop_remote_and_full_queue(void)
{
    ProtocolOutputBatch output;
    DebugUiRequest normal;
    DebugUiRequest stop;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    uint8_t slot_index;
    test_fixture();
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("2 bench move 1 position=1 speed=1",
        strlen("2 bench move 1 position=1 speed=1"), 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    normal = test_request(DEBUG_UI_OPERATION_ACQUIRE, 1001U);
    stop = test_request(DEBUG_UI_OPERATION_STOP, 1001U);
    assert(stop.identity.request_id == 2U);
    assert(aethor_app_debug_ui_submit(&normal) == DEBUG_UI_REASON_NONE);
    assert(aethor_app_debug_ui_submit(&stop) == DEBUG_UI_REASON_NONE);
    aethor_app_debug_ui_process(1002U);
    (void)aethor_app_service(1003U);
    assert(application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI);
    assert(application_action.command.type == PROTOCOL_COMMAND_STOP);
    assert(aethor_app_pop_protocol_result_output(&output));
    assert(strstr(output.messages[0].data, "cancelled") != NULL);
    application_action.frame_read_index = application_action.frames.count;
    test_feedback(1004U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1004U);
    aethor_app_debug_ui_process(1004U);
    for (slot_index = 0U; slot_index < 2U; ++slot_index)
    {
        assert(aethor_app_debug_ui_poll_admission(&admission));
        assert(aethor_app_debug_ui_poll_completion(&completion));
        if (completion.operation == DEBUG_UI_OPERATION_STOP)
        { assert(completion.code == DEBUG_UI_STOPPED); assert(completion.disabled_confirmed); }
    }
    test_fixture();
    for (slot_index = 0U; slot_index < PROTOCOL_ENGINE_COMMAND_CAPACITY; ++slot_index)
    { application_protocol_engine.commands[slot_index].type = PROTOCOL_COMMAND_CLEAR_FAULT; }
    application_protocol_engine.command_write_sequence = PROTOCOL_ENGINE_COMMAND_CAPACITY;
    admission = test_admission(test_request(DEBUG_UI_OPERATION_STOP, 1000U), 1000U);
    assert(admission.accepted);
    assert(application_protocol_engine.stop_write_sequence != application_protocol_engine.stop_read_sequence);
}

/** @brief Loss of either publisher triggers Arm-owned cleanup in every one-shot phase. */
static void test_independent_health_cleanup(void)
{
    static const AethorAppActionState phases[] = {
        AETHOR_APP_ACTION_ONE_SHOT_DISCOVERY, AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT,
        AETHOR_APP_ACTION_ONE_SHOT_MODE_SWITCH,
        AETHOR_APP_ACTION_ONE_SHOT_CLEAR_WAIT, AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT,
        AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT, AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT,
        AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT
    };
    uint8_t publisher;
    size_t phase_index;
    for (publisher = 0U; publisher < 2U; ++publisher)
    {
        for (phase_index = 0U; phase_index < sizeof(phases) / sizeof(phases[0]); ++phase_index)
        {
            DebugUiAdmission admission;
            ProtocolCommand command;
            test_fixture(); test_acquire();
            admission = test_admission(test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U), 1000U);
            assert(admission.accepted);
            assert(protocol_engine_pop_command(&application_protocol_engine, &command));
            application_action.command = command;
            application_action.state = phases[phase_index];
            application_action.enabled_by_action_mask = 1U;
            application_action.deadline_us = 1000000U;
            test_feedback(151001U, 0.2F, S3519_DRIVER_STATE_ENABLED);
            if (publisher == 1U) { test_health(151001U, false); }
            (void)aethor_app_service(151001U);
            assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
            assert(application_debug_ui.diagnostics.health_stop_count == 1U);
            /* Preserve fault-time evidence even after the live heartbeat recovers. */
            assert(application_debug_ui.health_failure_timestamp_us == 151001U);
            assert(application_debug_ui.health_failure_authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY);
            assert(application_debug_ui.health_failure_action_state == (uint32_t)phases[phase_index]);
            assert(application_debug_ui.health_failure_snapshot.protocol_timestamp_us == 1000U);
            assert(application_debug_ui.health_failure_snapshot.ui_timestamp_us ==
                   (publisher == 1U ? 151001U : 1000U));
            assert((application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT) ||
                   (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT));
            {
                DebugUiSnapshot snapshot;
                assert(aethor_app_debug_ui_get_snapshot(151001U, &snapshot));
                assert(snapshot.active_stage == (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_HOLD_WAIT
                    ? DEBUG_UI_STAGE_HOLD : DEBUG_UI_STAGE_DISABLE));
            }
            assert(application_protocol_engine.command_write_sequence == 1U);
            assert(test_critical_depth == 0U && test_critical_entries > 0U);
            application_action.frame_read_index = application_action.frames.count;
            (void)aethor_app_service(151002U);
            assert(application_debug_ui.health_failure_timestamp_us == 151001U);
            assert(application_debug_ui.health_failure_action_state == (uint32_t)phases[phase_index]);
            application_action.frame_read_index = application_action.frames.count;
            test_feedback(151003U, 0.2F, S3519_DRIVER_STATE_DISABLED);
            (void)aethor_app_service(151003U);
            assert(application_action.state == AETHOR_APP_ACTION_IDLE);
            assert(application_protocol_engine.result_write_sequence != application_protocol_engine.result_read_sequence);
            {
                DebugUiCompletion completion;
                aethor_app_debug_ui_process(151004U);
                assert(aethor_app_debug_ui_poll_completion(&completion));
                assert(completion.code == DEBUG_UI_FAILED);
                assert(completion.reason == DEBUG_UI_REASON_UNHEALTHY);
                assert(completion.disabled_confirmed);
            }
        }
    }
}

/** @brief MIT profile/public limits reject zero gains, nonfinite values and nonzero feedforward. */
static void test_mit_limits(void)
{
    uint8_t scenario;
    for (scenario = 0U; scenario < 5U; ++scenario)
    {
        DebugUiRequest request;
        DebugUiAdmission admission;
        test_fixture(); test_acquire();
        request = test_request(DEBUG_UI_OPERATION_MIT_HOLD, 1000U);
        if (scenario == 0U) { request.kp = 0.0F; }
        if (scenario == 1U) { request.kd = NAN; }
        if (scenario == 2U) { request.kp = 2.1F; }
        if (scenario == 3U) { request.torque_ff_nm = 0.1F; }
        if (scenario == 4U) { request.hold_duration_ms = 1001U; }
        admission = test_admission(request, 1000U);
        assert(!admission.accepted);
    }
}

/** @brief Healthy idle retains ownership; explicit release works with stale disabled POS feedback. */
static void test_results_and_idle_authority(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    uint32_t epoch;
    test_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_DISABLE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    application_action.frame_read_index = application_action.frames.count;
    test_feedback(1002U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1002U);
    aethor_app_debug_ui_process(1002U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    aethor_app_debug_ui_process(1002U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    epoch = application_debug_ui.epoch;
    test_health(30001002ULL, true);
    test_feedback(30001002ULL, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(30001002ULL);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    assert(application_debug_ui.epoch == epoch);
    assert(application_protocol_engine.local_control_locked);
    test_health(3600001002ULL, true);
    (void)aethor_app_service(3600001002ULL);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    assert(application_debug_ui.epoch == epoch);
    request = test_request(DEBUG_UI_OPERATION_RELEASE, 3600001002ULL);
    admission = test_admission(request, 3600001002ULL);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_COMPLETED);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE);
    assert(application_debug_ui.epoch != epoch);
    assert(!application_protocol_engine.local_control_locked);
}

/** @brief An idle UI fault can be explicitly recovered after health returns and POS feedback ages. */
static void test_idle_ui_fault_reacquire_with_stale_feedback(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    test_fixture(); test_acquire();
    (void)aethor_app_service(200000U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(application_debug_ui.unconfirmed_disable_mask == 0U);
    test_health(2000000U, true);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 2000000U), 2000000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    assert(!application_debug_ui.health_stop_started);
}

/** @brief Local STOP refreshes residual-speed disabled feedback before faulted reacquisition. */
static void test_fault_recovery_stop_refresh(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    test_fixture(); test_acquire();
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    application_debug_ui.target_motor_id = 1U;
    application_motor_runtime.bank.motors[0].feedback.velocity_rad_s = -0.5372467F;
    test_health(1000000U, true);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000000U), 1000000U);
    assert(!admission.accepted && admission.reason == DEBUG_UI_REASON_NOT_DISABLED);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    admission = test_admission(test_request(DEBUG_UI_OPERATION_STOP, 1000000U), 1000000U);
    assert(admission.accepted);
    (void)aethor_app_service(1000001U);
    assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
    assert(application_action.frames.count == 1U && application_action.frames.frames[0].data[7] == 0xFDU);
    application_action.frame_read_index = application_action.frames.count;
    test_feedback(1000002U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1000002U);
    aethor_app_debug_ui_process(1000002U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.operation == DEBUG_UI_OPERATION_STOP && completion.disabled_confirmed);
    aethor_app_debug_ui_process(1000002U);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 1000003U), 1000003U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
}

/** @brief Cleanup proof requires post-attempt feedback and survives later UI confirmation delays. */
static void test_recovery_proof_survives_feedback_expiry(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiSnapshot snapshot;
    test_fixture(); test_acquire();
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    aethor_app_debug_ui_lock_targets(1U, 2000U);
    test_health(2001U, true);
    (void)aethor_app_service(2001U);
    assert(application_debug_ui.unconfirmed_disable_mask == 1U);
    test_feedback(2002U, 0.2F, S3519_DRIVER_STATE_ENABLED);
    (void)aethor_app_service(2002U);
    assert(application_debug_ui.unconfirmed_disable_mask == 1U);
    test_feedback(2003U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(2003U);
    assert(application_debug_ui.unconfirmed_disable_mask == 0U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    test_health(2000000U, true);
    assert(aethor_app_debug_ui_get_snapshot(2000000U, &snapshot));
    assert(snapshot.unconfirmed_disable_mask == 0U && !snapshot.motors[0].feedback_valid);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 2000000U), 2000000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
}

/** @brief UI and Protocol stalls do not borrow the USB self-contained watchdog semantics. */
static void test_usb_session_disconnect_and_result_retention(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    ProtocolCommandResult usb_result;
    ProtocolOutputBatch output;
    uint8_t slot_index;
    test_fixture(); test_acquire();
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    request = test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    test_health(1101001U, true);
    test_feedback(1101001U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    application_action.deadline_us = 2000000U;
    (void)aethor_app_service(1101001U);
    assert(application_action.command.request_id == request.identity.request_id);
    assert(application_protocol_engine.watchdog_timeout_reported == 0U);
    assert(aethor_app_process_protocol_line("90 hello", 8U, 1101002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    memset(&usb_result, 0, sizeof(usb_result));
    usb_result.type = PROTOCOL_COMMAND_CLEAR_FAULT;
    usb_result.session_id = application_protocol_engine.session_id;
    usb_result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    for (slot_index = 0U; slot_index < PROTOCOL_ENGINE_RESULT_CAPACITY; ++slot_index)
    {
        usb_result.request_id = (uint32_t)(100U + slot_index);
        assert(protocol_engine_submit_command_result(&application_protocol_engine, &usb_result));
    }
    assert(aethor_app_complete_action(PROTOCOL_COMMAND_RESULT_FAILED, 3U, 1101003U));
    assert(aethor_app_deferred_result_count() == 1U);
    assert(application_protocol_engine.active_motion_request_id == request.identity.request_id);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(aethor_app_process_protocol_line("91 hello", 8U, 1101004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_pop_protocol_result_output(&output);
    (void)aethor_app_service(1101005U);
    aethor_app_debug_ui_process(1101005U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.identity.request_id == request.identity.request_id);
    assert(!aethor_app_debug_ui_poll_completion(&completion));
}

/** @brief A failed cleanup never unlocks LOCAL_FAULT or claims a disable acknowledgement. */
static void test_missing_disable_feedback_and_reacquire(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest stop;
    ProtocolOutputBatch output;
    test_fixture(); test_acquire();
    admission = test_admission(test_request(DEBUG_UI_OPERATION_POS_MOVE, 1000U), 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    stop = test_request(DEBUG_UI_OPERATION_STOP, 1001U);
    admission = test_admission(stop, 1001U);
    assert(admission.accepted);
    (void)aethor_app_service(1002U);
    aethor_app_debug_ui_process(1002U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_CANCELLED);
    application_action.frame_read_index = application_action.frames.count;
    test_health(501003U, true);
    (void)aethor_app_service(501003U);
    aethor_app_debug_ui_process(501003U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.code == DEBUG_UI_FAILED);
    assert(!completion.disabled_confirmed);
    assert(completion.stage == DEBUG_UI_STAGE_DISABLE);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(aethor_app_process_protocol_line("88 hello", 8U, 501004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    test_feedback(501005U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    test_health(501005U, true);
    admission = test_admission(test_request(DEBUG_UI_OPERATION_ACQUIRE, 501005U), 501005U);
    assert(admission.accepted);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED);
}

/** @brief Local DISABLE uses verified actual mode and waits for a newer disabled sample. */
static void test_local_disable_actual_mode_and_fresh_ack(void)
{
    DebugUiAdmission admission;
    DebugUiRequest request;
    test_fixture(); test_acquire();
    application_motor_runtime.discovery.results[0].observed_control_mode = 1U;
    request = test_request(DEBUG_UI_OPERATION_DISABLE, 1000U);
    admission = test_admission(request, 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    assert(application_action.frames.frames[0].identifier == 1U);
    application_action.frame_read_index = application_action.frames.count;
    (void)aethor_app_service(1002U);
    assert(application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT);
    test_feedback(1003U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1003U);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
}
/** @brief Publishes a fresh explicit fault/disable fixture for any selected motor. */
static void test_p1_feedback(uint8_t motor_id, uint64_t timestamp_us, uint8_t driver_state)
{
    MotorJointFeedback feedback;
    assert(motor_id >= 1U && motor_id <= ARM_JOINT_COUNT);
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.position_rad = 0.2F;
    feedback.driver_state = driver_state;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        application_motor_runtime.configuration->joints[motor_id - 1U].master_id,
        application_motor_runtime.configuration->joints[motor_id - 1U].esc_id,
        &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Adds a second motor before authority acquisition, with host-only evidence. */
static void test_p1_second_motor(void)
{
    DebugUiMotorProfile profile;
    assert(aethor_app_debug_ui_get_motor_profile(1U, &profile));
    assert(aethor_app_debug_ui_set_motor_profile(2U, &profile));
    application_motor_runtime.discovery.results[1] = application_motor_runtime.discovery.results[0];
    application_motor_runtime.discovery.results[1].observed_master_id = application_motor_runtime.configuration->joints[1].master_id;
    application_motor_runtime.discovery.results[1].observed_esc_id = application_motor_runtime.configuration->joints[1].esc_id;
    application_motor_runtime.discovery.verified_joint_mask |= 2U;
    test_p1_feedback(2U, 1000U, S3519_DRIVER_STATE_DISABLED);
}

/** @brief Neither RELEASE nor ACQUIRE may replace an unconfirmed original cleanup target. */
static void test_p1_fault_target_lock(void)
{
    uint8_t operation_index;
    for (operation_index = 0U; operation_index < 2U; ++operation_index)
    {
        DebugUiAdmission admission;
        DebugUiCompletion completion;
        DebugUiRequest request;
        ProtocolOutputBatch output;
        test_fixture(); test_p1_second_motor(); test_acquire();
        admission = test_admission(test_request(DEBUG_UI_OPERATION_DISABLE, 1000U), 1000U);
        assert(admission.accepted);
        (void)aethor_app_service(1001U);
        application_action.frame_read_index = application_action.frames.count;
        test_health(501002U, true);
        test_p1_feedback(1U, 501002U, S3519_DRIVER_STATE_ENABLED);
        test_p1_feedback(2U, 501002U, S3519_DRIVER_STATE_DISABLED);
        (void)aethor_app_service(501002U);
        aethor_app_debug_ui_process(501002U);
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(completion.code == DEBUG_UI_FAILED && !completion.disabled_confirmed);
        request = test_request(operation_index == 0U ? DEBUG_UI_OPERATION_RELEASE : DEBUG_UI_OPERATION_ACQUIRE, 501003U);
        request.target_motor_id = 2U;
        admission = test_admission(request, 501003U);
        assert(!admission.accepted);
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
        assert(application_debug_ui.target_motor_id == 1U);
        assert(aethor_app_process_protocol_line("88 hello", 8U, 501004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(aethor_app_process_protocol_line("89 bench enable 1", strlen("89 bench enable 1"),
            501005U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        test_p1_feedback(1U, 501006U, S3519_DRIVER_STATE_DISABLED);
        test_p1_feedback(2U, 501006U, S3519_DRIVER_STATE_DISABLED);
        request = test_request(DEBUG_UI_OPERATION_ACQUIRE, 501006U);
        request.target_motor_id = 2U;
        admission = test_admission(request, 501006U);
        assert(admission.accepted);
        assert(application_debug_ui.target_motor_id == 2U);
    }
}

/** @brief A saturated result consumer cannot prevent Arm-owned STOP or erase old identities. */
static void test_p1_stop_backpressure(void)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint8_t result_index;
    uint8_t disable_frame_count = 0U;
    uint64_t timestamp_us = 1000U;
    test_fixture();
    for (result_index = 0U; result_index < DEBUG_UI_RESULT_CAPACITY; ++result_index)
    {
        admission = test_admission(test_request(DEBUG_UI_OPERATION_STOP, timestamp_us), timestamp_us);
        assert(admission.accepted);
        (void)aethor_app_service(timestamp_us + 1U);
        application_action.frame_read_index = application_action.frames.count;
        test_feedback(timestamp_us + 2U, 0.2F, S3519_DRIVER_STATE_DISABLED);
        (void)aethor_app_service(timestamp_us + 2U);
        aethor_app_debug_ui_process(timestamp_us + 3U);
        timestamp_us += 10U;
    }
    assert(debug_ui_mailbox_result_count(&application_debug_ui.mailbox) == DEBUG_UI_RESULT_CAPACITY);
    assert(aethor_app_process_protocol_line("88 hello", 8U, timestamp_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench move 1 position=1 speed=1",
        strlen("89 bench move 1 position=1 speed=1"), timestamp_us + 1U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(timestamp_us + 2U);
    request = test_request(DEBUG_UI_OPERATION_STOP, timestamp_us + 3U);
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_STOP_LATCHED);
    assert(aethor_app_debug_ui_get_snapshot(timestamp_us + 3U, &snapshot) && snapshot.stop_pending);
    /* No ProtocolTask service: the independent control owner must consume the intent. */
    (void)aethor_app_service(timestamp_us + 4U);
    while (aethor_app_pop_emergency_can_frame(&frame))
    {
        assert(frame.data[7] == 0xFDU);
        ++disable_frame_count;
    }
    assert(disable_frame_count > 0U);
    assert(aethor_app_next_can_frame(timestamp_us + 4U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    test_feedback(timestamp_us + 5U, 0.2F, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(timestamp_us + 5U);
    for (result_index = 0U; result_index < DEBUG_UI_RESULT_CAPACITY; ++result_index)
    {
        assert(aethor_app_debug_ui_poll_completion(&completion));
        assert(completion.identity.request_id == (uint32_t)result_index + 1U);
        assert(completion.code == DEBUG_UI_STOPPED);
    }
    assert(!aethor_app_debug_ui_poll_completion(&completion));
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
}

/** @brief Retains exact STOP/cancel identities with full FIFOs at an arbitrary sequence boundary. */
static void test_p1_stop_full_result_queues_from_sequence(uint8_t initial_sequence)
{
    ProtocolOutputBatch output;
    ProtocolCommandResult result;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    DebugUiSnapshot snapshot;
    CanFrame frame;
    uint32_t result_index;
    uint8_t disable_count = 0U;
    test_fixture();
    application_deferred_result_read_sequence = initial_sequence;
    application_deferred_result_write_sequence = initial_sequence;
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench move 1 position=1 speed=1",
        strlen("89 bench move 1 position=1 speed=1"), 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(1002U);
    memset(&result, 0, sizeof(result));
    result.origin = DEBUG_UI_ORIGIN_USB;
    result.session_id = application_protocol_engine.session_id;
    result.type = PROTOCOL_COMMAND_DISABLE;
    result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    for (result_index = 0U; result_index < PROTOCOL_ENGINE_RESULT_CAPACITY + AETHOR_APP_DEFERRED_RESULT_CAPACITY; ++result_index)
    {
        result.request_id = 200U + result_index;
        assert(aethor_app_submit_or_defer_result(&result));
    }
    request = test_request(DEBUG_UI_OPERATION_STOP, 1003U);
    request.target_motor_id = 2U;
    admission = test_admission(request, 1003U);
    assert(admission.accepted);
    (void)aethor_app_service(1004U);
    while (aethor_app_pop_emergency_can_frame(&frame))
    { assert(frame.data[7] == 0xFDU); ++disable_count; }
    assert(disable_count >= 2U);
    assert(aethor_app_debug_ui_get_snapshot(1004U, &snapshot) && snapshot.stop_pending);
    assert(snapshot.active_motor_mask == 3U);
    /* Protocol is stalled and neither FIFO may be evicted to make room for STOP. */
    for (result_index = 0U; result_index < PROTOCOL_ENGINE_RESULT_CAPACITY + AETHOR_APP_DEFERRED_RESULT_CAPACITY; ++result_index)
    {
        assert(protocol_engine_peek_command_result(&application_protocol_engine, &result));
        assert(result.request_id == 200U + result_index);
        assert(result.origin == DEBUG_UI_ORIGIN_USB && result.code == PROTOCOL_COMMAND_RESULT_COMPLETED);
        assert(protocol_engine_pop_result_output(&application_protocol_engine, &output));
        (void)aethor_app_flush_deferred_results();
    }
    test_p1_feedback(1U, 1005U, S3519_DRIVER_STATE_DISABLED);
    test_p1_feedback(2U, 1005U, S3519_DRIVER_STATE_DISABLED);
    (void)aethor_app_service(1005U);
    assert(protocol_engine_peek_command_result(&application_protocol_engine, &result));
    assert(result.request_id == 89U && result.origin == DEBUG_UI_ORIGIN_USB);
    assert(result.code == PROTOCOL_COMMAND_RESULT_CANCELLED);
    assert(protocol_engine_pop_result_output(&application_protocol_engine, &output));
    aethor_app_debug_ui_process(1005U);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(completion.identity.request_id == request.identity.request_id && completion.code == DEBUG_UI_CANCELLED);
    assert(!aethor_app_debug_ui_poll_completion(&completion));
    assert(aethor_app_debug_ui_get_snapshot(1005U, &snapshot) && !snapshot.stop_pending);
}

/** @brief A first STOP bypasses full result FIFOs without losing terminals across uint8 wrap. */
static void test_p1_stop_all_result_queues_full(void)
{
    test_p1_stop_full_result_queues_from_sequence(0U);
    test_p1_stop_full_result_queues_from_sequence(254U);
}

/** @brief Coalescing preserves an older unadmitted STOP and a later USB target during cleanup. */
static void test_p1_stop_coalesced_scope(void)
{
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiSnapshot snapshot;
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t disable_mask = 0U;
    test_fixture(); test_acquire();
    admission = test_admission(test_request(DEBUG_UI_OPERATION_DISABLE, 1000U), 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    request = test_request(DEBUG_UI_OPERATION_STOP, 1002U);
    request.target_motor_id = 2U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    request = test_request(DEBUG_UI_OPERATION_STOP, 1003U);
    request.target_motor_id = 3U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_STOP_LATCHED);
    /* Protocol has not admitted STOP2, but its original requested target cannot be lost. */
    (void)aethor_app_service(1004U);
    assert(aethor_app_debug_ui_get_snapshot(1004U, &snapshot));
    assert(snapshot.active_motor_mask == 7U && snapshot.target_motor_id == 1U);
    while (aethor_app_pop_emergency_can_frame(&frame))
    { disable_mask |= (uint8_t)(1U << ((frame.identifier & 0xFFU) - 1U)); }
    assert(disable_mask == 7U);
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench stop 4", strlen("89 bench stop 4"),
        1006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(1007U);
    assert(aethor_app_debug_ui_get_snapshot(1007U, &snapshot));
    assert(snapshot.active_motor_mask == 15U);
    while (aethor_app_pop_emergency_can_frame(&frame))
    { disable_mask |= (uint8_t)(1U << ((frame.identifier & 0xFFU) - 1U)); }
    assert(disable_mask == 15U);
    assert(application_debug_ui.unconfirmed_disable_mask == 15U);
}

/** @brief A later USB STOP must retry the original failed local cleanup mask, even while idle. */
static void test_p1_stop_retries_locked_scope(void)
{
    DebugUiCompletion completion;
    ProtocolOutputBatch output;
    DebugUiAdmission admission;
    test_fixture(); test_p1_second_motor(); test_acquire();
    admission = test_admission(test_request(DEBUG_UI_OPERATION_DISABLE, 1000U), 1000U);
    assert(admission.accepted);
    (void)aethor_app_service(1001U);
    application_action.frame_read_index = application_action.frames.count;
    test_p1_feedback(1U, 501002U, S3519_DRIVER_STATE_ENABLED);
    test_health(501002U, true);
    (void)aethor_app_service(501002U);
    aethor_app_debug_ui_process(501002U);
    assert(aethor_app_debug_ui_poll_completion(&completion) && !completion.disabled_confirmed);
    assert(aethor_app_process_protocol_line("88 hello", 8U, 501003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench stop 2", strlen("89 bench stop 2"),
        501004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(501005U);
    assert(application_action.command.motor_mask == 3U);
    assert(application_action.command.cleanup_after_stop && application_action.frames.count >= 2U);
}

/** @brief STOP inherits local DISABLE and CLEAR_FAULT masks before replacing their frames. */
static void test_p1_stop_preserves_interrupted_scope(void)
{
    uint8_t operation_index;
    for (operation_index = 0U; operation_index < 2U; ++operation_index)
    {
        DebugUiAdmission admission;
        DebugUiCompletion completion;
        ProtocolOutputBatch output;
        test_fixture(); test_p1_second_motor(); test_acquire();
        test_p1_feedback(1U, 1001U, S3519_DRIVER_STATE_ENABLED);
        admission = test_admission(test_request(operation_index == 0U ? DEBUG_UI_OPERATION_DISABLE : DEBUG_UI_OPERATION_CLEAR_FAULT,
            1001U), 1001U);
        assert(admission.accepted);
        (void)aethor_app_service(1002U);
        assert(application_action.command.motor_mask == 1U && application_action.frame_read_index == 0U);
        assert(aethor_app_process_protocol_line("88 hello", 8U, 1003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(aethor_app_process_protocol_line("89 bench stop 2", strlen("89 bench stop 2"),
            1004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        (void)aethor_app_service(1005U);
        assert(application_action.command.type == PROTOCOL_COMMAND_STOP);
        assert(application_action.command.motor_mask == 3U);
        assert(application_action.frames.count >= 2U);
        application_action.frame_read_index = application_action.frames.count;
        test_p1_feedback(2U, 1006U, S3519_DRIVER_STATE_DISABLED);
        (void)aethor_app_service(1006U);
        assert(application_action.state != AETHOR_APP_ACTION_IDLE);
        test_p1_feedback(1U, 1007U, S3519_DRIVER_STATE_DISABLED);
        (void)aethor_app_service(1007U);
        aethor_app_debug_ui_process(1007U);
        assert(aethor_app_debug_ui_poll_completion(&completion) && completion.code == DEBUG_UI_CANCELLED);
        assert(aethor_app_pop_protocol_result_output(&output));
        assert(strstr(output.messages[0].data, "stopped=03") != NULL);
    }
}

/** @brief Pending and active USB targets never borrow the UI's selected or old lease target. */
static void test_p1_snapshot_actual_target(void)
{
    ProtocolOutputBatch output;
    DebugUiSnapshot snapshot;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest request;
    test_fixture(); test_acquire();
    admission = test_admission(test_request(DEBUG_UI_OPERATION_RELEASE, 1000U), 1000U);
    assert(admission.accepted);
    assert(aethor_app_debug_ui_poll_completion(&completion));
    assert(application_debug_ui.target_motor_id == 1U); /* Deliberately retained old lease. */
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench move 3 position=1 speed=1",
        strlen("89 bench move 3 position=1 speed=1"), 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
    assert(snapshot.active_motor_mask == 4U && snapshot.target_motor_id == 3U);
    assert(snapshot.active_identity.origin == DEBUG_UI_ORIGIN_USB && snapshot.active_identity.request_id == 89U);
    (void)aethor_app_service(1002U);
    assert(aethor_app_debug_ui_get_snapshot(1002U, &snapshot));
    assert(snapshot.active_motor_mask == 4U && snapshot.target_motor_id == 3U);
    assert(snapshot.active_identity.origin == DEBUG_UI_ORIGIN_USB && snapshot.active_identity.request_id == 89U);
    assert(snapshot.active_identity.epoch == application_protocol_engine.session_id);
    /* Non-motion pending commands have no active_motion ID but still own a true target. */
    test_fixture();
    assert(aethor_app_process_protocol_line("88 hello", 8U, 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_process_protocol_line("89 bench disable 3", strlen("89 bench disable 3"),
        1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_debug_ui_get_snapshot(1001U, &snapshot));
    assert(snapshot.active_motor_mask == 4U && snapshot.target_motor_id == 3U);
    assert(snapshot.active_identity.origin == DEBUG_UI_ORIGIN_USB && snapshot.active_identity.request_id == 89U);
    (void)aethor_app_service(1002U);
    assert(aethor_app_debug_ui_get_snapshot(1002U, &snapshot));
    assert(snapshot.active_motor_mask == 4U && snapshot.target_motor_id == 3U);
    test_fixture(); test_acquire();
    request = test_request(DEBUG_UI_OPERATION_STOP, 1003U);
    request.target_motor_id = 3U;
    assert(aethor_app_debug_ui_submit(&request) == DEBUG_UI_REASON_NONE);
    assert(aethor_app_debug_ui_get_snapshot(1003U, &snapshot));
    assert(snapshot.active_motor_mask == 4U && snapshot.target_motor_id == 3U);
    assert(snapshot.active_identity.origin == DEBUG_UI_ORIGIN_LOCAL_UI);
    assert(snapshot.active_identity.request_id == request.identity.request_id);
    assert(snapshot.stop_identity.request_id == request.identity.request_id);
}
#endif

/** @brief Runs contracts or an individually selectable persistent P1 regression. */
/** @brief LCD arrival uses output degrees while USB retains raw-coordinate tolerance. */
static void test_local_output_arrival_tolerance(void)
{
    MotorFeedbackSnapshot snapshot;
    float ratio;
    uint8_t failed = 0U;
    aethor_app_init(1U, 78U);
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&application_action, 0, sizeof(application_action));
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.bench_relative_scope = 1U;
    application_action.command.motor_mask = 64U;
    ratio = aethor_app_lcd_position_ratio(6U);
    snapshot.valid_joint_mask = 64U;
    snapshot.joints[6].position_rad = 0.49F * AETHOR_APP_DEG_TO_RAD * ratio;
    assert(aethor_app_one_shot_targets_arrived(&snapshot));
    snapshot.joints[6].position_rad = 0.51F * AETHOR_APP_DEG_TO_RAD * ratio;
    assert(!aethor_app_one_shot_targets_arrived(&snapshot));
    snapshot.joints[6].position_rad = 0.49F * AETHOR_APP_DEG_TO_RAD * ratio;
    application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
    snapshot.joints[6].timestamp_us = 1000U;
    assert(!aethor_app_local_pos_endpoint_arrived(&snapshot, 1000U, &failed));
    assert(application_action.local_pos_settle_start_us == 1000U && !failed);
    application_action.command.origin = DEBUG_UI_ORIGIN_USB;
    snapshot.joints[6].position_rad = 0.51F * AETHOR_APP_DEG_TO_RAD;
    assert(!aethor_app_one_shot_targets_arrived(&snapshot));
    snapshot.joints[6].position_rad = 0.49F * AETHOR_APP_DEG_TO_RAD;
    assert(aethor_app_one_shot_targets_arrived(&snapshot));
}

int main(int argument_count, char **arguments)
{
    test_local_output_arrival_tolerance();
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    if (argument_count == 2)
    {
        if (strcmp(arguments[1], "p1-lock") == 0) { test_p1_fault_target_lock(); }
        else if (strcmp(arguments[1], "p1-backpressure") == 0) { test_p1_stop_backpressure(); }
        else if (strcmp(arguments[1], "p1-scope") == 0) { test_p1_stop_preserves_interrupted_scope(); }
        else if (strcmp(arguments[1], "p1-snapshot") == 0) { test_p1_snapshot_actual_target(); }
        else if (strcmp(arguments[1], "p1-all-full") == 0) { test_p1_stop_all_result_queues_full(); }
        else if (strcmp(arguments[1], "p1-retry-scope") == 0) { test_p1_stop_retries_locked_scope(); }
        else if (strcmp(arguments[1], "p1-coalesced-scope") == 0) { test_p1_stop_coalesced_scope(); }
        else { return 2; }
        puts("debug_ui_app: selected P1 regression PASS");
        return 0;
    }
#else
    (void)argument_count;
    (void)arguments;
#endif
    test_engine_identity();
    test_runtime_stack_publication();
    test_idle_position_register_service();
    test_init_terminal_verification_snapshot();
    test_default_profile();
#if AETHOR_DEBUG_UI_ALLOW_MOTION
#if AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE
    test_motor7_protocol_envelope(DEBUG_UI_OPERATION_POS_MOVE);
#if AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE
    test_motor7_protocol_envelope(DEBUG_UI_OPERATION_MIT_MOVE);
#endif
#endif
    test_bench_pos_reference_trajectory();
    test_bench_pos_feedback_guard();
    test_local_pos_fixed_endpoint();
    test_local_pos_progress_and_settle();
    test_pos_preflight_fresh_anchor(DEBUG_UI_OPERATION_POS_MOVE);
#if AETHOR_DEBUG_UI_ALLOW_MIT
    test_pos_preflight_fresh_anchor(DEBUG_UI_OPERATION_MIT_MOVE);
    test_pos_preflight_fresh_anchor(DEBUG_UI_OPERATION_MIT_HOLD);
    test_pos_preflight_fresh_anchor(DEBUG_UI_OPERATION_SET_MODE);
    test_local_mit_feedback_guard();
    test_mit_ownership_and_independent_bounds();
#endif
    test_local_rejections();
    test_relative_start_and_queued_expiry();
    test_usb_local_arbitration();
    test_local_stop_remote_and_full_queue();
    test_independent_health_cleanup();
    test_mit_limits();
    test_results_and_idle_authority();
    test_idle_ui_fault_reacquire_with_stale_feedback();
    test_fault_recovery_stop_refresh();
    test_recovery_proof_survives_feedback_expiry();
    test_usb_session_disconnect_and_result_retention();
    test_missing_disable_feedback_and_reacquire();
    test_local_disable_actual_mode_and_fresh_ack();
    test_p1_fault_target_lock();
    test_p1_stop_backpressure();
    test_p1_stop_preserves_interrupted_scope();
    test_p1_snapshot_actual_target();
    test_p1_stop_all_result_queues_full();
    test_p1_stop_retries_locked_scope();
    test_p1_stop_coalesced_scope();
#endif
    puts("debug_ui_app: contracts PASS");
    return 0;
}
