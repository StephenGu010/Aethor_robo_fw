/** @file static_startup_sync_test.c
 * @brief Startup sequence, strict-token and passive trace regressions against the actual application facade.
 * Every receipt/feedback is synthetic; this executable has no HAL, serial, debugger or flash dependencies.
 */
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
#define STATIC_LOAD_TEST_ENTRY startup_existing_tests_main
#include "static_load_app_test.c"
#else
#define SINGLE_STATIC_TEST_ENTRY startup_existing_tests_main
#include "single_static_load_app_test.c"
#endif

#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Reach the production first PRELOAD using the original immutable request and disabled preflight. */
static void startup_fixture(ProtocolCommand *command, MotorFeedbackSnapshot *feedback)
{
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
    DebugUiRequest request = static_fixture(command, feedback);
#else
    DebugUiRequest request = single_fixture(7U, 1U, command, feedback);
#endif
    assert(test_admission(request, 1001U).accepted);
    assert(protocol_engine_pop_command(&application_protocol_engine, command));
    assert(!aethor_app_start_one_shot_move(command, feedback, 1002U));
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
    static_take_batch(1003U, S3519_MODE_COMMAND_DISABLE);
    static_feedback(2000U, S3519_DRIVER_STATE_DISABLED);
#else
    single_take_frame(7U, 1003U, S3519_MODE_COMMAND_DISABLE);
    single_feedback(7U, 2000U, S3519_DRIVER_STATE_DISABLED);
#endif
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 2000U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, feedback) == MOTOR_RUNTIME_STATUS_OK);
    assert(!aethor_app_advance_one_shot_setup(feedback, 2001U));
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
    assert(application_action.deadline_us == 501002ULL);
    assert(application_static_trace.header.active && !application_static_trace.header.frozen);
    assert(application_static_trace.header.started_us == 1002ULL);
}

/** @brief The application requests one exact ENABLE through MotorRuntime and fails closed on runtime/codec rejection. */
static void startup_enable_runtime_builder_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint8_t joint_index;
    unsigned scenario;
    startup_fixture(&command, &feedback);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        unsigned byte_index;
        uint64_t setup_deadline_us = application_action.static_setup_deadline_us;
        if (!(command.motor_mask & (1U << joint_index))) continue;
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_ENABLE, joint_index, 2002U);
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
        assert(application_action.frames.count == 1U && application_action.frame_read_index == 0U);
        assert(application_action.frames.frames[0].identifier == application_motor_runtime.configuration->joints[joint_index].esc_id);
        for (byte_index = 0U; byte_index < 7U; ++byte_index)
            assert(application_action.frames.frames[0].data[byte_index] == 0xFFU);
        assert(application_action.frames.frames[0].length == 8U &&
            application_action.frames.frames[0].data[7] == S3519_MODE_COMMAND_ENABLE);
        assert(application_action.static_setup_deadline_us == setup_deadline_us);
        assert(application_action.target_position_rad[joint_index] == command.values[joint_index]);
    }
    for (scenario = 0U; scenario < 2U; ++scenario) {
        ArmConfig invalid_configuration;
        const ArmConfig *original_configuration;
        startup_fixture(&command, &feedback);
        joint_index = (uint8_t)(application_action.static_identity.motor_id - 1U);
        original_configuration = application_motor_runtime.configuration;
        invalid_configuration = *original_configuration;
        if (!scenario) application_motor_runtime.initialized = 0U;
        else {
            invalid_configuration.joints[joint_index].esc_id = 0U;
            application_motor_runtime.configuration = &invalid_configuration;
        }
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_ENABLE, joint_index, 2002U);
        application_motor_runtime.configuration = original_configuration;
        application_motor_runtime.initialized = 1U;
        assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
        assert(!application_action.enabled_by_action_mask && !application_action.static_identity.phase);
        assert(application_static_trace.header.first_fault_valid);
        assert(!application_action.frames.count);
    }
    puts("STATIC_STARTUP_ENABLE_RUNTIME_BOUNDARY_PASS exact_single_axis_enable=1 uninitialized_and_codec_rejection_failclosed=2");
}

/** @brief Wrong epoch/request/phase/axis/sequence receipts cannot release debt or advance the first barrier. */
static void startup_identity_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    unsigned scenario;
    for (scenario = 0U; scenario < 9U; ++scenario) {
        DebugUiStaticTxIdentity identity, wrong;
        CanFrame frame;
        CanTxPriority priority;
        uint8_t phase_before;
        startup_fixture(&command, &feedback);
        assert(aethor_app_next_can_frame(2002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity));
        test_health(2003U, true);
        assert(aethor_app_static_submitted(&identity, 2003U));
        wrong = identity;
        if (scenario == 0U) ++wrong.epoch;
        if (scenario == 1U) ++wrong.request_id;
        if (scenario == 2U) ++wrong.phase;
        if (scenario == 3U) wrong.motor_id = identity.motor_id == 7U ? 1U : 7U;
        if (scenario == 4U) ++wrong.sequence;
        if (scenario == 5U) wrong.sequence = 0U;
        phase_before = application_action.static_identity.phase;
        if (scenario < 6U) {
            aethor_app_static_transmitted(&wrong, 1U, 100U, 2003U);
            assert(!application_action.single_static_tx_confirmed);
            assert(application_debug_ui.single_static_transport_mask);
            startup_test_feedback(identity.motor_id, S3519_DRIVER_STATE_DISABLED, 2004U, &wrong);
        } else {
            aethor_app_static_transmitted(&identity, 1U, 100U, 2003U);
            startup_test_feedback(identity.motor_id, S3519_DRIVER_STATE_DISABLED, 2004U,
                scenario == 6U ? NULL : &identity);
            if (scenario == 7U) startup_test_feedback(identity.motor_id, S3519_DRIVER_STATE_DISABLED, 2005U, NULL);
            if (scenario == 8U) {
                /* A fresh accepted other-ID frame cannot change this axis's ordered generation. */
                application_action.single_static_ordered_generation = application_action.single_static_rx_generation;
            }
        }
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 2005U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(&feedback, 2006U));
        assert(application_action.static_identity.phase == phase_before);
        assert(!application_action.static_preloaded_mask);
    }
    puts("STATIC_STARTUP_IDENTITY_PASS epoch_request_phase_axis_sequence_zero_missing_order_newer_unproved=9");
}

/** @brief All startup phases retain torque/temperature/drift/speed/driver/reference/stale guards and the absolute deadline. */
static void startup_guard_cases(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    uint8_t phase;
    unsigned scenario;
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase)
        for (scenario = 0U; scenario < 9U; ++scenario) {
            uint8_t joint_index, failure;
            startup_fixture(&command, &feedback);
            joint_index = (uint8_t)(application_action.static_identity.motor_id - 1U);
            aethor_app_static_prepare_phase(phase, joint_index, 2002U);
            if (scenario == 0U) feedback.joints[joint_index].torque_nm = 3.6F;
            if (scenario == 1U) feedback.joints[joint_index].mos_temperature_c = 56.0F;
            if (scenario == 2U) feedback.joints[joint_index].rotor_temperature_c = 56.0F;
            if (scenario == 3U) feedback.joints[joint_index].position_rad += aethor_app_lcd_position_ratio(joint_index);
            if (scenario == 4U) feedback.joints[joint_index].velocity_rad_s = 1000.0F;
            if (scenario == 5U) feedback.joints[joint_index].fault_flags = 1U;
            if (scenario == 6U) ++application_debug_ui.reference_generation[joint_index];
            if (scenario == 7U) feedback.valid_joint_mask &= (uint8_t)~(1U << joint_index);
            if (scenario == 8U) feedback.joints[joint_index].torque_nm = NAN;
            (void)aethor_app_check_one_shot_motor_safety(&feedback, 2003U, &failure);
            assert(failure && application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(application_static_trace.header.first_fault_valid);
            assert(!application_static_trace.header.frozen);
        }
    startup_fixture(&command, &feedback);
    aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_FIRST_HOLD,
        (uint8_t)(application_action.static_identity.motor_id - 1U), 500999U);
    assert(application_action.deadline_us == 501002ULL);
    assert(!aethor_app_static_advance_startup(&feedback, 501002U));
    assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    puts("STATIC_STARTUP_GUARDS_PASS phases=3 cases=27 frozen_setup_deadline=500000us");
}

/** @brief A real global transport fault retains startup/fault evidence through emitted emergency cleanup. */
static void startup_global_fault_trace_case(void)
{
    ProtocolCommand command;
    MotorFeedbackSnapshot feedback;
    CanFrame frame;
    unsigned scenario;
    for (scenario = 0U; scenario < 2U; ++scenario) {
        unsigned emitted = 0U, expected_frames;
        startup_fixture(&command, &feedback);
        startup_test_finish(&feedback, 2002U, 3001U);
        if (!scenario) assert(aethor_app_apply_transport_fault(123U, 4001U));
        else {
            application_diagnostics.counters.control_consecutive_miss_count = 3U;
            test_health(4001U, true);
            (void)aethor_app_service(4001U);
        }
        assert(application_static_trace.header.first_fault_valid);
        assert(application_static_trace.header.active && !application_static_trace.header.frozen);
        expected_frames = application_emergency_disable_batch.count;
        while (aethor_app_pop_emergency_can_frame(&frame)) {
            assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
            aethor_app_static_can_queued(&frame, 1U, 0U, 4002U);
            ++emitted;
        }
        assert(emitted == expected_frames && expected_frames >= 7U && expected_frames <= 14U &&
            !application_static_trace.header.frozen);
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
        static_feedback(5000U, S3519_DRIVER_STATE_DISABLED);
        test_health(5001U, true);
        (void)aethor_app_service(5001U);
        assert(application_static_trace.header.cleanup_outcome == 1U);
#else
        test_health(504002U, true);
        (void)aethor_app_service(504002U);
        assert(application_static_trace.header.cleanup_outcome == 2U);
#endif
        assert(application_static_trace.header.frozen && application_static_trace.header.terminal_valid);
        assert(application_static_trace.header.terminal.serial + 1U == application_static_trace.header.total);
    }
    puts("STATIC_STARTUP_GLOBAL_FAULT_TRACE_PASS transport_and_control_deadline=2 firstfault_before_terminal=1 emergency_scope_all7=1 both_identifier_if_unknown=1 safe_observation_or_timeout_explicit=1");
}

/** @brief Query every retained record/header chunk and prove exact generation, immutable bytes and no CAN side effects. */
static void startup_trace_cases(void)
{
    ProtocolCommand command, stop;
    ProtocolCommandResult result;
    MotorFeedbackSnapshot feedback;
    ProtocolOutputBatch output;
    uint32_t generation, total, index, part;
    uint8_t prefix[sizeof(DebugUiStaticTraceRecord)];
    char query[128], expected[8];
    startup_fixture(&command, &feedback);
    generation = application_static_trace.header.generation;
    assert(!aethor_app_process_protocol_line("97 static trace_status", 22U, 2002U, &output));
    assert(strstr(output.messages[0].data, "ok 97 static trace_status") && strstr(output.messages[0].data, "target_acceptance=unknown"));
    memcpy(prefix, &application_static_trace.records[0], sizeof(prefix));
    startup_test_finish(&feedback, 2002U, 3001U);
    assert(application_static_trace.header.record_size == 96U && application_static_trace.header.header_size == 832U);
    for (index = 0U; index < ARM_JOINT_COUNT; ++index)
        assert(application_static_trace.header.axes[index].target_rad == command.values[index]);
    aethor_app_static_trace_record(STATIC_TRACE_FIRST_FAULT, 1U, NULL, 4000U, 4000U, 0U, 0U, 7U);
    total = application_static_trace.header.total;
    aethor_app_static_trace_record(STATIC_TRACE_FIRST_FAULT, 2U, NULL, 4000U, 4000U, 0U, 0U, 99U);
    assert(application_static_trace.header.total == total);
    for (index = 0U; index < 300U; ++index)
        aethor_app_static_trace_record(STATIC_TRACE_RAW_RX, 1U, NULL, 4001U + index, 4001U + index, 0U, 0U, index);
    assert(application_static_trace.header.count == 128U && application_static_trace.header.overflow == application_static_trace.header.total - 128U);
    assert(!memcmp(prefix, &application_static_trace.records[0], sizeof(prefix)));
    assert(application_static_trace.header.first_fault.fault == 7U);
    memset(&stop, 0, sizeof(stop)); stop.origin = DEBUG_UI_ORIGIN_LOCAL_UI; stop.session_id = command.session_id;
    stop.request_id = command.request_id + 1U; stop.motor_mask = command.motor_mask;
    aethor_app_static_load_bind_stop(&stop);
    memset(&result, 0, sizeof(result)); result.origin = stop.origin; result.session_id = stop.session_id;
    result.request_id = stop.request_id; result.motor_mask = stop.motor_mask;
    result.local_operation = DEBUG_UI_OPERATION_STOP; result.code = PROTOCOL_COMMAND_RESULT_STOPPED;
    result.completed_at_us = 5000U;
    aethor_app_static_trace_terminal(&result);
    assert(application_static_trace.header.frozen && !application_static_trace.header.active && application_static_trace.header.terminal_valid);
    total = application_static_trace.header.total;
    for (index = 0U; index < application_static_trace.header.count; ++index) {
        (void)snprintf(query, sizeof(query), "98 static trace index=%lu part=0 generation=%lu", (unsigned long)index, (unsigned long)generation);
        assert(!aethor_app_process_protocol_line(query, strlen(query), 6000U, &output));
        assert(output.count == 1U && output.messages[0].length < 520U);
        assert(strstr(output.messages[0].data, "valid=1") && strstr(output.messages[0].data, "bytes=96 hex="));
        if (index == 0U) {
            (void)snprintf(expected, sizeof(expected), "%02x%02x", prefix[0], prefix[1]);
            assert(strstr(output.messages[0].data, expected));
        }
    }
    for (part = 0U; part < 7U; ++part) {
        (void)snprintf(query, sizeof(query), "99 static trace index=4294967295 part=%lu generation=%lu", (unsigned long)part, (unsigned long)generation);
        assert(!aethor_app_process_protocol_line(query, strlen(query), 6000U, &output));
        assert(strstr(output.messages[0].data, "valid=1") && output.messages[0].length < 520U);
    }
    (void)snprintf(query, sizeof(query), "100 static trace index=0 part=0 generation=%lu", (unsigned long)(generation + 1U));
    assert(!aethor_app_process_protocol_line(query, strlen(query), 6000U, &output));
    assert(strstr(output.messages[0].data, "valid=0"));
    assert(!aethor_app_process_protocol_line("static trace_status", strlen("static trace_status"), 6000U, &output));
    assert(strstr(output.messages[0].data, "valid=0"));
    assert(!aethor_app_process_protocol_line("0 static trace_status", strlen("0 static trace_status"), 6000U, &output));
    assert(strstr(output.messages[0].data, "valid=0"));
    for (index = 0U; index < 64U; ++index) {
        assert(application_static_trace.records[index].serial == index);
        assert(application_static_trace.records[64U + (application_static_trace.tail_next + index) % 64U].serial == total - 64U + index);
    }
    assert(application_static_trace.header.total == total && application_static_trace.header.generation == generation);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
    puts("STATIC_STARTUP_TRACE_PASS prefix_tail_overflow_firstfault_terminal_frozen_generation_header_records_bounds_readonly=1");
}
#endif

/** @brief Execute inherited full lifecycle regressions and focused startup contracts in each compile-time gate configuration. */
int main(void)
{
    assert(startup_existing_tests_main() == 0);
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    startup_enable_runtime_builder_cases();
    startup_global_fault_trace_case();
    startup_identity_cases();
    startup_guard_cases();
    startup_trace_cases();
#endif
    return 0;
}
