/** @file flange_home_test.c
 * @brief Exercise fixed absolute HOME through the production adapter and guarded POS executor; synthetic only.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Publish a synthetic standard ID7 sample without touching discovered mode or register telemetry. */
static void home_feedback(uint64_t now_us, float position)
{
    MotorJointFeedback feedback;
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = now_us; feedback.position_rad = position;
    feedback.driver_state = S3519_DRIVER_STATE_DISABLED;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Prepare a synthetic remotely owned POS identity with fresh ordinary feedback and checked UI health. */
static void home_fixture(void)
{
    ProtocolOutputBatch output; DebugUiHealth health; char line[180];
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 4000000123UL); fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    home_feedback(1000000U, 0.2F);
    memset(&health, 0, sizeof(health)); health.ui_timestamp_us = 1000000U;
    health.ui_sequence = 1U; health.ui_valid = health.input_valid = health.display_valid = 1U;
    aethor_app_debug_ui_update_health(&health); aethor_app_debug_ui_process(1000000U);
    request("1 ui caps", 1000001U, &output);
    snprintf(line, sizeof(line), "2 ui submit token=%lu op=acquire delta=0 speed=0.02 kp=120 kd=0.3 torque=0 duration=1000",
        (unsigned long)application_remote_ui.token);
    request(line, 1000002U, &output); aethor_app_debug_ui_process(1000003U);
    request("3 ui result id=2", 1000004U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    request("4 ui caps", 1000005U, &output);
}

/** @brief Serialize eight fields while preserving every uint32 identity digit. */
static void home_submit(const char *target, uint32_t boot, uint32_t reference, uint32_t duration, ProtocolOutputBatch *output)
{
    char line[210];
    snprintf(line, sizeof(line), "5 ui submit token=%lu op=home delta=%s speed=0.02 kp=%lu kd=%lu torque=0 duration=%lu",
        (unsigned long)application_remote_ui.token, target, (unsigned long)boot,
        (unsigned long)reference, (unsigned long)duration);
    request(line, 1000006U, output);
}

/** @brief HOME is independently detectable and queues its immutable target, including an absolute zero. */
static void test_home_protocol(void)
{
    ProtocolOutputBatch output; ProtocolCommand command; uint32_t reference;
    home_fixture(); request("4 ui caps", 1000005U, &output);
    assert(strstr(output.messages[0].data, "home=1"));
    assert(strstr(output.messages[0].data, "coord="));
    reference = application_remote_ui.review.motors[6].reference_generation;
    home_submit("0", 4000000123UL, reference, 1000U, &output);
    assert(strstr(output.messages[0].data, "result=queued"));
    aethor_app_debug_ui_process(1000007U);
    assert(protocol_engine_pop_command(&application_protocol_engine, &command));
    assert((unsigned)command.local_operation == 15U && !command.relative_target);
    assert(command.type == PROTOCOL_COMMAND_HOME_PREPARE && !command.home_prepared);
    assert(command.values[6] == 0.0F && command.mit_kp == 0.0F && command.mit_kd == 0.0F);
    assert(command.reference_generation == reference && command.hold_duration_us == 1000000ULL);
    home_fixture(); application_debug_ui.reference_generation[6] = 4000000456UL;
    request("4 ui caps", 1000005U, &output);
    assert(strstr(output.messages[0].data, "coord=4000000456"));
    home_submit("0", 4000000123UL, 4000000456UL, 1000U, &output);
    assert(strstr(output.messages[0].data, "result=queued"));
    aethor_app_debug_ui_process(1000007U);
    assert(protocol_engine_pop_command(&application_protocol_engine, &command));
    assert(command.home_boot_id == 4000000123UL && command.reference_generation == 4000000456UL);
    { ProtocolCommand forged = command;
      forged.local_operation = DEBUG_UI_OPERATION_POS_MOVE;
      assert(protocol_engine_validate_typed_command(&forged) == DEBUG_UI_REASON_INVALID_ARGUMENT); }
    home_fixture();
    application_remote_ui.sequence = UINT32_MAX - 1U;
    application_remote_ui.last_external_id = application_remote_ui.external_id = application_remote_ui.terminal_id = UINT32_MAX;
    application_remote_ui.active = 1U; application_debug_ui.reference_generation[6] = UINT32_MAX;
    assert(request("4294967295 ui caps", 1000005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U && strstr(output.messages[0].data, "home=1 coord=4294967295"));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    puts("HOME_PROTOCOL_PASS eight_fields absolute_zero uint32_identity");
}

/** @brief Encode a new standard feedback sample with explicit synthetic hardware ordering evidence. */
static void home_ordered_feedback(uint8_t state, uint64_t now_us, const DebugUiStaticTxIdentity *identity)
{
    CanFrame encoded, reply; const MotorJointFeedback *feedback = &application_motor_runtime.bank.motors[6].feedback;
    assert(s3519_pack_mit(7U, &application_motor_runtime.discovery.results[6].ranges,
        feedback->position_rad, 0.0F, 0.0F, 0.0F, 0.0F, &encoded) == S3519_CODEC_STATUS_OK);
    memset(&reply, 0, sizeof(reply)); reply.identifier = 23U; reply.length = 8U;
    reply.data[0] = (uint8_t)((state << 4U) | 7U); reply.data[1] = encoded.data[0]; reply.data[2] = encoded.data[1];
    reply.data[3] = encoded.data[2]; reply.data[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    reply.data[5] = encoded.data[7]; reply.data[6] = 30U; reply.data[7] = 31U;
    assert(aethor_app_receive_static_frame(&reply, now_us, now_us, identity, 101U, 1U) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Actual-TX and newer ordered RX barriers precede the fixed motion frame; STOP then emits selected DISABLE. */
static void home_startup_and_stop(MotorFeedbackSnapshot *snapshot, float target)
{
    unsigned phase; CanFrame frame; CanTxPriority priority; ProtocolOutputBatch output;
    for (phase = DEBUG_UI_STATIC_TX_PRELOAD; phase <= DEBUG_UI_STATIC_TX_FIRST_HOLD; ++phase) {
        uint64_t now_us = 1002000ULL + phase * 10ULL; DebugUiStaticTxIdentity identity;
        assert(aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity) && identity.phase == phase);
        assert(!aethor_app_advance_one_shot_setup(snapshot, now_us));
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
        assert(aethor_app_static_submitted(&identity, now_us + 1ULL));
        aethor_app_static_transmitted(&identity, 1U, 100U, now_us + 1ULL);
        if (phase == DEBUG_UI_STATIC_TX_ENABLE) continue;
        home_ordered_feedback(phase == DEBUG_UI_STATIC_TX_PRELOAD ? S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED,
            now_us + 2ULL, &identity);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, now_us + 3ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_advance_one_shot_setup(snapshot, now_us + 3ULL));
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
    assert(aethor_app_next_can_frame(1002100U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    { float sent_position; memcpy(&sent_position, frame.data, sizeof(sent_position));
      assert(frame.identifier == 0x107U && fabsf(sent_position - target) < 0.000001F); }
    request("20 bench stop 7", 1002101U, &output);
    (void)aethor_app_service(1002102U);
    assert(aethor_app_pop_emergency_can_frame(&frame));
    assert(frame.length == 8U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(!application_debug_ui.pos_load.disabled_confirmed);
    puts("HOME_ACTUAL_TX_STOP_PASS preload_enable_firsthold_order fixed_wire_target selected_disable no_false_confirmation");
}

/** @brief Missing HOME capability must fail before implementation; stale identity and invalid inputs fail closed afterward. */
static void test_home_rejections(void)
{
    ProtocolOutputBatch output; unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario) {
        uint32_t reference;
        home_fixture(); reference = application_remote_ui.review.motors[6].reference_generation;
        home_submit(scenario == 2U ? "nan" : scenario == 3U ? "99999" : "0",
            scenario == 0U ? 4000000122UL : 4000000123UL,
            scenario == 1U ? reference + 1U : reference, scenario == 4U ? 2000U : 1000U, &output);
        assert(strstr(output.messages[0].data, "code=rejected"));
        assert(!application_remote_ui.active);
    }
    puts("HOME_REJECTIONS_PASS boot reference nonfinite bounds duration");
}

/** @brief Preparation uses a new standard start while keeping the absolute target and qualification gates. */
static void test_home_prepared_target(void)
{
    ProtocolOutputBatch output; ProtocolCommand command; MotorFeedbackSnapshot snapshot; CanFrame frame; CanTxPriority priority;
    float native_target; unsigned scenario;
    for (scenario = 0U; scenario < 6U; ++scenario) {
        home_fixture();
        if (scenario == 2U) {
            application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
            request("4 ui caps", 1000005U, &output);
        }
        home_submit("0.1", 4000000123UL, application_remote_ui.review.motors[6].reference_generation, 1000U, &output);
        assert(strstr(output.messages[0].data, "result=queued"));
        aethor_app_debug_ui_process(1000007U);
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        native_target = command.home_target_position_rad;
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1000007U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!aethor_app_start_one_shot_move(&command, &snapshot, 1000008U));
        while (application_action.frame_read_index < application_action.frames.count)
            assert(aethor_app_next_can_frame(1000009U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        home_feedback(1001000U, scenario == 2U ? -150.0F : scenario == 5U ? native_target : 0.35F);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        if (scenario == 1U) snapshot.valid_joint_mask = 0U;
        if (scenario == 3U) application_protocol_engine.boot_id++;
        if (scenario == 4U) application_debug_ui.reference_generation[6]++;
        (void)aethor_app_advance_one_shot_setup(&snapshot, 1001002U);
        if (scenario == 0U || scenario == 5U) {
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(aethor_app_is_pos_load());
            assert(application_action.command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED);
            assert(application_action.command.home_prepared && application_action.command.relative_target);
            assert(fabsf(application_action.target_position_rad[6] - native_target) < 0.000001F);
            assert(fabsf(application_debug_ui.pos_load_start_position_rad - snapshot.joints[6].position_rad) < 0.000001F);
            assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
            if (scenario == 0U) {
                application_protocol_engine.boot_id++;
                assert(aethor_app_pos_load_guard(&snapshot, 6U, 1001003U) == DEBUG_UI_REASON_OLD_EPOCH);
                application_protocol_engine.boot_id--;
                home_startup_and_stop(&snapshot, native_target);
            }
        } else {
            assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(application_action.static_identity.phase != DEBUG_UI_STATIC_TX_PRELOAD);
            if (scenario == 2U) {
                ProtocolCommandResult result;
                assert(protocol_engine_peek_command_result(&application_protocol_engine, &result));
                printf("HOME_RANGE_RECEIPT reason=%u target=%.7g actual=%.7g bound=%.7g\n", result.local_reason,
                    (double)native_target, (double)snapshot.joints[6].position_rad, (double)aethor_app_pos_load_bounds(6U).max_delta_rad);
                assert(result.local_reason == DEBUG_UI_REASON_OUT_OF_RANGE);
            }
        }
    }
    puts("HOME_PREPARED_TARGET_PASS fresh_start fixed_target stale_feedback_range_boot_reference_closed zero_displacement");
}

/** @brief Passive position and HOME advertise the same integer boot/reference, with bounded worst-case output. */
static void test_home_passive_coordinate_identity(void)
{
    ProtocolOutputBatch output; char expected[80]; uint32_t reference;
    home_fixture();
    assert(request("6 show motor 7", 1000008U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    snprintf(expected, sizeof(expected), "coord_boot=%lu coord_ref=%lu",
        (unsigned long)application_protocol_engine.boot_id,
        (unsigned long)application_debug_ui.reference_generation[6]);
    assert(strstr(output.messages[0].data, expected));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    reference = application_debug_ui.reference_generation[6];
    application_motor_runtime.discovery.results[6].observed_master_id++;
    assert(request("7 show motor 7", 1000009U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_debug_ui.reference_generation[6] != reference);
    puts("HOME_PASSIVE_IDENTITY_PASS show_motor_matches_caps");
}

/** @brief A stale standard cache cannot reject a valid multi-turn HOME before the DISABLE preparation obtains its real start. */
static void test_home_multiturn_preflight_intent(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario) {
        ProtocolOutputBatch output; ProtocolCommand command; MotorFeedbackSnapshot snapshot;
        CanFrame frame; CanTxPriority priority; float fixed_target;
        home_fixture();
        application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
        application_motor_runtime.bank.motors[6].feedback.timestamp_us = 1U;
        application_motor_runtime.bank.motors[6].feedback.position_rad = 0.0F;
        request("4 ui caps", 1000005U, &output);
        home_submit("20.829862", 4000000123UL, application_remote_ui.review.motors[6].reference_generation, 1000U, &output);
        assert(strstr(output.messages[0].data, "result=queued"));
        aethor_app_debug_ui_process(1000007U);
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        fixed_target = command.home_target_position_rad; assert(fabsf(fixed_target - 400.0F) < 0.01F);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1000007U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        assert(!(snapshot.valid_joint_mask & 0x40U));
        assert(!aethor_app_start_one_shot_move(&command, &snapshot, 1000008U));
        assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
        assert(aethor_app_active_action_owns_link_lifecycle());
        assert(!application_action.static_identity.phase);
        while (application_action.frame_read_index < application_action.frames.count) {
            assert(aethor_app_next_can_frame(1000009U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            assert(frame.length == 8U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        }
        if (scenario == 2U) {
            request("20 bench stop 7", 1000010U, &output); (void)aethor_app_service(1000011U);
            assert(aethor_app_pop_emergency_can_frame(&frame));
            assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
            assert(application_action.static_identity.phase != DEBUG_UI_STATIC_TX_PRELOAD);
            continue;
        }
        if (scenario == 3U) {
            application_motor_runtime.idle_position_read.seen_mask = 3U;
            application_motor_runtime.idle_position_read.sample_us[0] = 1001000U;
            application_motor_runtime.idle_position_read.values[0] = 410.0F;
            (void)aethor_app_advance_one_shot_setup(&snapshot, 1001002U);
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT);
            assert(!application_action.command.home_prepared && !application_action.static_identity.phase);
            continue;
        }
        if (scenario == 4U) application_action.command.home_target_position_rad += 1.0F;
        home_feedback(1001000U, scenario ? 200.0F : 410.0F);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, 1001001U,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
        (void)aethor_app_advance_one_shot_setup(&snapshot, 1001002U);
        if (!scenario) {
            assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
            assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD);
            assert(application_action.target_position_rad[6] == fixed_target);
            assert(application_debug_ui.pos_load_start_position_rad == 410.0F);
        } else {
            ProtocolCommandResult result;
            assert(application_action.static_identity.phase != DEBUG_UI_STATIC_TX_PRELOAD);
            assert(protocol_engine_peek_command_result(&application_protocol_engine, &result));
            assert(result.local_reason == (scenario == 4U ? DEBUG_UI_REASON_STALE_FEEDBACK : DEBUG_UI_REASON_OUT_OF_RANGE));
        }
    }
    puts("HOME_MULTITURN_INTENT_PASS stale_cache_ignored disable_only_preparation fixed400_fresh410_allowed fixed400_fresh200_rejected stop_before_prepare register_no_qualification frozen_target_checked lifecycle_owned");
}

/** @brief Existing POS operations still reject excessive relative delta; a forged prepared HOME cannot enter motion directly. */
static void test_relative_bounds_and_forged_prepared_home(void)
{
    ProtocolOutputBatch output; ProtocolCommand command; MotorFeedbackSnapshot snapshot; char line[210];
    home_fixture();
    snprintf(line, sizeof(line), "5 ui submit token=%lu op=pos delta=100 speed=0.02 kp=120 kd=0.3 torque=0 duration=1000",
        (unsigned long)application_remote_ui.token);
    request(line, 1000006U, &output); assert(strstr(output.messages[0].data, "result=queued"));
    aethor_app_debug_ui_process(1000007U); request("6 ui result id=5", 1000008U, &output);
    assert(strstr(output.messages[0].data, "result=failed") && strstr(output.messages[0].data, "reason=11"));
    home_fixture();
    snprintf(line, sizeof(line), "5 ui submit token=%lu op=pos_load delta=100 speed=0.02 kp=0 kd=0 torque=0 duration=1000",
        (unsigned long)application_remote_ui.token);
    request(line, 1000006U, &output); assert(strstr(output.messages[0].data, "code=rejected reason=11"));
    home_fixture();
    home_submit("0", 4000000123UL, application_remote_ui.review.motors[6].reference_generation, 1000U, &output);
    aethor_app_debug_ui_process(1000007U); assert(protocol_engine_pop_command(&application_protocol_engine, &command));
    command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED; command.relative_target = command.home_prepared = 1U;
    assert(motor_runtime_get_snapshot(&application_motor_runtime, 1000007U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_validate_start(&command, &snapshot, 1000008U, false) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    (void)aethor_app_start_one_shot_move(&command, &snapshot, 1000008U);
    assert(application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT && !application_action.static_identity.phase);
    puts("HOME_NO_BYPASS_PASS relative_pos_bounds relative_pos_load_bounds forged_prepared_move_cannot_start");
}

/** @brief Run software-only production regressions with no serial, SWD or hardware access. */
int main(void)
{
    setbuf(stdout, NULL); test_home_protocol(); test_home_rejections(); test_home_prepared_target();
    test_home_passive_coordinate_identity(); test_home_multiturn_preflight_intent();
    test_relative_bounds_and_forged_prepared_home(); return 0;
}
