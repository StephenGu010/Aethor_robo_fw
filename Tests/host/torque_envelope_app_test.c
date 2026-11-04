/** @file Synthetic production-adapter tests for ID7 exposure maintenance and exact operation scope. */
#define main existing_torque_integration_main
#include "adrc_lcd_integration_test_main.c"
#undef main
static uint32_t torque_test_link;
static uint8_t torque_test_idle;
static uint8_t torque_test_change_link;
static void torque_reset_request(uint32_t token, uint64_t now_us, uint8_t expect_success);
/** @brief Synthetic USB configuration/generation evidence for maintenance identity binding. */
static uint32_t torque_link_probe(void)
{
    uint32_t observed_link = torque_test_link;
    if (torque_test_change_link) { ++torque_test_link; torque_test_change_link = 0U; }
    return observed_link;
}
/** @brief Synthetic transport debt probe; maintenance itself must not emit CAN. */
static uint8_t torque_idle_probe(void) { return torque_test_idle; }

/** @brief Publish explicit synthetic disabled evidence and healthy UI state. */
static void torque_fixture(uint8_t fresh)
{
    DebugUiHealth health;
    ProtocolOutputBatch output;
    aethor_app_init(1U, 1234U);
    torque_test_link = 0x80000001UL; torque_test_idle = 1U;
    torque_test_change_link = 0U;
    aethor_app_bind_remote_release_probes(torque_idle_probe, torque_link_probe);
    assert(request("1 hello ui=0123456789abcdef0123456789abcdef", 900000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    if (fresh) fixture_disabled_motor7(1000000U); else fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    memset(&health, 0, sizeof(health));
    health.ui_timestamp_us = 1000000U; health.ui_sequence = 1U;
    health.ui_valid = health.input_valid = health.display_valid = 1U;
    aethor_app_debug_ui_update_health(&health);
    aethor_app_debug_ui_process(1000000U);
}

/** @brief Capture a maintenance token from the real read-only route. */
static uint32_t torque_query(uint64_t now_us, uint8_t expected_reset)
{
    ProtocolOutputBatch output;
    assert(request("1000 ui torque", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U && strstr(output.messages[0].data, "window_ms=10000"));
    assert(strstr(output.messages[0].data, expected_reset ? "can_reset=1" : "can_reset=0"));
    return application_remote_ui.torque_token;
}

/** @brief New wire requests must atomically set millinewton-metre limits without any motion frame. */
static void adjustable_request(uint32_t token, const char *peak, uint64_t now_us, uint8_t accepted)
{
    char line[160];
    ProtocolOutputBatch output;
    TorqueEnvelope before = application_torque_envelope;
    snprintf(line, sizeof(line), "2001 ui torque_reset token=%lu cold=1 peak_mnm=%s", (unsigned long)token, peak);
    assert(request(line, now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U);
    assert(strstr(output.messages[0].data, accepted ? "result=reset" : "code=rejected"));
    if (!accepted) assert(!memcmp(&before, &application_torque_envelope, sizeof(before)));
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && !application_action.frames.count);
}

/** @brief Exercise the actual protocol boundary, strict units and unchanged ledger on reads/failures. */
static void adjustable_wire_contract(void)
{
    static const char *invalid[] = { "3499", "6241", "-3500", "+3500", "3.5", "NaN", "inf", "6240.0", "4294967296" };
    ProtocolOutputBatch output;
    uint32_t token;
    unsigned index;
    torque_fixture(1U);
    token = torque_query(1000001U, 1U);
    adjustable_request(token, "6240", 1000002U, 1U);
    assert(request("2002 ui torque", 1000003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "policy_v=2 min_mnm=3500 max_mnm=6240 peak_mnm=6240 blockers=0"));
    assert(strstr(output.messages[0].data, "peak=6.240 window_ms=10000"));
    assert(torque_envelope_observe(&application_torque_envelope, 1000003U, 1000003U, 6.24F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&application_torque_envelope, 1004003U, 1004003U, 6.24F) == TORQUE_ENVELOPE_OK);
    for (index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); ++index) {
        TorqueEnvelope before = application_torque_envelope;
        token = torque_query(1004004U + index * 2U, 1U);
        assert(!memcmp(&before, &application_torque_envelope, sizeof(before)));
        adjustable_request(token, invalid[index], 1004005U + index * 2U, 0U);
        adjustable_request(token, "6240", 1004005U + index * 2U, 0U);
    }
    token = torque_query(1004100U, 1U);
    adjustable_request(token, "3500", 1004101U, 1U);
    assert(torque_envelope_observe(&application_torque_envelope, 1004102U, 1004102U, -3.5F) == TORQUE_ENVELOPE_OK);
    assert(torque_envelope_observe(&application_torque_envelope, 1004103U, 1004103U, -3.501F) == TORQUE_ENVELOPE_PEAK);
    token = torque_query(1004200U, 1U);
    torque_reset_request(token, 1004201U, 1U);
    assert(torque_envelope_observe(&application_torque_envelope, 1004202U, 1004202U, 5.461F) == TORQUE_ENVELOPE_PEAK);
}

/** @brief Identity changes, busy transport and missing disable evidence invalidate single-use restoration. */
static void adjustable_identity_binding(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 11U; ++scenario) {
        uint32_t token;
        TorqueEnvelope before_identity_change;
        ProtocolOutputBatch output;
        torque_fixture(1U);
        assert(torque_envelope_reset_cold_selected(&application_torque_envelope, 6240U));
        assert(torque_envelope_observe(&application_torque_envelope, 992000U, 992000U, 5.0F) == TORQUE_ENVELOPE_OK);
        assert(torque_envelope_observe(&application_torque_envelope, 996000U, 996000U, 5.0F) == TORQUE_ENVELOPE_OK);
        before_identity_change = application_torque_envelope;
        token = torque_query(1000001U, 1U);
        switch (scenario) {
        case 0: ++application_protocol_engine.session_id; break;
        case 1: ++application_protocol_engine.next_session_nonce; break;
        case 2: ++torque_test_link; break;
        case 3: torque_test_link = 0U; break;
        case 4: torque_test_idle = 0U; break;
        case 5: application_protocol_engine.session_id = 0U; break;
        case 6: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
        case 7: assert(request("2 hello ui=fedcba9876543210fedcba9876543210", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK); break;
        case 8: assert(request("2 hello", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK); break;
        case 9: ++application_protocol_engine.boot_id; break;
        default: ++application_debug_ui.epoch; break;
        }
        assert(!memcmp(&before_identity_change, &application_torque_envelope, sizeof(before_identity_change)));
        adjustable_request(token, "6240", 1000002U, 0U);
        adjustable_request(token, "6240", 1000003U, 0U);
    }
}

/** @brief Reconnection requires a new successfully processed hello, not a cached response or a query rebind. */
static void adjustable_hello_link_binding(void)
{
    ProtocolOutputBatch output;
    TorqueEnvelope original;
    uint32_t token;
    torque_fixture(1U);
    original = application_torque_envelope;
    token = torque_query(1000001U, 1U);
    ++torque_test_link;
    assert(torque_query(1000002U, 0U) == 0U);
    adjustable_request(token, "6240", 1000003U, 0U);
    /* A cached old hello response cannot qualify the new physical connection. */
    assert(request("1 hello ui=0123456789abcdef0123456789abcdef", 1000004U, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(torque_query(1000005U, 0U) == 0U);
    assert(!memcmp(&original, &application_torque_envelope, sizeof(original)));
    assert(request("2 hello ui=fedcba9876543210fedcba9876543210", 1000006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    token = torque_query(1000007U, 1U);
    adjustable_request(token, "6240", 1000008U, 1U);
    torque_fixture(1U);
    application_protocol_engine.ui_link_word = NULL;
    assert(request("2 hello ui=fedcba9876543210fedcba9876543210", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(torque_query(1000002U, 0U) == 0U);
    protocol_engine_bind_ui_link_probe(&application_protocol_engine, torque_link_probe);
    assert(torque_query(1000003U, 0U) == 0U); /* Late probe binding cannot bless an old hello. */
    torque_test_change_link = 1U;
    assert(request("3 hello ui=0123456789abcdef0123456789abcdef", 1000004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(torque_query(1000005U, 0U) == 0U); /* A generation transition during hello is unqualified. */
    assert(request("4 hello ui=fedcba9876543210fedcba9876543210", 1000006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(torque_query(1000007U, 1U) != 0U);
    puts("ADJUSTABLE_HELLO_LINK_PASS reconnect_without_hello_closed_cached_hello_closed_fresh_hello_required");
}

/** @brief Every reported blocker must close maintenance and preserve both the selected line and exposure ledger. */
static void adjustable_blockers(void)
{
    static const uint32_t expected[] = { 1U, 2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U, 512U };
    unsigned scenario;
    for (scenario = 0U; scenario < sizeof(expected) / sizeof(expected[0]); ++scenario) {
        ProtocolOutputBatch output;
        char expected_field[32];
        TorqueEnvelope original;
        torque_fixture(1U);
        original = application_torque_envelope;
        switch (scenario) {
        case 0: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; break;
        case 1: torque_test_idle = 0U; break;
        case 2: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
        case 3: application_protocol_engine.session_id = 0U; break;
        case 4: application_controller.fault = ARM_FAULT_DRIVER; break;
        case 5: application_motor_runtime.bank.motors[6].feedback.timestamp_us = 0U; break;
        case 6: application_motor_runtime.bank.motors[6].feedback.driver_state = 1U; break;
        case 7: application_motor_runtime.bank.motors[6].feedback.velocity_rad_s = 1.0F; break;
        case 8: application_motor_runtime.bank.motors[6].feedback.torque_nm = 3.501F; break;
        default: application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 35.01F; break;
        }
        assert(request("1000 ui torque", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        snprintf(expected_field, sizeof(expected_field), "blockers=%lu\r\n", (unsigned long)expected[scenario]);
        assert(strstr(output.messages[0].data, expected_field));
        assert(strstr(output.messages[0].data, "can_reset=0 token=0"));
        assert(!memcmp(&original, &application_torque_envelope, sizeof(original)));
    }
}

/** @brief UINT32_MAX identity values and every required field fit the fixed protocol frame without truncation. */
static void adjustable_reply_capacity(void)
{
    ProtocolOutputBatch output;
    torque_fixture(1U);
    application_protocol_engine.boot_id = UINT32_MAX;
    application_debug_ui.reference_generation[6] = UINT32_MAX;
    application_remote_ui.sequence = UINT32_MAX - 1U;
    assert(request("4294967295 ui torque", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U && output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    assert(strstr(output.messages[0].data, "token=4294967295 boot=4294967295 coord=4294967295"));
    assert(strstr(output.messages[0].data, "used=175.616000"));
    assert(strstr(output.messages[0].data, "peak_mnm=5460 blockers=0\r\n"));
    printf("ADJUSTABLE_FRAME_PASS policy_bytes=%u capacity=%u\n", output.messages[0].length, PROTOCOL_ENGINE_MESSAGE_CAPACITY);
}

/** @brief Exercise the explicit one-use maintenance route without sending any CAN frame. */
static void torque_reset_request(uint32_t token, uint64_t now_us, uint8_t expect_success)
{
    char line[120];
    ProtocolOutputBatch output;
    (void)snprintf(line, sizeof(line), "1001 ui torque_reset token=%lu cold=1", (unsigned long)token);
    assert(request(line, now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U);
    assert(strstr(output.messages[0].data, expect_success ? "result=reset" : "code=rejected"));
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_action.frames.count == 0U && !application_remote_ui.owns && !application_remote_ui.active);
}

/** @brief Protection in the existing disable phase must retain a failed terminal without adding frames. */
static void torque_disable_failure(unsigned scenario)
{
    MotorFeedbackSnapshot snapshot;
    ProtocolCommandResult terminal;
    uint64_t now_us = 11004000ULL;
    torque_fixture(1U);
    torque_envelope_reset_cold(&application_torque_envelope);
    assert(torque_envelope_observe(&application_torque_envelope, now_us - 4000ULL, now_us - 4000ULL, 5.0F) == TORQUE_ENVELOPE_OK);
    memset(&application_action, 0, sizeof(application_action));
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.motor_mask = 0x40U;
    application_action.command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    application_action.command.type = PROTOCOL_COMMAND_MIT_ACTION;
    application_action.command.control_mode = ARM_CONTROL_MODE_MIT;
    application_action.command.request_id = 123U;
    application_action.state = AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT;
    memset(&snapshot, 0, sizeof(snapshot)); snapshot.valid_joint_mask = 0x40U;
    snapshot.joints[6].timestamp_us = scenario == 1U ? now_us - 21000ULL : now_us;
    snapshot.joints[6].torque_nm = scenario == 0U ? 5.47F : 5.0F;
    if (scenario == 2U) application_torque_envelope.exposure_nm2_s = TORQUE_ENVELOPE_CAPACITY_NM2_S - 0.001;
    if (scenario == 3U) application_torque_envelope.excess_started_us = now_us - TORQUE_ENVELOPE_MAX_EXCESS_US;
    aethor_app_torque_envelope_background(&snapshot, now_us);
    assert(application_torque_fault.valid && application_torque_fault.authorized_before);
    assert(application_torque_fault.request_id == 123U);
    assert(application_torque_fault.reason == (scenario == 0U ? TORQUE_ENVELOPE_PEAK :
        scenario == 1U ? TORQUE_ENVELOPE_FEEDBACK : scenario == 2U ? TORQUE_ENVELOPE_BUDGET : TORQUE_ENVELOPE_DURATION));
    assert(application_action.local_failure_reason == (scenario == 1U ? DEBUG_UI_REASON_STALE_FEEDBACK : DEBUG_UI_REASON_TORQUE_LIMIT));
    assert(application_action.failed_stage == PROTOCOL_COMMAND_STAGE_DISABLE);
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT && application_action.frames.count == 0U);
    snapshot.joints[6].timestamp_us = now_us + 1ULL;
    snapshot.joints[6].driver_state = S3519_DRIVER_STATE_DISABLED;
    snapshot.joints[6].torque_nm = 0.0F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        arm_config_get_production()->joints[6].master_id, 7U, &snapshot.joints[6]) == MOTOR_BANK_STATUS_OK);
    assert(aethor_app_advance_one_shot_setup(&snapshot, now_us + 2ULL));
    assert(protocol_engine_peek_command_result(&application_protocol_engine, &terminal));
    assert(terminal.code == PROTOCOL_COMMAND_RESULT_FAILED && terminal.disabled_confirmed);
    assert(!application_torque_envelope.authorized);
}

/** @brief Real direct failure/cancellation cannot keep credit even while a fresh disabled sample exists. */
static void torque_direct_failure(ProtocolCommandResultCode code)
{
    ProtocolCommandResult terminal;
    torque_fixture(1U);
    torque_envelope_reset_cold(&application_torque_envelope);
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.motor_mask = 0x40U;
    application_action.command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    application_action.command.type = PROTOCOL_COMMAND_MIT_ACTION;
    application_action.command.request_id = 124U;
    assert(aethor_app_complete_action(code, 0U, 1000001U));
    assert(protocol_engine_peek_command_result(&application_protocol_engine, &terminal));
    assert(terminal.code == code && terminal.disabled_confirmed);
    assert(!application_torque_envelope.authorized);
}

/** @brief Verify reset binding, exact scope, first failure and boot-time fail-closed behavior. */
int main(void)
{
    MotorJointFeedback feedback;
    ProtocolCommandResult completion;
    ProtocolOutputBatch output;
    uint32_t token;
    unsigned scenario;
    adjustable_wire_contract();
    adjustable_identity_binding();
    adjustable_hello_link_binding();
    adjustable_blockers();
    adjustable_reply_capacity();
    for (scenario = 0U; scenario < 4U; ++scenario) torque_disable_failure(scenario);
    torque_direct_failure(PROTOCOL_COMMAND_RESULT_FAILED);
    torque_direct_failure(PROTOCOL_COMMAND_RESULT_CANCELLED);
    torque_fixture(1U);
    assert(!application_torque_envelope.authorized);
    token = torque_query(1000001U, 1U);
    torque_reset_request(token, 1000002U, 1U);
    assert(application_torque_envelope.authorized);
    torque_reset_request(token, 1000003U, 0U);
    assert(application_torque_envelope.authorized);
    token = torque_query(1000004U, 1U);
    ++application_debug_ui.reference_generation[6];
    torque_reset_request(token, 1000005U, 0U);
    token = torque_query(1000006U, 1U);
    application_remote_ui.active = 1U;
    assert(request("1002 ui torque_reset token=1 cold=1", 1000007U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "code=rejected"));
    application_remote_ui.active = 0U;
    (void)torque_query(6000001U, 0U);
    torque_fixture(1U);
    application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 36.0F;
    (void)torque_query(1000001U, 0U);
    torque_fixture(1U);
    application_debug_ui.unconfirmed_disable_mask = 0x40U;
    (void)torque_query(1000001U, 0U);
    torque_fixture(1U);
    application_debug_ui.mailbox.transactions[0].used = 1U;
    application_debug_ui.mailbox.transactions[0].request.operation = DEBUG_UI_OPERATION_ACQUIRE;
    (void)torque_query(1000001U, 0U);
    torque_fixture(1U);
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = 1000001U; feedback.torque_nm = 4.0F;
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.motor_mask = 0x40U;
    application_action.command.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_TORQUE_LIMIT);
    torque_envelope_reset_cold(&application_torque_envelope);
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_NONE);
    application_action.command.local_operation = DEBUG_UI_OPERATION_HOME;
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_TORQUE_LIMIT);
    application_action.command.local_operation = DEBUG_UI_OPERATION_POS_MOVE;
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_TORQUE_LIMIT);
    application_action.command.local_operation = DEBUG_UI_OPERATION_STATIC_HOLD;
    application_action.command.motor_mask = 0x7FU;
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_TORQUE_LIMIT);
    application_action.command.local_operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    application_action.command.motor_mask = 1U;
    assert(aethor_app_torque_envelope_guard(&feedback, 0U, 1000001U) == DEBUG_UI_REASON_TORQUE_LIMIT);
    application_action.command.motor_mask = 0x40U;
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, 1000001U) == DEBUG_UI_REASON_NONE);
    memset(&completion, 0, sizeof(completion)); completion.disabled_confirmed = 1U;
    completion.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    aethor_app_torque_envelope_finish(&completion);
    assert(application_torque_envelope.authorized && !application_torque_envelope.tracking);
    completion.code = PROTOCOL_COMMAND_RESULT_FAILED;
    aethor_app_torque_envelope_finish(&completion);
    assert(!application_torque_envelope.authorized);
    torque_envelope_reset_cold(&application_torque_envelope);
    completion.code = PROTOCOL_COMMAND_RESULT_CANCELLED;
    aethor_app_torque_envelope_finish(&completion);
    assert(!application_torque_envelope.authorized);
    aethor_app_torque_envelope_fault();
    assert(!application_torque_envelope.authorized);
    aethor_app_init(2000000U, 4321U);
    assert(!application_torque_envelope.authorized);
    puts("TORQUE_ADAPTER_PASS one_use_token_reference_busy_stale_temperature_disabled_scope_boot_no_CAN");
    puts("ADJUSTABLE_ADAPTER_PASS strict_mnm_atomic_ledger_session_nonce_link_blockers_reply_capacity_no_CAN");
    return 0;
}
