/** @file Real proxy/executor regression with synthetic disabled feedback; no serial, probe or hardware calls. */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main
static uint8_t test_transport_idle;
static uint32_t test_usb_link;
/** @brief Trusted platform stand-in; false represents a still-active hardware slot even when App flags are clear. */
static uint8_t idle_probe(void) { return test_transport_idle; }
/** @brief Preserve a USB configured bit and independent generation for reconnect tests. */
static uint32_t link_probe(void) { return test_usb_link; }
/** @brief Freeze one ordinary request using the current capability token. */
static void submit_release(uint64_t now, ProtocolOutputBatch *output)
{
    char wire[160];
    snprintf(wire, sizeof(wire), "40 ui submit token=%lu op=release delta=0 speed=0.02 kp=120 kd=0.3 torque=0 duration=1000",
        (unsigned long)application_remote_ui.token);
    assert(request(wire, now, output) == PROTOCOL_ENGINE_STATUS_OK);
}
/** @brief Retain a synthetic admitted POS failure through the actual result-consumption path. */
static void failed_remote_fixture_with_torque(unsigned scenario)
{
    DebugUiHealth health;
    MotorJointFeedback feedback;
    DebugUiMailboxTransaction *transaction;
    ProtocolOutputBatch output;
    aethor_app_init(1U, 1234U);
    test_transport_idle = 1U; test_usb_link = 0x80000001UL;
    aethor_app_bind_remote_release_probes(idle_probe, link_probe);
    fixture_disabled_motor7(1000000U);
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    memset(&health, 0, sizeof(health)); health.ui_timestamp_us = 1000000U;
    health.ui_sequence = 1U; health.ui_valid = health.input_valid = health.display_valid = 1U;
    aethor_app_debug_ui_update_health(&health);
    assert(request("1 hello ui=0123456789abcdef0123456789abcdef", 1000000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_protocol_engine.session_id && application_protocol_engine.next_session_nonce);
    assert(!application_protocol_engine.session_active); /* UI identity must not arm the legacy USB watchdog. */
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_ARMED);
    application_debug_ui.target_motor_id = 7U;
    application_remote_ui.owner_epoch = application_debug_ui.epoch;
    application_remote_ui.external_id = application_remote_ui.last_external_id = 10U;
    application_remote_ui.internal_id = 0x80000002UL;
    application_remote_ui.owns = application_remote_ui.active = 1U;
    application_remote_ui.request_session = application_protocol_engine.session_id;
    application_remote_ui.request_nonce = application_protocol_engine.next_session_nonce;
    application_remote_ui.request_link = test_usb_link;
    memset(&feedback, 0, sizeof(feedback)); feedback.timestamp_us = 1000000U;
    feedback.torque_nm = scenario == 4U ? -6.241F : scenario == 1U ? 5.47F : scenario == 2U ? 4.0F : scenario == 3U ? 3.6F : 3.5238094F;
    application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    application_action.command.session_id = application_debug_ui.epoch;
    application_action.command.request_id = application_remote_ui.internal_id;
    application_action.command.motor_mask = 0x40U;
    application_action.command.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    if (scenario) torque_envelope_reset_cold(&application_torque_envelope);
    if (scenario == 4U) assert(torque_envelope_reset_cold_selected(&application_torque_envelope, 6240U));
    if (scenario == 2U) {
        application_torque_envelope.exposure_nm2_s = TORQUE_ENVELOPE_CAPACITY_NM2_S - 0.001;
        feedback.timestamp_us -= 4000U;
    }
    if (scenario == 3U) {
        application_torque_envelope.tracking = application_torque_envelope.excessive = 1U;
        application_torque_envelope.last_tick_us = application_torque_envelope.last_feedback_us = 996000U;
        application_torque_envelope.excess_started_us = 1U;
        feedback.timestamp_us = 11000000U;
        application_torque_envelope.last_tick_us = application_torque_envelope.last_feedback_us = 10996000U;
    }
    assert(aethor_app_torque_envelope_guard(&feedback, 6U, feedback.timestamp_us + (scenario == 2U ? 4000U : 0U)) == DEBUG_UI_REASON_TORQUE_LIMIT);
    /* A later policy choice and cleanup must never rewrite the original effective exit line. */
    assert(torque_envelope_reset_cold_selected(&application_torque_envelope, 3500U));
    (void)torque_envelope_revoke(&application_torque_envelope, TORQUE_ENVELOPE_FEEDBACK);
    torque_envelope_end(&application_torque_envelope, 1U);
    memset(&application_action, 0, sizeof(application_action)); /* Mimic completed cleanup; first-fault evidence must survive. */
    transaction = &application_debug_ui.mailbox.transactions[0];
    memset(transaction, 0, sizeof(*transaction));
    transaction->used = transaction->taken = transaction->admission_ready = transaction->completion_ready = 1U;
    transaction->request.remote_proxy = 1U;
    transaction->request.target_motor_id = 7U;
    transaction->request.operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    transaction->request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    transaction->request.identity.epoch = application_debug_ui.epoch;
    transaction->request.identity.request_id = application_remote_ui.internal_id;
    transaction->admission.accepted = 1U;
    transaction->completion.code = DEBUG_UI_FAILED;
    transaction->completion.reason = DEBUG_UI_REASON_TORQUE_LIMIT;
    transaction->completion.disabled_confirmed = 1U;
    transaction->completion.motor_mask = 0x40U;
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    request("11 ui result id=10", 1000001U, &output);
    assert(strstr(output.messages[0].data, "disabled=1"));
    assert(strstr(output.messages[0].data, scenario == 1U || scenario == 4U ? "tq_reason=2 tq_auth=1" :
        scenario == 2U ? "tq_reason=4 tq_auth=1" : scenario == 3U ? "tq_reason=3 tq_auth=1" : "tq_reason=1 tq_auth=0"));
    assert(strstr(output.messages[0].data, scenario == 2U ? "tq_used=175.615000" :
        scenario ? "tq_used=0.000000" : "tq_used=175.616000"));
    assert(strstr(output.messages[0].data, scenario == 4U ? "tq_limit=6.240\r\n" :
        scenario ? "tq_limit=5.460\r\n" : "tq_limit=3.500\r\n"));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    request("12 ui caps", 1000002U, &output);
    assert(aethor_remote_fault_release_ready());
    assert(strstr(output.messages[0].data, "authority=3 remote_fault_release=1"));
}
/** @brief Keep baseline identity/transport assertions independent of the selected torque branch. */
static void failed_remote_fixture(void) { failed_remote_fixture_with_torque(0U); }

/** @brief Challenge-bound UI handshakes rotate identity without enabling watchdog, queuing CAN or changing LCD authority. */
static void handshake_identity(void)
{
    ProtocolOutputBatch output;
    uint32_t nonce, epoch;
    aethor_app_init(1U, 1234U); epoch = application_debug_ui.epoch;
    assert(request("1 hello ui=0123456789abcdef0123456789abcdef", 1000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    nonce = application_protocol_engine.next_session_nonce;
    assert(nonce && application_protocol_engine.session_id && !application_protocol_engine.session_active);
    assert(strstr(output.messages[0].data, "ui=0123456789abcdef0123456789abcdef"));
    assert(request("1 hello ui=0123456789abcdef0123456789abcdef", 1001U, &output) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(application_protocol_engine.next_session_nonce == nonce);
    assert(request("1 hello ui=fedcba9876543210fedcba9876543210", 1002U, &output) == PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT);
    assert(application_protocol_engine.next_session_nonce == nonce);
    assert(request("2 hello ui=fedcba9876543210fedcba9876543210", 1003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_protocol_engine.next_session_nonce != nonce && !application_protocol_engine.session_active);
    nonce = application_protocol_engine.next_session_nonce;
    assert(request("3 hello ui=bad", 1004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(application_protocol_engine.next_session_nonce == nonce && application_debug_ui.epoch == epoch);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && !application_action.frames.count);
}
/** @brief No action or quota restoration is produced while the existing executor releases a safe failed session. */
static void safe_release(void)
{
    ProtocolOutputBatch output;
    TorqueEnvelope original;
    failed_remote_fixture(); original = application_torque_envelope;
    submit_release(1000003U, &output);
    assert(strstr(output.messages[0].data, "queued"));
    aethor_app_debug_ui_process(1000004U);
    request("41 ui result id=40", 1000005U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_REMOTE && !application_remote_ui.owns);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && !application_action.frames.count);
    assert(!memcmp(&original, &application_torque_envelope, sizeof(original)));
}
/** @brief Every independent identity/transport/safety blocker must keep the capability closed. */
static void negative_gates(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 17U; ++scenario) {
        failed_remote_fixture();
        switch (scenario) {
        case 0: application_remote_ui.terminal_disabled = 0U; break;
        case 1: application_remote_ui.terminal_accepted = 0U; break;
        case 2: application_remote_ui.terminal_mask = 1U; break;
        case 3: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; break;
        case 4: ++application_debug_ui.epoch; break;
        case 5: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
        case 6: test_transport_idle = 0U; break;
        case 7: ++test_usb_link; break;
        case 8: ++application_protocol_engine.next_session_nonce; break;
        case 9: ++application_protocol_engine.session_id; break;
        case 10: application_debug_ui.mailbox.stop_intent_mask = 0x40U; break;
        case 11: application_emergency_disable_batch.count = 1U; break;
        case 12: application_debug_ui.fallback_frames.count = 1U; break;
        case 13: application_action.control_group_ready = 1U; break;
        case 14: application_transport_fault_write_sequence++; break;
        case 15: application_remote_ui.terminal_code = DEBUG_UI_COMPLETED; break;
        default: application_controller.fault = ARM_FAULT_TRANSPORT; break;
        }
        assert(!aethor_remote_fault_release_ready());
    }
}
/** @brief Evidence that changes after submit still rejects at actual executor admission, without sending a frame. */
static void late_debt_and_stop(void)
{
    ProtocolOutputBatch output;
    failed_remote_fixture(); submit_release(1000003U, &output);
    test_transport_idle = 0U;
    aethor_app_debug_ui_process(1000004U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE && !application_action.frames.count);
    failed_remote_fixture(); submit_release(1000003U, &output);
    application_controller.fault = ARM_FAULT_TRANSPORT;
    aethor_app_debug_ui_process(1000004U);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    failed_remote_fixture();
    aethor_app_remote_stop_intent();
    application_debug_ui.mailbox.stop_intent_mask = 0U; /* Synthetic completed STOP debt cannot restore the invalidated original proof. */
    assert(!aethor_remote_fault_release_ready());
    failed_remote_fixture();
    { uint32_t epoch = application_debug_ui.epoch;
      aethor_app_remote_service(7000000U);
      assert(application_debug_ui.epoch == epoch && !application_remote_ui.owns);
      assert(!aethor_remote_fault_release_ready()); /* The unchanged STOP intent is still mandatory. */
    }
}
/** @brief Execute only injected host assertions; physical qualification is intentionally absent. */
int main(void)
{
    handshake_identity();
    failed_remote_fixture_with_torque(1U); failed_remote_fixture_with_torque(2U); failed_remote_fixture_with_torque(3U);
    failed_remote_fixture_with_torque(4U);
    safe_release(); negative_gates(); late_debt_and_stop();
    puts("REMOTE_FAULT_RELEASE_PASS safe_release_no_can_no_quota_restore identity_transport_stop_and_late_debt_closed");
    return 0;
}
