/** @file Real facade remote routing regression; all feedback and completions are explicitly synthetic. */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Reset the proxy and firmware fixture without any hardware transport. */
static void remote_fixture(uint8_t fresh)
{
    DebugUiHealth health;
    memset(&application_remote_ui, 0, sizeof(application_remote_ui));
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U);
    if (fresh) fixture_disabled_motor7(1000000U); else fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    memset(&health, 0, sizeof(health)); health.ui_timestamp_us = 1000000U;
    health.ui_sequence = 1U; health.ui_valid = health.input_valid = health.display_valid = 1U;
    aethor_app_debug_ui_update_health(&health);
    aethor_app_debug_ui_process(1000000U);
}

/** @brief Issue a bounded schema request through the real USB dispatch. */
static void remote_submit(uint32_t id, const char *operation, uint64_t now_us, ProtocolOutputBatch *output)
{
    char line[160];
    (void)snprintf(line, sizeof(line), "%lu ui submit token=%lu op=%s delta=0.01 speed=0.02 kp=120 kd=0.3 torque=0 duration=1000",
        (unsigned long)id, (unsigned long)application_remote_ui.token, operation);
    assert(request(line, now_us, output) == PROTOCOL_ENGINE_STATUS_OK);
}

/** @brief Retain complete failed-stage diagnostics without clearing ownership or disable interlocks. */
static void remote_failure_diagnostics(void)
{
    ProtocolOutputBatch output;
    DebugUiMailboxTransaction *transaction;
    remote_fixture(1U);
    transaction = &application_debug_ui.mailbox.transactions[0];
    memset(transaction, 0, sizeof(*transaction));
    transaction->used = transaction->admission_ready = transaction->completion_ready = 1U;
    transaction->request.remote_proxy = 1U;
    transaction->request.operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    transaction->request.identity.request_id = 0x80000002U;
    transaction->admission.accepted = 1U;
    transaction->completion.code = DEBUG_UI_FAILED;
    transaction->completion.stage = DEBUG_UI_STAGE_VALIDATE;
    transaction->completion.reason = DEBUG_UI_REASON_NOT_DISABLED;
    transaction->completion.error = UINT8_MAX;
    transaction->completion.detail = UINT16_MAX;
    transaction->completion.failed_motor_id = 7U;
    transaction->completion.feedback_timestamp_us = UINT64_MAX;
    application_remote_ui.external_id = UINT32_MAX;
    application_remote_ui.internal_id = 0x80000002U;
    application_remote_ui.owns = application_remote_ui.active = 1U;
    application_debug_ui.unconfirmed_disable_mask = 0x40U;
    request("900 ui result id=4294967295", 1000001ULL, &output);
    assert(output.count == 1U);
    assert(strstr(output.messages[0].data, "result=failed accepted=1 reason=9 disabled=0"));
    assert(strstr(output.messages[0].data, "stage=1 error=255 detail=65535 motor=7 fb_us=18446744073709551615"));
    assert(strlen(application_remote_ui.terminal_detail) < sizeof(application_remote_ui.terminal_detail));
    assert(application_remote_ui.owns && !application_remote_ui.active);
    assert(application_debug_ui.unconfirmed_disable_mask == 0x40U);
    request("901 ui result id=4294967295", 1000002ULL, &output);
    assert(strstr(output.messages[0].data, "stage=1 error=255 detail=65535 motor=7 fb_us=18446744073709551615"));
    assert(application_remote_ui.owns && application_debug_ui.unconfirmed_disable_mask == 0x40U);
    puts("REMOTE_FAILED_DIAGNOSTICS_PASS retained_stage_error_detail_axis_timestamp locked_no_release");
}

/** @brief Test missing feedback, stale review, local ownership, maintenance locks and private identity boundaries. */
int main(void)
{
    ProtocolOutputBatch output; DebugUiRequest local; DebugUiAdmission admission; DebugUiCompletion completion;
    setbuf(stdout, NULL);
    remote_failure_diagnostics();
    remote_fixture(0U);
    assert(request("1 ui caps", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "fresh=0"));
    remote_submit(2U, "configure", 1000002U, &output);
    assert(strstr(output.messages[0].data, "reason=8"));
    assert(!application_remote_ui.active && !application_remote_ui.owns);
    remote_fixture(1U);
    assert(request("1 ui caps", 1000001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    remote_submit(2U, "acquire", 1000002U, &output);
    assert(strstr(output.messages[0].data, "reason=7"));
    remote_fixture(1U); request("1 ui caps", 1000001U, &output);
    remote_submit(2U, "acquire", 31000002ULL, &output);
    assert(strstr(output.messages[0].data, "reason=5"));
    remote_fixture(1U); request("1 ui caps", 1000001U, &output);
    application_persistence.locked = 1U;
    remote_submit(2U, "acquire", 1000002U, &output);
    assert(strstr(output.messages[0].data, "reason=2"));
    remote_fixture(1U); request("1 ui caps", 1000001U, &output);
    remote_submit(2U, "acquire", 1000002U, &output);
    assert(strstr(output.messages[0].data, "result=queued"));
    assert(application_remote_ui.owns && application_remote_ui.active);
    assert(application_debug_ui.mailbox.last_request_id == 0U);
    memset(&local, 0, sizeof(local)); local.target_motor_id = 7U; local.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    local.identity.epoch = application_debug_ui.epoch; local.identity.request_id = 1U;
    local.operation = DEBUG_UI_OPERATION_ACQUIRE;
    assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_BUSY);
    local.remote_proxy = 1U;
    assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    local.remote_proxy = 0U; local.operation = DEBUG_UI_OPERATION_STOP;
    assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_STOP_LATCHED);
    assert(!application_remote_ui.owns && !application_remote_ui.token);
    assert(!aethor_app_debug_ui_poll_admission(&admission));
    memset(&admission, 0, sizeof(admission)); memset(&completion, 0, sizeof(completion));
    admission.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    admission.identity.epoch = application_remote_ui.review.epoch;
    admission.identity.request_id = application_remote_ui.internal_id;
    completion.identity = admission.identity; completion.code = DEBUG_UI_FAILED;
    assert(debug_ui_mailbox_admit(&application_debug_ui.mailbox, &admission));
    assert(debug_ui_mailbox_complete(&application_debug_ui.mailbox, &completion));
    request("3 ui result id=2", 1000003U, &output);
    assert(strstr(output.messages[0].data, "result=failed"));
    request("40 ui caps", 1000004U, &output);
    assert(strstr(output.messages[0].data, "terminal_id=2"));
    request("4 ui result id=2", 1000004U, &output);
    assert(strstr(output.messages[0].data, "result=failed"));
    request("5 ui caps", 1000005U, &output);
    remote_submit(2U, "acquire", 1000006U, &output);
    assert(strstr(output.messages[0].data, "reason=13"));
    application_remote_ui.owns = 1U;
    request("6 bench stop 7", 1000007U, &output);
    assert(!application_remote_ui.owns && !application_remote_ui.token);
    aethor_app_init(1U, 2345U);
    assert(!application_remote_ui.active && !application_remote_ui.terminal_id && !application_remote_ui.owns);
    remote_fixture(1U);
    request("10 ui caps", 1000001U, &output);
    remote_submit(11U, "configure", 1000002U, &output);
    assert(strstr(output.messages[0].data, "result=queued"));
    aethor_app_debug_ui_process(1000003U);
    request("12 ui result id=11", 1000004U, &output);
    puts(output.messages[0].data);
    assert(strstr(output.messages[0].data, "result=completed"));
    assert(application_debug_ui.single_static_profiles[6].configured);
    assert(application_debug_ui.single_static_gains[6].kp == 120.0F);
    request("13 ui caps", 1000005U, &output);
    remote_submit(14U, "acquire", 1000006U, &output);
    aethor_app_debug_ui_process(1000007U);
    request("15 ui result id=14", 1000008U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    assert(application_remote_ui.owns && application_remote_ui.owner_epoch == application_debug_ui.epoch);
    request("16 ui caps", 1000009U, &output);
    remote_submit(17U, "hold", 1000010U, &output);
    aethor_app_debug_ui_process(1000011U);
    assert(application_protocol_engine.command_write_sequence != application_protocol_engine.command_read_sequence);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY);
    request("30 ui caps", 1000012U, &output);
    assert(strstr(output.messages[0].data, "active_id=17"));
    remote_fixture(1U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    request("20 ui caps", 1000001U, &output);
    remote_submit(21U, "acquire", 1000002U, &output);
    aethor_app_debug_ui_process(1000003U);
    request("22 ui result id=21", 1000004U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    request("23 ui caps", 1000005U, &output);
    remote_submit(24U, "pos", 1000006U, &output);
    aethor_app_debug_ui_process(1000007U);
    assert(application_protocol_engine.command_write_sequence != application_protocol_engine.command_read_sequence);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY);
    remote_fixture(1U);
    application_remote_ui.owns = 1U; application_remote_ui.lease_token = 77U;
    application_remote_ui.heartbeat_us = 1000000U;
    request("8 ui heartbeat token=77", 4000000U, &output);
    assert(strstr(output.messages[0].data, "result=alive"));
    assert(application_remote_ui.heartbeat_us == 4000000U);
    aethor_app_service(9000001U);
    assert(!application_remote_ui.owns && !application_remote_ui.lease_token);
    assert(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    assert(application_debug_ui.fallback_stop_mask & 0x40U);
    assert(application_debug_ui.fallback_frames.count > 0U);
    { CanFrame frame;
      assert(aethor_app_pop_emergency_can_frame(&frame));
      assert(frame.length == 8U && frame.data[7] == 0xFDU);
    }
    { unsigned scenario;
      for (scenario = 0U; scenario < 3U; ++scenario) {
        CanFrame frame;
        remote_fixture(1U); application_remote_ui.owns = 1U;
        application_remote_ui.heartbeat_us = 1000000U; application_remote_ui.lease_token = 88U;
        if (scenario == 0U) request("99 bench stop nonsense=1", 1000001U, &output);
        else if (scenario == 1U) request("99 bench disable 1", 1000001U, &output);
        else { local.operation = DEBUG_UI_OPERATION_STOP; local.identity.request_id = 0U;
          assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_INVALID_ARGUMENT); }
        assert(!application_remote_ui.owns && (application_debug_ui.mailbox.stop_intent_mask & 0x40U));
        aethor_app_service(1000002U);
        assert(aethor_app_pop_emergency_can_frame(&frame));
        assert(frame.length == 8U && frame.data[7] == 0xFDU);
      }
    }
    /* Dedicated loaded POS must enter its own executor, with zero MIT gains. */
    remote_fixture(1U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    request("100 ui caps", 1000001U, &output);
    assert(strstr(output.messages[0].data, "load=1"));
    remote_submit(101U, "acquire", 1000002U, &output);
    aethor_app_debug_ui_process(1000003U);
    request("102 ui result id=101", 1000004U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    request("103 ui caps", 1000005U, &output);
    { char line[160];
      snprintf(line, sizeof(line), "104 ui submit token=%lu op=pos_load delta=0.01 speed=0.02 kp=0 kd=0 torque=0 duration=1000",
        (unsigned long)application_remote_ui.token);
      request(line, 1000006U, &output);
      assert(strstr(output.messages[0].data, "result=queued"));
      aethor_app_debug_ui_process(1000007U);
      assert(application_protocol_engine.command_write_sequence != application_protocol_engine.command_read_sequence);
      { ProtocolCommand command;
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        assert(command.local_operation == DEBUG_UI_OPERATION_POS_LOAD_TEST);
        assert(command.type == PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED);
        assert(command.planned_duration_us == 1000000ULL);
        assert(command.mit_kp == 0.0F && command.mit_kd == 0.0F && command.mit_torque_ff_nm == 0.0F);
      }
    }
    /* Explicit static preflight may collect missing feedback; it must only enqueue DISABLE. */
    remote_fixture(0U);
    request("200 ui caps", 1000001U, &output);
    assert(strstr(output.messages[0].data, "preflight=1"));
    remote_submit(201U, "preflight", 1000002U, &output);
    assert(strstr(output.messages[0].data, "result=queued"));
    assert(!application_remote_ui.owns);
    aethor_app_debug_ui_process(1000003U);
    { ProtocolCommand command;
      assert(protocol_engine_pop_command(&application_protocol_engine, &command));
      assert(command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT);
      assert(command.type == PROTOCOL_COMMAND_DISABLE);
      assert(command.motor_mask == 0x40U);
    }
    { unsigned target_mode;
      for (target_mode = 1U; target_mode <= 2U; ++target_mode) {
        ProtocolCommand command;
        remote_fixture(1U);
        application_motor_runtime.discovery.results[6].observed_control_mode = target_mode == 1U ? 2U : 1U;
        request("300 ui caps", 1000001U, &output);
        assert(strstr(output.messages[0].data, "switch=1"));
        remote_submit(301U, "acquire", 1000002U, &output);
        aethor_app_debug_ui_process(1000003U);
        request("302 ui result id=301", 1000004U, &output);
        assert(strstr(output.messages[0].data, "result=completed"));
        request("303 ui caps", 1000005U, &output);
        remote_submit(304U, target_mode == 1U ? "mode_mit" : "mode_pos", 1000006U, &output);
        assert(strstr(output.messages[0].data, "result=queued"));
        aethor_app_debug_ui_process(1000007U);
        assert(protocol_engine_pop_command(&application_protocol_engine, &command));
        assert(command.local_operation == DEBUG_UI_OPERATION_SET_MODE);
        assert(command.type == PROTOCOL_COMMAND_SET_MODE);
        assert(command.control_mode == (target_mode == 1U ? ARM_CONTROL_MODE_MIT : ARM_CONTROL_MODE_POSITION_VELOCITY));
      }
    }
    /* Cold mode2 can enter original DISABLE preflight, never the post-preflight execution gate. */
    remote_fixture(0U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    request("400 ui caps", 1000001U, &output);
    assert(strstr(output.messages[0].data, "defer=1"));
    assert(strstr(output.messages[0].data, "fresh=0"));
    remote_submit(401U, "acquire", 1000002U, &output);
    aethor_app_debug_ui_process(1000003U);
    request("402 ui result id=401", 1000004U, &output);
    assert(strstr(output.messages[0].data, "result=completed"));
    request("403 ui caps", 1000005U, &output);
    remote_submit(404U, "pos", 1000006U, &output);
    aethor_app_debug_ui_process(1000007U);
    { ProtocolCommand command; MotorFeedbackSnapshot feedback;
      assert(protocol_engine_pop_command(&application_protocol_engine, &command));
      motor_runtime_get_snapshot(&application_motor_runtime, 1000008U, MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback);
      assert(!(feedback.valid_joint_mask & 0x40U));
      assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000008U, false) == DEBUG_UI_REASON_NONE);
      assert(aethor_app_debug_ui_validate_start(&command, &feedback, 1000008U, true) == DEBUG_UI_REASON_STALE_FEEDBACK);
    }
    remote_fixture(0U);
    application_motor_runtime.discovery.results[6].verified_fields_mask = 0U;
    request("410 ui caps", 1000001U, &output);
    assert(strstr(output.messages[0].data, "defer=0"));
    remote_submit(411U, "acquire", 1000002U, &output);
    assert(strstr(output.messages[0].data, "reason=8"));
    puts("REMOTE_UI_ADAPTER PASS: guards, STOP, POS load, mode switching, deferred intent never grants execution");
    return 0;
}
