/** @file adrc_lcd_integration_test_main.c
 * @brief Offline facade contract for one LCD-MIT/ADRC image; synthetic inputs are never hardware evidence.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../App/aethor_app.c"

/** @brief Host generated-controller stub refuses actuation and never stands in for ERT output. */
int adrc_generated_controller_step(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output)
{
    (void)context;
    (void)input;
    (void)output;
    return 0;
}

/** @brief Provides verified MIT parameters without inventing a feedback sample. */
static void fixture_discovered_motor7(void)
{
    MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    const JointConfig *joint = &arm_config_get_production()->joints[6];
    memset(discovery, 0, sizeof(*discovery));
    discovery->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    discovery->observed_esc_id = joint->esc_id;
    discovery->observed_master_id = joint->master_id;
    discovery->observed_control_mode = 1U;
    discovery->ranges.position_max_rad = 12.5F;
    discovery->ranges.velocity_max_rad_s = 200.0F;
    discovery->ranges.torque_max_nm = 18.0F;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    application_motor_runtime.discovery.verified_joint_mask = 0x40U;
    application_motor_runtime.discovery_active = 0U;
}

/** @brief Adds deterministic fresh disabled motor 7 feedback after verified discovery. */
static void fixture_disabled_motor7(uint64_t timestamp_us)
{
    MotorJointFeedback feedback;
    const JointConfig *joint = &arm_config_get_production()->joints[6];
    fixture_discovered_motor7();
    memset(&feedback, 0, sizeof(feedback));
    feedback.timestamp_us = timestamp_us;
    feedback.driver_state = S3519_DRIVER_STATE_DISABLED;
    feedback.mos_temperature_c = 25.0F;
    feedback.rotor_temperature_c = 25.0F;
    assert(motor_bank_update_feedback(&application_motor_runtime.bank,
        joint->master_id, joint->esc_id, &feedback) == MOTOR_BANK_STATUS_OK);
}

/** @brief Sends one USB line through the real application facade. */
static ProtocolEngineStatus request(const char *line, uint64_t timestamp_us,
    ProtocolOutputBatch *output)
{
    return aethor_app_process_protocol_line(line, strlen(line), timestamp_us, output);
}

/** @brief Exposes each pre-handoff gate without changing owner or requesting a CAN frame. */
static void test_read_only_handoff_gate_diagnostics(void)
{
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc gate motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "motor=7") != NULL);
    assert(strstr(output.messages[0].data, "lcd_idle=1 can_idle=1 mit_ready=1") != NULL);
    assert(strstr(output.messages[0].data, "mode=1") != NULL);
    assert(strstr(output.messages[0].data, "active_samples=0 active_intervals=0") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(application_integrated_probe_submitted_us == 0ULL);

    application_motor_runtime.feedback_timing[6].sample_count = 3U;
    application_motor_runtime.feedback_timing[6].interval_count = 2U;
    application_motor_runtime.feedback_timing[6].minimum_interval_us = 3900U;
    application_motor_runtime.feedback_timing[6].maximum_interval_us = 4200U;
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    assert(request("2 adrc gate motor=7", 6003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "mit_ready=0 mode=2") != NULL);
    assert(strstr(output.messages[0].data,
        "active_samples=3 active_intervals=2 active_min_us=3900 active_max_us=4200") != NULL);
    assert(request("3 adrc gate motor=1", 6004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Requires a separate discovery command to emit parameter reads without rewriting mode. */
static void test_adrc_discover_does_not_switch_motor_mode(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc discover motor=1", 6002U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("2 adrc discover motor=7", 6003U, &output) ==
        PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "discover=accepted") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_motor_runtime.discovery_active == 1U);
    assert(aethor_app_debug_ui_executor_busy());
    assert(aethor_app_next_can_frame(6004U, &frame, &priority) ==
        MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
    assert(frame.data[2] == 0x33U);
    assert(priority == CAN_TX_PRIORITY_PARAMETER);
    assert(request("3 adrc discover motor=7", 6005U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("4 bench init 7", 6006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6007U);
    assert(application_action.state == AETHOR_APP_ACTION_IDLE);
    assert(application_motor_runtime.discovery_active == 1U);
}

/** @brief Refuses diagnostic discovery while ownership or a control source is busy. */
static void test_adrc_discover_requires_idle_sources(void)
{
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    assert(request("1 adrc discover motor=7", 2000U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    aethor_app_integrated_set_can_idle(1U, 2001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    assert(request("2 adrc discover motor=7", 6002U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_REMOTE;
    application_action.state = AETHOR_APP_ACTION_MOTION;
    assert(request("3 adrc discover motor=7", 6003U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    application_action.state = AETHOR_APP_ACTION_IDLE;
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
    assert(request("4 adrc discover motor=7", 6004U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(application_motor_runtime.discovery_active == 0U);
}

/** @brief Sends one MIT feedback query for diagnosis without changing LCD ownership. */
static void test_diagnostic_probe_requires_mit_and_preserves_lcd_owner(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    assert(request("1 adrc probe motor=7", 6002U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 0U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    assert(request("2 adrc probe motor=1", 6004U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    fixture_disabled_motor7(6000U);
    assert(request("3 adrc probe motor=7", 6005U, &output) ==
        PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "probe=accepted") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6006U) == 1U);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER && frame.length == 4U &&
        frame.data[0] == 7U && frame.data[2] == 0xCCU);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6007U) == 0U);
    aethor_app_integrated_report_probe_transmit(1U, 6008U);
    assert(request("4 adrc probe", 6009U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=transmitted") != NULL);
    assert(strstr(output.messages[0].data, "sample_after_tx=0") != NULL);
    assert(request("41 adrc acquire motor=7", 6010U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    fixture_disabled_motor7(7000U);
    assert(request("5 adrc probe", 7001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=feedback") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
}

/** @brief A failed CAN receipt or competing legacy submit invalidates diagnostic evidence. */
static void test_diagnostic_probe_rejects_failed_or_competing_transmission(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc probe motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(0U, 6004U);
    fixture_disabled_motor7(7000U);
    assert(request("2 adrc probe", 7001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=failed") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 7002U) == 0U);

    assert(request("3 adrc probe motor=7", 106003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    aethor_app_integrated_note_legacy_can_activity();
    assert(request("4 adrc probe", 106004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=failed") != NULL);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 106005U) == 0U);
}

/** @brief Missing post-query feedback stays diagnostic and cannot gain ADRC authority. */
static void test_diagnostic_probe_times_out_without_feedback(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc probe motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    assert(request("2 adrc probe", 106005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=timeout") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 106006U) == 0U);
    fixture_disabled_motor7(106010U);
    assert(request("3 adrc probe", 106011U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=timeout") != NULL);
    assert(strstr(output.messages[0].data, "sample_after_tx=1") != NULL);
}

/** @brief A padded query records accepted and rejected raw replies without ADRC ownership. */
static void test_diagnostic_probe_padded_query_and_rx_evidence(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    const JointConfig *joint = &arm_config_get_production()->joints[6];
    const uint8_t malformed_payload[4] = {0x17U, 0U, 0U, 0U};
    const uint8_t feedback_payload[8] = {0x17U, 0U, 0U, 0U, 0U, 0U, 25U, 25U};
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc probe motor=7 len=5", 6002U, &output) ==
        PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("2 adrc probe motor=7 len=8", 6003U, &output) ==
        PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6004U) == 1U);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER && frame.length == 8U);
    assert(frame.data[0] == 7U && frame.data[2] == 0xCCU && frame.data[7] == 0U);
    aethor_app_integrated_report_probe_transmit(1U, 6005U);
    assert(can_frame_init(&frame, joint->master_id, malformed_payload,
        sizeof(malformed_payload)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, 6006U) == MOTOR_RUNTIME_STATUS_CODEC_ERROR);
    assert(can_frame_init(&frame, joint->master_id, feedback_payload,
        sizeof(feedback_payload)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, 6007U) == MOTOR_RUNTIME_STATUS_OK);
    assert(request("3 adrc probe", 6008U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=feedback") != NULL);
    assert(strstr(output.messages[0].data, "query_len=8") != NULL);
    assert(strstr(output.messages[0].data, "rx_after_tx=2 rx_valid=1 rx_rejected=1") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
}

/** @brief A drained CAN FIFO cannot override an unpublished LCD control group. */
static void test_pending_lcd_control_rejects_acquire(void)
{
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 1234U);
    fixture_disabled_motor7(2000U);
    application_action.control_group_ready = 1U;
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("2 adrc status", 6004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=unsafe") != NULL);
}

/** @brief A new LCD CAN submission restarts the measured quiet interval. */
static void test_recent_lcd_can_rejects_acquire(void)
{
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 1234U);
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    aethor_app_integrated_note_legacy_can_activity();
    fixture_disabled_motor7(7000U);
    aethor_app_integrated_set_can_idle(1U, 7001U);
    assert(request("1 adrc acquire motor=7", 7002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(7003U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
}

/** @brief Defaults to LCD-MIT and exposes ADRC status without reserving motor 7. */
static void test_lcd_default_and_safe_acquire(void)
{
    ProtocolOutputBatch output;
    DebugUiSnapshot snapshot;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 1234U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_debug_ui_get_snapshot(1000U, &snapshot));
    assert(snapshot.motion_enabled == 1U && snapshot.mit_enabled == 1U);
    assert(request("1 hello", 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request("2 adrc status", 1002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "owner=lcd") != NULL);
    assert(request("31 adrc acquire motor=1", 1002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("3 adrc run", 1003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);

    assert(request("4 adrc acquire motor=7", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    aethor_app_integrated_set_can_idle(1U, 2001U);
    (void)aethor_app_service(4000U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("40 adrc status", 4001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=unsafe") != NULL);
    assert(strstr(output.messages[0].data, "last_id=4") != NULL);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);

    fixture_disabled_motor7(8000U);
    assert(request("5 adrc acquire motor=7", 8001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    aethor_app_integrated_set_can_idle(1U, 8002U);
    (void)aethor_app_service(8002U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 8002U) == 1U);
    assert(frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER && frame.length == 4U &&
        frame.data[0] == 7U && frame.data[2] == 0xCCU);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 8002U) == 0U);
    aethor_app_integrated_report_probe_transmit(1U, 8003U);
    (void)aethor_app_service(8004U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    fixture_disabled_motor7(9000U);
    (void)aethor_app_service(9001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC);
    assert(application_motor_runtime.discovery.target_joint_mask == 0x40U);
    assert(request("6 adrc status", 8003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "owner=adrc") != NULL);
    assert(strstr(output.messages[0].data, "handoff=transferred") != NULL);
    assert(request("61 adrc prepare motor=1", 8003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(application_adrc_bridge.bench.draft_config.axis_index == 6U);
    assert(request("7 bench move 7 1", 8004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("5 adrc acquire motor=7", 8005U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Completed register discovery without a post-probe sample cannot grant ADRC ownership. */
static void test_no_feedback_after_probe_times_out(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    (void)aethor_app_service(100004U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    (void)aethor_app_service(106004U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("2 adrc status", 106005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=timeout") != NULL);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
}

/** @brief A dedicated-buffer rejection cannot be promoted into disabled feedback evidence. */
static void test_failed_probe_keeps_lcd_owner(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(0U, 6004U);
    fixture_disabled_motor7(7000U);
    (void)aethor_app_service(7001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 7002U) == 0U);
    assert(request("2 adrc status", 7003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=unsafe") != NULL);
}

/** @brief Keeps LCD locked until actual disable transmit and newer disabled feedback. */
static void test_release_requires_new_feedback(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    /* An already-disabled supervisor still needs a new actual CAN disable receipt. */
    application_adrc_bridge.bench.experiment.status.disabled_confirmed = 1U;
    assert(request("8 adrc release", 12000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(12001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_adrc_report_transmit_at(kind, decoded, 0U, 12500U);
    (void)aethor_app_service(12600U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 13000U);
    (void)aethor_app_service(13001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
    fixture_disabled_motor7(14000U);
    (void)aethor_app_service(14001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("41 adrc status", 14002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=released") != NULL);
    assert(strstr(output.messages[0].data, "last_id=8") != NULL);
}

/** @brief LCD motion is rejected while its STOP still reaches the selected-axis owner. */
static void test_lcd_stop_during_adrc(void)
{
    ProtocolOutputBatch output;
    DebugUiRequest ui_request;
    DebugUiSnapshot snapshot;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 1234U);
    fixture_disabled_motor7(6000U);
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    fixture_disabled_motor7(7000U);
    (void)aethor_app_service(7001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC);
    assert(aethor_app_debug_ui_get_snapshot(6004U, &snapshot));
    assert(snapshot.motion_enabled == 0U && snapshot.mit_enabled == 0U);
    memset(&ui_request, 0, sizeof(ui_request));
    ui_request.operation = DEBUG_UI_OPERATION_MIT_MOVE;
    assert(aethor_app_debug_ui_submit(&ui_request) == DEBUG_UI_REASON_DISABLED);
    ui_request.operation = DEBUG_UI_OPERATION_STOP;
    assert(aethor_app_debug_ui_submit(&ui_request) == DEBUG_UI_REASON_STOP_LATCHED);
    (void)aethor_app_service(6005U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 6006U);
    assert(request("2 stop", 6007U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6008U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
}

/** @brief Runs one deterministic integrated handoff without USB or CAN hardware. */
int main(void)
{
    test_read_only_handoff_gate_diagnostics();
    test_adrc_discover_does_not_switch_motor_mode();
    test_adrc_discover_requires_idle_sources();
    test_diagnostic_probe_requires_mit_and_preserves_lcd_owner();
    test_diagnostic_probe_rejects_failed_or_competing_transmission();
    test_diagnostic_probe_times_out_without_feedback();
    test_diagnostic_probe_padded_query_and_rx_evidence();
    test_pending_lcd_control_rejects_acquire();
    test_recent_lcd_can_rejects_acquire();
    test_lcd_default_and_safe_acquire();
    test_release_requires_new_feedback();
    test_no_feedback_after_probe_times_out();
    test_failed_probe_keeps_lcd_owner();
    test_lcd_stop_during_adrc();
    puts("ADRC_LCD_INTEGRATION_TESTS_PASSED");
    return 0;
}
