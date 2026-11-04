/** @file adrc_lcd_integration_test_main.c
 * @brief Offline facade contract for one LCD-MIT/ADRC image; synthetic inputs are never hardware evidence.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../App/aethor_app.c"
#include "motor_adrc_command.h"

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
    assert(frame.identifier == 7U && frame.length == 8U &&
        frame.data[0] == 0xFFU && frame.data[6] == 0xFFU &&
        frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 8002U) == 0U);
    memset(&frame, 0, sizeof(frame));
    frame.identifier = arm_config_get_production()->joints[6].master_id;
    frame.length = 7U;
    frame.data[0] = (uint8_t)arm_config_get_production()->joints[6].esc_id;
    frame.data[1] = 0x80U; frame.data[2] = 0U;
    frame.data[3] = 0x80U; frame.data[4] = 0x08U;
    frame.data[6] = 25U; frame.data[7] = 25U;
    assert(aethor_app_receive_can_frame(&frame, 8003U) == MOTOR_RUNTIME_STATUS_CODEC_ERROR);
    frame.length = 8U;
    assert(aethor_app_receive_can_frame(&frame, 8004U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_integrated_acquire_rx_count == 2U);
    assert(application_integrated_acquire_valid_count == 1U);
    assert(application_integrated_acquire_rejected_count == 1U);
    assert(application_integrated_acquire_phases[0].raw_count == 2U);
    assert(application_integrated_acquire_phases[0].target_valid_count == 1U);
    assert(application_integrated_acquire_phases[1].raw_count == 0U);
    (void)aethor_app_service(8005U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    aethor_app_integrated_report_probe_transmit(1U, 8006U);
    (void)aethor_app_service(8007U);
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
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6005U) == 1U);
    assert(frame.identifier == 7U && frame.length == 8U);
    assert(frame.data[4] == 0U && frame.data[5] == 0U &&
        (frame.data[6] & 0xF0U) == 0U);
    assert(((((uint16_t)frame.data[6] & 0x0FU) << 8U) | frame.data[7]) >= 2047U);
    assert(((((uint16_t)frame.data[6] & 0x0FU) << 8U) | frame.data[7]) <= 2048U);
    aethor_app_integrated_report_probe_transmit(1U, 6006U);
    (void)aethor_app_service(100004U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    (void)aethor_app_service(106004U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("2 adrc status", 106005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=timeout") != NULL);
    assert(request("3 adrc gate motor=7", 106006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data,
        "acq_submit=1 acq_tx=1 acq_failed=0 acq_disable_tx=1 acq_neutral=1 acq_rx=0 acq_valid=0 acq_rejected=0") != NULL);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
}

/** @brief Requires a fresh disabled state reply after the neutral MIT fallback. */
static void test_neutral_probe_requires_actual_reply(void)
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
    assert(frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6005U) == 1U);
    assert(frame.identifier == 7U && frame.data[4] == 0U && frame.data[5] == 0U);
    (void)aethor_app_service(6006U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    aethor_app_integrated_report_probe_transmit(1U, 6007U);
    (void)aethor_app_service(6008U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
    fixture_disabled_motor7(6009U);
    (void)aethor_app_service(6010U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC);
}

/** @brief A bounded DISABLE retry must retain the first successful causal challenge. */
static void test_acquire_disable_retry_retains_first_challenge(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    unsigned retry_index;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6005U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6006U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 10004U) == 0U);
    for (retry_index = 0U; retry_index < 3U; ++retry_index)
    {
        uint64_t submitted_us = 10005ULL + retry_index * 4000ULL;
        assert(aethor_app_integrated_pop_probe_frame(&frame, submitted_us) == 1U);
        assert(frame.identifier == 7U && frame.length == 8U);
        assert(frame.data[0] == 0xFFU && frame.data[6] == 0xFFU && frame.data[7] == 0xFDU);
        aethor_app_integrated_report_probe_transmit(1U, submitted_us + 1ULL);
    }
    assert(aethor_app_integrated_pop_probe_frame(&frame, 22005U) == 0U);
    /* This lower bound is after the first transmitted DISABLE, before the last retry.
     * Moving the anchor to each retry would reject a causally valid fresh reply. */
    fixture_disabled_motor7(18004U);
    (void)aethor_app_service(22007U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC);
}

/** @brief Retries never bypass unknown/old evidence, failed TX, or the original deadline. */
static void test_acquire_disable_retry_preserves_guards(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 5U; ++scenario)
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
        aethor_app_integrated_report_probe_transmit(1U, 6004U);
        assert(aethor_app_integrated_pop_probe_frame(&frame, 6005U) == 1U);
        aethor_app_integrated_report_probe_transmit(1U, 6006U);
        if (scenario == 0U)
        {
            /* No service call at the deadline: pop itself must refuse late traffic. */
            assert(aethor_app_integrated_pop_probe_frame(&frame, 106003U) == 0U);
            fixture_disabled_motor7(106002U);
            (void)aethor_app_service(106003U);
            assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
            assert(application_integrated_handoff == ADRC_LCD_OWNERSHIP_TIMEOUT);
            continue;
        }
        assert(aethor_app_integrated_pop_probe_frame(&frame, 10005U) == 1U);
        assert(aethor_app_integrated_pop_probe_frame(&frame, 14005U) == 0U);
        aethor_app_integrated_report_probe_transmit(scenario == 1U ? 0U : 1U, 14006U);
        fixture_disabled_motor7(scenario == 2U ? 6003U : scenario == 3U ? 0U : 14007U);
        (void)aethor_app_service(scenario == 4U ? 22008U : 14008U);
        assert(application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_ADRC);
        if (scenario == 1U)
        {
            assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
            assert(application_integrated_handoff == ADRC_LCD_OWNERSHIP_UNSAFE);
        }
    }
}

/** @brief Acquisition counters retain DISABLE-window replies after neutral submission. */
static void test_acquire_diagnostics_keep_original_window_and_target(void)
{
    ProtocolOutputBatch output;
    CanFrame frame = {0};
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 6001U);
    assert(request("1 adrc acquire motor=7", 6002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(6003U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6003U) == 1U);
    aethor_app_integrated_report_probe_transmit(1U, 6004U);
    assert(aethor_app_integrated_pop_probe_frame(&frame, 6005U) == 1U);
    frame.identifier = arm_config_get_production()->joints[6].master_id;
    frame.length = 7U;
    (void)aethor_app_receive_can_frame(&frame, 6004U);
    assert(application_integrated_acquire_rx_count == 1U);
    assert(application_integrated_acquire_rejected_count == 1U);
    frame.identifier = arm_config_get_production()->joints[0].master_id;
    (void)aethor_app_receive_can_frame(&frame, 6006U);
    assert(application_integrated_acquire_rx_count == 2U);
    assert(application_integrated_acquire_rejected_count == 1U);
    frame.identifier = arm_config_get_production()->joints[6].master_id;
    (void)aethor_app_receive_can_frame(&frame, 0ULL);
    assert(application_integrated_acquire_rx_count == 3U);
    assert(application_integrated_acquire_other_count == 1U);
    assert(application_integrated_acquire_unknown_time_count == 1U);
    assert(application_integrated_acquire_first_submit_us == 6003ULL);
    assert(request("2 adrc acquisition motor=7", 6007U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "disable_us=6003 neutral_us=6005") != NULL);
    assert(strstr(output.messages[0].data, "other=1 unknown=1 old=0 unassigned=2") != NULL);
    assert(strstr(output.messages[0].data, "d_rx=0 d_valid=0 d_other=0 n_rx=1 n_valid=0 n_other=1") != NULL);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ACQUIRING);
}

/** @brief Maximum diagnostic integers fit the wire response without mutating control state. */
static void test_acquisition_diagnostics_maximum_fields_are_read_only(void)
{
    ProtocolOutputBatch output;
    AdrcLcdOwnership owner_before;
    uint8_t phase_index;
    aethor_app_init(1000U, 1234U);
    application_integrated_acquire_rx_count = UINT32_MAX;
    application_integrated_acquire_valid_count = UINT32_MAX;
    application_integrated_acquire_rejected_count = UINT32_MAX;
    application_integrated_acquire_other_count = UINT32_MAX;
    application_integrated_acquire_unknown_time_count = UINT32_MAX;
    application_integrated_acquire_old_count = UINT32_MAX;
    application_integrated_acquire_unassigned_count = UINT32_MAX;
    for (phase_index = 0U; phase_index < 2U; ++phase_index)
    {
        application_integrated_acquire_phases[phase_index].submitted_us = UINT64_MAX;
        application_integrated_acquire_phases[phase_index].raw_count = UINT32_MAX;
        application_integrated_acquire_phases[phase_index].target_valid_count = UINT32_MAX;
        application_integrated_acquire_phases[phase_index].other_count = UINT32_MAX;
    }
    owner_before = application_adrc_lcd_ownership;
    assert(request("4294967295 adrc acquisition motor=7", 1001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output.count == 1U);
    assert(output.messages[0].length == strlen(output.messages[0].data));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    assert(strstr(output.messages[0].data, "neutral_us=18446744073709551615") != NULL);
    assert(strstr(output.messages[0].data, "n_other=4294967295\r\n") != NULL);
    assert(memcmp(&owner_before, &application_adrc_lcd_ownership, sizeof(owner_before)) == 0);
    assert(application_integrated_probe_submitted_us == 0ULL);
    assert(application_adrc_bridge.frame_pending == 0U);
    assert(request("2 adrc acquisition motor=7 qualify=1", 1002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("3 adrc acquisition motor=1", 1003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Motor-owned diagnostic encoders retain exact wire bytes and reject invalid inputs. */
static void test_motor_diagnostic_wrappers_preserve_wire_contract(void)
{
    CanFrame actual;
    CanFrame expected;
    CanFrame sentinel;
    uint8_t length;
    uint8_t initialized;
    aethor_app_init(1000U, 1234U);
    fixture_discovered_motor7();
    memset(&actual, 0xA5, sizeof(actual));
    sentinel = actual;
    assert(motor_adrc_build_neutral_probe(NULL, 6U, &actual) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(motor_adrc_build_neutral_probe(&application_motor_runtime, ARM_JOINT_COUNT, &actual) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(motor_adrc_build_neutral_probe(&application_motor_runtime, 6U, NULL) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(memcmp(&actual, &sentinel, sizeof(actual)) == 0);
    initialized = application_motor_runtime.initialized;
    application_motor_runtime.initialized = 0U;
    assert(motor_adrc_build_neutral_probe(&application_motor_runtime, 6U, &actual) == MOTOR_RUNTIME_STATUS_NOT_INITIALIZED);
    assert(motor_adrc_build_feedback_query(&application_motor_runtime, 6U, 4U, &actual) == MOTOR_RUNTIME_STATUS_NOT_INITIALIZED);
    application_motor_runtime.initialized = initialized;
    application_motor_runtime.discovery.results[6].verified_fields_mask = 0U;
    assert(motor_adrc_build_neutral_probe(&application_motor_runtime, 6U, &actual) == MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE);
    assert(memcmp(&actual, &sentinel, sizeof(actual)) == 0);
    fixture_discovered_motor7();
    assert(s3519_pack_mit(7U, &application_motor_runtime.discovery.results[6].ranges,
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, &expected) == S3519_CODEC_STATUS_OK);
    assert(motor_adrc_build_neutral_probe(&application_motor_runtime, 6U, &actual) == MOTOR_RUNTIME_STATUS_OK);
    assert(actual.identifier == expected.identifier && actual.length == expected.length);
    assert(memcmp(actual.data, expected.data, expected.length) == 0);
    assert(motor_adrc_build_feedback_query(NULL, 6U, 4U, &actual) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(motor_adrc_build_feedback_query(&application_motor_runtime, ARM_JOINT_COUNT, 4U, &actual) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(motor_adrc_build_feedback_query(&application_motor_runtime, 6U, 4U, NULL) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    assert(motor_adrc_build_feedback_query(&application_motor_runtime, 6U, 7U, &actual) == MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT);
    for (length = 4U; length <= 8U; length += 4U)
    {
        assert(s3519_pack_feedback_query_with_length(7U, length, &expected) == S3519_CODEC_STATUS_OK);
        assert(motor_adrc_build_feedback_query(&application_motor_runtime, 6U, length, &actual) == MOTOR_RUNTIME_STATUS_OK);
        assert(actual.identifier == expected.identifier && actual.length == expected.length);
        assert(memcmp(actual.data, expected.data, expected.length) == 0);
    }
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
    aethor_app_adrc_report_disable_submit_at(kind, 12002U);
    aethor_app_adrc_report_transmit_at(kind, decoded, 0U, 12500U);
    (void)aethor_app_service(12600U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U);
    aethor_app_adrc_report_disable_submit_at(kind, 12601U);
    /* A prompt reply can have a conservative lower bound equal to submission. */
    fixture_disabled_motor7(12601U);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 13000U);
    (void)aethor_app_service(13001U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
    (void)aethor_app_service(16600U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
    (void)aethor_app_service(16601U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
    assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_adrc_report_disable_submit_at(kind, 16602U);
    assert(application_adrc_lcd_ownership.disable_submitted_us == 12601U);
    fixture_disabled_motor7(16603U);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 16700U);
    (void)aethor_app_service(16701U);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
    assert(request("41 adrc status", 17002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "handoff=released") != NULL);
    assert(strstr(output.messages[0].data, "last_id=8") != NULL);
}

/** @brief Release retries stop at the count/deadline limits and never fabricate confirmation. */
static void test_release_retry_limits_without_feedback(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    uint8_t scenario;
    uint8_t retry;
    uint64_t submitted_us;
    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        test_lcd_default_and_safe_acquire();
        assert(request("8 adrc release", 12000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        (void)aethor_app_service(12001U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
        assert(kind == 2U);
        submitted_us = 12002U;
        aethor_app_adrc_report_disable_submit_at(kind, submitted_us);
        aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 12003U);
        if (scenario == 0U)
        {
            for (retry = 0U; retry < 3U; ++retry)
            {
                (void)aethor_app_service(submitted_us + 4000U);
                assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
                assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
                submitted_us += 4001U;
                aethor_app_adrc_report_disable_submit_at(kind, submitted_us);
                aethor_app_adrc_report_transmit_at(kind, decoded, 1U, submitted_us + 1U);
            }
            (void)aethor_app_service(submitted_us + 4000U);
            assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
        }
        (void)aethor_app_service(512000U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
        (void)aethor_app_service(512001U);
        assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 0U);
        assert(request("41 adrc status", 512002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "handoff=timeout") != NULL);
    }
}

/** @brief Failed or missing supplementary receipts retain the existing safety DISABLE path. */
static void test_release_retry_failure_remains_fail_closed(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    uint8_t scenario;
    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        test_lcd_default_and_safe_acquire();
        assert(request("8 adrc release", 12000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        (void)aethor_app_service(12001U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
        aethor_app_adrc_report_disable_submit_at(kind, 12002U);
        aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 12003U);
        (void)aethor_app_service(511999U);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
        assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        aethor_app_adrc_report_disable_submit_at(kind, 511999U);
        if (scenario == 0U)
        { aethor_app_adrc_report_transmit_at(kind, decoded, 0U, 512000U); }
        (void)aethor_app_service(516001U);
        assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
        assert(application_integrated_release_disable_retries == 1U);
        /* The handoff deadline does not cancel an unconfirmed safety DISABLE. */
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) == 1U);
        assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        assert(application_adrc_lcd_ownership.disable_submitted_us == 12002U);
        assert(request("41 adrc status", 516002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "handoff=timeout") != NULL);
    }
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

/** @brief Diagnostics require ADRC ownership and cancellation keeps DISABLE on motor 7. */
static void test_integrated_disabled_capture(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 1234U);
    assert(request("100 adrc sample", 2000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required"));
    assert(request("101 adrc capture", 2001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    test_lcd_default_and_safe_acquire();
    fixture_disabled_motor7(12000U);
    assert(request("100 adrc sample", 12001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(16000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded));
    assert(kind == 2U && frame.identifier == 7U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 16001U);
    assert(request("101 stop", 16002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_adrc_bridge.capture.state == 3U);
    (void)aethor_app_service(20000U);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded));
    assert(kind == 2U && frame.identifier == 7U);
    assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
}

/** @brief A pending or failed independent stop cannot hand control back to legacy motion. */
static void test_guard_release_gate(void)
{
    uint8_t state;
    ProtocolOutputBatch output;
    aethor_app_init(1000U, 1234U);
    assert(request("100 adrc guard", 2000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required"));
    assert(request("101 adrc guard_status", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    for (state = 1U; state <= 6U; ++state)
    {
        test_lcd_default_and_safe_acquire();
        application_adrc_bridge.guard_test.state = state; /* Synthetic state injection only. */
        assert(request("100 adrc release", 12000U, &output) ==
            (state == 5U ? PROTOCOL_ENGINE_STATUS_OK : PROTOCOL_ENGINE_STATUS_BAD_REQUEST));
        if (state != 5U)
        { assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC); }
    }
}

/** @brief Watchdog parameter routing and owner locks cannot promote an incomplete trial. */
static void test_watchdog_owner_gate(void)
{
    uint8_t state;
    ProtocolOutputBatch output;
    CanFrame reply;
    aethor_app_init(1000U, 1234U);
    assert(request("100 adrc watchdog", 2000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required"));
    assert(request("101 adrc watchdog_status", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    /* Read-only fault snapshots remain available in every owner state and grant no authority. */
    for (state = ADRC_LCD_OWNER_LCD; state <= ADRC_LCD_OWNER_RELEASING; ++state)
    {
        application_adrc_lcd_ownership.state = (AdrcLcdOwnerState)state;
        assert(request("4294967295 adrc watchdog_fault", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "reason=0 phase=0 observed_us=0 rx_lower_us=0"));
        assert(application_adrc_lcd_ownership.state == (AdrcLcdOwnerState)state);
        assert(!application_adrc_bridge.frame_pending);
        assert(application_adrc_bridge.bench.trusted_qualification.verified_flags == 0U);
    }
    for (state = 1U; state <= 17U; ++state)
    {
        test_lcd_default_and_safe_acquire();
        application_adrc_bridge.guard_test.state = 5U;
        application_adrc_bridge.watchdog.state = state;
        assert(request("100 adrc release", 12000U, &output) ==
            ((state == 15U || state == 17U) ? PROTOCOL_ENGINE_STATUS_OK : PROTOCOL_ENGINE_STATUS_BAD_REQUEST));
    }
    test_lcd_default_and_safe_acquire();
    memset(&reply, 0, sizeof(reply)); reply.identifier = 23U; reply.length = 8U;
    reply.data[0] = 7U; reply.data[2] = 0x33U; reply.data[3] = 9U; reply.data[4] = 120U;
    application_adrc_bridge.watchdog.state = 3U;
    application_adrc_bridge.watchdog.first_request_us = 12000U;
    assert(aethor_app_receive_can_frame(&reply, 12000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(!application_adrc_bridge.watchdog.parameter_rx_us);
    assert(aethor_app_receive_can_frame(&reply, 16000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_adrc_bridge.watchdog.parameter_rx_us == 16000U);
    assert(application_adrc_bridge.watchdog.parameter_value == 120U);
    application_adrc_bridge.watchdog_quarantine_until_us = 100000U;
    adrc_app_bridge_complete_release(&application_adrc_bridge);
    assert(adrc_app_bridge_accept_watchdog_parameter(&application_adrc_bridge, &reply, 20000U));
    assert(!application_adrc_bridge.watchdog.parameter_rx_us);
    assert(!adrc_app_bridge_accept_watchdog_parameter(&application_adrc_bridge, &reply, 104000U));
}

/** @brief Simple commissioning cannot bypass ownership or release without both stop proofs. */
static void test_simple_owner_gate(void)
{
    uint8_t owner, acknowledged;
    ProtocolOutputBatch output;
    CanFrame frame;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 1234U);
    assert(request("100 adrc simple", 2000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required"));
    for (owner = ADRC_LCD_OWNER_LCD; owner <= ADRC_LCD_OWNER_RELEASING; ++owner)
    {
        application_adrc_lcd_ownership.state = (AdrcLcdOwnerState)owner;
        assert(request("101 adrc simple_status", 2000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "id=0 phase=0 result=0"));
        assert(!application_adrc_bridge.frame_pending);
        assert(application_adrc_lcd_ownership.state == (AdrcLcdOwnerState)owner);
    }
    for (acknowledged = 0U; acknowledged <= 1U; ++acknowledged)
    {
        test_lcd_default_and_safe_acquire();
        application_adrc_bridge.simple_request_id = 88U;
        application_adrc_bridge.simple.status.phase = ADRC_SIMPLE_DONE;
        application_adrc_bridge.simple.status.result = ADRC_SIMPLE_COMPLETED_NO_RESPONSE;
        application_adrc_bridge.simple.status.safe_release = 1U;
        application_adrc_bridge.simple_guard_ack = acknowledged;
        assert(request("100 adrc release", 12000U, &output) ==
            (acknowledged ? PROTOCOL_ENGINE_STATUS_OK : PROTOCOL_ENGINE_STATUS_BAD_REQUEST));
        if (acknowledged)
        {
            (void)aethor_app_service(16000U);
            assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 2U);
            assert(application_adrc_bridge.simple.status.result == ADRC_SIMPLE_COMPLETED_NO_RESPONSE);
            aethor_app_adrc_report_disable_submit_at(kind, 16001U);
            fixture_disabled_motor7(20000U);
            aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 20001U);
            (void)aethor_app_service(20002U);
            assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
            assert(application_adrc_bridge.simple_request_id == 0U);
        }
        else { assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_ADRC); }
    }
}

/** @brief LCD ownership cannot launch capabilities; STOP/release retain quarantine and zero qualification. */
static void test_capability_owner_gate(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanFrame reply;
    CanTxPriority priority;
    uint32_t feedback_count;
    uint8_t kind;
    float decoded;
    aethor_app_init(1000U, 123U);
    fixture_disabled_motor7(4000ULL);
    assert(request("501 adrc capabilities", 4001ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required"));
    assert(!application_adrc_bridge.capabilities.used);
    assert(request("502 adrc capabilities_status", 4001ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
    application_adrc_lcd_ownership.axis_index = 6U;
    application_adrc_bridge.bench.draft_config.axis_index = 6U;
    application_adrc_bridge.discovery_axis = 6U;
    assert(request("503 adrc capabilities", 4001ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_next_can_frame(4002ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
    (void)aethor_app_service(4002ULL);
    assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded) && kind == 2U && frame.identifier == 7U);
    aethor_app_adrc_report_disable_submit_at(kind, 4003ULL);
    aethor_app_adrc_report_transmit_at(kind, decoded, 1U, 4004ULL);
    fixture_disabled_motor7(8000ULL);
    (void)aethor_app_service(8001ULL);
    assert(application_adrc_bridge.capability_prepare_state == 2U);
    assert(aethor_app_next_can_frame(8002ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
    aethor_app_adrc_observe_capability_rx_boundary(12000ULL);
    assert(aethor_app_next_can_frame(12000ULL, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.data[2] == 0x33U && frame.data[3] == 1U);
    reply = frame; reply.identifier = 23U; reply.length = 8U;
    reply.data[4] = 0xcdU; reply.data[5] = 0xccU; reply.data[6] = 0xccU; reply.data[7] = 0x3dU;
    feedback_count = application_motor_runtime.accepted_feedback_count;
    assert(request("504 adrc simple", 12001ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("505 stop", 12001ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_adrc_bridge.capabilities.state == MOTOR_CAPABILITY_ABORTED);
    aethor_app_adrc_report_capability_transmit(&frame, 1U, 12002ULL);
    assert(aethor_app_receive_can_frame_observed(&reply, 12000ULL, 12003ULL) == MOTOR_RUNTIME_STATUS_OK);
    assert(!application_adrc_bridge.capabilities.count);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    adrc_app_bridge_complete_release(&application_adrc_bridge);
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_LCD;
    assert(aethor_app_receive_can_frame_observed(&reply, 12000ULL, 12001ULL) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    assert(request("506 adrc capabilities_status", 12001ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "current=0") && strstr(output.messages[0].data, "qualified=0"));
    assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
}

/** @brief Retained shaft diagnostics remain read-only after LCD handback; mutations still require ADRC ownership. */
static void test_retained_shaft_diagnostics_under_lcd_owner(void)
{
    static const char *queries[] = {"1 adrc shaft_status", "2 adrc shaft_binding index=0",
        "3 adrc shaft_trace index=0 part=0"};
    static const char *invalid[] = {"4 adrc shaft_status qualified=1", "5 adrc shaft_binding index=4",
        "6 adrc shaft_trace index=1 part=0"};
    ProtocolOutputBatch output;
    ShaftPilotStatus retained;
    unsigned index;
    aethor_app_init(1000U, 1234U);
    application_adrc_bridge.shaft.used = 1U;
    application_adrc_bridge.shaft.released = 1U;
    application_adrc_bridge.shaft.boot_id = 1234U;
    application_adrc_bridge.shaft.request_id = 71U;
    application_adrc_bridge.shaft.guard_ack = 1U;
    application_adrc_bridge.shaft.prepare = 2U;
    application_adrc_bridge.shaft.core->initialized = 1U;
    application_adrc_bridge.shaft.core->trace_frozen = 1U;
    application_adrc_bridge.shaft.core->status.phase = SHAFT_PHASE_DONE;
    application_adrc_bridge.shaft.core->status.outcome = SHAFT_END_TIME;
    application_adrc_bridge.shaft.core->status.safe_release = 1U;
    application_adrc_bridge.shaft.core->status.trace_count = 1U;
    retained = application_adrc_bridge.shaft.core->status;
    for (index = 0U; index < sizeof(queries)/sizeof(queries[0]); ++index)
    {
        assert(request(queries[index], 2000ULL + index, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "owner_required") == NULL);
        assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
        assert(!application_adrc_bridge.frame_pending && !application_adrc_bridge.stop_pending);
    }
    for (index = 0U; index < sizeof(invalid)/sizeof(invalid[0]); ++index)
    {
        assert(request(invalid[index], 3000ULL + index, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(strstr(output.messages[0].data, "owner_required") == NULL);
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
        assert(!application_adrc_bridge.frame_pending && !application_adrc_bridge.stop_pending);
    }
    assert(request("7 adrc shaft_pilot direction=positive profile=low", 4000ULL, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output.messages[0].data, "owner_required") != NULL);
    assert(!application_adrc_bridge.frame_pending && !application_adrc_bridge.stop_pending);
    assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
}
/** @brief Handback preserves the completed pilot while new DISABLE receipts/feedback still gate LCD ownership. */
static void test_shaft_terminal_survives_release_transaction(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 4U; ++scenario)
    {
        ProtocolOutputBatch output;
        CanFrame frame;
        ShaftPilotStatus retained;
        uint8_t kind;
        float decoded;
        test_lcd_default_and_safe_acquire();
        application_adrc_bridge.shaft.used = 1U;
        application_adrc_bridge.shaft.prepare = 2U;
        application_adrc_bridge.shaft.guard_ack = 1U;
        application_adrc_bridge.shaft.core->initialized = 1U;
        application_adrc_bridge.shaft.core->trace_frozen = 1U;
        application_adrc_bridge.shaft.core->status.phase = SHAFT_PHASE_DONE;
        application_adrc_bridge.shaft.core->status.outcome = scenario & 1U ? SHAFT_END_ABORT : SHAFT_END_TIME;
        application_adrc_bridge.shaft.core->status.safe_release = 1U;
        application_adrc_bridge.shaft.core->status.caller_next_transaction_ready = (uint8_t)(!(scenario & 1U));
        application_adrc_bridge.shaft.core->status.trace_count = 1U;
        retained = application_adrc_bridge.shaft.core->status;
        assert(request("100 adrc release", 12000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        (void)aethor_app_service(12001U);
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
        assert(application_adrc_bridge.shaft.bridge_fault == 0U);
        assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
        assert(aethor_app_adrc_pop_frame(&frame, &kind, &decoded));
        assert(kind == 2U && frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        aethor_app_adrc_report_disable_submit_at(kind, 12002U);
        aethor_app_adrc_report_transmit_at(kind, decoded, (uint8_t)(scenario < 2U), 12500U);
        (void)aethor_app_service(12600U);
        assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING);
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
        fixture_disabled_motor7(12601U);
        (void)aethor_app_service(12602U);
        if (scenario < 2U)
        { assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD); }
        else
        { assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_RELEASING); }
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
        assert(application_adrc_bridge.shaft.used);
        assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
        assert(!memcmp(&retained, &application_adrc_bridge.shaft.core->status, sizeof(retained)));
    }
}
/** @brief Establish a current LCD-owned disabled fixture for one raw coordinate pair. */
static void fixture_coordinate_pair(void)
{
    aethor_app_init(1000U, 1234U);
    fixture_disabled_motor7(100000U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    application_motor_runtime.bank.motors[6].feedback.position_rad = 0.25F;
    aethor_app_integrated_set_can_idle(1U, 1001U);
    aethor_app_integrated_set_can_idle(1U, 100000U);
}

/** @brief Supply an exact read response without modifying normal feedback validity. */
static MotorRuntimeStatus coordinate_reply(uint8_t register_id, float value, uint64_t now_us)
{
    CanFrame reply;
    uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
    payload[3] = register_id;
    memcpy(payload + 4U, &value, sizeof(value));
    assert(can_frame_init(&reply, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
    return aethor_app_receive_can_frame(&reply, now_us);
}

/** @brief Read exactly 0x50/0x51 once; no ENABLE, write, normal feedback or qualification side effect. */
static void test_coordinate_pair_readonly_contract(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint32_t feedback_count;
    fixture_coordinate_pair();
    feedback_count = application_motor_runtime.accepted_feedback_count;
    assert(request("400 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "coordinates accepted=1") != NULL);
    assert(aethor_app_next_can_frame(100002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[0] == 7U &&
        frame.data[1] == 0U && frame.data[2] == 0x33U && frame.data[3] == 0x50U);
    assert(coordinate_reply(0x50U, 0.25001F, 100003U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_next_can_frame(200002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == 0x51U);
    assert(coordinate_reply(0x51U, 0.0F, 200003U) == MOTOR_RUNTIME_STATUS_OK);
    (void)aethor_app_next_can_frame(204000U, &frame, &priority);
    assert(request("401 adrc coordinates_status", 204001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "id=400 state=2 reason=0 seen=3") != NULL);
    assert(strstr(output.messages[0].data, "native_p=0.25") != NULL);
    assert(strstr(output.messages[0].data, "xout=0") != NULL);
    assert(strstr(output.messages[0].data, "qualified=0 control=0") != NULL);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    assert(application_motor_runtime.bank.motors[6].feedback.position_rad == 0.25F);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("402 adrc coordinates motor=7", 204002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief 首次原始读取等时钟推进；同发送毫秒的首回执必须仍严格晚于事务起点。 */
static void test_coordinate_pair_waits_for_later_tick(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint32_t feedback_count;
    unsigned same_tick_call;
    fixture_coordinate_pair();
    feedback_count = application_motor_runtime.accepted_feedback_count;
    assert(request("403 adrc coordinates motor=7", 101000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    for (same_tick_call = 0U; same_tick_call < 3U; ++same_tick_call)
    {
        assert(aethor_app_next_can_frame(101000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_WAITING);
        assert(!application_motor_runtime.position_read.pending);
        assert(application_motor_runtime.position_read.not_before_us == 0U);
        assert(application_coordinate_pair.state == 1U);
    }
    assert(aethor_app_next_can_frame(102000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == 0x50U);
    assert(coordinate_reply(0x50U, 0.25001F, 102000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.position_read.sample_us[0] > application_coordinate_pair.started_us);
    assert(aethor_app_next_can_frame(202000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == 0x51U);
    assert(coordinate_reply(0x51U, 0.0F, 202000U) == MOTOR_RUNTIME_STATUS_OK);
    (void)aethor_app_next_can_frame(206000U, &frame, &priority);
    assert(request("404 adrc coordinates_status", 206001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "id=403 state=2 reason=0 seen=3") != NULL);
    assert(application_motor_runtime.position_read.accepted_count == 2U);
    assert(application_motor_runtime.position_read.rejected_count == 0U);
    assert(application_motor_runtime.position_read.timeout_count == 0U);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    assert(application_motor_runtime.bank.motors[6].feedback.position_rad == 0.25F);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
}

/** @brief 首发等待保留原期限、失权、STOP、非法/重复回执以及严格起点时间门。 */
static void test_coordinate_pair_wait_preserves_failures(void)
{
    static const uint8_t expected_reasons[7] = {2U, 2U, 3U, 1U, 4U, 4U, 4U};
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < sizeof(expected_reasons) / sizeof(expected_reasons[0]); ++scenario)
    {
        fixture_coordinate_pair();
        assert(request("405 adrc coordinates motor=7", 101000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        if (scenario == 0U) { (void)aethor_app_next_can_frame(100999U, &frame, &priority); }
        if (scenario == 1U) { (void)aethor_app_next_can_frame(501000U, &frame, &priority); }
        if (scenario == 2U)
        {
            application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
            (void)aethor_app_next_can_frame(101000U, &frame, &priority);
        }
        if (scenario == 3U)
        { assert(request("406 bench stop 7", 101000U, &output) == PROTOCOL_ENGINE_STATUS_OK); }
        if (scenario == 4U || scenario == 5U)
        {
            assert(aethor_app_next_can_frame(102000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            if (scenario == 4U)
            { assert(coordinate_reply(0x51U, 1.0F, 102000U) != MOTOR_RUNTIME_STATUS_OK); }
            else
            {
                assert(coordinate_reply(0x50U, 0.25001F, 102000U) == MOTOR_RUNTIME_STATUS_OK);
                assert(coordinate_reply(0x50U, 0.25001F, 102001U) != MOTOR_RUNTIME_STATUS_OK);
            }
            (void)aethor_app_next_can_frame(104000U, &frame, &priority);
        }
        if (scenario == 6U)
        {
            /* 单独验证原终态门；注入边界时间不代表真实CAN接收。 */
            application_motor_runtime.position_read.seen_mask = 3U;
            application_motor_runtime.position_read.sample_us[0] = 101000U;
            application_motor_runtime.position_read.sample_us[1] = 201000U;
            (void)aethor_app_next_can_frame(202000U, &frame, &priority);
        }
        assert(application_coordinate_pair.state == 3U);
        assert(application_coordinate_pair.reason == expected_reasons[scenario]);
        assert(!application_motor_runtime.position_read.pending);
        assert(application_motor_runtime.bank.motors[6].feedback.position_rad == 0.25F);
        assert(request("407 adrc coordinates motor=7", 502000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
}

/** @brief Wrong arguments and unhealthy or competing states cannot initiate any raw read. */
static void test_coordinate_pair_admission(void)
{
    ProtocolOutputBatch output;
    unsigned scenario;
    const char *invalid[] = {"coordinates", "410 adrc coordinates motor=1",
        "411 adrc coordinates motor=7 rid=80", "adrc coordinates motor=7"};
    for (scenario = 0U; scenario < sizeof(invalid)/sizeof(invalid[0]); ++scenario)
    {
        fixture_coordinate_pair();
        assert(request(invalid[scenario], 100001U, &output) != PROTOCOL_ENGINE_STATUS_OK);
        assert(!application_motor_runtime.position_read.pending);
    }
    for (scenario = 0U; scenario < 7U; ++scenario)
    {
        fixture_coordinate_pair();
        if (scenario == 0U) application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
        if (scenario == 1U) application_integrated_can_idle = 0U;
        if (scenario == 2U) application_motor_runtime.bank.motors[6].feedback.driver_state = S3519_DRIVER_STATE_ENABLED;
        if (scenario == 3U) application_motor_runtime.bank.motors[6].feedback.timestamp_us = 0U;
        if (scenario == 4U) application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 50.0F;
        if (scenario == 5U) application_motor_runtime.discovery_active = 1U;
        if (scenario == 6U) application_motor_runtime.position_read.pending = 1U;
        assert(request("420 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
}

/** @brief Timeout never retries; STOP cancels and permanently quarantines late parameter replies. */
static void test_coordinate_pair_timeout_and_stop(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    fixture_coordinate_pair();
    assert(request("430 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_next_can_frame(100002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_next_can_frame(200003U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request("431 adrc coordinates_status", 200004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=3 reason=2") != NULL);
    fixture_coordinate_pair();
    assert(request("440 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_next_can_frame(100002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request("441 adrc acquire motor=7", 100003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD);
    assert(request("442 bench stop 7", 100004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(request("443 adrc coordinates_status", 100005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=3 reason=1") != NULL);
    (void)coordinate_reply(0x50U, 9.0F, 100006U);
    assert(application_motor_runtime.bank.motors[6].feedback.position_rad == 0.25F);
    assert(request("444 adrc coordinates motor=7", 100007U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Reject invalid/late register values and owner changes without refreshing feedback or rearming. */
static void test_coordinate_pair_rejections_and_owner_change(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 3U; ++scenario)
    {
        fixture_coordinate_pair();
        assert(request("450 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(aethor_app_next_can_frame(100002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        if (scenario == 0U) (void)coordinate_reply(0x51U, 1.0F, 100003U);
        if (scenario == 1U) (void)coordinate_reply(0x50U, NAN, 100003U);
        if (scenario == 2U) application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
        (void)aethor_app_next_can_frame(104000U, &frame, &priority);
        assert(request("451 adrc coordinates_status", 104001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, scenario == 2U ? "state=3 reason=3" : "state=3 reason=4") != NULL);
        assert(application_motor_runtime.bank.motors[6].feedback.position_rad == 0.25F);
        assert(!application_motor_runtime.position_read.pending);
    }
}

/** @brief Runs one deterministic integrated handoff without USB or CAN hardware. */
/** @brief Match audited hardware facts while keeping all input evidence synthetic. */
static void fixture_low_speed_mapping(void)
{
    fixture_coordinate_pair();
    application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 10.0F;
    application_motor_runtime.discovery.results[6].software_version = 808597046U;
    application_motor_runtime.discovery.results[6].sub_version = 49U;
}

/** @brief Simulate the real pre-service FIFO-empty boundary; no physical timing claim. */
static MotorRuntimeStatus mapping_next_frame(uint64_t now_us, CanFrame *frame, CanTxPriority *priority)
{
    aethor_app_adrc_observe_capability_rx_boundary(now_us);
    return aethor_app_next_can_frame(now_us, frame, priority);
}

/** @brief A fixed low-speed transaction must acknowledge before any volatile write is possible. */
static void test_low_speed_mapping_entry(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    fixture_low_speed_mapping();
    assert(request("470 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "low_speed accepted=1") != NULL);
    assert(request("471 adrc acquire motor=7", 100002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(mapping_next_frame(599999U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U &&
        frame.data[0] == 7U && frame.data[1] == 0U && frame.data[2] == 0x33U && frame.data[3] == 0x16U);
}

/** @brief Supply a complete disabled packed frame with an exact velocity code. */
static void mapping_feedback(uint16_t velocity_code, uint64_t now_us)
{
    CanFrame reply;
    uint8_t payload[8] = {7U, 0x80U, 0U, 0U, 0U, 0xFFU, 25U, 25U};
    payload[3] = (uint8_t)(velocity_code >> 4U);
    payload[4] = (uint8_t)(((velocity_code & 0xFU) << 4U) | 7U);
    assert(can_frame_init(&reply, 23U, payload, 8U) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&reply, now_us) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Complete real discovery requests using schema-specific synthetic register responses. */
static uint64_t mapping_discover(uint64_t now_us, float position_range, float velocity_range,
    uint8_t wrong_version)
{
    CanFrame frame;
    CanFrame reply;
    CanTxPriority priority;
    unsigned count = 0U;
    while (application_low_speed_mapping.state == AETHOR_MAPPING_DISCOVERY ||
           application_low_speed_mapping.state == AETHOR_MAPPING_DRAIN)
    {
        MotorRuntimeStatus status = mapping_next_frame(now_us, &frame, &priority);
        if (status == MOTOR_RUNTIME_STATUS_FRAME_READY)
        {
            uint32_t raw = 0U;
            float value = 5.0F;
            uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
            assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U);
            payload[3] = frame.data[3];
            switch (frame.data[3])
            {
                case 0x07U: raw = 23U; break;
                case 0x08U: raw = 7U; break;
                case 0x0AU: raw = 2U; break;
                case 0x09U: raw = 1000U; break;
                case 0x0DU: raw = 1U; break;
                case 0x0EU: raw = wrong_version ? 1U : 808597046U; break;
                case 0x24U: raw = 49U; break;
                case 0x15U: value = position_range; break;
                case 0x16U: value = velocity_range; break;
                case 0x17U: value = 10.0F; break;
                case 0x06U: value = 25.0F; break;
                default: break;
            }
            if (frame.data[3] == 0x15U || frame.data[3] == 0x16U || frame.data[3] == 0x17U ||
                frame.data[3] == 0x04U || frame.data[3] == 0x05U || frame.data[3] == 0x06U)
            { memcpy(&raw, &value, sizeof(raw)); }
            memcpy(payload + 4U, &raw, sizeof(raw));
            assert(can_frame_init(&reply, 23U, payload, 8U) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame(&reply, now_us + 1U) == MOTOR_RUNTIME_STATUS_OK);
            ++count;
        }
        now_us += 1000U;
        assert(count <= 13U && now_us < 2800000ULL);
    }
    assert(count == 13U);
    return now_us;
}

/** @brief Exercise fixed bytes, write-ACK isolation, full rediscovery and a new disabled baseline. */
static uint64_t mapping_complete_register_transaction(uint8_t register_id, uint64_t now_us,
    float source_range, float target_range, uint8_t wrong_version)
{
    CanFrame frame;
    CanTxPriority priority;
    uint32_t target_bits;
    uint32_t feedback_count = application_motor_runtime.accepted_feedback_count;
    now_us += 500000U;
    assert(mapping_next_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == register_id);
    assert(coordinate_reply(register_id, source_range, now_us + 1U) == MOTOR_RUNTIME_STATUS_OK);
    assert(mapping_next_frame(now_us + 2U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 8U && frame.data[2] == 0x55U && frame.data[3] == register_id);
    memcpy(&target_bits, frame.data + 4U, sizeof(target_bits));
    assert(!memcmp(&target_bits, &target_range, sizeof(float)));
    frame.identifier = 23U;
    assert(aethor_app_receive_can_frame(&frame, now_us + 3U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_SETTLE);
    mapping_feedback(2088U, now_us + 4U);
    assert(application_motor_runtime.accepted_feedback_count == feedback_count);
    now_us += 100003U;
    assert(mapping_next_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == register_id);
    assert(coordinate_reply(register_id, target_range, now_us + 1U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_mapping_locked());
    now_us = mapping_discover(now_us + 500002U,
        register_id == 0x15U ? target_range : 12.5F,
        register_id == 0x16U ? target_range : 200.0F, wrong_version);
    if (wrong_version)
    { assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED); return now_us; }
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DISABLE);
    assert(mapping_next_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 7U && frame.length == 8U && frame.data[7] == 0xFDU);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(aethor_app_mapping_locked());
    mapping_feedback(2048U, now_us + 1U);
    (void)mapping_next_frame(now_us + 2U, &frame, &priority);
    if (application_low_speed_mapping.state != AETHOR_MAPPING_DONE)
    { fprintf(stderr, "mapping baseline state=%u reason=%u valid=%u stamp=%llu phase=%llu vel=%g count=%u busy=%u\n",
        (unsigned)application_low_speed_mapping.state, application_low_speed_mapping.reason,
        application_motor_runtime.bank.motors[6].feedback_valid,
        (unsigned long long)application_motor_runtime.bank.motors[6].feedback.timestamp_us,
        (unsigned long long)application_low_speed_mapping.phase_us,
        (double)application_motor_runtime.bank.motors[6].feedback.velocity_rad_s,
        application_motor_runtime.accepted_feedback_count, (unsigned)aethor_app_debug_ui_executor_busy()); }
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DONE && !aethor_app_mapping_locked());
    assert(application_motor_runtime.accepted_feedback_count == feedback_count + 1U);
    assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
    return now_us + 3U;
}

/** @brief Preserve the existing VMAX fixture through the common serialized maintenance path. */
static uint64_t mapping_complete_transaction(uint64_t now_us, float source_range, float target_range,
    uint8_t wrong_version)
{
    return mapping_complete_register_transaction(0x16U, now_us, source_range, target_range, wrong_version);
}

/** @brief Verify both fixed directions and that every subsequent normal sample uses the new VMAX. */
static void test_low_speed_mapping_roundtrip(void)
{
    ProtocolOutputBatch output;
    uint64_t now_us;
    float low_speed;
    fixture_low_speed_mapping();
    assert(request("480 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    now_us = mapping_complete_transaction(100001U, 200.0F, 5.0F, 0U);
    mapping_feedback(2088U, now_us);
    low_speed = application_motor_runtime.bank.motors[6].feedback.velocity_rad_s;
    assert(low_speed > 0.098F && low_speed < 0.100F);
    mapping_feedback(2048U, now_us + 1U);
    assert(request("481 adrc restore_speed motor=7", now_us + 2U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    now_us = mapping_complete_transaction(now_us + 2U, 5.0F, 200.0F, 0U);
    mapping_feedback(2088U, now_us);
    assert(fabsf(application_motor_runtime.bank.motors[6].feedback.velocity_rad_s / low_speed - 40.0F) < 0.001F);
}

/** @brief Wrong readback, late replies, source mismatch, timeout, STOP and bad discovery cannot unlock motion. */
static void test_low_speed_mapping_failures(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 7U; ++scenario)
    {
        fixture_low_speed_mapping();
        assert(request("490 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        if (scenario == 0U)
        { assert(request("491 bench stop 7", 100002U, &output) == PROTOCOL_ENGINE_STATUS_OK); }
        else if (scenario == 1U)
        {
            (void)mapping_next_frame(600001U, &frame, &priority);
            (void)mapping_next_frame(700001U, &frame, &priority);
        }
        else if (scenario == 6U)
        { (void)mapping_complete_transaction(100001U, 200.0F, 5.0F, 1U); }
        else
        {
            assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            if (scenario == 2U) { (void)coordinate_reply(0x16U, NAN, 600002U); }
            if (scenario == 3U) { (void)coordinate_reply(0x16U, 5.0F, 600002U); }
            if (scenario == 4U) { (void)coordinate_reply(0x16U, 200.0F, 700001U); }
            if (scenario == 5U)
            {
                (void)coordinate_reply(0x16U, 200.0F, 600002U);
                (void)mapping_next_frame(600003U, &frame, &priority);
                (void)mapping_next_frame(700003U, &frame, &priority);
                (void)coordinate_reply(0x16U, 200.0F, 700004U);
            }
        }
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED && aethor_app_mapping_locked());
        assert(!application_motor_runtime.bank.motors[6].feedback_valid);
        assert(request("492 adrc acquire motor=7", 2500000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        (void)coordinate_reply(0x16U, 5.0F, 2500001U);
        assert(aethor_app_mapping_locked());
    }
}

/** @brief Missing FIFO proof blocks emission; coarse-clock equality is admitted only after that boundary. */
static void test_low_speed_mapping_timing_and_recovery(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t now_us;
    fixture_low_speed_mapping();
    assert(request("500 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_next_can_frame(600001U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(mapping_next_frame(600002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(coordinate_reply(0x16U, 200.0F, 600002U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_WRITE);
    assert(request("501 bench stop 7", 600003U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
    /* A separate recovery fixture has no queued STOP; failure is retained until explicit restore. */
    fixture_low_speed_mapping();
    assert(request("502 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)mapping_next_frame(600001U, &frame, &priority);
    (void)coordinate_reply(0x16U, 200.0F, 700001U);
    assert(request("503 adrc restore_speed motor=7", 700002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(aethor_app_pop_emergency_can_frame(&frame) == 1U);
    assert(frame.identifier == 7U && frame.data[7] == 0xFDU);
    mapping_feedback(2048U, 700003U);
    assert(!application_motor_runtime.bank.motors[6].feedback_valid);
    assert(request("504 adrc restore_speed motor=7", 700004U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    now_us = mapping_complete_transaction(700004U, 200.0F, 200.0F, 0U);
    assert(request("505 adrc low_speed_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=10 reason=0 locked=0 target=200") != NULL);
}

/** @brief Reject arbitrary ranges, missing identities, stale feedback, busy local mailboxes and owner changes. */
static void test_low_speed_mapping_admission_and_local_stop(void)
{
    ProtocolOutputBatch output;
    DebugUiRequest local;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    const char *invalid[] = {"adrc low_speed motor=7", "510 adrc low_speed motor=1",
        "511 adrc low_speed motor=7 vmax=5", "512 adrc restore_speed motor=7"};
    for (scenario = 0U; scenario < sizeof(invalid)/sizeof(invalid[0]); ++scenario)
    {
        fixture_low_speed_mapping();
        assert(request(invalid[scenario], 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_IDLE);
    }
    for (scenario = 0U; scenario < 6U; ++scenario)
    {
        fixture_low_speed_mapping();
        if (scenario == 0U) application_motor_runtime.discovery.results[6].software_version = 1U;
        if (scenario == 1U) application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 18.0F;
        if (scenario == 2U) application_motor_runtime.bank.motors[6].feedback.timestamp_us = 0U;
        if (scenario == 3U) application_integrated_can_idle = 0U;
        if (scenario == 4U) application_motor_runtime.bank.motors[6].feedback.driver_state = 1U;
        if (scenario == 5U) application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 50.0F;
        assert(request("513 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    }
    fixture_low_speed_mapping();
    assert(request("514 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    memset(&local, 0, sizeof(local));
    local.operation = DEBUG_UI_OPERATION_ACQUIRE;
    assert(aethor_app_debug_ui_submit(&local) == DEBUG_UI_REASON_DISABLED);
    local.operation = DEBUG_UI_OPERATION_STOP;
    (void)aethor_app_debug_ui_submit(&local);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
    {
        CanFrame group[ARM_JOINT_COUNT];
        application_action.control_group_ready = 1U;
        assert(aethor_app_pop_control_group(group) == 0U);
        assert(aethor_app_pop_emergency_can_frame(&frame) == 1U);
        assert(frame.identifier == 7U && frame.data[7] == 0xFDU);
    }
    fixture_low_speed_mapping();
    assert(request("515 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC;
    (void)mapping_next_frame(600001U, &frame, &priority);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
}

/** @brief PMAX admission invalidates stale feedback/model trust and exposes the fixed exact status contract. */
static void test_position_mapping_entry_and_status(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    const char *expected = "position_range_status id=600 state=1 reason=0 locked=1 target=512 source=12.5 writes=0 pmax=12.5 vmax=200 tmax=10 register=21 qualified=0 control=0 tx_proven=0";
    fixture_low_speed_mapping();
    application_adrc_bridge.bench.trusted_qualification.verified_flags = 0xFFU;
    application_adrc_bridge.bench.experiment.qualification.verified_flags = 0xFFU;
    application_adrc_bridge.bench.experiment.configured = 1U;
    application_adrc_bridge.evidence_bound = 1U;
    assert(request("600 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "wide_position accepted=1 qualified=0 control=0") != NULL);
    assert(!application_motor_runtime.bank.motors[6].feedback_valid);
    assert(!(application_motor_runtime.bank.valid_joint_mask & 0x40U));
    assert(!(application_motor_runtime.discovery.verified_joint_mask & 0x40U));
    assert(!application_adrc_bridge.bench.trusted_qualification.verified_flags);
    assert(!application_adrc_bridge.bench.experiment.qualification.verified_flags);
    assert(!application_adrc_bridge.bench.experiment.configured && !application_adrc_bridge.evidence_bound);
    assert(request("601 adrc position_range_status", 100002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, expected) != NULL);
    assert(request("602 adrc position_range_status motor=7", 100003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("603 adrc coordinates motor=7", 100004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("604 adrc acquire motor=7", 100005U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(mapping_next_frame(600000U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_next_can_frame(600001U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(mapping_next_frame(600002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[3] == 0x15U);
}

/** @brief Full rediscovery synchronizes native position decoding in both explicit fixed directions. */
static void test_position_mapping_roundtrip(void)
{
    ProtocolOutputBatch output;
    MotorPositionVelocityLimits limits;
    uint64_t now_us;
    float wide_position;
    fixture_low_speed_mapping();
    assert(request("610 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    now_us = mapping_complete_register_transaction(0x15U, 100001U, 12.5F, 512.0F, 0U);
    wide_position = application_motor_runtime.bank.motors[6].feedback.position_rad;
    assert(fabsf(wide_position - 512.0F / 65535.0F) < 0.00002F);
    assert(motor_runtime_get_position_velocity_limits(&application_motor_runtime, 6U, &limits) == MOTOR_RUNTIME_STATUS_OK);
    assert(limits.position_max_rad == 512.0F && limits.velocity_mapping_max_rad_s == 200.0F);
    assert(application_motor_runtime.bank.motors[6].configuration_consistent);
    assert(application_motor_runtime.bank.motors[6].parameter_valid_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    assert(application_motor_runtime.discovery.results[6].ranges.torque_max_nm == 10.0F);
    assert(request("611 adrc position_range_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=10 reason=0 locked=0 target=512 source=12.5 writes=1 pmax=512 vmax=200 tmax=10 register=21 qualified=0 control=0 tx_proven=0") != NULL);
    assert(request("612 adrc restore_position motor=7", now_us + 1U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "restore_position accepted=1 qualified=0 control=0") != NULL);
    now_us = mapping_complete_register_transaction(0x15U, now_us + 1U, 512.0F, 12.5F, 0U);
    assert(fabsf(wide_position / application_motor_runtime.bank.motors[6].feedback.position_rad - 40.96F) < 0.3F);
    assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == 12.5F);
    assert(application_low_speed_mapping.writes == 1U);
    assert(request("613 adrc restore_position motor=7", now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
}

/** @brief Reject arbitrary mappings, non-REMOTE authority, unsafe evidence, identity drift and queued competitors. */
static void test_position_mapping_admission(void)
{
    ProtocolOutputBatch output;
    unsigned scenario;
    const char *invalid[] = {"adrc wide_position motor=7", "620 adrc wide_position motor=1",
        "621 adrc wide_position motor=7 pmax=512", "622 adrc restore_position motor=7"};
    for (scenario = 0U; scenario < sizeof(invalid) / sizeof(invalid[0]); ++scenario)
    {
        fixture_low_speed_mapping();
        assert(request(invalid[scenario], 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_IDLE);
    }
    for (scenario = 0U; scenario < 25U; ++scenario)
    {
        fixture_low_speed_mapping();
        switch (scenario)
        {
            case 0U: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; break;
            case 1U: application_motor_runtime.discovery.results[6].software_version = 1U; break;
            case 2U: application_motor_runtime.discovery.results[6].sub_version = 1U; break;
            case 3U: application_motor_runtime.discovery.results[6].observed_esc_id = 6U; break;
            case 4U: application_motor_runtime.discovery.results[6].observed_master_id = 22U; break;
            case 5U: application_motor_runtime.discovery.results[6].observed_control_mode = 1U; break;
            case 6U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F; break;
            case 7U: application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; break;
            case 8U: application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 18.0F; break;
            case 9U: application_motor_runtime.bank.motors[6].feedback.timestamp_us = 0U; break;
            case 10U: application_motor_runtime.bank.motors[6].feedback.driver_state = 1U; break;
            case 11U: application_motor_runtime.bank.motors[6].feedback.fault_flags = 8U; break;
            case 12U: application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 50.0F; break;
            case 13U: application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 50.0F; break;
            case 14U: application_motor_runtime.bank.motors[6].feedback.position_rad = NAN; break;
            case 15U: application_motor_runtime.bank.motors[6].feedback.velocity_rad_s = 0.051F; break;
            case 16U: application_motor_runtime.bank.motors[6].feedback.torque_nm = 0.151F; break;
            case 17U: application_integrated_can_idle = 0U; break;
            case 18U: application_action.control_group_ready = 1U; break;
            case 19U: application_emergency_disable_batch.count = 1U; break;
            case 20U: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
            case 21U: application_protocol_engine.result_write_sequence = 1U; break;
            case 22U: application_motor_runtime.position_read.pending = 1U; break;
            case 23U: application_protocol_engine.command_write_sequence = 1U; break;
            default: application_motor_runtime.mode_switch_state = MOTOR_MODE_SWITCH_READ_READY; break;
        }
        assert(request("623 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_IDLE);
    }
}

/** @brief Unexpected reads, exact readback mismatch, timeout, STOP and bad rediscovery retain the failure lock. */
static void test_position_mapping_failures(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 10U; ++scenario)
    {
        fixture_low_speed_mapping();
        assert(request("630 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        if (scenario == 0U) { (void)request("631 bench stop 7", 100002U, &output); }
        else if (scenario == 9U) { (void)mapping_complete_register_transaction(0x15U, 100001U, 12.5F, 512.0F, 1U); }
        else
        {
            assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            if (scenario == 1U) { (void)coordinate_reply(0x15U, NAN, 600002U); }
            if (scenario == 2U) { (void)coordinate_reply(0x15U, 512.0F, 600002U); }
            if (scenario == 3U) { (void)coordinate_reply(0x15U, 12.5F, 700001U); }
            if (scenario == 4U)
            {
                (void)coordinate_reply(0x16U, 12.5F, 600002U);
                assert(application_low_speed_mapping.state == AETHOR_MAPPING_SOURCE_WAIT);
                (void)mapping_next_frame(700001U, &frame, &priority);
            }
            if (scenario == 5U)
            {
                uint8_t payload[8] = {7U, 0U, 0x33U, 0x15U, 0U, 0U, 0U, 0U};
                float source = 12.5F;
                memcpy(payload + 4U, &source, sizeof(source));
                assert(can_frame_init(&frame, 22U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                (void)aethor_app_receive_can_frame(&frame, 600002U);
            }
            if (scenario >= 6U && scenario <= 8U)
            {
                (void)coordinate_reply(0x15U, 12.5F, 600002U);
                assert(mapping_next_frame(600003U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
                assert(mapping_next_frame(700003U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
                if (scenario == 6U) { (void)coordinate_reply(0x15U, 511.99997F, 700004U); }
                if (scenario == 7U) { (void)mapping_next_frame(800003U, &frame, &priority); }
                if (scenario == 8U)
                {
                    uint8_t payload[8] = {0x87U, 0x80U, 0U, 0x80U, 7U, 0xFFU, 25U, 25U};
                    assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                    (void)aethor_app_receive_can_frame(&frame, 700004U);
                }
            }
        }
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED && aethor_app_mapping_locked());
        assert(!application_motor_runtime.bank.motors[6].feedback_valid);
        assert(application_low_speed_mapping.writes <= 1U);
        assert(request("632 adrc acquire motor=7", 2500000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        (void)coordinate_reply(0x15U, 512.0F, 2500001U);
        assert(aethor_app_mapping_locked());
    }
}

/** @brief Late write ACKs and old feedback cannot prove readback or a newly disabled baseline. */
static void test_position_mapping_ack_drain_and_baseline(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint32_t accepted;
    uint64_t now_us;
    fixture_low_speed_mapping();
    accepted = application_motor_runtime.accepted_feedback_count;
    assert(request("640 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    (void)coordinate_reply(0x15U, 12.5F, 600002U);
    assert(mapping_next_frame(600003U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    frame.identifier = 23U;
    assert(aethor_app_receive_can_frame(&frame, 700003U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_SETTLE);
    assert(mapping_next_frame(700004U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    frame.length = 8U;
    frame.identifier = 23U;
    frame.data[2] = 0x55U;
    assert(aethor_app_receive_can_frame(&frame, 700005U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_TARGET_WAIT);
    mapping_feedback(2048U, 700006U);
    assert(application_motor_runtime.accepted_feedback_count == accepted);
    (void)coordinate_reply(0x15U, 512.0F, 700007U);
    assert(mapping_next_frame(1200006U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DRAIN);
    now_us = mapping_discover(1200007U, 512.0F, 200.0F, 0U);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DISABLE);
    assert(mapping_next_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    mapping_feedback(2048U, now_us - 1U);
    assert(!application_motor_runtime.bank.motors[6].feedback_valid);
    assert(mapping_next_frame(now_us + 1U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_mapping_locked());
    mapping_feedback(2048U, now_us + 2U);
    (void)mapping_next_frame(now_us + 3U, &frame, &priority);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DONE);
    assert(application_motor_runtime.accepted_feedback_count == accepted + 1U);
}

/** @brief Failure recovery needs the same register/axis, an explicit command and fresh healthy disabled evidence. */
static void test_position_mapping_explicit_recovery_and_cross_lock(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t now_us;
    unsigned position_failure;
    for (position_failure = 0U; position_failure <= 1U; ++position_failure)
    {
        fixture_low_speed_mapping();
        assert(request(position_failure ? "650 adrc wide_position motor=7" : "650 adrc low_speed motor=7",
            100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        (void)mapping_next_frame(700001U, &frame, &priority);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
        assert(request("651 adrc restore_position motor=7", 700002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(aethor_app_pop_emergency_can_frame(&frame));
        mapping_feedback(2048U, 700003U);
        assert(!application_motor_runtime.bank.motors[6].feedback_valid);
        assert(request(position_failure ? "652 adrc restore_speed motor=7" : "652 adrc restore_position motor=7",
            700004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(request("653 adrc restore_position motor=1", 700005U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(aethor_app_mapping_locked());
        if (position_failure)
        {
            application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
            assert(request("654 adrc restore_position motor=7", 700006U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
            application_debug_ui.authority = DEBUG_UI_AUTHORITY_REMOTE;
            assert(request("655 adrc restore_position motor=7", 800004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
            mapping_feedback(2048U, 800005U);
            assert(request("656 adrc restore_position motor=7", 800006U, &output) == PROTOCOL_ENGINE_STATUS_OK);
            now_us = mapping_complete_register_transaction(0x15U, 800006U, 12.5F, 12.5F, 0U);
            assert(request("657 adrc position_range_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
            assert(strstr(output.messages[0].data, "state=10 reason=0 locked=0 target=12.5 source=12.5 writes=1") != NULL);
        }
    }
    fixture_low_speed_mapping();
    assert(request("658 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    (void)mapping_next_frame(600001U, &frame, &priority);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
}

/** @brief A same-register restore still refuses changed identity, revoked health and clocks before the transaction. */
static void test_position_mapping_recovery_identity_and_feedback(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    unsigned rejected = 0U;
    for (scenario = 0U; scenario < 14U; ++scenario)
    {
        uint64_t feedback_us = scenario == 12U ? 100000U : scenario == 13U ? 600000U : 700003U;
        fixture_low_speed_mapping();
        assert(request("670 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        if (scenario == 12U) { (void)mapping_next_frame(99999U, &frame, &priority); }
        else
        {
            (void)mapping_next_frame(600001U, &frame, &priority);
            (void)mapping_next_frame(scenario == 13U ? 599999U : 700001U, &frame, &priority);
        }
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
        assert(aethor_app_pop_emergency_can_frame(&frame));
        mapping_feedback(2048U, feedback_us);
        switch (scenario)
        {
            case 0U: application_motor_runtime.discovery.results[6].software_version = 1U; break;
            case 1U: application_motor_runtime.discovery.results[6].sub_version = 1U; break;
            case 2U: application_motor_runtime.discovery.results[6].observed_esc_id = 6U; break;
            case 3U: application_motor_runtime.discovery.results[6].observed_master_id = 22U; break;
            case 4U: application_motor_runtime.discovery.results[6].observed_control_mode = 1U; break;
            case 5U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = NAN; break;
            case 6U: application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; break;
            case 7U: application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 18.0F; break;
            case 8U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 100.0F; break;
            case 9U: mapping_feedback(2088U, feedback_us + 1U); break;
            case 10U:
            case 11U:
            {
                uint8_t payload[8] = {7U, 0x80U, 0U, 0x80U, 7U, 0xFFU, 25U, 25U};
                if (scenario == 10U) { payload[6] = 50U; }
                else { payload[0] = 0x87U; }
                assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                (void)aethor_app_receive_can_frame(&frame, feedback_us + 1U);
                break;
            }
            default: break;
        }
        if (request("671 adrc restore_position motor=7", feedback_us + 2U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST)
        { ++rejected; }
        else { fprintf(stderr, "unsafe PMAX recovery admitted scenario=%u\n", scenario); }
    }
    assert(rejected == 14U);
}

/** @brief Position-status consumers can distinguish VMAX activity/terminals without mutating or hiding its lock. */
static void test_position_status_identifies_shared_velocity_transaction(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    AethorLowSpeedMapping retained;
    unsigned scenario;
    for (scenario = 0U; scenario < 2U; ++scenario)
    {
        uint64_t now_us;
        fixture_low_speed_mapping();
        assert(request("680 adrc position_range_status", 100000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "state=0 reason=0 locked=0") != NULL);
        assert(strstr(output.messages[0].data, "register=21 qualified=0 control=0 tx_proven=0") != NULL);
        assert(request("681 adrc low_speed motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        retained = application_low_speed_mapping;
        assert(request("682 adrc position_range_status", 100002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, "state=1 reason=0 locked=1 target=5 source=200 writes=0") != NULL);
        assert(strstr(output.messages[0].data, "register=22 qualified=0 control=0 tx_proven=0") != NULL);
        assert(!memcmp(&retained, &application_low_speed_mapping, sizeof(retained)));
        if (scenario == 0U)
        { now_us = mapping_complete_transaction(100001U, 200.0F, 5.0F, 0U); }
        else
        {
            (void)mapping_next_frame(600001U, &frame, &priority);
            (void)mapping_next_frame(700001U, &frame, &priority);
            now_us = 700002U;
        }
        retained = application_low_speed_mapping;
        assert(request("683 adrc position_range_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, scenario ? "state=11 reason=2 locked=1" : "state=10 reason=0 locked=0") != NULL);
        assert(strstr(output.messages[0].data, "target=5 source=200") != NULL);
        assert(strstr(output.messages[0].data, "register=22 qualified=0 control=0 tx_proven=0") != NULL);
        assert(!memcmp(&retained, &application_low_speed_mapping, sizeof(retained)));
        assert(aethor_app_mapping_locked() == (uint8_t)scenario);
        assert(request("684 adrc low_speed_status", now_us + 1U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        assert(strstr(output.messages[0].data, scenario ? "state=11 reason=2 locked=1" : "state=10 reason=0 locked=0") != NULL);
        assert(strstr(output.messages[0].data, "register=") == NULL);
    }
}

/** @brief Start the independent fixed rated70 family from the audited PMAX512 contract. */
static void fixture_rated70_position_mapping(void)
{
    fixture_low_speed_mapping();
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
}

/** @brief Both fixed rated70 directions retain exact status fields and synchronize the actual decoder. */
static void test_rated70_position_mapping_roundtrip(void)
{
    ProtocolOutputBatch output;
    MotorPositionVelocityLimits limits;
    uint64_t now_us;
    float expanded_position;
    fixture_rated70_position_mapping();
    assert(request("700 adrc rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strcmp(output.messages[0].data,
        "ok 700 adrc rated70_position accepted=1 qualified=0 control=0\r\n") == 0);
    assert(request("701 adrc position_range_status", 100002U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strcmp(output.messages[0].data,
        "ok 701 adrc position_range_status id=700 state=1 reason=0 locked=1 target=1024 source=512 writes=0 pmax=512 vmax=200 tmax=10 register=21 qualified=0 control=0 tx_proven=0\r\n") == 0);
    now_us = mapping_complete_register_transaction(0x15U, 100001U, 512.0F, 1024.0F, 0U);
    expanded_position = application_motor_runtime.bank.motors[6].feedback.position_rad;
    assert(fabsf(expanded_position - 1024.0F / 65535.0F) < 0.00004F);
    assert(motor_runtime_get_position_velocity_limits(&application_motor_runtime, 6U, &limits) == MOTOR_RUNTIME_STATUS_OK);
    assert(limits.position_max_rad == 1024.0F && limits.velocity_mapping_max_rad_s == 200.0F);
    assert(application_motor_runtime.bank.motors[6].configuration_consistent);
    assert(application_motor_runtime.bank.motors[6].parameter_valid_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == 1024.0F);
    assert(request("702 adrc restore_position motor=7", now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("703 adrc rated70_position motor=7", now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(request("704 adrc restore_rated70_position motor=7", now_us + 1U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "restore_rated70_position accepted=1 qualified=0 control=0") != NULL);
    now_us = mapping_complete_register_transaction(0x15U, now_us + 1U, 1024.0F, 512.0F, 0U);
    assert(fabsf(expanded_position / application_motor_runtime.bank.motors[6].feedback.position_rad - 2.0F) < 0.02F);
    assert(application_motor_runtime.discovery.results[6].ranges.position_max_rad == 512.0F);
    assert(application_low_speed_mapping.writes == 1U);
    assert(request("705 adrc restore_rated70_position motor=7", now_us, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    fixture_rated70_position_mapping();
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 1024.0F;
    assert(request("706 adrc restore_rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)mapping_complete_register_transaction(0x15U, 100001U, 1024.0F, 512.0F, 0U);
}

/** @brief Fixed rated70 admission rejects source, identity, freshness, health and competing producers. */
static void test_rated70_position_mapping_admission(void)
{
    ProtocolOutputBatch output;
    unsigned scenario;
    const char *invalid[] = {"adrc rated70_position motor=7", "710 adrc rated70_position motor=1",
        "711 adrc rated70_position motor=7 pmax=1024", "712 adrc restore_rated70_position motor=7",
        "713 adrc rated70_position motor=7 extra", "0 adrc rated70_position motor=7"};
    for (scenario = 0U; scenario < sizeof(invalid) / sizeof(invalid[0]); ++scenario)
    {
        fixture_rated70_position_mapping();
        assert(request(invalid[scenario], 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_IDLE);
    }
    for (scenario = 0U; scenario < 27U; ++scenario)
    {
        fixture_rated70_position_mapping();
        switch (scenario)
        {
            case 0U: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; break;
            case 1U: application_motor_runtime.discovery.results[6].software_version = 1U; break;
            case 2U: application_motor_runtime.discovery.results[6].sub_version = 1U; break;
            case 3U: application_motor_runtime.discovery.results[6].observed_esc_id = 6U; break;
            case 4U: application_motor_runtime.discovery.results[6].observed_master_id = 22U; break;
            case 5U: application_motor_runtime.discovery.results[6].observed_control_mode = 1U; break;
            case 6U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 12.5F; break;
            case 7U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 1024.0F; break;
            case 8U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = NAN; break;
            case 9U: application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; break;
            case 10U: application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 18.0F; break;
            case 11U: application_motor_runtime.bank.motors[6].feedback.timestamp_us = 0U; break;
            case 12U: application_motor_runtime.bank.motors[6].feedback.timestamp_us = 100002U; break;
            case 13U: application_motor_runtime.bank.motors[6].feedback.driver_state = 1U; break;
            case 14U: application_motor_runtime.bank.motors[6].feedback.fault_flags = 8U; break;
            case 15U: application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 50.0F; break;
            case 16U: application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 50.0F; break;
            case 17U: application_motor_runtime.bank.motors[6].feedback.position_rad = NAN; break;
            case 18U: application_motor_runtime.bank.motors[6].feedback.velocity_rad_s = 0.051F; break;
            case 19U: application_motor_runtime.bank.motors[6].feedback.torque_nm = 0.151F; break;
            case 20U: application_integrated_can_idle = 0U; break;
            case 21U: application_action.control_group_ready = 1U; break;
            case 22U: application_emergency_disable_batch.count = 1U; break;
            case 23U: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
            case 24U: application_protocol_engine.result_write_sequence = 1U; break;
            case 25U: application_motor_runtime.position_read.pending = 1U; break;
            default: application_protocol_engine.command_write_sequence = 1U; break;
        }
        assert(request("714 adrc rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_IDLE);
    }
}

/** @brief Wrong reads, source/target values, clocks, STOP, discovery and timeout cannot release rated70. */
static void test_rated70_position_mapping_failures(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 13U; ++scenario)
    {
        fixture_rated70_position_mapping();
        assert(request("720 adrc rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        if (scenario == 0U) { (void)request("721 bench stop 7", 100002U, &output); }
        else if (scenario == 1U) { (void)mapping_next_frame(99999U, &frame, &priority); }
        else if (scenario == 2U) { (void)mapping_complete_register_transaction(0x15U, 100001U, 512.0F, 1024.0F, 1U); }
        else
        {
            assert(mapping_next_frame(600001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
            if (scenario == 3U) { (void)coordinate_reply(0x15U, 12.5F, 600002U); }
            if (scenario == 4U) { (void)coordinate_reply(0x15U, 1024.0F, 600002U); }
            if (scenario == 5U) { (void)coordinate_reply(0x15U, NAN, 600002U); }
            if (scenario == 6U) { (void)coordinate_reply(0x15U, 512.0F, 600000U); }
            if (scenario == 7U) { (void)coordinate_reply(0x15U, 512.0F, 700001U); }
            if (scenario == 8U)
            {
                uint8_t payload[8] = {7U, 0U, 0x33U, 0x15U, 0U, 0U, 0U, 0U};
                float source = 512.0F;
                memcpy(payload + 4U, &source, sizeof(source));
                assert(can_frame_init(&frame, 22U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                (void)aethor_app_receive_can_frame(&frame, 600002U);
            }
            if (scenario == 9U)
            {
                (void)coordinate_reply(0x16U, 512.0F, 600002U);
                assert(application_low_speed_mapping.state == AETHOR_MAPPING_SOURCE_WAIT);
                (void)mapping_next_frame(700001U, &frame, &priority);
            }
            if (scenario >= 10U)
            {
                (void)coordinate_reply(0x15U, 512.0F, 600002U);
                assert(mapping_next_frame(600003U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
                assert(mapping_next_frame(700003U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
                if (scenario == 10U) { (void)coordinate_reply(0x15U, 1023.99994F, 700004U); }
                if (scenario == 11U) { (void)mapping_next_frame(800003U, &frame, &priority); }
                if (scenario == 12U)
                {
                    uint8_t payload[8] = {0x87U, 0x80U, 0U, 0x80U, 7U, 0xFFU, 25U, 25U};
                    assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                    (void)aethor_app_receive_can_frame(&frame, 700004U);
                }
            }
        }
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED && aethor_app_mapping_locked());
        assert(!application_motor_runtime.bank.motors[6].feedback_valid);
        assert(application_low_speed_mapping.writes <= 1U);
        assert(request("722 adrc acquire motor=7", 2500000U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        (void)coordinate_reply(0x15U, 1024.0F, 2500001U);
        assert(aethor_app_mapping_locked());
    }
}

/** @brief ACKs and old decoded samples cannot bypass real readback, 500ms draining or a new DISABLE. */
static void test_rated70_position_mapping_ack_and_old_feedback(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t now_us;
    uint32_t accepted;
    fixture_rated70_position_mapping();
    accepted = application_motor_runtime.accepted_feedback_count;
    assert(request("730 adrc rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_next_can_frame(600001U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(mapping_next_frame(600002U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    (void)coordinate_reply(0x15U, 512.0F, 600003U);
    assert(mapping_next_frame(600004U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    frame.identifier = 23U;
    assert(aethor_app_receive_can_frame(&frame, 700004U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_SETTLE);
    assert(mapping_next_frame(700005U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    frame.identifier = 23U;
    frame.length = 8U;
    frame.data[2] = 0x55U;
    assert(aethor_app_receive_can_frame(&frame, 700006U) == MOTOR_RUNTIME_STATUS_OK);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_TARGET_WAIT);
    mapping_feedback(2048U, 700007U);
    assert(application_motor_runtime.accepted_feedback_count == accepted);
    (void)coordinate_reply(0x15U, 1024.0F, 700008U);
    assert(mapping_next_frame(1200007U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DRAIN);
    now_us = mapping_discover(1200008U, 1024.0F, 200.0F, 0U);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DISABLE);
    assert(mapping_next_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    mapping_feedback(2048U, now_us - 1U);
    assert(!application_motor_runtime.bank.motors[6].feedback_valid);
    (void)mapping_next_frame(now_us + 1U, &frame, &priority);
    assert(aethor_app_mapping_locked());
    mapping_feedback(2048U, now_us + 2U);
    (void)mapping_next_frame(now_us + 3U, &frame, &priority);
    assert(application_low_speed_mapping.state == AETHOR_MAPPING_DONE);
    assert(application_motor_runtime.accepted_feedback_count == accepted + 1U);
}

/** @brief Failure recovery matches register and family, then checks current audited identity and disabled evidence. */
static void test_rated70_position_mapping_recovery_family_and_health(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    CanTxPriority priority;
    unsigned scenario;
    for (scenario = 0U; scenario < 18U; ++scenario)
    {
        fixture_rated70_position_mapping();
        assert(request("740 adrc rated70_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
        (void)mapping_next_frame(600001U, &frame, &priority);
        (void)mapping_next_frame(700001U, &frame, &priority);
        assert(application_low_speed_mapping.state == AETHOR_MAPPING_FAILED);
        assert(request("741 adrc restore_rated70_position motor=7", 700002U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(aethor_app_pop_emergency_can_frame(&frame));
        mapping_feedback(2048U, 700003U);
        assert(request("742 adrc restore_position motor=7", 700004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(request("743 adrc restore_speed motor=7", 700004U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
        assert(aethor_app_mapping_locked());
        switch (scenario)
        {
            case 0U: application_motor_runtime.discovery.results[6].software_version = 1U; break;
            case 1U: application_motor_runtime.discovery.results[6].sub_version = 1U; break;
            case 2U: application_motor_runtime.discovery.results[6].observed_esc_id = 6U; break;
            case 3U: application_motor_runtime.discovery.results[6].observed_master_id = 22U; break;
            case 4U: application_motor_runtime.discovery.results[6].observed_control_mode = 1U; break;
            case 5U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = NAN; break;
            case 6U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 12.5F; break;
            case 7U: application_motor_runtime.discovery.results[6].ranges.position_max_rad = 100.0F; break;
            case 8U: application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 5.0F; break;
            case 9U: application_motor_runtime.discovery.results[6].ranges.torque_max_nm = 18.0F; break;
            case 10U: mapping_feedback(2088U, 700004U); break;
            case 11U: application_debug_ui.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; break;
            case 12U: application_low_speed_mapping.disabled_observed_us = 600000U; break;
            case 13U: application_low_speed_mapping.disabled_observed_us = 800000U; break;
            case 14U:
            case 15U:
            {
                uint8_t payload[8] = {7U, 0x80U, 0U, 0x80U, 7U, 0xFFU, 25U, 25U};
                if (scenario == 14U) { payload[6] = 50U; }
                else { payload[0] = 0x87U; }
                assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
                (void)aethor_app_receive_can_frame(&frame, 700004U);
                break;
            }
            default: break;
        }
        if (scenario < 16U)
        {
            assert(request("744 adrc restore_rated70_position motor=7", 700005U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
            assert(aethor_app_mapping_locked());
        }
        else
        {
            uint64_t now_us;
            float actual_source = scenario == 16U ? 512.0F : 1024.0F;
            application_motor_runtime.discovery.results[6].ranges.position_max_rad = actual_source;
            assert(request("745 adrc restore_rated70_position motor=7", 700005U, &output) == PROTOCOL_ENGINE_STATUS_OK);
            now_us = mapping_complete_register_transaction(0x15U, 700005U, actual_source, 512.0F, 0U);
            assert(request("746 adrc position_range_status", now_us, &output) == PROTOCOL_ENGINE_STATUS_OK);
            assert(strstr(output.messages[0].data, "state=10 reason=0 locked=0 target=512") != NULL);
        }
    }
    fixture_low_speed_mapping();
    assert(request("747 adrc wide_position motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)mapping_next_frame(600001U, &frame, &priority);
    (void)mapping_next_frame(700001U, &frame, &priority);
    assert(aethor_app_pop_emergency_can_frame(&frame));
    mapping_feedback(2048U, 700002U);
    /* PMAX512 satisfies both families' recovery endpoints; only the retained family may decide. */
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
    assert(request("748 adrc restore_rated70_position motor=7", 700003U, &output) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(aethor_app_mapping_locked());
}

/** @brief Runs deterministic integrated contracts without USB, motor motion or CAN hardware. */
int main(void)
{
    test_rated70_position_mapping_roundtrip();
    test_rated70_position_mapping_admission();
    test_rated70_position_mapping_failures();
    test_rated70_position_mapping_ack_and_old_feedback();
    test_rated70_position_mapping_recovery_family_and_health();
    test_position_mapping_entry_and_status();
    test_position_mapping_roundtrip();
    test_position_mapping_admission();
    test_position_mapping_failures();
    test_position_mapping_ack_drain_and_baseline();
    test_position_mapping_explicit_recovery_and_cross_lock();
    test_position_mapping_recovery_identity_and_feedback();
    test_position_status_identifies_shared_velocity_transaction();
    test_low_speed_mapping_entry();
    test_low_speed_mapping_roundtrip();
    test_low_speed_mapping_failures();
    test_low_speed_mapping_timing_and_recovery();
    test_low_speed_mapping_admission_and_local_stop();
    test_coordinate_pair_readonly_contract();
    test_coordinate_pair_waits_for_later_tick();
    test_coordinate_pair_wait_preserves_failures();
    test_coordinate_pair_admission();
    test_coordinate_pair_timeout_and_stop();
    test_coordinate_pair_rejections_and_owner_change();
    test_shaft_terminal_survives_release_transaction();
    test_retained_shaft_diagnostics_under_lcd_owner();
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
    test_release_retry_limits_without_feedback();
    test_release_retry_failure_remains_fail_closed();
    test_no_feedback_after_probe_times_out();
    test_neutral_probe_requires_actual_reply();
    test_acquire_disable_retry_retains_first_challenge();
    test_acquire_disable_retry_preserves_guards();
    test_acquire_diagnostics_keep_original_window_and_target();
    test_acquisition_diagnostics_maximum_fields_are_read_only();
    test_motor_diagnostic_wrappers_preserve_wire_contract();
    test_failed_probe_keeps_lcd_owner();
    test_lcd_stop_during_adrc();
    test_integrated_disabled_capture();
    test_guard_release_gate();
    test_watchdog_owner_gate();
    test_simple_owner_gate();
    test_capability_owner_gate();
    puts("ADRC_LCD_INTEGRATION_TESTS_PASSED");
    return 0;
}
