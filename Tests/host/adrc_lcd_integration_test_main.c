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

/** @brief Adds deterministic fresh disabled motor 7 feedback and verified MIT discovery. */
static void fixture_disabled_motor7(uint64_t timestamp_us)
{
    MotorJointFeedback feedback;
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
    test_pending_lcd_control_rejects_acquire();
    test_recent_lcd_can_rejects_acquire();
    test_lcd_default_and_safe_acquire();
    test_release_requires_new_feedback();
    test_lcd_stop_during_adrc();
    puts("ADRC_LCD_INTEGRATION_TESTS_PASSED");
    return 0;
}
