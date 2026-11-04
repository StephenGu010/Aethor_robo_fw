/** @file idle_engineering_test.c
 * @brief Offline evidence for engineering telemetry, distinct CAN sample timing and invalidation.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Build a mode-2 fixture with verified identity and no standard feedback. */
static void position_fixture(void)
{
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U);
    fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
}

/** @brief Encode a matching read reply without sending any hardware command. */
static CanFrame position_reply(uint8_t rid, float value)
{
    CanFrame frame;
    uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
    payload[3] = rid;
    memcpy(&payload[4], &value, sizeof(value));
    assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
    return frame;
}

/** @brief Deliver one pending register response through the production runtime. */
static void engineering_sample(uint64_t sent_us, uint8_t rid, float value)
{
    CanFrame frame;
    assert(motor_runtime_next_idle_position_read(&application_motor_runtime, 6U,
        sent_us, 1U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[3] == rid);
    frame = position_reply(rid, value);
    assert(aethor_app_receive_can_frame(&frame, sent_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief A repeated USB read or a different register must not fabricate a derivative. */
int main(void)
{
    ProtocolOutputBatch output;
    MotorRegisterPosition *reader;
    position_fixture();
    reader = &application_motor_runtime.idle_position_read;
    engineering_sample(1000000U, 0x50U, 1.0F);
    assert(!reader->estimated_velocity_valid);
    engineering_sample(1116000U, 0x51U, 90.0F);
    assert(!reader->estimated_velocity_valid);
    engineering_sample(1232000U, 0x50U, 1.232F);
    assert(reader->estimated_velocity_valid);
    assert(fabsf(reader->estimated_native_rad_s - 1.0F) < 0.00001F);
    assert(request("900 show motor 7", 1234000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "pm_contract=1 pm_vel="));
    assert(strstr(output.messages[0].data, "state=absent"));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    assert(application_motor_runtime.accepted_feedback_count == 0U);
    engineering_sample(1348000U, 0x51U, -90.0F);
    assert(fabsf(reader->estimated_native_rad_s - 1.0F) < 0.00001F);
    engineering_sample(2000000U, 0x50U, 2.0F);
    assert(!reader->estimated_velocity_valid); /* Gap >500ms breaks differentiation. */
    aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_REMOTE);
    assert(!reader->estimated_velocity_valid && reader->seen_mask == 0U);
    engineering_sample(2116000U, 0x51U, 0.0F);
    engineering_sample(2232000U, 0x50U, 2.232F);
    assert(!reader->estimated_velocity_valid); /* First native sample in the new epoch. */
    engineering_sample(2348000U, 0x51U, 0.0F);
    engineering_sample(2464000U, 0x50U, 2.464F);
    assert(reader->estimated_velocity_valid);
    engineering_sample(2580000U, 0x51U, 0.0F);
    engineering_sample(2696000U, 0x50U, 10000.0F);
    assert(!reader->estimated_velocity_valid); /* No fabricated unwrap across a discontinuity. */
    reader->estimated_velocity_valid = 1U;
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    aethor_app_idle_position_preempt(2698000U);
    assert(!reader->estimated_velocity_valid && reader->seen_mask == 0U);
    assert(request("901 show motor 7", 2698000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "pm_contract=0 pm_vel=?"));
    position_fixture();
    fixture_disabled_motor7(1U);
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    application_motor_runtime.discovery.results[6].ranges.position_max_rad = 512.0F;
    application_motor_runtime.discovery.results[6].ranges.velocity_max_rad_s = 50.0F;
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    application_motor_runtime.bank.motors[6].feedback.position_rad = -512.0F;
    application_motor_runtime.bank.motors[6].feedback.velocity_rad_s = -50.0F;
    application_motor_runtime.bank.motors[6].feedback.torque_nm = -18.0F;
    application_motor_runtime.bank.motors[6].feedback.mos_temperature_c = 125.0F;
    application_motor_runtime.bank.motors[6].feedback.rotor_temperature_c = 125.0F;
    application_motor_runtime.bank.motors[6].feedback.fault_flags = UINT32_MAX;
    reader->joint_index = 6U; reader->seen_mask = 3U;
    reader->values[0] = -3.4028234e38F; reader->values[1] = 3.4028234e38F;
    reader->sample_us[0] = reader->sample_us[1] = 1U;
    application_protocol_engine.boot_id = UINT32_MAX;
    application_debug_ui.epoch = UINT32_MAX;
    assert(request("4294967295 show motor 7", 4294967296000ULL, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "pm_boot=4294967295 pm_epoch=4294967295"));
    assert(output.messages[0].length < PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    printf("MODE2_ENGINEERING_CAPACITY_PASS bytes=%u capacity=%u\n",
        (unsigned)output.messages[0].length, (unsigned)PROTOCOL_ENGINE_MESSAGE_CAPACITY);
    puts("MODE2_ENGINEERING_PASS distinct_rx_clock gap_preemption no_control_feedback");
    return 0;
}
