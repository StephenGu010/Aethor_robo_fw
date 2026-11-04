/** @file idle_feedback_test.c
 * @brief Exercise real idle CAN scheduling and shared LCD/USB feedback with synthetic replies only.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Simulate power-on static storage; maintenance locks intentionally survive facade re-init. */
static void idle_fixture(void)
{
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U);
    fixture_discovered_motor7();
}

/** @brief Check the exact vendor read-only packet; no control opcode can pass. */
static void expect_query(const CanFrame *frame, CanTxPriority priority, uint8_t motor_id)
{
    static const uint8_t suffix[6] = {0xCCU, 0U, 0U, 0U, 0U, 0U};
    assert(frame->identifier == 0x7FFU && frame->length == 8U);
    assert(frame->data[0] == motor_id && frame->data[1] == 0U);
    assert(memcmp(&frame->data[2], suffix, sizeof(suffix)) == 0);
    assert(priority == CAN_TX_PRIORITY_FEEDBACK_QUERY);
}

/** @brief A motion-capable LCD must acquire feedback even with no browser and no local lease. */
static void test_idle_query_and_freshness(void)
{
    CanFrame frame;
    CanTxPriority priority;
    DebugUiSnapshot snapshot;
    ProtocolOutputBatch output;
    const uint8_t reply[8] = {7U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 26U};
    idle_fixture();
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    expect_query(&frame, priority, 7U);
    assert(aethor_app_debug_ui_get_snapshot(1000001U, &snapshot));
    assert(!snapshot.motors[6].feedback_valid);
    assert(can_frame_init(&frame, 23U, reply, sizeof(reply)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, 1001000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_get_snapshot(1001001U, &snapshot));
    assert(snapshot.motors[6].feedback_valid);
    assert(request("1 show motor 7", 1001001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=disabled") != NULL);
    assert(strstr(output.messages[0].data, "age_ms=0") != NULL);
    assert(aethor_app_debug_ui_get_snapshot(1101001U, &snapshot));
    assert(!snapshot.motors[6].feedback_valid);
    assert(application_motor_runtime.bank.motors[6].feedback.timestamp_us == 1001000U);
    puts("IDLE_FEEDBACK_SHARED_CACHE_PASS real_rx_lcd_usb stale_threshold_unchanged");
}

/** @brief All existing owners, queues and safety work preempt background monitoring. */
static void test_preemption(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 14U; ++scenario)
    {
        CanFrame frame;
        CanTxPriority priority;
        idle_fixture();
        switch (scenario)
        {
            case 0: application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT; break;
            case 1: application_protocol_engine.stop_write_sequence++; break;
            case 2: application_protocol_engine.command_write_sequence++; break;
            case 3: application_debug_ui.mailbox.stop_intent_mask = 0x40U; break;
            case 4: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ACQUIRING; break;
            case 5: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_RELEASING; break;
            case 6: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC; break;
            case 7: application_emergency_disable_batch.count = 1U; break;
            case 8: application_action.control_group_ready = 1U; break;
            case 9: application_controller.enabled = 1U; break;
            case 10: application_controller.fault = ARM_FAULT_TRANSPORT; break;
            case 11: application_integrated_diag_requested_us = 999000U; break;
            case 12: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
            case 13: application_persistence.locked = 1U; break;
        }
        if (aethor_app_next_can_frame(1000000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        {
            /* Other owners may legitimately emit discovery/control; background CC must yield. */
            if (frame.identifier == 0x7FFU && frame.data[2] == 0xCCU)
            {
                /* Existing ADRC owner has its own 4-byte query; preserve that path. */
                assert((scenario == 5U || scenario == 6U) && frame.length == 4U &&
                    priority == CAN_TX_PRIORITY_PARAMETER);
            }
        }
    }
    puts("IDLE_FEEDBACK_PREEMPTION_PASS cases=14");
}

/** @brief Missing replies never create samples; repeated failures back off, then recover on real RX. */
static void test_no_reply_backoff(void)
{
    uint64_t now_us;
    unsigned query_count = 0U;
    CanFrame frame;
    CanTxPriority priority;
    idle_fixture();
    for (now_us = 1000000U; now_us < 2000000U; now_us += 1000U)
    {
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        { expect_query(&frame, priority, 7U); ++query_count; }
    }
    assert(query_count == 3U);
    assert(application_motor_runtime.bank.motors[6].feedback_generation == 0U);
    fixture_disabled_motor7(2000000U);
    assert(aethor_app_next_can_frame(2000001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    expect_query(&frame, priority, 7U);
    puts("IDLE_FEEDBACK_NO_REPLY_PASS three_attempts_then_backoff no_synthetic_samples");
}

/** @brief All seven identities receive fair 4ms-task scheduling; duplicate reads cannot manufacture freshness. */
static void test_seven_axis_fairness(void)
{
    CanFrame frame;
    CanTxPriority priority;
    uint64_t now_us, last_query_us[ARM_JOINT_COUNT] = {0U};
    unsigned counts[ARM_JOINT_COUNT] = {0U};
    uint8_t joint_index;
    idle_fixture();
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        *discovery = application_motor_runtime.discovery.results[6];
        discovery->observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
        discovery->observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
    }
    application_motor_runtime.discovery.verified_joint_mask = 0x7FU;
    for (now_us = 1000000U; now_us < 2000000U; now_us += 4000U)
    {
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        {
            uint8_t reply[8] = {0U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 26U};
            joint_index = (uint8_t)(frame.data[0] - 1U);
            assert(joint_index < ARM_JOINT_COUNT);
            expect_query(&frame, priority, (uint8_t)(joint_index + 1U));
            if (last_query_us[joint_index])
            { assert(now_us - last_query_us[joint_index] <= 70000U); }
            last_query_us[joint_index] = now_us;
            ++counts[joint_index];
            reply[0] = (uint8_t)(joint_index + 1U);
            assert(can_frame_init(&frame, application_motor_runtime.configuration->joints[joint_index].master_id,
                reply, sizeof(reply)) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
        }
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    { assert(counts[joint_index] >= 14U); }
    puts("IDLE_FEEDBACK_SEVEN_AXIS_PASS task_period_4ms no_axis_starvation");
}

/** @brief Missing identity and malformed/unrelated frames cannot restore feedback freshness. */
static void test_identity_and_preemption_resume(void)
{
    CanFrame frame;
    CanTxPriority priority;
    DebugUiSnapshot snapshot;
    const uint8_t wrong_reply[8] = {6U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 26U};
    idle_fixture();
    application_motor_runtime.discovery.results[6].observed_esc_id = 6U;
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    fixture_discovered_motor7();
    assert(aethor_app_next_can_frame(1010000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(can_frame_init(&frame, 23U, wrong_reply, sizeof(wrong_reply)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, 1011000U) != MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_get_snapshot(1011001U, &snapshot));
    assert(!snapshot.motors[6].feedback_valid);
    application_protocol_engine.stop_write_sequence++;
    assert(aethor_app_next_can_frame(1200000U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    application_protocol_engine.stop_read_sequence++;
    assert(aethor_app_next_can_frame(1200001U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_next_can_frame(1220000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    expect_query(&frame, priority, 7U);
    puts("IDLE_FEEDBACK_IDENTITY_RESUME_PASS bad_identity_bad_rx_stop_gap");
}

/** @brief A retained completed probe is historical evidence, not permanent ownership of the bus. */
static void test_monitor_resumes_after_probe(void)
{
    CanFrame frame;
    CanTxPriority priority;
    idle_fixture();
    application_integrated_diag_requested_us = 1000000U;
    application_integrated_probe_transmitted = 1U;
    assert(aethor_app_next_can_frame(1050000U, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_next_can_frame(1100001U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    expect_query(&frame, priority, 7U);
    aethor_app_integrated_note_idle_feedback_activity();
    assert(!application_integrated_probe_failed && application_integrated_probe_transmitted);
    assert(application_integrated_diag_requested_us == 1000000U);
    puts("IDLE_FEEDBACK_PROBE_RESUME_PASS historical_probe_does_not_lock_monitor");
}

/** @brief Mirror ControlTask's idle sample/send order; ProtocolTask must see a retained quiet window. */
static void test_protocol_quiet_window(void)
{
    uint64_t now_us;
    uint8_t joint_index;
    unsigned quiet_observations = 0U;
    CanFrame frame;
    CanTxPriority priority;
    idle_fixture();
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        *discovery = application_motor_runtime.discovery.results[6];
        discovery->observed_esc_id = application_motor_runtime.configuration->joints[joint_index].esc_id;
        discovery->observed_master_id = application_motor_runtime.configuration->joints[joint_index].master_id;
    }
    application_motor_runtime.discovery.verified_joint_mask = 0x7FU;
    for (now_us = 1000000U; now_us < 1200000U; now_us += 4000U)
    {
        aethor_app_integrated_set_can_idle(1U, now_us);
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY)
        {
            uint8_t reply[8] = {0U, 0x80U, 0U, 0x80U, 0x08U, 0U, 25U, 26U};
            joint_index = (uint8_t)(frame.data[0] - 1U);
            reply[0] = frame.data[0];
            aethor_app_integrated_note_idle_feedback_activity();
            assert(can_frame_init(&frame, application_motor_runtime.configuration->joints[joint_index].master_id,
                reply, sizeof(reply)) == CAN_FRAME_STATUS_OK);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
        }
        if (application_integrated_can_idle) { ++quiet_observations; }
    }
    assert(quiet_observations >= 3U);
    puts("IDLE_FEEDBACK_QUIET_WINDOW_PASS protocol_can_observe_idle_after_control_task");
}

/** @brief A discovery timeout on another axis must not suppress monitoring of a verified axis. */
static void test_partial_discovery_failure(void)
{
    CanFrame frame;
    CanTxPriority priority;
    idle_fixture();
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_FAILED;
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    expect_query(&frame, priority, 7U);
    puts("IDLE_FEEDBACK_PARTIAL_DISCOVERY_PASS verified_motor_survives_unrelated_failure");
}

/** @brief Run offline regression cases without opening any serial port or hardware interface. */
int main(int argc, char **argv)
{
    if (argc > 1 && strcmp(argv[1], "quiet") == 0) { test_protocol_quiet_window(); return 0; }
    test_idle_query_and_freshness();
    test_preemption();
    test_no_reply_backoff();
    test_seven_axis_fairness();
    test_identity_and_preemption_resume();
    test_monitor_resumes_after_probe();
    test_protocol_quiet_window();
    test_partial_discovery_failure();
    return 0;
}
