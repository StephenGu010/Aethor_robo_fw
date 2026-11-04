/** @file idle_position_test.c
 * @brief Reproduce mode-2 idle display without fabricating control feedback or hardware qualification.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Configure the discovered mode-2 driver with no standard feedback. */
static void position_fixture(void)
{
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U);
    fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
}

/** @brief Build a documented little-endian read-only register reply. */
static CanFrame position_reply(uint8_t rid, float value)
{
    CanFrame frame;
    uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
    payload[3] = rid;
    memcpy(&payload[4], &value, sizeof(value));
    assert(can_frame_init(&frame, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
    return frame;
}

/** @brief Observe the first mode-2 read and prove the full feedback cache stays invalid. */
static void test_position_display(void)
{
    CanFrame frame;
    CanTxPriority priority;
    DebugUiSnapshot snapshot;
    ProtocolOutputBatch output;
    position_fixture();
    assert(aethor_app_next_can_frame(1000000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[2] == 0x33U && frame.data[3] == 0x50U);
    assert(priority == CAN_TX_PRIORITY_FEEDBACK_QUERY);
    (void)aethor_app_next_can_frame(1000400U, &frame, &priority);
    assert(application_motor_runtime.idle_position_read.pending);
    application_motor_runtime.position_read.values[0] = 9.0F;
    application_motor_runtime.position_read.sample_us[0] = 77U;
    frame = position_reply(0x50U, 1.25F);
    assert(aethor_app_receive_can_frame(&frame, 1001000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_next_can_frame(1116000U, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[2] == 0x33U && frame.data[3] == 0x51U);
    frame = position_reply(0x51U, 0.25F);
    assert(aethor_app_receive_can_frame(&frame, 1117000U) == MOTOR_RUNTIME_STATUS_OK);
    assert(aethor_app_debug_ui_get_snapshot(1118000U, &snapshot));
    assert(snapshot.motors[6].register_seen_mask == 3U);
    assert(snapshot.motors[6].register_position[1] == 0.25F);
    assert(!snapshot.motors[6].feedback_valid && !snapshot.motors[6].feedback_seen);
    assert(application_motor_runtime.accepted_feedback_count == 0U);
    assert(application_motor_runtime.position_read.values[0] == 9.0F);
    assert(application_motor_runtime.position_read.sample_us[0] == 77U);
    assert(request("31 show motor 7", 1118000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "state=absent"));
    assert(strstr(output.messages[0].data, "reg51_raw=0.25 reg51_age_ms=1"));
    assert(request("32 show motor 6", 1118000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(!strstr(output.messages[0].data, "reg51_raw="));
    puts("MODE2_POSITION_DISPLAY_PASS lcd_usb_separate_from_control");
}

/** @brief STOP cancels only background work, rejects late replies, and never generates control frames. */
static void test_position_preemption(void)
{
    CanFrame frame;
    CanTxPriority priority;
    position_fixture();
    (void)aethor_app_next_can_frame(1000000U, &frame, &priority);
    assert(application_motor_runtime.idle_position_read.pending);
    application_debug_ui.mailbox.stop_intent_mask = 0x40U;
    (void)aethor_app_next_can_frame(1004000U, &frame, &priority);
    assert(!application_motor_runtime.idle_position_read.pending);
    frame = position_reply(0x50U, 7.0F);
    assert(aethor_app_receive_can_frame(&frame, 1005000U) != MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.idle_position_read.seen_mask == 0U);
    puts("MODE2_POSITION_PREEMPTION_PASS stop_and_late_reply");
}

/** @brief Keep missing identity and non-mode-2 drivers on the existing safe path. */
static void test_position_identity(void)
{
    CanFrame frame;
    CanTxPriority priority;
    position_fixture();
    application_motor_runtime.discovery.verified_joint_mask = 0U;
    (void)aethor_app_next_can_frame(1000000U, &frame, &priority);
    assert(!application_motor_runtime.idle_position_read.pending);
    puts("MODE2_POSITION_IDENTITY_PASS");
}

/** @brief Invalid replies and timeouts cannot publish samples; no-response polling remains bounded. */
static void test_position_invalid_and_timeout(void)
{
    CanFrame frame;
    CanTxPriority priority;
    unsigned created = 1U;
    uint64_t now_us;
    position_fixture();
    (void)aethor_app_next_can_frame(1000000U, &frame, &priority);
    frame = position_reply(0x51U, 1.0F);
    assert(aethor_app_receive_can_frame(&frame, 1001000U) != MOTOR_RUNTIME_STATUS_OK);
    frame = position_reply(0x50U, NAN);
    assert(aethor_app_receive_can_frame(&frame, 1002000U) != MOTOR_RUNTIME_STATUS_OK);
    frame = position_reply(0x50U, 1.0F); frame.identifier = 22U;
    assert(aethor_app_receive_can_frame(&frame, 1003000U) != MOTOR_RUNTIME_STATUS_OK);
    for (now_us = 1004000U; now_us < 2000000U; now_us += 4000U)
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY) ++created;
    assert(created == 2U);
    assert(application_motor_runtime.idle_position_read.seen_mask == 0U);
    assert(application_motor_runtime.accepted_feedback_count == 0U);
    puts("MODE2_POSITION_INVALID_PASS wrong_rid_master_nan bounded_timeouts");
}

/** @brief Every ownership or maintenance preemption cancels a pending background read before early returns. */
static void test_position_owner_preemption(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 6U; ++scenario) {
        CanFrame frame;
        CanTxPriority priority;
        position_fixture();
        (void)aethor_app_next_can_frame(1000000U, &frame, &priority);
        switch (scenario) {
            case 0: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ACQUIRING; break;
            case 1: application_persistence.locked = 1U; break;
            case 2: application_coordinate_pair.state = 1U; break;
            case 3: application_controller.enabled = 1U; break;
            case 4: application_motor_runtime.discovery_active = 1U; break;
            default: application_motor_runtime.discovery.results[6].observed_control_mode = 1U; break;
        }
        (void)aethor_app_next_can_frame(1004000U, &frame, &priority);
        assert(!application_motor_runtime.idle_position_read.pending);
        assert(application_motor_runtime.idle_position_read.not_before_us >= 1504000U);
    }
    puts("MODE2_POSITION_OWNERS_PASS six_preemption_paths");
}

/** @brief A historical idle reader cannot swallow duplicate replies during an explicit coordinate pair. */
static void test_coordinate_reservation(void)
{
    CanFrame frame;
    CanTxPriority priority;
    ProtocolOutputBatch output;
    memset(&application_persistence, 0, sizeof(application_persistence));
    fixture_coordinate_pair();
    application_motor_runtime.idle_position_read.joint_index = 6U;
    application_motor_runtime.idle_position_read.not_before_us = 50000U;
    assert(request("400 adrc coordinates motor=7", 100001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(application_motor_runtime.position_read_reserved);
    (void)aethor_app_next_can_frame(100002U, &frame, &priority);
    assert(coordinate_reply(0x50U, 0.25F, 100003U) == MOTOR_RUNTIME_STATUS_OK);
    assert(coordinate_reply(0x50U, 0.25F, 100004U) != MOTOR_RUNTIME_STATUS_OK);
    assert(application_motor_runtime.position_read.rejected_count == 1U);
    assert(application_motor_runtime.idle_position_read.rejected_count == 0U);
    (void)aethor_app_next_can_frame(100005U, &frame, &priority);
    assert(application_coordinate_pair.state == 3U && application_coordinate_pair.reason == 4U);
    assert(!application_motor_runtime.position_read_reserved);
    puts("MODE2_POSITION_RESERVATION_PASS duplicate_reply_remains_coordinate_error");
}

/** @brief Mode changes invalidate raw display samples and mapping waits for pending background reads. */
static void test_position_epoch(void)
{
    CanFrame frame;
    CanTxPriority priority;
    ProtocolOutputBatch output;
    position_fixture();
    assert(aethor_app_mapping_sources_idle(0U));
    (void)aethor_app_next_can_frame(1000000U, &frame, &priority);
    assert(!aethor_app_mapping_sources_idle(0U));
    frame = position_reply(0x50U, 0.5F);
    assert(aethor_app_receive_can_frame(&frame, 1001000U) == MOTOR_RUNTIME_STATUS_OK);
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    assert(request("480 show motor 7", 1001001U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output.messages[0].data, "reg50_raw=?"));
    (void)aethor_app_next_can_frame(1004000U, &frame, &priority);
    assert(application_motor_runtime.idle_position_read.seen_mask == 0U);
    puts("MODE2_POSITION_EPOCH_PASS mode_change_invalidates_raw_display");
}

/** @brief ProtocolTask runs after the control tick and can still reserve a foreground coordinate read. */
static void test_position_quiet_window(void)
{
    uint64_t now_us;
    unsigned reads = 0U;
    uint8_t admitted = 0U;
    CanFrame frame;
    CanTxPriority priority;
    ProtocolOutputBatch output;
    memset(&application_persistence, 0, sizeof(application_persistence));
    fixture_coordinate_pair();
    for (now_us = 100000U; now_us < 400000U; now_us += 4000U) {
        aethor_app_integrated_set_can_idle(1U, now_us);
        if (aethor_app_next_can_frame(now_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            uint8_t rid = frame.data[3];
            assert(frame.data[2] == 0x33U);
            ++reads;
            aethor_app_integrated_note_idle_feedback_activity();
            frame = position_reply(rid, 0.25F);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
        }
        /* A separate synthetic standard sample supplies the existing safety gate;
         * the register reply above must never supply it. */
        application_motor_runtime.bank.motors[6].feedback.timestamp_us = now_us;
        if (now_us % 20000U == 0U && now_us > 100000U &&
            request("600 adrc coordinates motor=7", now_us + 1U, &output) == PROTOCOL_ENGINE_STATUS_OK) {
            admitted = 1U;
            break;
        }
    }
    assert(reads >= 1U && admitted);
    puts("MODE2_POSITION_QUIET_PASS protocol_admission_after_control_tick");
}

/** @brief Run only offline fixtures. */
int main(void)
{
    test_position_display();
    test_position_preemption();
    test_position_identity();
    test_position_invalid_and_timeout();
    test_position_owner_preemption();
    test_coordinate_reservation();
    test_position_epoch();
    test_position_quiet_window();
    return 0;
}

