/** @file standby_recovery_test.c
 * @brief Offline late-24V recovery, priority and read-only traffic regression fixtures.
 */
#define main existing_integrated_main
#include "adrc_lcd_integration_test_main.c"
#undef main

/** @brief Reset all facade storage and reproduce completed boot discovery without motor power. */
static void recovery_fixture(void)
{
    memset(&application_persistence, 0, sizeof(application_persistence));
    aethor_app_init(1U, 1234U);
    application_motor_runtime.discovery_active = 0U;
    application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_FAILED;
    memset(&application_motor_runtime.discovery.results[6], 0, sizeof(MotorDiscoveryResult));
    application_motor_runtime.discovery.verified_joint_mask = 0U;
}

/** @brief Supply trusted fresh CAN-idle observations before asking the real scheduler for a frame. */
static MotorRuntimeStatus recovery_next(uint64_t now_us, CanFrame *frame)
{
    CanTxPriority priority;
    aethor_app_integrated_set_can_idle(1U, now_us - 4000U);
    aethor_app_integrated_set_can_idle(1U, now_us);
    return aethor_app_next_can_frame(now_us, frame, &priority);
}

/** @brief Decode the requested RID into the documented fixed driver identity and positive ranges. */
static CanFrame recovery_reply(const CanFrame *request_frame)
{
    CanFrame reply;
    uint8_t payload[8] = {7U, 0U, 0x33U, 0U, 0U, 0U, 0U, 0U};
    uint32_t raw = 1U;
    float value = 10.0F;
    payload[3] = request_frame->data[3];
    switch (payload[3]) {
        case S3519_REGISTER_MASTER_ID: raw = 23U; break;
        case S3519_REGISTER_ESC_ID: raw = 7U; break;
        case S3519_REGISTER_CONTROL_MODE: raw = 2U; break;
        case S3519_REGISTER_ACCELERATION:
        case S3519_REGISTER_DECELERATION:
        case S3519_REGISTER_MAXIMUM_SPEED:
        case S3519_REGISTER_POSITION_RANGE:
        case S3519_REGISTER_VELOCITY_RANGE:
        case S3519_REGISTER_TORQUE_RANGE:
            memcpy(&raw, &value, sizeof(raw)); break;
        default: break;
    }
    memcpy(&payload[4], &raw, sizeof(raw));
    assert(can_frame_init(&reply, 23U, payload, sizeof(payload)) == CAN_FRAME_STATUS_OK);
    return reply;
}

/** @brief A motor powered after failed boot discovery must become identified using only 13 read requests. */
static void test_late_power_recovers(void)
{
    CanFrame frame;
    uint64_t now_us;
    unsigned reads = 0U;
    recovery_fixture();
    for (now_us = 1000000U; now_us < 5000000U; now_us += 4000U) {
        if (recovery_next(now_us, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[0] == 7U && frame.data[2] == 0x33U);
            if (frame.data[3] == 0x50U || frame.data[3] == 0x51U) break;
            ++reads;
            frame = recovery_reply(&frame);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
        }
    }
    assert(reads == MOTOR_DISCOVERY_REGISTER_COUNT);
    assert(application_motor_runtime.discovery.verified_joint_mask == 0x40U);
    assert(application_motor_runtime.discovery.results[6].observed_control_mode == 2U);
    assert(application_motor_runtime.accepted_feedback_count == 0U);
    assert(!application_motor_runtime.bank.motors[6].feedback_valid);
    frame = recovery_reply(&frame);
    assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
    { DebugUiSnapshot snapshot;
      uint32_t reference;
      assert(aethor_app_debug_ui_get_snapshot(now_us + 1001U, &snapshot));
      reference = snapshot.motors[6].reference_generation;
      for (now_us += 4000U; now_us < 15000000U; now_us += 4000U) {
          if (recovery_next(now_us, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
              assert(frame.data[2] == 0x33U && (frame.data[3] == 0x50U || frame.data[3] == 0x51U));
              frame = recovery_reply(&frame);
              assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
          }
          assert(aethor_app_debug_ui_get_snapshot(now_us + 1001U, &snapshot));
          assert(snapshot.motors[6].reference_generation == reference);
      }
    }
    puts("STANDBY_RECOVERY_LATEPOWER_PASS read_only_13_no_feedback_qualification stable_12_seconds");
}

/** @brief A queued foreground discovery replaces the background lease before the next CAN call. */
static void test_recovery_foreground_handoff(void)
{
    ProtocolOutputBatch output;
    CanFrame frame;
    recovery_fixture();
    assert(recovery_next(3000000U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(request("1 bench init 7", 3001000U, &output) == PROTOCOL_ENGINE_STATUS_OK);
    (void)aethor_app_service(3002000U);
    assert(!application_standby_recovery.owned);
    assert(application_motor_runtime.discovery_active);
    (void)recovery_next(3004000U, &frame);
    assert(application_motor_runtime.discovery_active);
    puts("STANDBY_RECOVERY_HANDOFF_PASS queued_foreground_keeps_its_discovery");
}

/** @brief Five seconds of authorized foreground traffic is a sampling pause, never evidence of power loss. */
static void test_recovery_busy_pause(void)
{
    CanFrame frame;
    DebugUiSnapshot snapshot;
    uint64_t now_us;
    uint32_t reference;
    recovery_fixture(); fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    assert(aethor_app_debug_ui_get_snapshot(1000000U, &snapshot));
    reference = snapshot.motors[6].reference_generation;
    (void)recovery_next(1000000U, &frame);
    application_remote_ui.owns = 1U;
    for (now_us = 1004000U; now_us < 7000000U; now_us += 4000U)
        (void)recovery_next(now_us, &frame);
    application_remote_ui.owns = 0U;
    (void)recovery_next(7000000U, &frame);
    assert(!application_standby_recovery.owned);
    assert(aethor_app_debug_ui_get_snapshot(7001000U, &snapshot));
    assert(snapshot.motors[6].reference_generation == reference);
    puts("STANDBY_RECOVERY_PAUSE_PASS foreground_pause_retains_reference");
}

/** @brief A foreground discovery completion releases outage reference suppression and permits later real mode changes. */
static void test_recovery_foreground_reference_resume(void)
{
    CanFrame frame;
    DebugUiSnapshot snapshot;
    uint64_t now_us;
    uint32_t reference;
    recovery_fixture(); fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    assert(aethor_app_debug_ui_get_snapshot(1000000U, &snapshot));
    for (now_us = 1000000U; now_us < 6000000U; now_us += 4000U) {
        (void)recovery_next(now_us, &frame);
        if (application_standby_recovery.owned) break;
    }
    assert(application_standby_recovery.lost_reference);
    application_remote_ui.owns = 1U;
    (void)recovery_next(now_us + 4000U, &frame);
    assert(!application_standby_recovery.owned);
    /* The real foreground owner may complete its own discovery after this cancellation. */
    fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    (void)recovery_next(now_us + 8000U, &frame);
    assert(!application_standby_recovery.lost_reference);
    assert(aethor_app_debug_ui_get_snapshot(now_us + 8001U, &snapshot));
    reference = snapshot.motors[6].reference_generation;
    application_motor_runtime.discovery.results[6].observed_control_mode = 1U;
    assert(aethor_app_debug_ui_get_snapshot(now_us + 9000U, &snapshot));
    assert(snapshot.motors[6].reference_generation > reference);
    puts("STANDBY_RECOVERY_REFERENCE_RESUME_PASS foreground_identity_no_permanent_suppression");
}

/** @brief Every foreground and safety condition prevents a new recovery pass. */
static void test_recovery_gates(void)
{
    unsigned scenario;
    for (scenario = 0U; scenario < 23U; ++scenario) {
        CanFrame frame;
        recovery_fixture();
        switch (scenario) {
            case 0: application_action.state = AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT; break;
            case 1: application_protocol_engine.command_write_sequence++; break;
            case 2: application_protocol_engine.stop_write_sequence++; break;
            case 3: application_debug_ui.mailbox.stop_intent_mask = 0x40U; break;
            case 4: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ACQUIRING; break;
            case 5: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_RELEASING; break;
            case 6: application_adrc_lcd_ownership.state = ADRC_LCD_OWNER_ADRC; break;
            case 7: application_emergency_disable_batch.count = 1U; break;
            case 8: application_action.control_group_ready = 1U; break;
            case 9: application_controller.enabled = 1U; break;
            case 10: application_controller.fault = ARM_FAULT_TRANSPORT; break;
            case 11: application_integrated_diag_requested_us = 2999000U; break;
            case 12: application_debug_ui.unconfirmed_disable_mask = 0x40U; break;
            case 13: application_persistence.locked = 1U; break;
            case 14: application_remote_ui.owns = 1U; break;
            case 15: application_remote_ui.active = 1U; break;
            case 16: application_coordinate_pair.state = 1U; break;
            case 17: application_motor_runtime.position_read_reserved = 1U; break;
            case 18: application_transport_fault_write_sequence++; break;
            case 19: application_protocol_engine.result_write_sequence++; break;
            case 20: application_debug_ui.pos_disable_pending = 1U; break;
            case 21: application_debug_ui.single_static_transport_mask = 0x40U; break;
            default: application_controller.moving = 1U; break;
        }
        (void)recovery_next(3000000U, &frame);
        assert(!application_standby_recovery.owned);
    }
    recovery_fixture();
    { CanFrame frame; CanTxPriority priority;
      (void)aethor_app_next_can_frame(3000000U, &frame, &priority);
      assert(!application_standby_recovery.owned); }
    puts("STANDBY_RECOVERY_GATES_PASS cases=24 foreground_remote_stop_maintenance_bus");
}

/** @brief STOP takeover aborts this read-only transaction and rejects its delayed parameter response. */
static void test_recovery_preemption(void)
{
    CanFrame frame, late_reply;
    recovery_fixture();
    assert(recovery_next(3000000U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    late_reply = recovery_reply(&frame);
    application_debug_ui.mailbox.stop_intent_mask = 0x40U;
    (void)recovery_next(3004000U, &frame);
    assert(!application_standby_recovery.owned && !application_motor_runtime.discovery_active);
    assert(!application_motor_runtime.discovery.results[6].verified_fields_mask);
    (void)aethor_app_receive_can_frame(&late_reply, 3005000U);
    assert(application_motor_runtime.accepted_parameter_response_count == 0U);
    assert(!application_motor_runtime.discovery.verified_joint_mask);
    puts("STANDBY_RECOVERY_PREEMPT_PASS late_reply_no_qualification");
}

/** @brief Arbitrarily long no-response operation has a closed-form traffic bound and failure cooldown. */
static void test_recovery_bound(void)
{
    CanFrame frame;
    uint64_t now_us, previous_round_us = 0U;
    unsigned reads = 0U, rounds = 0U;
    recovery_fixture();
    for (now_us = 1000000U; now_us < 61000000U; now_us += 4000U) {
        uint8_t was_owned = application_standby_recovery.owned;
        if (recovery_next(now_us, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            assert(frame.identifier == 0x7FFU && frame.length == 4U && frame.data[0] == 7U && frame.data[2] == 0x33U);
            ++reads;
        }
        if (!was_owned && application_standby_recovery.owned) {
            assert(!previous_round_us || now_us - previous_round_us >= MOTOR_STANDBY_RECOVERY_COOLDOWN_US);
            previous_round_us = now_us; ++rounds;
        }
    }
    assert(rounds >= 20U && reads == rounds * MOTOR_DISCOVERY_MAX_ATTEMPTS);
    assert(reads <= MOTOR_DISCOVERY_MAX_ATTEMPTS * (1U + 60000000U / MOTOR_STANDBY_RECOVERY_COOLDOWN_US));
    assert(!application_motor_runtime.accepted_feedback_count && !application_motor_runtime.discovery.verified_joint_mask);
    puts("STANDBY_RECOVERY_BOUND_PASS 60_seconds_four_reads_per_failed_round");
}

/** @brief A round blocked by CAN traffic retires at two seconds and cannot resume its old request. */
static void test_recovery_round_deadline(void)
{
    CanFrame frame;
    CanTxPriority priority;
    uint64_t now_us;
    recovery_fixture();
    assert(recovery_next(3000000U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    for (now_us = 3004000U; now_us < 5000000U; now_us += 4000U) {
        aethor_app_integrated_set_can_idle(0U, now_us);
        assert(aethor_app_next_can_frame(now_us, &frame, &priority) != MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(application_standby_recovery.owned);
    }
    aethor_app_integrated_set_can_idle(0U, 5000000U);
    (void)aethor_app_next_can_frame(5000000U, &frame, &priority);
    assert(!application_standby_recovery.owned && !application_motor_runtime.discovery_active);
    assert(application_standby_recovery.next_attempt_us == 7000000U);
    assert(!application_motor_runtime.discovery.verified_joint_mask);
    puts("STANDBY_RECOVERY_DEADLINE_PASS bus_busy_round_hard_limit_2_seconds");
}

/** @brief A monotonic rollback and the uint64 boundary cannot trigger retry storms or wrapped deadlines. */
static void test_recovery_clock(void)
{
    CanFrame frame;
    recovery_fixture();
    assert(recovery_next(3000000U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    (void)recovery_next(1000000U, &frame);
    assert(!application_standby_recovery.owned);
    assert(application_standby_recovery.next_attempt_us == 3000000U);
    (void)recovery_next(2999000U, &frame);
    assert(!application_standby_recovery.owned);
    assert(recovery_next(3000000U, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY);
    recovery_fixture();
    (void)recovery_next(UINT64_MAX - 1000U, &frame);
    assert(!application_standby_recovery.owned);
    assert(aethor_app_standby_deadline(UINT64_MAX - 1000U, 2000000U) == UINT64_MAX);
    puts("STANDBY_RECOVERY_CLOCK_PASS rollback_uint64_no_burst");
}

/** @brief Healthy mode-2 register samples suppress rediscovery and retain the coordinate generation. */
static void test_recovery_stable_and_outage(void)
{
    CanFrame frame;
    DebugUiSnapshot snapshot;
    uint64_t now_us;
    uint32_t reference;
    unsigned discoveries = 0U;
    recovery_fixture(); fixture_discovered_motor7();
    application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
    application_motor_runtime.discovery.results[6].maximum_speed_rad_s = 10.0F;
    assert(aethor_app_debug_ui_get_snapshot(1000000U, &snapshot));
    reference = snapshot.motors[6].reference_generation;
    for (now_us = 1000000U; now_us < 12000000U; now_us += 4000U) {
        if (recovery_next(now_us, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            assert(frame.data[2] == 0x33U);
            if (frame.data[3] != 0x50U && frame.data[3] != 0x51U) ++discoveries;
            frame = recovery_reply(&frame);
            assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
        }
        assert(aethor_app_debug_ui_get_snapshot(now_us + 1001U, &snapshot));
        assert(snapshot.motors[6].reference_generation == reference);
    }
    assert(!discoveries);
    /* Only 0x51 continues: that raw constant cannot stand in for real 0x50 position continuity. */
    for (; now_us < 20000000U; now_us += 4000U) {
        if (recovery_next(now_us, &frame) == MOTOR_RUNTIME_STATUS_FRAME_READY) {
            if (frame.data[3] != 0x50U && frame.data[3] != 0x51U) { ++discoveries; break; }
            if (frame.data[3] == 0x51U) {
                frame = recovery_reply(&frame);
                assert(aethor_app_receive_can_frame(&frame, now_us + 1000U) == MOTOR_RUNTIME_STATUS_OK);
            }
        }
    }
    assert(discoveries == 1U);
    assert(application_debug_ui.reference_generation[6] == reference + 1U);
    /* Repeated failed passes cannot repeatedly erase a retained client zero. */
    for (; now_us < 25000000U; now_us += 4000U) {
        (void)recovery_next(now_us, &frame);
        assert(aethor_app_debug_ui_get_snapshot(now_us + 1001U, &snapshot));
        assert(snapshot.motors[6].reference_generation == reference + 1U);
    }
    assert(!application_motor_runtime.discovery.verified_joint_mask);
    puts("STANDBY_RECOVERY_STABLE_PASS position_only_silence_long_outage_invalidates_once");
}

/** @brief Run only deterministic offline fixtures; no hardware or controller execution occurs. */
int main(void)
{
    test_late_power_recovers();
    test_recovery_foreground_handoff();
    test_recovery_busy_pause();
    test_recovery_foreground_reference_resume();
    test_recovery_gates();
    test_recovery_preemption();
    test_recovery_bound();
    test_recovery_round_deadline();
    test_recovery_clock();
    test_recovery_stable_and_outage();
    return 0;
}
