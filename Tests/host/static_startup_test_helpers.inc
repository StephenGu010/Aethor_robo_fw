/** @file static_startup_test_helpers.inc
 * @brief Offline production startup driver; hardware receipts and SOF tags are explicit synthetic inputs.
 */
/** @brief Encode one original selected feedback frame and route it through the real decode/order boundary. */
static void startup_test_feedback(uint8_t motor_id, uint8_t driver_state, uint64_t timestamp_us,
    const DebugUiStaticTxIdentity *identity)
{
    uint8_t joint_index = (uint8_t)(motor_id - 1U);
    CanFrame encoded, reply;
    const MotorJointFeedback *feedback = &application_motor_runtime.bank.motors[joint_index].feedback;
    assert(s3519_pack_mit(motor_id, &application_motor_runtime.discovery.results[joint_index].ranges,
        feedback->position_rad, feedback->velocity_rad_s, 0.0F, 0.0F, feedback->torque_nm, &encoded) == S3519_CODEC_STATUS_OK);
    memset(&reply, 0, sizeof(reply));
    reply.identifier = application_motor_runtime.configuration->joints[joint_index].master_id;
    reply.length = 8U;
    reply.data[0] = (uint8_t)((driver_state << 4U) | motor_id);
    reply.data[1] = encoded.data[0]; reply.data[2] = encoded.data[1];
    reply.data[3] = encoded.data[2]; reply.data[4] = (uint8_t)((encoded.data[3] & 0xF0U) | (encoded.data[6] & 0x0FU));
    reply.data[5] = encoded.data[7]; reply.data[6] = 30U; reply.data[7] = 31U;
    assert(aethor_app_receive_static_frame(&reply, timestamp_us, timestamp_us, identity, 101U, 1U) == MOTOR_RUNTIME_STATUS_OK);
}

/** @brief Drive every disabled PRELOAD before ENABLE, then immediate identical first HOLD for each selected axis. */
static void startup_test_finish(MotorFeedbackSnapshot *snapshot, uint64_t first_us, uint64_t hold_started_us)
{
    uint64_t timestamp_us = first_us;
    uint8_t count = 0U;
    while (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT && application_action.static_identity.phase) {
        DebugUiStaticTxIdentity identity;
        CanFrame frame;
        CanTxPriority priority;
        uint8_t final;
        assert(++count <= 21U);
        test_health(timestamp_us, true);
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) == MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(aethor_app_static_frame_identity(&frame, &identity));
        if (identity.phase == DEBUG_UI_STATIC_TX_ENABLE) {
            assert(application_action.static_preloaded_mask == application_action.command.motor_mask);
            assert(frame.data[7] == S3519_MODE_COMMAND_ENABLE);
        } else {
            assert(!memcmp(frame.data, application_action.static_frozen_frames[identity.motor_id - 1U].data, 8U));
        }
        assert(aethor_app_static_submitted(&identity, timestamp_us + 1ULL));
        aethor_app_static_transmitted(&identity, 1U, 100U, timestamp_us + 1ULL);
        if (identity.phase == DEBUG_UI_STATIC_TX_ENABLE) {
            assert(application_action.static_identity.phase == DEBUG_UI_STATIC_TX_FIRST_HOLD);
            timestamp_us += 10ULL;
            continue;
        }
        startup_test_feedback(identity.motor_id, identity.phase == DEBUG_UI_STATIC_TX_PRELOAD ?
            S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED, timestamp_us + 2ULL, &identity);
        assert(motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us + 3ULL,
            MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, snapshot) == MOTOR_RUNTIME_STATUS_OK);
        final = (uint8_t)(identity.phase == DEBUG_UI_STATIC_TX_FIRST_HOLD &&
            (application_action.static_first_hold_mask | (1U << (identity.motor_id - 1U))) == application_action.command.motor_mask);
        assert(!aethor_app_advance_one_shot_setup(snapshot, final ? hold_started_us : timestamp_us + 3ULL));
        timestamp_us += 10ULL;
    }
    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT);
}
