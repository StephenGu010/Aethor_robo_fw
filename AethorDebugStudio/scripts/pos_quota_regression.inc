/** @file Synthetic ID7 POS protection regression included after the frozen adapter fixture. No hardware. */
/** @brief Drive the actual POS enable/motion/hold guards after the real policy token/reset route. */
static void pos_quota_stage_regression(void)
{
    static const AethorAppActionState stages[] = { AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT,
        AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT, AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT };
    unsigned stage_index, sign_index, scenario;
    for (stage_index = 0U; stage_index < 3U; ++stage_index) {
        for (sign_index = 0U; sign_index < 2U; ++sign_index) {
            for (scenario = 0U; scenario < 5U; ++scenario) {
                MotorFeedbackSnapshot snapshot;
                uint8_t failed = 0U;
                uint64_t now_us = 1004000ULL;
                torque_fixture(1U);
                adjustable_request(torque_query(1000001U, 1U), "6240", 1000002U, 1U);
                application_motor_runtime.discovery.results[6].observed_control_mode = 2U;
                application_debug_ui.profiles[6].pos_valid = 1U;
                application_debug_ui.pos_load_discovery = application_motor_runtime.discovery.results[6];
                application_debug_ui.pos_load_profile = aethor_app_pos_load_bounds(6U);
                memset(&application_action, 0, sizeof(application_action));
                application_action.command.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
                application_action.command.motor_mask = 0x40U;
                application_action.command.local_operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
                application_action.command.type = PROTOCOL_COMMAND_MOVE_ABSOLUTE_SELF_CONTAINED;
                application_action.command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
                application_action.command.reference_generation = application_debug_ui.reference_generation[6];
                application_action.state = stages[stage_index];
                application_action.last_pos_reference_timestamp_us = now_us;
                memset(&snapshot, 0, sizeof(snapshot));
                snapshot.valid_joint_mask = 0x40U;
                snapshot.joints[6].timestamp_us = now_us;
                snapshot.joints[6].driver_state = S3519_DRIVER_STATE_ENABLED;
                snapshot.joints[6].mos_temperature_c = snapshot.joints[6].rotor_temperature_c = 25.0F;
                snapshot.joints[6].torque_nm = sign_index ? -5.46F : 5.46F;
                if (scenario == 1U) snapshot.joints[6].torque_nm = sign_index ? -6.25F : 6.25F;
                if (scenario == 2U) snapshot.joints[6].mos_temperature_c = 55.1F;
                if (scenario == 3U) snapshot.joints[6].timestamp_us = now_us - 20001ULL;
                if (scenario == 4U) torque_envelope_init(&application_torque_envelope);
                (void)aethor_app_check_one_shot_motor_safety(&snapshot, now_us, &failed);
                if (scenario == 0U) {
                    assert(!failed && application_torque_envelope.authorized);
                    assert(application_action.state == stages[stage_index]);
                } else {
                    assert(failed && !application_torque_envelope.authorized);
                    assert(application_action.local_failure_reason == (scenario == 2U ? DEBUG_UI_REASON_TEMPERATURE_LIMIT :
                        scenario == 3U ? DEBUG_UI_REASON_STALE_FEEDBACK : DEBUG_UI_REASON_TORQUE_LIMIT));
                    assert(application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT);
                }
            }
        }
    }
    puts("POS_QUOTA_STAGES_PASS enable_move_hold_positive_negative_5p46_under_6p24_peak_temp_stale_boot_locked");
}

/** @brief Run the unchanged frozen adapter suite plus actual POS-stage protection assertions. */
int main(void)
{
    existing_quota_adapter_main();
    pos_quota_stage_regression();
    return 0;
}
