/** @file seven_motor_preset_tests.inc
 * @brief Synthetic per-ID integration checks for the user-confirmed S3519 preset.
 * Included after the shared facade fixtures; never used by firmware.
 */
#if (AETHOR_DEBUG_UI_S3519_PROFILE_MASK & 0x3FU)
/** @brief Verify independent discovery, output units, CAN routing and stale-profile invalidation. */
static void test_seven_motor_presets(void)
{
    unsigned joint_index, other_index, mode_index;
    for (joint_index=0U; joint_index<7U; ++joint_index) {
        for (mode_index=0U; mode_index<(AETHOR_DEBUG_UI_ALLOW_MIT ? 2U : 1U); ++mode_index) {
            DebugUiSnapshot snapshot;
            DebugUiRequest request;
            ProtocolCommand command;
            MotorFeedbackSnapshot feedback;
            CanFrame frame;
            uint32_t generation;
            uint8_t mask=(uint8_t)(1U<<joint_index);
            aethor_app_init(1U,77U);
            assert(aethor_app_debug_ui_get_snapshot(1000U,&snapshot));
            for (other_index=0U; other_index<7U; ++other_index) {
                assert(!snapshot.motors[other_index].profile.pos_valid);
                assert(!snapshot.motors[other_index].profile.mit_valid);
            }
            application_motor_runtime.discovery.results[joint_index].verified_fields_mask=MOTOR_DISCOVERY_ALL_FIELDS_MASK;
            application_motor_runtime.discovery.verified_joint_mask=mask;
            application_motor_runtime.discovery.results[joint_index].observed_control_mode=mode_index ? 1U : 2U;
            application_motor_runtime.discovery.results[joint_index].ranges.position_max_rad=12.5F;
            application_motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s=30.0F;
            application_motor_runtime.discovery.results[joint_index].ranges.torque_max_nm=10.0F;
            application_motor_runtime.discovery.results[joint_index].maximum_speed_rad_s=10.0F;
            assert(aethor_app_debug_ui_get_snapshot(1000U,&snapshot));
            for (other_index=0U; other_index<7U; ++other_index) {
                assert(snapshot.motors[other_index].profile.pos_valid==(other_index==joint_index));
                assert(snapshot.motors[other_index].profile.mit_valid==
                    (other_index==joint_index && AETHOR_DEBUG_UI_ALLOW_MIT));
            }
            assert(fabsf(snapshot.motors[joint_index].profile.position_max_rad-12.5F/19.2032F)<0.000001F);
            assert(snapshot.motors[joint_index].profile.max_speed_rad_s==10.0F);
            if (AETHOR_DEBUG_UI_ALLOW_MIT) {
                assert(fabsf(snapshot.motors[joint_index].profile.default_kp-80.0F)<0.0001F);
                assert(snapshot.motors[joint_index].profile.default_kd==0.2F);
            }
            application_debug_ui.authority=DEBUG_UI_AUTHORITY_LOCAL_ARMED;
            application_debug_ui.target_motor_id=(uint8_t)(joint_index+1U);
            request=test_request(mode_index ? DEBUG_UI_OPERATION_MIT_MOVE : DEBUG_UI_OPERATION_POS_MOVE,1000U);
            request.target_motor_id=(uint8_t)(joint_index+1U);
            request.reference_generation=snapshot.motors[joint_index].reference_generation;
            request.delta_rad=AETHOR_APP_DEG_TO_RAD;
            request.speed_rad_s=10.0F*AETHOR_APP_DEG_TO_RAD;
            request.kp=80.0F; request.kd=0.2F;
            aethor_app_debug_ui_build_command(&request,1000U,&command);
            assert(command.motor_mask==mask);
            assert(fabsf(command.values[joint_index]-19.2032F*AETHOR_APP_DEG_TO_RAD)<0.000001F);
            assert(command.speeds[joint_index]==request.speed_rad_s);
            assert(fabsf(command.mit_kp-80.0F/19.2032F)<0.00001F);
            for (other_index=0U; other_index<7U; ++other_index)
                if (other_index!=joint_index) assert(command.values[other_index]==0.0F && command.speeds[other_index]==0.0F);
            assert(s3519_pack_mode_command((uint8_t)(joint_index+1U),S3519_CONTROL_MODE_POSITION_VELOCITY,
                S3519_MODE_COMMAND_DISABLE,&frame)==S3519_CODEC_STATUS_OK);
            assert(frame.identifier==joint_index+1U && frame.data[7]==0xFDU);
            memset(&feedback,0,sizeof(feedback)); feedback.valid_joint_mask=mask;
            memset(&application_action,0,sizeof(application_action));
            application_action.command=command;
            application_action.target_position_rad[joint_index]=command.values[joint_index];
            application_action.target_speed_rad_s[joint_index]=command.speeds[joint_index];
            assert(!aethor_app_start_one_shot_motion(&feedback,1000U));
            assert(application_action.state==AETHOR_APP_ACTION_ONE_SHOT_MOVE_WAIT);
            if (mode_index) {
                uint64_t duration_us=application_action.motion_plan.duration_us;
                assert(duration_us>=187499U && duration_us<=187501U);
                assert(aethor_app_build_one_shot_reference(1000U+duration_us/2U)==MOTOR_RUNTIME_STATUS_OK);
                assert(application_action.frames.frames[0].identifier==joint_index+1U);
            } else assert(application_action.frames.frames[0].identifier==0x101U+joint_index);
            generation=snapshot.motors[joint_index].reference_generation;
            application_motor_runtime.discovery.results[joint_index].maximum_speed_rad_s=5.0F;
            assert(aethor_app_debug_ui_get_snapshot(1001U,&snapshot));
            assert(snapshot.motors[joint_index].profile.max_speed_rad_s==5.0F);
            assert(snapshot.motors[joint_index].reference_generation!=generation);
            application_motor_runtime.discovery.results[joint_index].verified_fields_mask=0U;
            assert(aethor_app_debug_ui_get_snapshot(1002U,&snapshot));
            assert(!snapshot.motors[joint_index].profile.pos_valid && !snapshot.motors[joint_index].profile.mit_valid);
        }
    }
    puts("SEVEN_MOTOR_PRESETS_PASS independent_discovery=1 absent_locked=1 output_units=1 single_target=1 can_routes=1 invalidation=1");
}
#endif
