/** @file debug_ui_static_startup_app.inc
 * @brief Frozen-target startup barriers sharing the original setup deadline and existing safety path.
 * Included by aethor_app.c. PRELOAD is a handover measure; disabled feedback cannot prove MIT acceptance.
 */
/* Engineering defaults: fixed 1 s window includes a 50 ms final proof budget. */
#define AETHOR_STATIC_TAKEOVER_WINDOW_US 1000000ULL
#define AETHOR_STATIC_TAKEOVER_STEP_US 50000ULL
#define AETHOR_STATIC_TAKEOVER_STEP_COUNT 19U
#define AETHOR_STATIC_TAKEOVER_REFRESH_US 4000ULL
#define AETHOR_STATIC_TAKEOVER_MAX_LATENESS_US 5000ULL

/** @brief Publish exactly one next phase without recapturing target/gains or extending the setup budget. */
static void aethor_app_static_prepare_phase(uint8_t phase, uint8_t joint_index, uint64_t timestamp_us)
{
    DebugUiStaticTxIdentity *identity = &application_action.static_identity;
    identity->epoch = application_action.command.session_id;
    identity->request_id = application_action.command.request_id;
    ++identity->sequence;
    if (!identity->sequence) ++identity->sequence;
    identity->motor_id = (uint8_t)(joint_index + 1U);
    identity->phase = phase;
    application_action.single_static_tx_submitted = 0U;
    application_action.single_static_tx_confirmed = 0U;
    application_action.single_static_tx_failed = 0U;
    application_action.single_static_ordered_generation = 0U;
    application_action.frames.count = 1U;
    application_action.frame_read_index = 0U;
    application_action.priority = CAN_TX_PRIORITY_JOINT_CONTROL;
    application_action.frames.frames[0] = application_action.static_frozen_frames[joint_index];
    if (phase == DEBUG_UI_STATIC_TX_TAKEOVER) {
        float zero_speed[ARM_JOINT_COUNT] = {0.0F};
        MotorRuntimeStatus status = motor_runtime_build_mit_subset(&application_motor_runtime,
            application_action.command.motor_mask, application_action.target_position_rad, zero_speed,
            application_action.command.mit_kp, application_action.command.mit_kd,
            application_debug_ui.static_load.requested_torque_ff_nm, &application_action.frames);
        if (status != MOTOR_RUNTIME_STATUS_OK || application_action.frames.count != 1U) {
            (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, (uint8_t)(joint_index + 1U), timestamp_us);
            return;
        }
    }
    if (phase == DEBUG_UI_STATIC_TX_ENABLE) {
        MotorRuntimeStatus runtime_status = motor_runtime_build_mode_command_batch(&application_motor_runtime,
            aethor_app_is_pos_load() ? S3519_CONTROL_MODE_POSITION_VELOCITY : S3519_CONTROL_MODE_MIT,
            S3519_MODE_COMMAND_ENABLE, (uint8_t)(1U << joint_index),
            &application_action.frames);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK || application_action.frames.count != 1U) {
            (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
                runtime_status == MOTOR_RUNTIME_STATUS_OK ? PROTOCOL_COMMAND_ERROR_ACTION_FAILED :
                    aethor_app_map_runtime_error(runtime_status), (uint8_t)(joint_index + 1U), timestamp_us);
            return;
        }
    }
    aethor_app_static_trace_record(STATIC_TRACE_PHASE, identity->motor_id,
        &application_action.frames.frames[0], timestamp_us, timestamp_us, 0U, 0U, 0U);
}

/** @brief Start a fixed 1000 ms single-axis takeover after the unchanged zero-FF startup proof. */
static uint8_t aethor_app_static_begin_takeover(uint64_t timestamp_us)
{
    application_action.static_takeover_started_us = timestamp_us;
    application_action.static_takeover_last_poll_us = timestamp_us;
    application_action.static_takeover_last_refresh_us = timestamp_us;
    application_action.static_takeover_step = 0U;
    application_action.static_takeover_proven_us = 0ULL;
    application_action.static_identity.phase = DEBUG_UI_STATIC_TX_TAKEOVER;
    application_action.frames.count = application_action.frame_read_index = 0U;
    if (!aethor_app_calculate_deadline_us(timestamp_us, AETHOR_STATIC_TAKEOVER_WINDOW_US, &application_action.deadline_us))
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
            PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
    aethor_app_static_load_begin(timestamp_us);
    application_debug_ui.static_load.takeover_active = 1U;
    return 0U;
}

/** @brief Require actual TX and a newer ordered selected-axis sample for the currently prepared refresh. */
static uint8_t aethor_app_static_takeover_proven(const MotorFeedbackSnapshot *snapshot, uint8_t joint_index)
{
    uint32_t generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
    return (uint8_t)(application_action.single_static_tx_confirmed && !application_action.single_static_tx_failed &&
        snapshot && (snapshot->valid_joint_mask & (1U << joint_index)) &&
        snapshot->joints[joint_index].driver_state == S3519_DRIVER_STATE_ENABLED &&
        generation != application_action.single_static_rx_generation &&
        generation == application_action.single_static_ordered_generation);
}

/** @brief Refresh the current 50 ms increment every 4 ms within the original fixed one-second window.
 * The preceding refresh must have complete actual-TX/ordered-RX proof and no transport debt.
 * A late refresh never catches up; lateness above 5 ms fails closed without extending either deadline.
 * These are engineering timing values, not manufacturer-qualified motor parameters.
 */
static uint8_t aethor_app_static_advance_takeover(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    uint8_t joint_index, failure = 0U, current_step;
    uint64_t elapsed, next_refresh_due;
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        if (application_action.command.motor_mask & (1U << joint_index)) break;
    (void)aethor_app_check_one_shot_motor_safety(snapshot, timestamp_us, &failure);
    if (failure) return 0U;
    if (joint_index >= ARM_JOINT_COUNT || timestamp_us < application_action.static_takeover_last_poll_us ||
        timestamp_us - application_action.static_takeover_last_poll_us >
            AETHOR_STATIC_TAKEOVER_REFRESH_US + AETHOR_STATIC_TAKEOVER_MAX_LATENESS_US ||
        application_action.single_static_tx_failed)
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
            PROTOCOL_COMMAND_ERROR_TIMEOUT, 0U, timestamp_us);
    application_action.static_takeover_last_poll_us = timestamp_us;
    elapsed = timestamp_us - application_action.static_takeover_started_us;
    report->takeover_elapsed_ms = elapsed >= AETHOR_STATIC_TAKEOVER_WINDOW_US ? 1000U : (uint32_t)(elapsed / 1000ULL);
    if (elapsed >= AETHOR_STATIC_TAKEOVER_WINDOW_US) {
        if (application_action.static_takeover_step != AETHOR_STATIC_TAKEOVER_STEP_COUNT || !application_action.static_takeover_proven_us ||
            application_action.static_takeover_proven_us > application_action.deadline_us ||
            application_debug_ui.single_static_transport_mask ||
            application_action.frame_read_index != application_action.frames.count ||
            !aethor_app_static_takeover_proven(snapshot, joint_index))
            return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
                PROTOCOL_COMMAND_ERROR_TIMEOUT, (uint8_t)(joint_index + 1U), timestamp_us);
        report->takeover_active = 0U;
        report->takeover_complete = 1U;
        application_action.static_identity.phase = DEBUG_UI_STATIC_TX_NONE;
        return aethor_app_start_one_shot_hold(application_action.deadline_us);
    }
    next_refresh_due = application_action.static_takeover_last_refresh_us + AETHOR_STATIC_TAKEOVER_REFRESH_US;
    if (timestamp_us < next_refresh_due) return 0U;
    current_step = (uint8_t)(elapsed / AETHOR_STATIC_TAKEOVER_STEP_US);
    if (current_step > AETHOR_STATIC_TAKEOVER_STEP_COUNT) current_step = AETHOR_STATIC_TAKEOVER_STEP_COUNT;
    if (timestamp_us - next_refresh_due > AETHOR_STATIC_TAKEOVER_MAX_LATENESS_US ||
        current_step > application_action.static_takeover_step + 1U ||
        (current_step != application_action.static_takeover_step &&
         elapsed - (uint64_t)current_step * AETHOR_STATIC_TAKEOVER_STEP_US > AETHOR_STATIC_TAKEOVER_MAX_LATENESS_US) ||
        application_debug_ui.single_static_transport_mask ||
        application_action.frame_read_index != application_action.frames.count ||
        (application_action.static_takeover_last_refresh_us != application_action.static_takeover_started_us &&
         !aethor_app_static_takeover_proven(snapshot, joint_index)))
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
            PROTOCOL_COMMAND_ERROR_TIMEOUT, (uint8_t)(joint_index + 1U), timestamp_us);
    application_action.static_takeover_step = current_step;
    application_action.static_takeover_last_refresh_us = timestamp_us;
    application_action.static_takeover_proven_us = 0ULL;
    report->requested_torque_ff_nm = application_action.command.mit_torque_ff_nm *
        ((float)application_action.static_takeover_step / (float)AETHOR_STATIC_TAKEOVER_STEP_COUNT);
    aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_TAKEOVER, joint_index, timestamp_us);
    return 0U;
}

/** @brief Advance only this selected axis's actual TX plus newer ordered, safe feedback barrier. */
static uint8_t aethor_app_static_advance_startup(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    DebugUiStaticTxIdentity identity = application_action.static_identity;
    if (application_debug_ui.static_load.takeover_active && application_action.static_takeover_started_us)
        return aethor_app_static_advance_takeover(snapshot, timestamp_us);
    uint8_t joint_index = (uint8_t)(identity.motor_id - 1U), next_index, failure = 0U;
    uint8_t required_state = identity.phase == DEBUG_UI_STATIC_TX_PRELOAD ?
        S3519_DRIVER_STATE_DISABLED : S3519_DRIVER_STATE_ENABLED;
    uint32_t generation;
    (void)aethor_app_check_one_shot_motor_safety(snapshot, timestamp_us, &failure);
    if (failure) return 0U;
    if (timestamp_us >= application_action.static_setup_deadline_us || application_action.single_static_tx_failed)
        return aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_ENABLE,
            application_action.single_static_tx_failed ? PROTOCOL_COMMAND_ERROR_ACTION_FAILED : PROTOCOL_COMMAND_ERROR_TIMEOUT,
            identity.motor_id, timestamp_us);
    if (identity.phase == DEBUG_UI_STATIC_TX_ENABLE || !application_action.single_static_tx_confirmed) return 0U;
    generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
    if (!snapshot || !(snapshot->valid_joint_mask & (1U << joint_index)) ||
        generation == application_action.single_static_rx_generation ||
        generation != application_action.single_static_ordered_generation ||
        snapshot->joints[joint_index].driver_state != required_state) return 0U;
    if (identity.phase == DEBUG_UI_STATIC_TX_PRELOAD) {
        application_action.static_preloaded_mask |= (uint8_t)(1U << joint_index);
        for (next_index = 0U; next_index < ARM_JOINT_COUNT; ++next_index)
            if ((application_action.command.motor_mask & (1U << next_index)) &&
                !(application_action.static_preloaded_mask & (1U << next_index))) break;
        if (next_index < ARM_JOINT_COUNT) {
            aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_PRELOAD, next_index, timestamp_us);
            return 0U;
        }
    } else {
        application_action.static_first_hold_mask |= (uint8_t)(1U << joint_index);
    }
    for (next_index = 0U; next_index < ARM_JOINT_COUNT; ++next_index)
        if ((application_action.command.motor_mask & (1U << next_index)) &&
            !(application_action.static_first_hold_mask & (1U << next_index))) break;
    if (next_index < ARM_JOINT_COUNT) {
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_ENABLE, next_index, timestamp_us);
        return 0U;
    }
    application_action.static_identity.phase = DEBUG_UI_STATIC_TX_NONE;
    if (aethor_app_is_pos_load()) {
        aethor_app_enter_task_critical();
        application_debug_ui.pos_load.stage = DEBUG_UI_STAGE_MOTION;
        aethor_app_exit_task_critical();
        return aethor_app_start_one_shot_motion(snapshot, timestamp_us);
    }
    if (application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        application_action.command.mit_torque_ff_nm != 0.0F)
        return aethor_app_static_begin_takeover(timestamp_us);
    return aethor_app_start_one_shot_hold(timestamp_us);
}
