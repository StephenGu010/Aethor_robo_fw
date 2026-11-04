/** @file debug_ui_pos_load_app.inc
 * @brief Bounded POS-only trial observations and guards, included by the production facade.
 * Angles in reports are S3519 output-shaft radians; load mass is never a control input.
 */
#if !AETHOR_ADRC_BENCH
/** @brief Separate confirmed ID7 POS trial bounds from the preserved ordinary motor pages. */
static DebugUiMotorProfile aethor_app_pos_load_bounds(uint8_t joint_index)
{
    DebugUiMotorProfile profile = application_debug_ui.profiles[joint_index];
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    if (joint_index == 6U) {
        float maximum = application_motor_runtime.discovery.results[joint_index].ranges.position_max_rad;
        float cap = 3.14159265358979323846F * AETHOR_S3519_MOTOR7_FLANGE_TOTAL_RATIO;
        if (isfinite(maximum) && maximum > 0.0F && isfinite(2.0F * maximum)) {
            profile.position_min_rad = -maximum;
            profile.position_max_rad = maximum;
            profile.max_delta_rad = fminf(2.0F * maximum, cap);
        }
    }
#endif
    return profile;
}

/** @brief Identify only the independently gated local POS loaded-motion command. */
static uint8_t aethor_app_is_pos_load(void)
{
    return (uint8_t)(application_action.command.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        debug_ui_is_pos_load_operation(application_action.command.local_operation));
}

/** @brief Match only the explicit local STOP bound to the retained POS trial, excluding unrelated cleanup. */
static uint8_t aethor_app_is_pos_load_stop(void)
{
    const ProtocolCommand *command = &application_action.command;
    const DebugUiIdentity *identity = &application_debug_ui.pos_load.stop_identity;
    uint8_t motor_id = application_debug_ui.pos_load.motor_id;
    return (uint8_t)(command->origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        command->local_operation == DEBUG_UI_OPERATION_STOP && identity->request_id &&
        command->bench_relative_scope && motor_id > 0U && motor_id <= ARM_JOINT_COUNT &&
        command->motor_mask == (uint8_t)(1U << (motor_id - 1U)) &&
        command->origin == identity->origin && command->session_id == identity->epoch &&
        command->request_id == identity->request_id);
}

/** @brief Freeze actual hold elapsed at exit, excluding subsequent disable/confirmation waiting; caller holds critical. */
static void aethor_app_pos_load_freeze_elapsed_locked(uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    if (aethor_app_is_pos_load()) application_debug_ui.pos_load_cleanup_required = 1U;
    if (aethor_app_is_pos_load() && application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
        application_debug_ui.pos_load_hold_started_us && timestamp_us >= application_debug_ui.pos_load_hold_started_us) {
        uint64_t elapsed_ms = (timestamp_us - application_debug_ui.pos_load_hold_started_us) / 1000ULL;
        report->elapsed_ms = elapsed_ms > report->duration_ms ? report->duration_ms : (uint32_t)elapsed_ms;
    }
}

/** @brief Publish bounded hold elapsed at normal expiry, STOP or protection before phase replacement. */
static void aethor_app_pos_load_freeze_elapsed(uint64_t timestamp_us)
{
    if (!aethor_app_is_pos_load()) return;
    aethor_app_enter_task_critical();
    aethor_app_pos_load_freeze_elapsed_locked(timestamp_us);
    aethor_app_exit_task_critical();
}

/** @brief Bind a fresh report before any selected preflight/enable frame can be emitted. */
static void aethor_app_pos_load_begin(uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    uint8_t joint_index;
    if (!aethor_app_is_pos_load()) return;
    aethor_app_enter_task_critical();
    memset(report, 0, sizeof(*report));
    report->identity.origin = application_action.command.origin;
    report->identity.epoch = application_action.command.session_id;
    report->identity.request_id = application_action.command.request_id;
    report->stage = DEBUG_UI_STAGE_VALIDATE;
    report->active = 1U;
    report->started_at_us = timestamp_us;
    report->duration_ms = (uint32_t)(application_action.command.hold_duration_us / 1000ULL);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        if (application_action.command.motor_mask & (1U << joint_index)) {
            report->motor_id = (uint8_t)(joint_index + 1U);
            application_debug_ui.pos_load_discovery = application_motor_runtime.discovery.results[joint_index];
            application_debug_ui.pos_load_profile = aethor_app_pos_load_bounds(joint_index);
            report->target_position_rad = application_action.target_position_rad[joint_index] /
                aethor_app_lcd_position_ratio(joint_index);
        }
    application_debug_ui.pos_load_sample_us = 0U;
    application_debug_ui.pos_load_observation_us = 0U;
    application_debug_ui.pos_load_hold_started_us = 0U;
    application_debug_ui.pos_load_recovery_started_us = 0U;
    application_debug_ui.pos_load_stable_started_us = 0U;
    application_debug_ui.pos_load_fallback_after_us = 0U;
    application_debug_ui.pos_load_disable_after_us = 0U;
    memset(&application_debug_ui.pos_disable_identity, 0, sizeof(application_debug_ui.pos_disable_identity));
    application_debug_ui.pos_disable_submitted = application_debug_ui.pos_disable_confirmed = 0U;
    application_debug_ui.pos_disable_failed = application_debug_ui.pos_disable_pending = 0U;
    application_debug_ui.pos_disable_attempt_count = 0U;
    application_debug_ui.pos_disable_restart_requested = 0U;
    application_debug_ui.pos_load_cleanup_required = 0U;
    aethor_app_exit_task_critical();
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
/** @brief Match only this trial's terminal DISABLE and preserve its independent identity across action cancellation. */
static uint8_t aethor_app_pos_disable_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    unsigned byte_index;
    /* Immediate STOP still uses the independent emergency path while prior strict TX debt retires. */
    if (application_debug_ui.pos_disable_restart_requested) return 0U;
    if (!AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST || !application_debug_ui.pos_load_cleanup_required ||
        !frame || !identity || !report->motor_id || frame->length != 8U || frame->data[7] != S3519_MODE_COMMAND_DISABLE ||
        (frame->identifier != report->motor_id && frame->identifier != 0x100U + report->motor_id)) return 0U;
    if (!report->active && !aethor_app_is_pos_load_stop() && application_debug_ui.pos_disable_identity.phase &&
        !application_debug_ui.pos_disable_pending) return 0U;
    for (byte_index = 0U; byte_index < 7U; ++byte_index) if (frame->data[byte_index] != 0xFFU) return 0U;
    if (!application_debug_ui.pos_disable_identity.phase) {
        ++application_debug_ui.pos_disable_attempt_count;
        ++application_debug_ui.pos_disable_next_sequence;
        if (!application_debug_ui.pos_disable_next_sequence) ++application_debug_ui.pos_disable_next_sequence;
        application_debug_ui.pos_disable_identity.epoch = report->identity.epoch;
        application_debug_ui.pos_disable_identity.request_id = report->identity.request_id;
        application_debug_ui.pos_disable_identity.sequence = application_debug_ui.pos_disable_next_sequence;
        application_debug_ui.pos_disable_identity.motor_id = report->motor_id;
        application_debug_ui.pos_disable_identity.phase = DEBUG_UI_STATIC_TX_POS_DISABLE;
        application_debug_ui.pos_disable_frame = *frame;
        application_debug_ui.pos_disable_pending = 1U;
        application_debug_ui.pos_disable_rx_generation = application_debug_ui.pos_disable_ordered_generation = 0U;
        application_debug_ui.pos_disable_submitted = application_debug_ui.pos_disable_confirmed = application_debug_ui.pos_disable_failed = 0U;
    }
    if (frame->identifier != application_debug_ui.pos_disable_frame.identifier ||
        memcmp(frame->data, application_debug_ui.pos_disable_frame.data, 8U)) return 0U;
    *identity = application_debug_ui.pos_disable_identity;
    return 1U;
}

/** @brief Admit the exact emergency cleanup HAL boundary without normal motion authority or health gates. */
static uint8_t aethor_app_pos_disable_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    uint8_t joint_index;
    if (!identity || !application_debug_ui.pos_load_cleanup_required ||
        !debug_ui_static_tx_identity_equal(identity, &application_debug_ui.pos_disable_identity) ||
        application_debug_ui.pos_disable_submitted || application_debug_ui.pos_disable_failed ||
        boundary_us <= application_debug_ui.pos_load.started_at_us) return 0U;
    joint_index = (uint8_t)(identity->motor_id - 1U);
    application_debug_ui.pos_disable_submitted = 1U;
    application_debug_ui.pos_disable_rx_generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
    application_debug_ui.pos_disable_ordered_generation = 0U;
    application_debug_ui.pos_load_disable_after_us = boundary_us;
    application_action.phase_feedback_baseline_us[joint_index] = boundary_us;
    return 1U;
}

/** @brief A matching real TX receipt releases only terminal transport debt; old startup receipts cannot confirm cleanup. */
static void aethor_app_pos_disable_transmitted(const DebugUiStaticTxIdentity *identity, uint8_t succeeded)
{
    if (!identity || !debug_ui_static_tx_identity_equal(identity, &application_debug_ui.pos_disable_identity)) return;
    application_debug_ui.pos_disable_pending = 0U;
    if (application_debug_ui.pos_disable_failed || application_debug_ui.pos_disable_confirmed) return;
    if (!succeeded || !application_debug_ui.pos_disable_submitted) application_debug_ui.pos_disable_failed = 1U;
    else application_debug_ui.pos_disable_confirmed = 1U;
}
#endif

/** @brief Require exact terminal actual-TX proof and a newer accepted hardware-ordered same-axis disabled sample. */
static uint8_t aethor_app_pos_disable_proven(const MotorFeedbackSnapshot *snapshot)
{
    const DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    const MotorJointFeedback *feedback;
    uint8_t joint_index;
    uint32_t generation;
    if (!snapshot || !application_debug_ui.pos_load_cleanup_required || !report->motor_id ||
        application_debug_ui.pos_disable_restart_requested ||
        !application_debug_ui.pos_disable_confirmed || application_debug_ui.pos_disable_failed) return 0U;
    joint_index = (uint8_t)(report->motor_id - 1U);
    generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
    feedback = &snapshot->joints[joint_index];
    return (uint8_t)((snapshot->valid_joint_mask & (1U << joint_index)) &&
        generation != application_debug_ui.pos_disable_rx_generation &&
        generation == application_debug_ui.pos_disable_ordered_generation &&
        feedback->timestamp_us >= application_debug_ui.pos_load_disable_after_us &&
        feedback->driver_state == S3519_DRIVER_STATE_DISABLED && !feedback->fault_flags &&
        isfinite(feedback->velocity_rad_s) && fabsf(feedback->velocity_rad_s) <= AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S);
}

/** @brief Request at most eight terminal DISABLE samples, 50ms apart, within the original deadline.
 * Only completed TX may be followed by a new identity; queued/failed transport,
 * pending STOP, fault feedback and every nonterminal phase remain untouched.
 * This obtains a new full feedback sample without enabling, moving or weakening proof.
 */
static uint8_t aethor_app_pos_load_next_disable_probe(uint64_t timestamp_us, CanFrame *frame)
{
    MotorFeedbackSnapshot snapshot;
    uint8_t ready = 0U;
    if (!AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST || !frame) return 0U;
    aethor_app_enter_task_critical();
    if ((aethor_app_is_pos_load() &&
         (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT ||
          application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT)) ||
        (aethor_app_is_pos_load_stop() && application_action.state == AETHOR_APP_ACTION_DISABLE_WAIT)) {
        uint8_t joint_index = (uint8_t)(application_debug_ui.pos_load.motor_id - 1U);
        if (joint_index < ARM_JOINT_COUNT && application_debug_ui.pos_load_cleanup_required &&
            application_action.frame_read_index >= application_action.frames.count &&
            application_action.deadline_us && timestamp_us < application_action.deadline_us &&
            application_protocol_engine.stop_read_sequence == application_protocol_engine.stop_write_sequence &&
            application_debug_ui.pos_disable_attempt_count < 8U &&
            ((application_debug_ui.pos_disable_confirmed && !application_debug_ui.pos_disable_failed) ||
             application_debug_ui.pos_disable_restart_requested) &&
            !application_debug_ui.pos_disable_pending &&
            timestamp_us >= application_debug_ui.pos_load_disable_after_us &&
            timestamp_us - application_debug_ui.pos_load_disable_after_us >= 50000ULL &&
            !application_motor_runtime.bank.motors[joint_index].feedback.fault_flags) {
            memset(&snapshot, 0, sizeof(snapshot));
            (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
            if (!aethor_app_pos_disable_proven(&snapshot)) {
                *frame = application_debug_ui.pos_disable_frame;
                memset(&application_debug_ui.pos_disable_identity, 0, sizeof(application_debug_ui.pos_disable_identity));
                application_debug_ui.pos_disable_submitted = application_debug_ui.pos_disable_confirmed = 0U;
                application_debug_ui.pos_disable_failed = application_debug_ui.pos_disable_restart_requested = 0U;
                application_debug_ui.pos_disable_rx_generation = application_debug_ui.pos_disable_ordered_generation = 0U;
                ready = 1U;
            }
        }
    }
    aethor_app_exit_task_critical();
    return ready;
}

/** @brief Preserve finite fresh phase measurements, including the sample that first crosses a protection threshold. */
static void aethor_app_pos_load_observe(const MotorFeedbackSnapshot *snapshot, uint8_t joint_index, uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    const MotorJointFeedback *feedback;
    if (!report->active || !snapshot || !(snapshot->valid_joint_mask & (1U << joint_index))) return;
    feedback = &snapshot->joints[joint_index];
    if (feedback->timestamp_us <= application_debug_ui.pos_load_observation_us || feedback->timestamp_us > timestamp_us ||
        timestamp_us - feedback->timestamp_us > MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US) return;
    aethor_app_enter_task_critical();
    application_debug_ui.pos_load_observation_us = feedback->timestamp_us;
    if (isfinite(feedback->torque_nm)) report->peak_torque_nm = fmaxf(report->peak_torque_nm, fabsf(feedback->torque_nm));
    if (isfinite(feedback->mos_temperature_c)) report->peak_mos_c = fmaxf(report->peak_mos_c, feedback->mos_temperature_c);
    if (isfinite(feedback->rotor_temperature_c)) report->peak_rotor_c = fmaxf(report->peak_rotor_c, feedback->rotor_temperature_c);
    /* Before arrival the remaining commanded distance is not a tracking-error measurement. */
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT && isfinite(feedback->position_rad)) {
        report->error_rad = fabsf(feedback->position_rad - application_action.target_position_rad[joint_index]) /
            aethor_app_lcd_position_ratio(joint_index);
        report->peak_error_rad = fmaxf(report->peak_error_rad, report->error_rad);
    }
    if (isfinite(feedback->position_rad) && isfinite(feedback->velocity_rad_s) && isfinite(feedback->torque_nm) &&
        isfinite(feedback->mos_temperature_c) && isfinite(feedback->rotor_temperature_c) && report->samples < UINT32_MAX)
        ++report->samples;
    aethor_app_exit_task_critical();
}

/** @brief Retain every existing local protection during POS preload, movement and continuous hold. */
static DebugUiReason aethor_app_pos_load_guard(const MotorFeedbackSnapshot *snapshot,
    uint8_t joint_index, uint64_t timestamp_us)
{
    const MotorJointFeedback *feedback = &snapshot->joints[joint_index];
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
    DebugUiMotorProfile bounds = aethor_app_pos_load_bounds(joint_index);
    const DebugUiMotorProfile *profile = &bounds;
    float speed_margin = 2.0F * discovery->ranges.velocity_max_rad_s / 4095.0F;
    if (application_action.command.local_operation == DEBUG_UI_OPERATION_HOME &&
        application_action.command.home_boot_id != application_protocol_engine.boot_id)
        return DEBUG_UI_REASON_OLD_EPOCH;
    if (!(snapshot->valid_joint_mask & (1U << joint_index)) || feedback->timestamp_us > timestamp_us ||
        timestamp_us - feedback->timestamp_us > MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US)
        return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (feedback->fault_flags) return DEBUG_UI_REASON_NOT_READY;
    if (!isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) ||
        !isfinite(feedback->torque_nm) || !isfinite(feedback->mos_temperature_c) ||
        !isfinite(feedback->rotor_temperature_c)) return DEBUG_UI_REASON_INVALID_FEEDBACK;
    if (discovery->observed_control_mode != 2U || !profile->pos_valid ||
        discovery->verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK)
        return DEBUG_UI_REASON_NOT_READY;
    if (!aethor_app_single_static_discovery_equal(discovery, &application_debug_ui.pos_load_discovery) ||
        profile->position_min_rad != application_debug_ui.pos_load_profile.position_min_rad ||
        profile->position_max_rad != application_debug_ui.pos_load_profile.position_max_rad ||
        profile->max_delta_rad != application_debug_ui.pos_load_profile.max_delta_rad ||
        profile->max_speed_rad_s != application_debug_ui.pos_load_profile.max_speed_rad_s)
        return DEBUG_UI_REASON_OLD_EPOCH;
    if (application_action.command.reference_generation != application_debug_ui.reference_generation[joint_index])
        return DEBUG_UI_REASON_OLD_EPOCH;
    {
        DebugUiReason torque_reason = aethor_app_torque_envelope_guard(feedback, joint_index, timestamp_us);
        if (torque_reason != DEBUG_UI_REASON_NONE) return torque_reason;
    }
    if (feedback->mos_temperature_c > 55.0F || feedback->rotor_temperature_c > 55.0F)
        return DEBUG_UI_REASON_TEMPERATURE_LIMIT;
    if (fabsf(feedback->velocity_rad_s) > profile->max_speed_rad_s + speed_margin)
        return DEBUG_UI_REASON_SPEED_LIMIT;
    if (feedback->position_rad < profile->position_min_rad || feedback->position_rad > profile->position_max_rad)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
        fabsf(feedback->position_rad - application_debug_ui.pos_load_start_position_rad) /
        aethor_app_lcd_position_ratio(joint_index) > 0.5F * AETHOR_APP_DEG_TO_RAD)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
        application_action.static_identity.phase == DEBUG_UI_STATIC_TX_PRELOAD) {
        if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED) return DEBUG_UI_REASON_NOT_DISABLED;
        if (fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S)
            return DEBUG_UI_REASON_SPEED_LIMIT;
    } else if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
        feedback->driver_state != S3519_DRIVER_STATE_ENABLED) return DEBUG_UI_REASON_NOT_READY;
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
        fabsf(feedback->position_rad - application_action.target_position_rad[joint_index]) /
        aethor_app_lcd_position_ratio(joint_index) > 0.5F * AETHOR_APP_DEG_TO_RAD)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
        (timestamp_us < application_action.last_pos_reference_timestamp_us ||
         timestamp_us - application_action.last_pos_reference_timestamp_us > AETHOR_APP_POS_REFERENCE_MAX_GAP_US))
        return DEBUG_UI_REASON_UNHEALTHY;
    return DEBUG_UI_REASON_NONE;
}

/** @brief Count fresh nonduplicate selected samples and record hysteretic recovery without extending the hold deadline. */
static uint8_t aethor_app_pos_load_sample_locked(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    uint8_t joint_index = (uint8_t)(report->motor_id - 1U);
    const MotorJointFeedback *feedback;
    float error_rad, speed_tolerance;
    uint64_t recovery_us;
    if (!aethor_app_is_pos_load() || !report->active || !report->motor_id ||
        application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT) return 0U;
    report->stage = DEBUG_UI_STAGE_HOLD;
    report->elapsed_ms = (uint32_t)((timestamp_us - application_debug_ui.pos_load_hold_started_us) / 1000ULL);
    if (report->elapsed_ms > report->duration_ms) report->elapsed_ms = report->duration_ms;
    if (report->recovering && timestamp_us >= application_debug_ui.pos_load_recovery_started_us &&
        timestamp_us - application_debug_ui.pos_load_recovery_started_us >= 2000000ULL) return 1U;
    feedback = &snapshot->joints[joint_index];
    if (!(snapshot->valid_joint_mask & (1U << joint_index)) ||
        feedback->timestamp_us <= application_debug_ui.pos_load_sample_us ||
        feedback->timestamp_us > timestamp_us) return 0U;
    if (feedback->timestamp_us - application_debug_ui.pos_load_sample_us > MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US)
        application_debug_ui.pos_load_stable_started_us = 0U;
    application_debug_ui.pos_load_sample_us = feedback->timestamp_us;
    error_rad = fabsf(feedback->position_rad - application_action.target_position_rad[joint_index]) /
        aethor_app_lcd_position_ratio(joint_index);
    report->error_rad = error_rad;
    report->peak_error_rad = fmaxf(report->peak_error_rad, error_rad);
    if (!report->recovering && error_rad >= 0.25F * AETHOR_APP_DEG_TO_RAD) {
        report->recovering = 1U;
        if (report->disturbances < UINT32_MAX) ++report->disturbances;
        application_debug_ui.pos_load_recovery_started_us = feedback->timestamp_us;
        application_debug_ui.pos_load_stable_started_us = 0U;
    }
    if (!report->recovering) return 0U;
    recovery_us = feedback->timestamp_us - application_debug_ui.pos_load_recovery_started_us;
    if (recovery_us >= 2000000ULL) return 1U;
    speed_tolerance = 2.0F * application_motor_runtime.discovery.results[joint_index].ranges.velocity_max_rad_s / 4095.0F;
    if (error_rad <= 0.125F * AETHOR_APP_DEG_TO_RAD && fabsf(feedback->velocity_rad_s) <= speed_tolerance) {
        if (!application_debug_ui.pos_load_stable_started_us)
            application_debug_ui.pos_load_stable_started_us = feedback->timestamp_us;
        if (feedback->timestamp_us - application_debug_ui.pos_load_stable_started_us >= 250000ULL) {
            report->recovering = 0U;
            report->recovery_observed = 1U;
            if (report->recoveries < UINT32_MAX) ++report->recoveries;
            report->last_recovery_ms = (uint32_t)(recovery_us / 1000ULL);
            if (report->last_recovery_ms > report->max_recovery_ms)
                report->max_recovery_ms = report->last_recovery_ms;
            application_debug_ui.pos_load_stable_started_us = 0U;
        }
    } else application_debug_ui.pos_load_stable_started_us = 0U;
    return 0U;
}

/** @brief Publish one bounded coherent observation while the UI snapshot uses the same critical boundary. */
static uint8_t aethor_app_pos_load_sample(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    uint8_t timed_out;
    aethor_app_enter_task_critical();
    timed_out = aethor_app_pos_load_sample_locked(snapshot, timestamp_us);
    aethor_app_exit_task_critical();
    return timed_out;
}

/** @brief Bind the explicit STOP to the active report before the original action is cancelled. */
static void aethor_app_pos_load_bind_stop(const ProtocolCommand *stop_command, uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    if (!AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST || !stop_command ||
        stop_command->origin != DEBUG_UI_ORIGIN_LOCAL_UI || !stop_command->bench_relative_scope ||
        !report->identity.request_id || !report->motor_id || report->motor_id > ARM_JOINT_COUNT ||
        stop_command->motor_mask != (uint8_t)(1U << (report->motor_id - 1U)) ||
        (!aethor_app_is_pos_load() && !application_debug_ui.pos_load_cleanup_required)) return;
    if (report->stop_identity.origin == stop_command->origin &&
        report->stop_identity.epoch == stop_command->session_id &&
        report->stop_identity.request_id == stop_command->request_id) return;
    aethor_app_enter_task_critical();
    if (aethor_app_is_pos_load()) aethor_app_pos_load_freeze_elapsed_locked(timestamp_us);
    if (!application_debug_ui.pos_disable_pending) {
        memset(&application_debug_ui.pos_disable_identity, 0, sizeof(application_debug_ui.pos_disable_identity));
        application_debug_ui.pos_disable_submitted = application_debug_ui.pos_disable_confirmed = 0U;
        application_debug_ui.pos_disable_failed = 0U;
        application_debug_ui.pos_disable_restart_requested = 0U;
        application_debug_ui.pos_disable_attempt_count = 0U;
        application_debug_ui.pos_disable_rx_generation = application_debug_ui.pos_disable_ordered_generation = 0U;
    } else {
        application_debug_ui.pos_disable_restart_requested = 1U;
        /* Count the immediate unproved safety DISABLE in this STOP's finite budget too. */
        application_debug_ui.pos_disable_attempt_count = 1U;
    }
    application_debug_ui.pos_load.stop_identity.origin = stop_command->origin;
    application_debug_ui.pos_load.stop_identity.epoch = stop_command->session_id;
    application_debug_ui.pos_load.stop_identity.request_id = stop_command->request_id;
    aethor_app_exit_task_critical();
}

/** @brief Finalize a matching trial/STOP only; caller holds the mailbox publication critical section. */
static void aethor_app_pos_load_finish(const ProtocolCommandResult *result)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    uint8_t trial = (uint8_t)(debug_ui_is_pos_load_operation(result->local_operation) &&
        result->origin == report->identity.origin && result->session_id == report->identity.epoch &&
        result->request_id == report->identity.request_id);
    uint8_t stop = (uint8_t)(result->local_operation == DEBUG_UI_OPERATION_STOP &&
        result->origin == report->stop_identity.origin && result->session_id == report->stop_identity.epoch &&
        result->request_id == report->stop_identity.request_id && report->stop_identity.request_id);
    if (!trial && !stop) return;
    /* A delayed original cancellation cannot rewrite the later causal STOP/fallback confirmation. */
    if (report->stopped && report->disabled_confirmed) return;
    report->active = 0U;
    report->stage = result->stage == PROTOCOL_COMMAND_STAGE_NONE ? DEBUG_UI_STAGE_DISABLE : (DebugUiStage)result->stage;
    report->disabled_confirmed = result->disabled_confirmed;
    if (trial && result->code == PROTOCOL_COMMAND_RESULT_CANCELLED) report->disabled_confirmed = 0U;
    report->stopped = (uint8_t)(stop || result->code == PROTOCOL_COMMAND_RESULT_CANCELLED ||
        result->code == PROTOCOL_COMMAND_RESULT_STOPPED);
    report->complete = (uint8_t)(trial && result->code == PROTOCOL_COMMAND_RESULT_COMPLETED &&
        !report->recovering && result->disabled_confirmed);
    if (report->disabled_confirmed && !application_debug_ui.pos_disable_pending)
        application_debug_ui.pos_load_cleanup_required = 0U;
}

/** @brief Anchor terminal/STOP disable confirmation without replacing the initial preflight's receive baseline. */
static void aethor_app_pos_load_disable_queued(const CanFrame *frame, uint8_t accepted, uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    unsigned byte_index;
    /* Initial preflight defers TX to a later HAL tick. Its prompt RX may share
     * that tick, so retain the earlier baseline and the strict newer-RX check.
     * Terminal and STOP confirmation still use their own scheduling boundary. */
    if (aethor_app_is_pos_load() &&
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_PREFLIGHT_DISABLE_WAIT &&
        !application_debug_ui.pos_load_fallback_after_us) return;
    if (!accepted || !frame || !report->motor_id || frame->length != 8U || frame->data[7] != S3519_MODE_COMMAND_DISABLE ||
        (frame->identifier != report->motor_id && frame->identifier != 0x100U + report->motor_id)) return;
    for (byte_index = 0U; byte_index < 7U; ++byte_index) if (frame->data[byte_index] != 0xFFU) return;
    if (!aethor_app_is_pos_load() && !application_debug_ui.pos_load_fallback_after_us &&
        !(application_action.command.local_operation == DEBUG_UI_OPERATION_STOP &&
          application_action.command.request_id == report->stop_identity.request_id &&
          application_action.command.session_id == report->stop_identity.epoch &&
          application_action.command.origin == report->stop_identity.origin)) return;
    aethor_app_enter_task_critical();
    application_debug_ui.pos_load_disable_after_us = timestamp_us;
    if (!application_debug_ui.pos_load_fallback_after_us)
        application_action.phase_feedback_baseline_us[report->motor_id - 1U] = timestamp_us;
    aethor_app_exit_task_critical();
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Bind a result-less saturated STOP to this trial before the shared emergency path cancels it. */
static void aethor_app_pos_load_bind_fallback(uint64_t timestamp_us)
{
    if (!aethor_app_is_pos_load()) return;
    aethor_app_pos_load_freeze_elapsed_locked(timestamp_us);
    application_debug_ui.pos_load_fallback_after_us = timestamp_us;
    application_debug_ui.pos_load.complete = 0U;
    application_debug_ui.pos_load.stopped = 1U;
}

/** @brief Mark saturated cleanup confirmed only after emitted scope and newer selected disabled feedback. */
static void aethor_app_pos_load_confirm_fallback(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    DebugUiPosLoadReport *report = &application_debug_ui.pos_load;
    const MotorJointFeedback *feedback;
    uint8_t selected_mask;
    if (!application_debug_ui.pos_load_fallback_after_us || !report->motor_id ||
        application_debug_ui.pos_load_disable_after_us < application_debug_ui.pos_load_fallback_after_us) return;
    selected_mask = (uint8_t)(1U << (report->motor_id - 1U));
    feedback = &snapshot->joints[report->motor_id - 1U];
    if (!(application_debug_ui.fallback_stop_mask & selected_mask) || !(snapshot->valid_joint_mask & selected_mask) ||
        !aethor_app_pos_disable_proven(snapshot) ||
        !application_debug_ui.fallback_frames.count ||
        application_debug_ui.fallback_frame_read_index < application_debug_ui.fallback_frames.count ||
        feedback->timestamp_us < application_debug_ui.pos_load_disable_after_us || feedback->timestamp_us > timestamp_us ||
        feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
        !isfinite(feedback->velocity_rad_s) || fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S)
        return;
    report->active = report->complete = 0U;
    report->stopped = report->disabled_confirmed = 1U;
    report->stage = DEBUG_UI_STAGE_DISABLE;
    application_debug_ui.pos_load_cleanup_required = 0U;
    application_debug_ui.pos_load_fallback_after_us = 0U;
}
#endif
#endif
