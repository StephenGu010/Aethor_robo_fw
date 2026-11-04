/** @file debug_ui_static_load_app.inc
 * @brief Exact-scope local static HOLD validation and observed statistics; included by the facade.
 * No hardware authority, ratios, gains or thermal limits are manufactured here.
 */
#if !AETHOR_ADRC_BENCH
/** @brief Rechecks selected frozen axes and the independent long-window binding before ENABLE. */
static DebugUiReason aethor_app_static_load_validate(const ProtocolCommand *command,
    const MotorFeedbackSnapshot *snapshot)
{
#if !AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD && !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    (void)command;
    (void)snapshot;
    return DEBUG_UI_REASON_DISABLED;
#else
    uint8_t joint_index;
    if (command->local_operation == DEBUG_UI_OPERATION_STATIC_HOLD)
    {
#if AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
        if (command->motor_mask != 0x7FU) { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
#else
        return DEBUG_UI_REASON_DISABLED;
#endif
    }
    else if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
    {
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
        if (!command->motor_mask || (command->motor_mask & 0x80U) ||
            (command->motor_mask & (command->motor_mask - 1U))) { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
#else
        return DEBUG_UI_REASON_DISABLED;
#endif
    }
    else { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    if (command->origin != DEBUG_UI_ORIGIN_LOCAL_UI ||
        command->type != PROTOCOL_COMMAND_MIT_ACTION || command->mit_action != PROTOCOL_MIT_ACTION_HOLD ||
        command->control_mode != ARM_CONTROL_MODE_MIT || !command->values_in_radians || !command->bench_relative_scope ||
        command->relative_target || !isfinite(command->mit_torque_ff_nm) ||
        (command->local_operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && command->mit_torque_ff_nm != 0.0F))
    { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback = &snapshot->joints[joint_index];
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        DebugUiMotorProfile scoped_profile = command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            aethor_app_single_static_bounds(joint_index) : application_debug_ui.profiles[joint_index];
        const DebugUiMotorProfile *profile = &scoped_profile;
        const DebugUiStaticGains *gains = &application_debug_ui.single_static_gains[joint_index];
        float expected_kp = profile->default_kp;
        float expected_kd = profile->default_kd;
        float target = command->values[joint_index];
        float ratio = aethor_app_lcd_position_ratio(joint_index);
        if ((command->motor_mask & (1U << joint_index)) == 0U) { continue; }
        if (command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
        {
            const DebugUiSingleStaticProfile *long_profile = &application_debug_ui.single_static_profiles[joint_index];
            if (gains->kp != 0.0F || gains->kd != 0.0F) {
                if (joint_index != 6U || !application_debug_ui.single_static_lcd_owned[joint_index] ||
                    !debug_ui_static_gains_valid(gains->kp, gains->kd) || !isfinite(ratio) || ratio <= 0.0F)
                    return DEBUG_UI_REASON_OUT_OF_RANGE;
                expected_kp = gains->kp / ratio; expected_kd = gains->kd;
            }
            if (!aethor_app_single_static_profile_current(joint_index)) { return DEBUG_UI_REASON_UNCALIBRATED; }
            if (!debug_ui_single_static_feedforward_valid(command->mit_torque_ff_nm, discovery->ranges.torque_max_nm) ||
                command->mit_torque_ff_nm != long_profile->torque_ff_nm) { return DEBUG_UI_REASON_OUT_OF_RANGE; }
            if (long_profile->minimum_ms < DEBUG_UI_SINGLE_STATIC_MIN_MS ||
                long_profile->maximum_ms > DEBUG_UI_SINGLE_STATIC_MAX_MS ||
                long_profile->minimum_ms > long_profile->maximum_ms ||
                command->hold_duration_us < (uint64_t)long_profile->minimum_ms * 1000ULL ||
                command->hold_duration_us > (uint64_t)long_profile->maximum_ms * 1000ULL)
            { return DEBUG_UI_REASON_OUT_OF_RANGE; }
        }
        if (command->static_reference_generations[joint_index] != application_debug_ui.reference_generation[joint_index])
        { return DEBUG_UI_REASON_OLD_EPOCH; }
        if ((snapshot->valid_joint_mask & (1U << joint_index)) == 0U)
        { return DEBUG_UI_REASON_STALE_FEEDBACK; }
        if (discovery->observed_control_mode != 1U) { return DEBUG_UI_REASON_NOT_READY; }
        if (!profile->mit_valid) { return DEBUG_UI_REASON_UNCALIBRATED; }
        if (!isfinite(target) || !isfinite(ratio) || ratio <= 0.0F ||
            !isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) ||
            !isfinite(feedback->torque_nm) || !isfinite(feedback->mos_temperature_c) ||
            !isfinite(feedback->rotor_temperature_c)) { return DEBUG_UI_REASON_INVALID_FEEDBACK; }
        if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
            fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S)
        { return DEBUG_UI_REASON_NOT_DISABLED; }
        if (fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM)
        { return DEBUG_UI_REASON_TORQUE_LIMIT; }
        if (feedback->mos_temperature_c > 55.0F || feedback->rotor_temperature_c > 55.0F)
        { return DEBUG_UI_REASON_TEMPERATURE_LIMIT; }
        if (target < profile->mit_position_min_rad || target > profile->mit_position_max_rad ||
            fabsf(target - feedback->position_rad) / ratio > 0.5F * AETHOR_APP_DEG_TO_RAD ||
            !isfinite(profile->default_kp) || !isfinite(profile->default_kd) ||
            !isfinite(command->mit_kp) || !isfinite(command->mit_kd) ||
            fabsf(command->mit_kp - expected_kp) > 0.000001F ||
            fabsf(command->mit_kd - expected_kd) > 0.000001F)
        { return DEBUG_UI_REASON_OUT_OF_RANGE; }
    }
    return DEBUG_UI_REASON_NONE;
#endif
}
/** @brief Starts a scope-labelled report only after the static HOLD was actually prepared. */
static void aethor_app_static_load_begin(uint64_t timestamp_us)
{
    uint8_t joint_index;
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    if (!debug_ui_is_static_operation(application_action.command.local_operation) ||
        application_action.command.origin != DEBUG_UI_ORIGIN_LOCAL_UI) { return; }
    aethor_app_enter_task_critical();
    if (report->active && report->takeover_complete &&
        report->identity.request_id == application_action.command.request_id &&
        report->identity.epoch == application_action.command.session_id) {
        report->started_at_us = timestamp_us;
        report->phase = DEBUG_UI_STATIC_LOAD_PHASE_HOLD;
        report->elapsed_ms = 0U;
        application_debug_ui.static_last_batch_us = timestamp_us;
        aethor_app_exit_task_critical();
        return;
    }
    memset(report, 0, sizeof(*report));
    application_debug_ui.static_fallback_bound = 0U;
    report->identity.origin = application_action.command.origin;
    report->identity.epoch = application_action.command.session_id;
    report->identity.request_id = application_action.command.request_id;
    report->operation = application_action.command.local_operation;
    report->motor_mask = application_action.command.motor_mask;
    memset(application_debug_ui.static_sample_us, 0, sizeof(application_debug_ui.static_sample_us));
    report->started_at_us = timestamp_us;
    application_debug_ui.static_last_batch_us = timestamp_us;
    report->duration_ms = (uint32_t)(application_action.command.hold_duration_us / 1000ULL);
    report->active = 1U;
    report->phase = application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT ?
        DEBUG_UI_STATIC_LOAD_PHASE_HOLD : DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER;
    report->target_torque_ff_nm = application_action.command.mit_torque_ff_nm;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    { report->sample_generation[joint_index] = application_motor_runtime.bank.motors[joint_index].feedback_generation;
      if ((report->motor_mask & (1U << joint_index)) == 0U) { continue; }
      report->target_position_rad[joint_index] = application_action.target_position_rad[joint_index] /
          aethor_app_lcd_position_ratio(joint_index); }
    aethor_app_exit_task_critical();
}

/** @brief Freeze only the first causal exit of this exact trial; cleanup and STOP cannot replace it. */
static void aethor_app_static_load_latch_exit(DebugUiReason reason, DebugUiStage stage,
    uint8_t error, DebugUiCompletionCode code)
{
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    const ProtocolCommand *command = &application_action.command;
    if (!report->active || report->first_exit_latched || command->local_operation != report->operation ||
        command->motor_mask != report->motor_mask || command->origin != report->identity.origin ||
        command->session_id != report->identity.epoch || command->request_id != report->identity.request_id) { return; }
    aethor_app_enter_task_critical();
    report->first_exit_phase = report->phase;
    report->first_exit_reason = reason;
    report->first_exit_stage = stage;
    report->first_exit_error = error;
    report->first_exit_code = code;
    report->first_exit_latched = 1U;
    aethor_app_exit_task_critical();
}

/** @brief Finalizes only this trial or its explicitly bound STOP; unrelated and delayed cancelled identities cannot confirm it. */
static void aethor_app_static_load_finish(const ProtocolCommandResult *result)
{
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    if (result->local_operation == DEBUG_UI_OPERATION_STOP && report->stop_identity.request_id != 0U &&
        result->origin == report->stop_identity.origin && result->session_id == report->stop_identity.epoch &&
        result->request_id == report->stop_identity.request_id && report->motor_mask != 0U &&
        (result->motor_mask & report->motor_mask) == report->motor_mask)
    {
        report->disabled_confirmed = result->disabled_confirmed;
        report->stopped = (uint8_t)(result->code == PROTOCOL_COMMAND_RESULT_STOPPED && result->disabled_confirmed);
        report->complete = 0U;
        return;
    }
    if (!report->active || !debug_ui_is_static_operation(result->local_operation) ||
        result->local_operation != report->operation || result->motor_mask != report->motor_mask ||
        result->origin != report->identity.origin || result->session_id != report->identity.epoch ||
        result->request_id != report->identity.request_id) { return; }
    if (!report->first_exit_latched) {
        report->first_exit_phase = report->phase;
        report->first_exit_reason = result->local_reason;
        report->first_exit_stage = (DebugUiStage)result->stage;
        report->first_exit_error = (uint8_t)result->error;
        report->first_exit_code = (DebugUiCompletionCode)result->code;
        report->first_exit_latched = 1U;
    }
    report->active = 0U;
    report->disabled_confirmed = result->disabled_confirmed;
    report->complete = (uint8_t)(result->code == PROTOCOL_COMMAND_RESULT_COMPLETED &&
        result->disabled_confirmed && report->valid_mask == report->motor_mask &&
        report->frames_issued_mask == report->motor_mask);
}

/** @brief Binds only the STOP that actually preempts this trial or its already bound cleanup. */
static void aethor_app_static_load_bind_stop(const ProtocolCommand *stop_command)
{
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    const ProtocolCommand *active = &application_action.command;
    uint8_t trial = (uint8_t)(debug_ui_is_static_operation(active->local_operation) &&
        active->local_operation == report->operation && active->motor_mask == report->motor_mask &&
        active->origin == report->identity.origin && active->request_id == report->identity.request_id &&
        active->session_id == report->identity.epoch);
    uint8_t cleanup = (uint8_t)(active->local_operation == DEBUG_UI_OPERATION_STOP &&
        active->origin == report->stop_identity.origin && active->request_id == report->stop_identity.request_id &&
        active->session_id == report->stop_identity.epoch);
    aethor_app_static_trace_bind_stop(stop_command);
    if ((!trial && !cleanup) || (!report->active && report->disabled_confirmed) ||
        stop_command->origin != DEBUG_UI_ORIGIN_LOCAL_UI || !report->motor_mask ||
        (stop_command->motor_mask & report->motor_mask) != report->motor_mask) { return; }
    aethor_app_static_load_latch_exit(DEBUG_UI_REASON_STOP_LATCHED, DEBUG_UI_STAGE_HOLD,
        0U, DEBUG_UI_STOPPED);
    aethor_app_enter_task_critical();
    report->stop_identity.origin = stop_command->origin;
    report->stop_identity.epoch = stop_command->session_id;
    report->stop_identity.request_id = stop_command->request_id;
    aethor_app_exit_task_critical();
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Records the exact original trial and causal anchor for a result-less saturated STOP. Caller holds critical. */
static void aethor_app_static_load_bind_fallback(uint64_t timestamp_us)
{
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    const ProtocolCommand *active = &application_action.command;
    uint8_t trial = (uint8_t)(debug_ui_is_static_operation(active->local_operation) &&
        active->local_operation == report->operation && active->motor_mask == report->motor_mask &&
        active->origin == report->identity.origin && active->request_id == report->identity.request_id &&
        active->session_id == report->identity.epoch);
    uint8_t cleanup = (uint8_t)(active->local_operation == DEBUG_UI_OPERATION_STOP &&
        active->origin == report->stop_identity.origin && active->request_id == report->stop_identity.request_id &&
        active->session_id == report->stop_identity.epoch);
    aethor_app_static_trace_bind_fallback(timestamp_us);
    if ((!trial && !cleanup) || (!report->active && report->disabled_confirmed)) { return; }
    if (report->active && !report->first_exit_latched) {
        /* This emergency entry already owns the task critical section. */
        report->first_exit_phase = report->phase;
        report->first_exit_reason = DEBUG_UI_REASON_STOP_LATCHED;
        report->first_exit_stage = DEBUG_UI_STAGE_HOLD;
        report->first_exit_error = 0U;
        report->first_exit_code = DEBUG_UI_STOPPED;
        report->first_exit_latched = 1U;
    }
    application_debug_ui.static_fallback_identity = report->identity;
    application_debug_ui.static_fallback_after_us = timestamp_us;
    application_debug_ui.static_fallback_bound = 1U;
}

/** @brief Confirms this trial's emitted emergency scope followed by newer selected safe ACKs. Caller holds critical. */
static void aethor_app_static_load_confirm_fallback(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    uint8_t index;
    uint8_t expected_frames = 0U;
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    aethor_app_static_trace_confirm_fallback(snapshot, timestamp_us);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
    { if (application_debug_ui.fallback_stop_mask & (1U << index)) { ++expected_frames; } }
    if (!application_debug_ui.static_fallback_bound || !report->motor_mask ||
        (snapshot->valid_joint_mask & report->motor_mask) != report->motor_mask ||
        (application_debug_ui.fallback_stop_mask & report->motor_mask) != report->motor_mask ||
        application_debug_ui.fallback_frames.count != expected_frames ||
        application_debug_ui.fallback_frame_read_index < expected_frames ||
        application_debug_ui.static_fallback_identity.origin != report->identity.origin ||
        application_debug_ui.static_fallback_identity.epoch != report->identity.epoch ||
        application_debug_ui.static_fallback_identity.request_id != report->identity.request_id) { return; }
    for (index = 0U; index < 7U; ++index)
    {
        const MotorJointFeedback *feedback = &snapshot->joints[index];
        if ((report->motor_mask & (1U << index)) == 0U) { continue; }
        if (feedback->timestamp_us <= application_debug_ui.static_fallback_after_us ||
            feedback->timestamp_us > timestamp_us || feedback->driver_state != S3519_DRIVER_STATE_DISABLED ||
            feedback->fault_flags || !isfinite(feedback->velocity_rad_s) ||
            fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) { return; }
    }
    report->active = report->complete = 0U;
    report->stopped = report->disabled_confirmed = 1U;
    application_debug_ui.static_fallback_bound = 0U;
}
#endif

/** @brief Count accepted generations during takeover/HOLD, including a violating RX before safety exits. */
static void aethor_app_static_load_sample(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    uint8_t joint_index;
    DebugUiStaticLoadReport *report = &application_debug_ui.static_load;
    uint8_t takeover = (uint8_t)(report->takeover_active &&
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT);
    if (!report->active || report->first_exit_latched ||
        (!takeover && report->frames_issued_mask != report->motor_mask) ||
        application_action.command.local_operation != report->operation ||
        application_action.command.motor_mask != report->motor_mask ||
        application_action.command.request_id != report->identity.request_id ||
        application_action.command.session_id != report->identity.epoch ||
        application_action.command.origin != report->identity.origin ||
        (!takeover && application_action.state != AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT)) { return; }
    aethor_app_enter_task_critical();
    if (!takeover && timestamp_us >= report->started_at_us)
    {
        uint64_t elapsed = (timestamp_us - report->started_at_us) / 1000ULL;
        report->elapsed_ms = elapsed > report->duration_ms ? report->duration_ms : (uint32_t)elapsed;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback = &snapshot->joints[joint_index];
        uint32_t generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        float error, torque;
        if ((report->motor_mask & (1U << joint_index)) == 0U) { continue; }
        if ((snapshot->valid_joint_mask & (1U << joint_index)) == 0U ||
            !generation || generation == report->sample_generation[joint_index] ||
            feedback->timestamp_us < report->started_at_us ||
            feedback->timestamp_us != application_motor_runtime.bank.motors[joint_index].feedback.timestamp_us ||
            feedback->timestamp_us > timestamp_us ||
            feedback->driver_state != S3519_DRIVER_STATE_ENABLED ||
            !isfinite(feedback->position_rad) || !isfinite(feedback->torque_nm) ||
            !isfinite(feedback->mos_temperature_c) || !isfinite(feedback->rotor_temperature_c)) { continue; }
        application_debug_ui.static_sample_us[joint_index] = feedback->timestamp_us;
        report->sample_generation[joint_index] = generation;
        error = fabsf(feedback->position_rad / aethor_app_lcd_position_ratio(joint_index) - report->target_position_rad[joint_index]);
        torque = fabsf(feedback->torque_nm);
        if (error > report->peak_error_rad[joint_index]) { report->peak_error_rad[joint_index] = error; }
        if (torque > report->peak_torque_nm[joint_index]) { report->peak_torque_nm[joint_index] = torque; }
        if (feedback->mos_temperature_c > report->peak_mos_c[joint_index]) { report->peak_mos_c[joint_index] = feedback->mos_temperature_c; }
        if (feedback->rotor_temperature_c > report->peak_rotor_c[joint_index]) { report->peak_rotor_c[joint_index] = feedback->rotor_temperature_c; }
        if (report->samples[joint_index] < UINT32_MAX) { ++report->samples[joint_index]; }
        report->valid_mask |= (uint8_t)(1U << joint_index);
    }
    aethor_app_exit_task_critical();
}
#endif
