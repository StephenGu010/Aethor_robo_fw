/** @file debug_ui_single_static_config_app.inc
 * @brief ProtocolTask-only LCD long-window installation and scoped safe relocking.
 * Included by aethor_app.c; configuration never creates an actuator command or CAN frame.
 */
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION
#if AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
/** @brief Recover only the same fault owner and its current LCD installation; caller separately checks idle/transaction state. */
static uint8_t aethor_app_single_static_fault_lock_owned(uint8_t joint_index)
{
    return (uint8_t)(application_debug_ui.target_motor_id == joint_index + 1U &&
        application_debug_ui.single_static_lcd_owned[joint_index] &&
        aethor_app_single_static_profile_current(joint_index));
}
#endif
/** @brief Validate one exact local DISABLE phase without granting authority or modifying RAM configuration. */
static DebugUiReason aethor_app_single_static_preflight_validate(const ProtocolCommand *command,
    const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us, bool require_feedback)
{
#if !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !AETHOR_DEBUG_UI_ALLOW_MIT
    (void)command; (void)snapshot; (void)timestamp_us; (void)require_feedback;
    return DEBUG_UI_REASON_DISABLED;
#else
    uint8_t joint_index, selected_index = 0U;
    const MotorDiscoveryResult *discovery;
    const DebugUiMotorProfile *profile;
    DebugUiMotorProfile scoped_profile;
    const MotorJointFeedback *feedback;
    DebugUiReason reason = protocol_engine_validate_typed_command(command);
    if (reason != DEBUG_UI_REASON_NONE) return reason;
    if (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE) return DEBUG_UI_REASON_DISABLED;
    if (command->session_id != application_debug_ui.epoch) return DEBUG_UI_REASON_OLD_EPOCH;
    if (timestamp_us < command->created_at_us || (!require_feedback &&
        timestamp_us - command->created_at_us > DEBUG_UI_REQUEST_MAX_AGE_US)) return DEBUG_UI_REASON_STALE_REQUEST;
    if (!debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us)) return DEBUG_UI_REASON_UNHEALTHY;
    if (application_debug_ui.unconfirmed_disable_mask || application_controller.fault) return DEBUG_UI_REASON_NOT_DISABLED;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        if (command->motor_mask & (1U << joint_index)) selected_index = joint_index;
    if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE &&
        ((application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED ||
          application_debug_ui.target_motor_id != selected_index + 1U) &&
         !(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
           command->single_static_preflight_purpose == DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK &&
           aethor_app_single_static_fault_lock_owned(selected_index)))) return DEBUG_UI_REASON_NOT_ARMED;
    if (application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
        (application_debug_ui.fallback_stop_mask || application_debug_ui.mailbox.stop_intent_mask ||
         application_protocol_engine.active_stop_request_id ||
         application_protocol_engine.stop_write_sequence != application_protocol_engine.stop_read_sequence ||
         (application_debug_ui.single_static_transport_mask & (uint8_t)~command->motor_mask))) return DEBUG_UI_REASON_BUSY;
    if (application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
        application_motor_runtime.bank.motors[selected_index].feedback.fault_flags) return DEBUG_UI_REASON_NOT_DISABLED;
    for (joint_index = 0U; joint_index < DEBUG_UI_MOTOR_COUNT; ++joint_index)
        if (joint_index != selected_index && application_debug_ui.single_static_lcd_owned[joint_index])
            return DEBUG_UI_REASON_NOT_ARMED;
    aethor_app_single_static_refresh_evidence(selected_index);
    if (command->reference_generation != application_debug_ui.reference_generation[selected_index] ||
        command->single_static_evidence_generation != application_debug_ui.single_static_evidence_generation[selected_index])
        return DEBUG_UI_REASON_OLD_EPOCH;
    discovery = &application_motor_runtime.discovery.results[selected_index];
    scoped_profile = aethor_app_single_static_bounds(selected_index);
    profile = &scoped_profile;
    if (discovery->observed_control_mode != 1U || discovery->verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
        discovery->observed_master_id != application_motor_runtime.configuration->joints[selected_index].master_id ||
        discovery->observed_esc_id != application_motor_runtime.configuration->joints[selected_index].esc_id)
        return DEBUG_UI_REASON_NOT_READY;
    if (!profile->mit_valid || !aethor_app_debug_ui_profile_valid(profile)) return DEBUG_UI_REASON_UNCALIBRATED;
    if (!require_feedback) return DEBUG_UI_REASON_NONE;
    feedback = &snapshot->joints[selected_index];
    if (!application_action.single_static_tx_submitted ||
        application_motor_runtime.bank.motors[selected_index].feedback_generation == application_action.single_static_rx_generation ||
        application_motor_runtime.bank.motors[selected_index].feedback_generation != application_action.single_static_ordered_generation ||
        !(snapshot->valid_joint_mask & command->motor_mask) || !feedback->timestamp_us ||
        feedback->timestamp_us > timestamp_us || feedback->timestamp_us <= application_action.phase_feedback_baseline_us[selected_index])
        return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (!isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) || !isfinite(feedback->torque_nm) ||
        !isfinite(feedback->mos_temperature_c) || !isfinite(feedback->rotor_temperature_c))
        return DEBUG_UI_REASON_INVALID_FEEDBACK;
    if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags ||
        fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) return DEBUG_UI_REASON_NOT_DISABLED;
    if (fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM) return DEBUG_UI_REASON_TORQUE_LIMIT;
    if (feedback->mos_temperature_c > 55.0F || feedback->rotor_temperature_c > 55.0F) return DEBUG_UI_REASON_TEMPERATURE_LIMIT;
    if (feedback->position_rad < profile->mit_position_min_rad || feedback->position_rad > profile->mit_position_max_rad)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    return DEBUG_UI_REASON_NONE;
#endif
}

/** @brief Admit only this immutable mailbox transaction under fresh selected-axis evidence. */
static DebugUiReason aethor_app_single_static_config_request(
    const DebugUiRequest *request, uint64_t timestamp_us)
{
#if !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
    (void)request;
    (void)timestamp_us;
    return DEBUG_UI_REASON_DISABLED;
#else
    MotorFeedbackSnapshot snapshot;
    const MotorDiscoveryResult *discovery;
    const DebugUiMotorProfile *motor_profile;
    DebugUiMotorProfile scoped_profile;
    const MotorJointFeedback *feedback;
    uint8_t joint_index = (uint8_t)(request->target_motor_id - 1U);
    uint8_t motor_mask = (uint8_t)(1U << joint_index);
    uint8_t other_index;
    uint8_t configure = (uint8_t)(request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE);
    uint8_t own_reservation = 0U;
    if (request->identity.origin != DEBUG_UI_ORIGIN_LOCAL_UI)
    { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    if (request->identity.epoch != application_debug_ui.epoch)
    { return DEBUG_UI_REASON_OLD_EPOCH; }
    if (timestamp_us < request->created_at_us ||
        timestamp_us - request->created_at_us > DEBUG_UI_REQUEST_MAX_AGE_US)
    { return DEBUG_UI_REASON_STALE_REQUEST; }
    if (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE)
    { return DEBUG_UI_REASON_DISABLED; }
    aethor_app_single_static_refresh_evidence(joint_index);
    if (request->reference_generation != application_debug_ui.reference_generation[joint_index] ||
        request->single_static_evidence_generation !=
            application_debug_ui.single_static_evidence_generation[joint_index])
    { return DEBUG_UI_REASON_OLD_EPOCH; }
    if (configure && (request->hold_duration_ms < DEBUG_UI_SINGLE_STATIC_MIN_MS ||
        request->hold_duration_ms > DEBUG_UI_SINGLE_STATIC_MAX_MS))
    { return DEBUG_UI_REASON_OUT_OF_RANGE; }
    if (!debug_ui_mailbox_healthy(&application_debug_ui.mailbox, timestamp_us))
    { return DEBUG_UI_REASON_UNHEALTHY; }
    if (aethor_app_debug_ui_executor_busy() ||
        application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_BUSY)
    { return DEBUG_UI_REASON_BUSY; }
    if (application_debug_ui.unconfirmed_disable_mask != 0U)
    { return DEBUG_UI_REASON_NOT_DISABLED; }
    if (configure && application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE)
    { return DEBUG_UI_REASON_NOT_ARMED; }
    if (!configure && application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE &&
        application_debug_ui.target_motor_id != request->target_motor_id)
    { return DEBUG_UI_REASON_NOT_ARMED; }
    if (!configure && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
        !aethor_app_single_static_fault_lock_owned(joint_index))
    { return DEBUG_UI_REASON_NOT_ARMED; }
    if (!configure && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && application_controller.fault)
    { return DEBUG_UI_REASON_NOT_DISABLED; }
    for (other_index = 0U; other_index < DEBUG_UI_MOTOR_COUNT; ++other_index)
    {
        if (other_index != joint_index && application_debug_ui.single_static_lcd_owned[other_index])
        { return DEBUG_UI_REASON_NOT_ARMED; }
    }
    /* A count of one is necessary but not enough: it must be this exact taken request. */
    for (other_index = 0U; other_index < DEBUG_UI_RESULT_CAPACITY; ++other_index)
    {
        const DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[other_index];
        if (transaction->used && transaction->taken && !transaction->admission_ready &&
            !transaction->completion_ready && transaction->request.operation == request->operation &&
            transaction->request.identity.origin == request->identity.origin &&
            transaction->request.identity.epoch == request->identity.epoch &&
            transaction->request.identity.request_id == request->identity.request_id)
        { own_reservation = 1U; }
    }
    if (debug_ui_mailbox_result_count(&application_debug_ui.mailbox) != 1U || !own_reservation)
    { return DEBUG_UI_REASON_RESULT_BACKPRESSURE; }
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    if ((snapshot.valid_joint_mask & motor_mask) != motor_mask)
    { return DEBUG_UI_REASON_STALE_FEEDBACK; }
    feedback = &snapshot.joints[joint_index];
    if (feedback->timestamp_us == 0ULL || feedback->timestamp_us > timestamp_us)
    { return DEBUG_UI_REASON_STALE_FEEDBACK; }
    if (!isfinite(feedback->position_rad) || !isfinite(feedback->velocity_rad_s) ||
        !isfinite(feedback->torque_nm) || !isfinite(feedback->mos_temperature_c) ||
        !isfinite(feedback->rotor_temperature_c))
    { return DEBUG_UI_REASON_INVALID_FEEDBACK; }
    if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags != 0U ||
        fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S)
    { return DEBUG_UI_REASON_NOT_DISABLED; }
    if (!configure && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT)
    {
        /* A successful preflight cannot qualify a later unsafe sample at the RAM withdrawal boundary. */
        scoped_profile = aethor_app_single_static_bounds(joint_index);
        motor_profile = &scoped_profile;
        if (fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM)
        { return DEBUG_UI_REASON_TORQUE_LIMIT; }
        if (feedback->mos_temperature_c > 55.0F || feedback->rotor_temperature_c > 55.0F)
        { return DEBUG_UI_REASON_TEMPERATURE_LIMIT; }
        if (feedback->position_rad < motor_profile->mit_position_min_rad ||
            feedback->position_rad > motor_profile->mit_position_max_rad)
        { return DEBUG_UI_REASON_OUT_OF_RANGE; }
    }
    if (configure)
    {
        DebugUiSingleStaticProfile profile;
        discovery = &application_motor_runtime.discovery.results[joint_index];
        scoped_profile = aethor_app_single_static_bounds(joint_index);
        motor_profile = &scoped_profile;
        if (!motor_profile->mit_valid || !aethor_app_debug_ui_profile_valid(motor_profile))
        { return DEBUG_UI_REASON_UNCALIBRATED; }
        if (discovery->observed_control_mode != 1U ||
            discovery->verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
            discovery->observed_master_id != application_motor_runtime.configuration->joints[joint_index].master_id ||
            discovery->observed_esc_id != application_motor_runtime.configuration->joints[joint_index].esc_id)
        { return DEBUG_UI_REASON_NOT_READY; }
        if (fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM)
        { return DEBUG_UI_REASON_TORQUE_LIMIT; }
        if (feedback->mos_temperature_c > 55.0F || feedback->rotor_temperature_c > 55.0F)
        { return DEBUG_UI_REASON_TEMPERATURE_LIMIT; }
        if (feedback->position_rad < motor_profile->mit_position_min_rad ||
            feedback->position_rad > motor_profile->mit_position_max_rad)
        { return DEBUG_UI_REASON_OUT_OF_RANGE; }
        if (!debug_ui_single_static_feedforward_valid(request->torque_ff_nm, discovery->ranges.torque_max_nm))
        { return DEBUG_UI_REASON_OUT_OF_RANGE; }
        if ((request->kp != 0.0F || request->kd != 0.0F) &&
            (joint_index != 6U || !debug_ui_static_gains_valid(request->kp, request->kd)))
        { return DEBUG_UI_REASON_OUT_OF_RANGE; }
        profile.torque_ff_nm = request->torque_ff_nm;
        profile.configured = 1U;
        profile.minimum_ms = DEBUG_UI_SINGLE_STATIC_MIN_MS;
        profile.maximum_ms = request->hold_duration_ms;
        aethor_app_single_static_install(joint_index, &profile, 1U);
        application_debug_ui.single_static_gains[joint_index].kp = request->kp;
        application_debug_ui.single_static_gains[joint_index].kd = request->kd;
    }
    else
    {
        if (application_debug_ui.single_static_lcd_owned[joint_index])
        {
            DebugUiSingleStaticProfile profile;
            memset(&profile, 0, sizeof(profile));
            aethor_app_single_static_install(joint_index, &profile, 0U);
        }
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE)
        { aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_REMOTE); }
    }
    application_debug_ui.last_activity_us = timestamp_us;
    return DEBUG_UI_REASON_NONE;
#endif
}
#endif
