/** @file motor_standby_recovery_app.inc
 * @brief Idle LCD-owned recovery uses the existing bounded 0x33 discovery and quarantine only.
 */
#if AETHOR_IDLE_FEEDBACK_ENABLE
/** @brief Saturate scheduling deadlines instead of wrapping at the uint64 clock boundary. */
static uint64_t aethor_app_standby_deadline(uint64_t now_us, uint64_t delay_us)
{
    return now_us > UINT64_MAX - delay_us ? UINT64_MAX : now_us + delay_us;
}

/** @brief Exclude every foreground owner, pending command/result/STOP, maintenance and unsafe control state. */
static uint8_t aethor_app_standby_safe(uint64_t now_us)
{
    return (uint8_t)(application_initialized &&
        application_action.state == AETHOR_APP_ACTION_IDLE &&
        !application_debug_ui.pos_disable_pending && !application_debug_ui.single_static_transport_mask &&
        !application_controller.enabled && !application_controller.moving &&
        application_controller.fault == ARM_FAULT_NONE &&
        !application_debug_ui.unconfirmed_disable_mask && !application_action.control_group_ready &&
        !application_debug_ui.fallback_stop_mask && !application_debug_ui.mailbox.stop_intent_mask &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) &&
        !debug_ui_mailbox_busy(&application_debug_ui.mailbox, false) &&
        !debug_ui_mailbox_result_count(&application_debug_ui.mailbox) &&
        application_protocol_engine.command_read_sequence == application_protocol_engine.command_write_sequence &&
        application_protocol_engine.stop_read_sequence == application_protocol_engine.stop_write_sequence &&
        application_protocol_engine.result_read_sequence == application_protocol_engine.result_write_sequence &&
        !application_protocol_engine.active_motion_request_id && !application_protocol_engine.active_stop_request_id &&
        !aethor_app_deferred_result_count() && !application_remote_ui.owns && !application_remote_ui.active &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        application_action.frame_read_index >= application_action.frames.count &&
        application_transport_fault_read_sequence == application_transport_fault_write_sequence &&
        !application_motor_runtime.position_read.pending && !application_motor_runtime.position_read_reserved &&
        now_us >= application_motor_runtime.position_read.not_before_us &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_WRITING &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_READ_READY &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_READ_WAITING
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
        && !aethor_app_persistence_locked()
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
        && application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
        !aethor_app_mapping_locked() && application_coordinate_pair.state != 1U &&
        (!application_integrated_diag_requested_us ||
         (now_us >= application_integrated_diag_requested_us &&
          now_us - application_integrated_diag_requested_us > ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US))
#endif
        );
}

/** @brief Retire only this background round; failed partial identity never becomes a published reference. */
static void aethor_app_standby_finish(uint64_t now_us, uint8_t success)
{
    if (!success) {
        (void)motor_runtime_abort_active_parameter_sequences(&application_motor_runtime, now_us);
        memset(&application_motor_runtime.discovery.results[6], 0, sizeof(MotorDiscoveryResult));
        application_motor_runtime.discovery.verified_joint_mask &= (uint8_t)~0x40U;
        application_motor_runtime.bank.motors[6].parameter_valid_mask = 0U;
        application_motor_runtime.bank.motors[6].configuration_consistent = 0U;
        application_motor_runtime.discovery.state = MOTOR_DISCOVERY_STATE_FAILED;
    }
    if (success) application_standby_recovery.lost_reference = 0U;
    application_standby_recovery.owned = 0U;
    application_standby_recovery.healthy_since_us = success ? now_us : 0U;
    application_standby_recovery.next_attempt_us = aethor_app_standby_deadline(now_us,
        MOTOR_STANDBY_RECOVERY_COOLDOWN_US);
}

/** @brief Cancel before owner/maintenance early returns and rebase safely if the clock moves backwards. */
static void aethor_app_standby_service(uint64_t now_us)
{
    MotorStandbyRecovery *recovery = &application_standby_recovery;
    uint8_t rollback = (uint8_t)(now_us < recovery->last_service_us);
    if (recovery->owned) {
        if (rollback || !aethor_app_standby_safe(now_us) || now_us < recovery->started_us ||
            now_us - recovery->started_us >= MOTOR_STANDBY_RECOVERY_BUDGET_US) {
            aethor_app_standby_finish(now_us, 0U);
        } else if (!application_motor_runtime.discovery_active) {
            aethor_app_standby_finish(now_us, (uint8_t)(
                application_motor_runtime.discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE &&
                (application_motor_runtime.discovery.verified_joint_mask & 0x40U) &&
                application_motor_runtime.discovery.results[6].verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK));
        }
    }
    if (!recovery->owned && recovery->lost_reference && !application_motor_runtime.discovery_active &&
        (application_motor_runtime.discovery.verified_joint_mask & 0x40U) &&
        application_motor_runtime.discovery.results[6].verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK) {
        /* A foreground owner can complete the replacement discovery after preempting our round. */
        recovery->lost_reference = 0U;
        recovery->healthy_since_us = now_us;
    }
    if (!aethor_app_standby_safe(now_us)) {
        /* Foreground work intentionally pauses display reads; give 0x50 a new observation window. */
        recovery->healthy_since_us = now_us;
        recovery->next_attempt_us = aethor_app_standby_deadline(now_us, MOTOR_STANDBY_RECOVERY_COOLDOWN_US);
    }
    if (rollback) {
        recovery->next_attempt_us = aethor_app_standby_deadline(now_us, MOTOR_STANDBY_RECOVERY_COOLDOWN_US);
        recovery->healthy_since_us = now_us;
    }
    recovery->last_service_us = now_us;
}

/** @brief Wait for long position silence, then start at most one bounded pass without changing modes or feedback. */
static MotorRuntimeStatus aethor_app_standby_next(uint64_t now_us, CanFrame *frame)
{
    MotorStandbyRecovery *recovery = &application_standby_recovery;
    const MotorRegisterPosition *reader = &application_motor_runtime.idle_position_read;
    uint8_t verified = (uint8_t)((application_motor_runtime.discovery.verified_joint_mask & 0x40U) &&
        application_motor_runtime.discovery.results[6].verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK);
    uint64_t latest_us = recovery->healthy_since_us;
    if (!aethor_app_standby_safe(now_us)) return MOTOR_RUNTIME_STATUS_WAITING;
#if AETHOR_ADRC_LCD_INTEGRATED
    if (!application_integrated_can_idle || now_us < application_integrated_can_quiet_since_us ||
        now_us - application_integrated_can_quiet_since_us < 4000ULL) return MOTOR_RUNTIME_STATUS_WAITING;
#endif
    if (!recovery->owned) {
        if (application_motor_runtime.discovery_active || reader->pending ||
            now_us < reader->not_before_us || now_us < recovery->next_attempt_us ||
            now_us > UINT64_MAX - MOTOR_STANDBY_RECOVERY_BUDGET_US) return MOTOR_RUNTIME_STATUS_WAITING;
        if (verified) {
            if (!latest_us) { recovery->healthy_since_us = now_us; return MOTOR_RUNTIME_STATUS_WAITING; }
            if (reader->joint_index == 6U && (reader->seen_mask & 1U) &&
                reader->sample_us[0] <= now_us && reader->sample_us[0] > latest_us)
                latest_us = reader->sample_us[0];
            if (application_motor_runtime.discovery.results[6].observed_control_mode != 2U) {
                uint64_t feedback_us = application_motor_runtime.bank.motors[6].feedback.timestamp_us;
                if (feedback_us <= now_us && feedback_us > latest_us) latest_us = feedback_us;
            }
            if (now_us < latest_us || now_us - latest_us < MOTOR_STANDBY_RECOVERY_STALE_US)
                return MOTOR_RUNTIME_STATUS_WAITING;
        }
        if (motor_runtime_begin_discovery(&application_motor_runtime, 0x40U) != MOTOR_RUNTIME_STATUS_OK)
            return MOTOR_RUNTIME_STATUS_WAITING;
        if (verified && !recovery->lost_reference) {
            /* A long outage may reset the encoder origin even with identical parameters. */
            ++application_debug_ui.reference_generation[6];
            recovery->lost_reference = 1U;
        }
        recovery->owned = 1U;
        recovery->started_us = now_us;
    }
    return motor_runtime_next_discovery_frame(&application_motor_runtime, now_us, frame);
}
#endif

