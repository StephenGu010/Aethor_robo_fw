/** @file ID7-only experimental exposure adapter; all actuation remains in the existing executor. */
#if !AETHOR_ADRC_BENCH
/** @brief Immutable first guard failure for one local trial; independent of cleanup's mutable quota ledger. */
typedef struct {
    uint32_t epoch, request_id;
    double used_before;
    float absolute_nm, limit_nm;
    TorqueEnvelopeReason reason;
    uint8_t authorized_before, valid;
} AethorTorqueFaultEvidence;
static AethorTorqueFaultEvidence application_torque_fault;

/** @brief Restrict additional headroom to explicitly compiled ID7 single static/POS load operations. */
static uint8_t aethor_app_torque_envelope_scope(const ProtocolCommand *command, uint8_t joint_index)
{
    if (!command || joint_index != 6U || command->motor_mask != 0x40U ||
        command->origin != DEBUG_UI_ORIGIN_LOCAL_UI ||
        AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE) return 0U;
    return (uint8_t)((AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD &&
        command->local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) ||
        (AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST && command->local_operation == DEBUG_UI_OPERATION_POS_LOAD_TEST));
}

/** @brief Preserve the pre-revocation ledger for a first failure, including ordinary disable; cleanup cannot replace it. */
static TorqueEnvelopeReason aethor_app_torque_envelope_observe(const MotorJointFeedback *feedback, uint64_t timestamp_us)
{
    TorqueEnvelopeReason reason;
    uint8_t authorized_before = application_torque_envelope.authorized;
    double used_before = application_torque_envelope.exposure_nm2_s;
    float limit_before = torque_envelope_effective_limit(&application_torque_envelope);
    reason = torque_envelope_observe(&application_torque_envelope, timestamp_us,
        feedback->timestamp_us, feedback->torque_nm);
    if (reason != TORQUE_ENVELOPE_OK && application_action.failed_stage == PROTOCOL_COMMAND_STAGE_NONE &&
        application_action.local_failure_reason == DEBUG_UI_REASON_NONE && (!application_torque_fault.valid ||
        application_torque_fault.epoch != application_action.command.session_id ||
        application_torque_fault.request_id != application_action.command.request_id)) {
        application_torque_fault.epoch = application_action.command.session_id;
        application_torque_fault.request_id = application_action.command.request_id;
        application_torque_fault.used_before = used_before;
        application_torque_fault.absolute_nm = fabsf(feedback->torque_nm);
        application_torque_fault.limit_nm = limit_before;
        application_torque_fault.reason = reason;
        application_torque_fault.authorized_before = authorized_before;
        application_torque_fault.valid = 1U;
    }
    return reason;
}

/** @brief Map detailed exposure failures without weakening the original public error namespace. */
static DebugUiReason aethor_app_torque_envelope_guard(const MotorJointFeedback *feedback,
    uint8_t joint_index, uint64_t timestamp_us)
{
    TorqueEnvelopeReason reason;
    if (!aethor_app_torque_envelope_scope(&application_action.command, joint_index))
        return fabsf(feedback->torque_nm) > AETHOR_APP_LOCAL_MIT_TORQUE_TRIP_NM ?
            DEBUG_UI_REASON_TORQUE_LIMIT : DEBUG_UI_REASON_NONE;
    reason = aethor_app_torque_envelope_observe(feedback, timestamp_us);
    if (reason == TORQUE_ENVELOPE_OK) return DEBUG_UI_REASON_NONE;
    if (reason == TORQUE_ENVELOPE_FEEDBACK) return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (reason == TORQUE_ENVELOPE_CLOCK) return DEBUG_UI_REASON_UNHEALTHY;
    if (reason == TORQUE_ENVELOPE_INVALID) return DEBUG_UI_REASON_INVALID_FEEDBACK;
    return DEBUG_UI_REASON_TORQUE_LIMIT;
}

/** @brief Any failed scoped transaction consumes its remaining experimental credit before existing cleanup. */
static void aethor_app_torque_envelope_fault(void)
{
    if (aethor_app_torque_envelope_scope(&application_action.command, 6U))
        (void)torque_envelope_revoke(&application_torque_envelope,
            application_torque_envelope.last_reason == TORQUE_ENVELOPE_OK ?
                TORQUE_ENVELOPE_FEEDBACK : application_torque_envelope.last_reason);
}

/** @brief Preserve cumulative usage and only end observation after the original strict disabled completion. */
static void aethor_app_torque_envelope_finish(const ProtocolCommandResult *result)
{
    if (aethor_app_torque_envelope_scope(&application_action.command, 6U)) {
        if (result->code != PROTOCOL_COMMAND_RESULT_COMPLETED) aethor_app_torque_envelope_fault();
        torque_envelope_end(&application_torque_envelope, result->disabled_confirmed);
    }
}

/** @brief Keep charging during the ordinary disable phase; no new motion or recovery is generated. */
static void aethor_app_torque_envelope_background(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    if (!application_torque_envelope.tracking) return;
    if (!aethor_app_torque_envelope_scope(&application_action.command, 6U)) {
        torque_envelope_end(&application_torque_envelope, 0U);
        return;
    }
    if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT ||
        application_action.state == AETHOR_APP_ACTION_ONE_SHOT_CLEANUP_DISABLE_WAIT) {
        TorqueEnvelopeReason reason;
        if (!(snapshot->valid_joint_mask & 0x40U))
            reason = torque_envelope_revoke(&application_torque_envelope, TORQUE_ENVELOPE_FEEDBACK);
        else reason = aethor_app_torque_envelope_observe(&snapshot->joints[6], timestamp_us);
        if (reason != TORQUE_ENVELOPE_OK && application_action.state == AETHOR_APP_ACTION_ONE_SHOT_DISABLE_WAIT &&
            application_action.local_failure_reason == DEBUG_UI_REASON_NONE) {
            application_action.local_failure_reason = reason == TORQUE_ENVELOPE_FEEDBACK ? DEBUG_UI_REASON_STALE_FEEDBACK :
                reason == TORQUE_ENVELOPE_CLOCK ? DEBUG_UI_REASON_UNHEALTHY :
                reason == TORQUE_ENVELOPE_INVALID ? DEBUG_UI_REASON_INVALID_FEEDBACK : DEBUG_UI_REASON_TORQUE_LIMIT;
            aethor_app_record_one_shot_failure_once(PROTOCOL_COMMAND_STAGE_DISABLE,
                PROTOCOL_COMMAND_ERROR_ACTION_FAILED, 7U);
            aethor_app_static_load_latch_exit(application_action.local_failure_reason,
                DEBUG_UI_STAGE_DISABLE, PROTOCOL_COMMAND_ERROR_ACTION_FAILED, DEBUG_UI_FAILED);
        }
    }
}

/** @brief Enumerate maintenance blockers; the five-second cache never qualifies motion or enable.
 * Stable wire bits: 1 authority, 2 execution/queue, 4 unconfirmed disable, 8 discovery/identity,
 * 16 controller, 32 standard feedback, 64 driver state, 128 speed, 256 torque, 512 temperature,
 * 1024 compiled feature/configuration. Missing feedback reports bit 32 rather than fabricated field failures.
 */
static uint32_t aethor_app_torque_envelope_blockers(uint64_t timestamp_us)
{
    MotorFeedbackSnapshot snapshot;
    ArmSnapshot arm;
    const MotorJointFeedback *feedback;
    uint32_t blockers = 0U;
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    if (!(AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST) ||
        AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE) blockers |= 1024U;
    if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE) blockers |= 1U;
    if (aethor_app_debug_ui_executor_busy() ||
        debug_ui_mailbox_busy(&application_debug_ui.mailbox, false) ||
        debug_ui_mailbox_busy(&application_debug_ui.mailbox, true) ||
        application_emergency_disable_read_index < application_emergency_disable_batch.count ||
        application_debug_ui.fallback_frame_read_index < application_debug_ui.fallback_frames.count ||
        application_action.control_group_ready || application_action.frame_read_index < application_action.frames.count ||
        application_transport_fault_read_sequence != application_transport_fault_write_sequence ||
        !application_remote_release_can_idle || !application_remote_release_can_idle()) blockers |= 2U;
    if (application_debug_ui.unconfirmed_disable_mask) blockers |= 4U;
    if (application_motor_runtime.discovery_active ||
        discovery->verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
        discovery->observed_esc_id != 7U ||
        discovery->observed_master_id != arm_config_get_production()->joints[6].master_id ||
        !application_debug_ui.reference_generation[6] || !application_protocol_engine.session_id ||
        !application_protocol_engine.next_session_nonce ||
        application_protocol_engine.ui_session_nonce != application_protocol_engine.next_session_nonce ||
        !application_remote_release_link_word || !(application_remote_release_link_word() & 0x80000000UL) ||
        !application_protocol_engine.ui_session_link ||
        application_protocol_engine.ui_session_link != application_remote_release_link_word()) blockers |= 8U;
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD) blockers |= 1U;
#endif
    memset(&snapshot, 0, sizeof(snapshot));
    memset(&arm, 0, sizeof(arm));
    if (!arm_controller_get_snapshot(&application_controller, &arm) ||
        arm.enabled || arm.moving || arm.fault != ARM_FAULT_NONE) blockers |= 16U;
    (void)motor_runtime_get_snapshot(&application_motor_runtime, timestamp_us, 5000000ULL, &snapshot);
    if (!(snapshot.valid_joint_mask & 0x40U)) return blockers | 32U;
    feedback = &snapshot.joints[6];
    if (!feedback->timestamp_us || feedback->timestamp_us > timestamp_us ||
        timestamp_us - feedback->timestamp_us > 5000000ULL) blockers |= 32U;
    if (feedback->driver_state != S3519_DRIVER_STATE_DISABLED || feedback->fault_flags) blockers |= 64U;
    if (!isfinite(feedback->velocity_rad_s) || fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) blockers |= 128U;
    if (!isfinite(feedback->torque_nm) || fabsf(feedback->torque_nm) > TORQUE_ENVELOPE_CONTINUOUS_NM) blockers |= 256U;
    if (!isfinite(feedback->mos_temperature_c) || feedback->mos_temperature_c > 35.0F ||
        !isfinite(feedback->rotor_temperature_c) || feedback->rotor_temperature_c > 35.0F) blockers |= 512U;
    return blockers;
}
#endif
