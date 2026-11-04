/** @file debug_ui_static_transport_app.inc
 * @brief Full phase/axis/sequence identities at the shared App/Core/Platform boundary; no hardware access here.
 */
/** @brief Expose a POS-only emergency classifier with no legacy PRELOAD/PREFLIGHT debt side effects. */
uint8_t aethor_app_pos_load_terminal_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    return aethor_app_pos_disable_frame_identity(frame, identity);
#else
    (void)frame; (void)identity;
    return 0U;
#endif
}

/** @brief Select only exact active startup bytes or the unchanged strict selected-DISABLE preflight. */
uint8_t aethor_app_static_frame_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    DebugUiIdentity legacy;
    if (!frame || !identity) return 0U;
    if (aethor_app_pos_load_terminal_frame_identity(frame, identity)) return 1U;
    memset(identity, 0, sizeof(*identity));
    if (aethor_app_debug_ui_preflight_frame_identity(frame, &legacy)) {
        identity->epoch = legacy.epoch;
        identity->request_id = legacy.request_id;
        identity->sequence = 1U;
        identity->phase = DEBUG_UI_STATIC_TX_PREFLIGHT;
        identity->motor_id = (uint8_t)frame->identifier;
    } else {
        if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ||
            (!debug_ui_is_static_operation(application_action.command.local_operation) && !aethor_app_is_pos_load()) ||
            !application_action.static_identity.phase || application_action.frames.count != 1U ||
            frame->identifier != application_action.frames.frames[0].identifier || frame->length != 8U ||
            memcmp(frame->data, application_action.frames.frames[0].data, 8U)) return 0U;
        *identity = application_action.static_identity;
    }
    application_debug_ui.static_transport_identity = *identity;
    application_debug_ui.static_transport_frame = *frame;
    application_debug_ui.static_transport_submit_us = 0U;
    application_debug_ui.single_static_transport_mask = (uint8_t)(1U << (identity->motor_id - 1U));
    return 1U;
#else
    (void)frame; (void)identity;
    return 0U;
#endif
}

/** @brief Guard the actual HAL boundary, rejecting expired state, STOP/revoked ownership or any unsafe axis. */
uint8_t aethor_app_static_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    MotorFeedbackSnapshot snapshot;
    uint8_t failure = 0U, joint_index;
    if (!identity) return 0U;
    if (identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE)
        return aethor_app_pos_disable_submitted(identity, boundary_us);
    if (identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT)
        return (uint8_t)(debug_ui_static_tx_identity_equal(identity, &application_debug_ui.static_transport_identity) &&
            aethor_app_debug_ui_preflight_submitted(identity->epoch, identity->request_id, boundary_us));
    if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ||
        !debug_ui_static_tx_identity_equal(identity, &application_action.static_identity) ||
        application_action.single_static_tx_submitted ||
        boundary_us >= (identity->phase == DEBUG_UI_STATIC_TX_TAKEOVER ? application_action.deadline_us :
            application_action.static_setup_deadline_us) ||
        (identity->phase == DEBUG_UI_STATIC_TX_TAKEOVER &&
         (boundary_us < application_action.static_takeover_last_refresh_us ||
          boundary_us - application_action.static_takeover_last_refresh_us > AETHOR_STATIC_TAKEOVER_MAX_LATENESS_US)) ||
        boundary_us <= application_action.command.created_at_us ||
        application_debug_ui.epoch != identity->epoch || (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
         application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_BUSY) ||
        application_debug_ui.fallback_stop_mask || (application_debug_ui.mailbox.stop_intent_mask || debug_ui_mailbox_busy(&application_debug_ui.mailbox, true)) ||
        !debug_ui_mailbox_healthy(&application_debug_ui.mailbox, boundary_us)) return 0U;
    memset(&snapshot, 0, sizeof(snapshot));
    (void)motor_runtime_get_snapshot(&application_motor_runtime, boundary_us,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &snapshot);
    (void)aethor_app_check_one_shot_motor_safety(&snapshot, boundary_us, &failure);
    if (failure || application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT) return 0U;
    joint_index = (uint8_t)(identity->motor_id - 1U);
    application_action.single_static_rx_generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
    application_action.single_static_tx_submitted = 1U;
    application_action.static_submit_us = boundary_us;
    application_debug_ui.static_transport_submit_us = boundary_us;
    aethor_app_static_trace_record_identity(STATIC_TRACE_HAL_SUBMIT, identity->motor_id,
        &application_action.frames.frames[0], boundary_us, boundary_us, 0U, 0U, 0U, identity);
    return 1U;
#else
    (void)identity; (void)boundary_us;
    return 0U;
#endif
}

/** @brief Release only the matching transport debt; ENABLE completion immediately prepares identical first HOLD. */
void aethor_app_static_transmitted(const DebugUiStaticTxIdentity *identity, uint8_t succeeded, uint16_t sof, uint64_t observed_us)
{
#if !AETHOR_ADRC_BENCH && AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (!identity) return;
    if (identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE) {
        aethor_app_pos_disable_transmitted(identity, succeeded);
        return;
    }
    if (identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT) {
        if (!debug_ui_static_tx_identity_equal(identity, &application_debug_ui.static_transport_identity)) return;
        aethor_app_debug_ui_preflight_transmitted(identity->epoch, identity->request_id, succeeded);
        return;
    }
    if (debug_ui_static_tx_identity_equal(identity, &application_debug_ui.static_transport_identity)) {
        application_debug_ui.single_static_transport_mask = 0U;
        /* Cleanup may have replaced the action. Keep this late receipt's original bytes, phase and observation. */
        aethor_app_static_trace_record_identity(succeeded ? STATIC_TRACE_ACTUAL_TX : STATIC_TRACE_TX_UNKNOWN,
            identity->motor_id, &application_debug_ui.static_transport_frame,
            application_debug_ui.static_transport_submit_us, observed_us, sof, succeeded, 0U, identity);
    }
    if (application_action.state != AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ||
        !debug_ui_static_tx_identity_equal(identity, &application_action.static_identity) ||
        application_action.single_static_tx_confirmed || application_action.single_static_tx_failed) return;
    succeeded = (uint8_t)(succeeded && application_action.single_static_tx_submitted);
    if (!succeeded) application_action.single_static_tx_failed = 1U;
    else if (identity->phase == DEBUG_UI_STATIC_TX_ENABLE) {
        application_action.enabled_by_action_mask |= (uint8_t)(1U << (identity->motor_id - 1U));
        aethor_app_static_prepare_phase(DEBUG_UI_STATIC_TX_FIRST_HOLD,
            (uint8_t)(identity->motor_id - 1U), observed_us);
    } else {
        application_action.single_static_tx_confirmed = 1U;
        if (identity->phase == DEBUG_UI_STATIC_TX_TAKEOVER) {
            const CanFrame *transmitted = &application_action.frames.frames[0];
            uint16_t encoded_torque = (uint16_t)(((uint16_t)(transmitted->data[6] & 0x0FU) << 8U) |
                transmitted->data[7]);
            float torque_limit = application_motor_runtime.discovery.results[identity->motor_id - 1U].ranges.torque_max_nm;
            /* Decode the immutable actual-TX bytes, retaining the driver's 12-bit quantization. */
            application_debug_ui.static_load.transmitted_torque_ff_nm =
                (float)encoded_torque * (2.0F * torque_limit) / 4095.0F - torque_limit;
        }
    }
#else
    (void)identity; (void)succeeded; (void)sof; (void)observed_us;
#endif
}

/** @brief Record original raw RX and accepted decode independently; only a newer same-ID sample may satisfy a phase. */
MotorRuntimeStatus aethor_app_receive_static_frame(const CanFrame *frame, uint64_t lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *identity, uint16_t sof, uint8_t sof_valid)
{
#if !AETHOR_ADRC_BENCH
    uint32_t before[ARM_JOINT_COUNT];
    uint8_t joint_index;
    MotorRuntimeStatus status;
    DebugUiIdentity legacy = { DEBUG_UI_ORIGIN_USB, 0U, 0U };
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    const DebugUiStaticTxIdentity *trace_identity = application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT ? identity : NULL;
#endif
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        before[joint_index] = application_motor_runtime.bank.motors[joint_index].feedback_generation;
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    aethor_app_static_trace_record_identity(STATIC_TRACE_RAW_RX, 0U, frame, lower_us, observed_us, sof, sof_valid, 0U, trace_identity);
#else
    aethor_app_static_trace_record(STATIC_TRACE_RAW_RX, 0U, frame, lower_us, observed_us, sof, sof_valid, 0U);
#endif
    if (identity && identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT &&
        debug_ui_static_tx_identity_equal(identity, &application_debug_ui.static_transport_identity)) {
        legacy.origin = DEBUG_UI_ORIGIN_LOCAL_UI; legacy.epoch = identity->epoch; legacy.request_id = identity->request_id;
    }
    status = aethor_app_debug_ui_receive_preflight_frame(frame, lower_us, observed_us, &legacy);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        uint32_t after = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        if (after == before[joint_index]) continue;
        if (identity && identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE &&
            identity->motor_id == joint_index + 1U &&
            debug_ui_static_tx_identity_equal(identity, &application_debug_ui.pos_disable_identity) &&
            application_debug_ui.pos_disable_submitted && application_debug_ui.pos_disable_confirmed &&
            !application_debug_ui.pos_disable_failed && after != application_debug_ui.pos_disable_rx_generation)
            application_debug_ui.pos_disable_ordered_generation = after;
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        aethor_app_static_trace_record_identity(STATIC_TRACE_ACCEPTED_RX, (uint8_t)(joint_index + 1U), frame,
            lower_us, observed_us, sof, sof_valid, 0U, trace_identity);
#else
        aethor_app_static_trace_record(STATIC_TRACE_ACCEPTED_RX, (uint8_t)(joint_index + 1U), frame,
            lower_us, observed_us, sof, sof_valid, 0U);
#endif
        /* Sample each accepted static RX before safety may freeze its first exit and before a later RX replaces it. */
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD)
        if (application_debug_ui.static_load.active && debug_ui_is_static_operation(application_action.command.local_operation) &&
            (application_action.command.motor_mask & (1U << joint_index))) {
            MotorFeedbackSnapshot received_snapshot;
            memset(&received_snapshot, 0, sizeof(received_snapshot));
            (void)motor_runtime_get_snapshot(&application_motor_runtime, observed_us,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &received_snapshot);
            aethor_app_static_load_sample(&received_snapshot, observed_us);
        }
#endif
        /* Observe every accepted selected-axis takeover sample before another RX can replace it.
         * Cleanup is the sticky failure latch, including for samples without a transport identity. */
#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
        if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
            application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
            application_debug_ui.static_load.takeover_active &&
            (application_action.command.motor_mask & (1U << joint_index))) {
            MotorFeedbackSnapshot received_snapshot;
            uint8_t failure_detected = 0U;
            memset(&received_snapshot, 0, sizeof(received_snapshot));
            (void)motor_runtime_get_snapshot(&application_motor_runtime, observed_us,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &received_snapshot);
            (void)aethor_app_check_one_shot_motor_safety(&received_snapshot, observed_us, &failure_detected);
            if (failure_detected) continue;
            if (received_snapshot.joints[joint_index].driver_state != S3519_DRIVER_STATE_ENABLED) {
                application_action.local_failure_reason = DEBUG_UI_REASON_NOT_READY;
                (void)aethor_app_begin_one_shot_cleanup(PROTOCOL_COMMAND_STAGE_HOLD,
                    PROTOCOL_COMMAND_ERROR_ACTION_FAILED, (uint8_t)(joint_index + 1U), observed_us);
                continue;
            }
        }
#endif
        /* A same-trial single-axis HOLD must also fail on the accepted violating
         * sample, before a second RX can replace it ahead of the next service poll.
         * The seven-axis safety path keeps its existing control-service behavior. */
#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD
        if (application_action.state == AETHOR_APP_ACTION_ONE_SHOT_HOLD_WAIT &&
            application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
            application_debug_ui.static_load.active &&
            application_debug_ui.static_load.operation == application_action.command.local_operation &&
            application_debug_ui.static_load.motor_mask == application_action.command.motor_mask &&
            application_debug_ui.static_load.identity.origin == application_action.command.origin &&
            application_debug_ui.static_load.identity.epoch == application_action.command.session_id &&
            application_debug_ui.static_load.identity.request_id == application_action.command.request_id &&
            (application_action.command.motor_mask & (1U << joint_index))) {
            MotorFeedbackSnapshot received_snapshot;
            uint8_t failure_detected = 0U;
            memset(&received_snapshot, 0, sizeof(received_snapshot));
            (void)motor_runtime_get_snapshot(&application_motor_runtime, observed_us,
                MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &received_snapshot);
            (void)aethor_app_check_one_shot_motor_safety(&received_snapshot, observed_us, &failure_detected);
            if (failure_detected) continue;
        }
#endif
        if (identity && identity->motor_id == joint_index + 1U &&
            debug_ui_static_tx_identity_equal(identity, &application_action.static_identity) &&
            application_action.state == AETHOR_APP_ACTION_ONE_SHOT_ENABLE_WAIT &&
            application_action.single_static_tx_confirmed && !application_action.single_static_tx_failed &&
            after != application_action.single_static_rx_generation) {
            application_action.single_static_ordered_generation = after;
            if (identity->phase == DEBUG_UI_STATIC_TX_TAKEOVER && application_action.static_takeover_step == AETHOR_STATIC_TAKEOVER_STEP_COUNT &&
                !application_action.static_takeover_proven_us && observed_us <= application_action.deadline_us)
                application_action.static_takeover_proven_us = observed_us;
        }
    }
    return status;
#else
    (void)identity; (void)sof; (void)sof_valid;
    return aethor_app_receive_can_frame_observed(frame, lower_us, observed_us);
#endif
}

/** @brief Retain original scheduler-admitted bytes while distinguishing ordinary transmission's unknown outcome. */
void aethor_app_static_can_queued(const CanFrame *frame, uint8_t accepted, uint8_t strict, uint64_t observed_us)
{
#if !AETHOR_ADRC_BENCH
    aethor_app_pos_load_disable_queued(frame, accepted, observed_us);
    if (accepted) {
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        aethor_app_static_trace_record_identity(STATIC_TRACE_QUEUED, 0U, frame,
            observed_us, observed_us, 0U, 0U, 0U, strict ? &application_action.static_identity : NULL);
#else
        aethor_app_static_trace_record(STATIC_TRACE_QUEUED, 0U, frame, observed_us, observed_us, 0U, 0U, 0U);
#endif
    }
    if (!accepted || !strict) aethor_app_static_trace_record(STATIC_TRACE_TX_UNKNOWN, 0U, frame,
        observed_us, observed_us, 0U, 0U, 0U);
#else
    (void)frame; (void)accepted; (void)strict; (void)observed_us;
#endif
}
