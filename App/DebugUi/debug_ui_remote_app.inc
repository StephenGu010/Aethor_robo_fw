/** @file debug_ui_remote_app.inc
 * @brief Fixed ID7 remote proxy into the existing guarded LCD executor. No CAN construction.
 * Include before aethor_app_process_protocol_line. All helpers run under App critical hooks.
 */
#include "debug_ui_pos_load_units.h"
#if !AETHOR_ADRC_BENCH
/** @brief One frozen review and one retained remote transaction; IDs never consume LCD IDs. */
typedef struct {
    DebugUiSnapshot review;
    uint32_t token, sequence, external_id, internal_id, owner_epoch;
    uint32_t terminal_id;
    uint32_t last_external_id, lease_token;
    uint64_t heartbeat_us;
    uint64_t torque_review_us;
    uint32_t torque_token, torque_boot, torque_epoch, torque_reference;
    uint32_t torque_session, torque_nonce, torque_link;
    char terminal_detail[320];
    uint8_t owns, active;
    uint32_t request_session, review_session, terminal_session, terminal_epoch;
    uint8_t terminal_accepted, terminal_disabled, terminal_mask, terminal_operation;
    uint32_t request_nonce, review_nonce, terminal_nonce;
    uint32_t request_link, review_link, terminal_link;
    uint32_t stop_generation, request_stop_generation, terminal_stop_generation;
    uint32_t release_internal_id, release_terminal_id;
    uint8_t terminal_code;
} AethorRemoteUi;
static AethorRemoteUi application_remote_ui;

/** @brief Mirror LCD's intent-only preflight eligibility; cached state is never execution feedback. */
static uint8_t aethor_remote_can_defer_feedback(const DebugUiSnapshot *snapshot)
{
    const DebugUiMotorView *motor = &snapshot->motors[6];
    return (uint8_t)(snapshot->motion_enabled && snapshot->bench_profile &&
        !snapshot->unconfirmed_disable_mask && !snapshot->arm_fault &&
        motor->motor_id == 7U && motor->reference_generation && motor->identity_verified &&
        motor->mode_verified && motor->ranges_verified &&
        ((motor->actual_mode == DEBUG_UI_MODE_POS_VEL && motor->profile.pos_valid) ||
         (motor->actual_mode == DEBUG_UI_MODE_MIT && snapshot->mit_enabled && motor->profile.mit_valid)) &&
        !motor->enabled && motor->driver_state == S3519_DRIVER_STATE_DISABLED && !motor->fault_flags &&
        isfinite(motor->position_rad) && isfinite(motor->velocity_rad_s) &&
        fabsf(motor->velocity_rad_s) <= AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S);
}

/** @brief Limit deferral to ownership-only actions and operations with the existing mandatory DISABLE preflight. */
static uint8_t aethor_remote_operation_defers_feedback(DebugUiOperation operation)
{
    return (uint8_t)(operation == DEBUG_UI_OPERATION_ACQUIRE || operation == DEBUG_UI_OPERATION_RELEASE ||
        operation == DEBUG_UI_OPERATION_POS_MOVE || debug_ui_is_pos_load_operation(operation) ||
        operation == DEBUG_UI_OPERATION_SET_MODE);
}

/** @brief Audit only the immediately preceding remote motor trial; never borrow a real LCD session or old hello. */
static uint8_t aethor_remote_fault_release_core(void)
{
    ArmSnapshot current_arm;
    uint32_t next_epoch = application_remote_ui.terminal_epoch + 1U;
    DebugUiOperation operation = (DebugUiOperation)application_remote_ui.terminal_operation;
    if (!arm_controller_get_snapshot(&application_controller, &current_arm) ||
        current_arm.fault != ARM_FAULT_NONE || current_arm.enabled || current_arm.moving) return 0U;
    if (!next_epoch) next_epoch = 1U;
    return (uint8_t)(application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
        application_debug_ui.target_motor_id == 7U && application_remote_ui.terminal_epoch &&
        application_remote_ui.terminal_epoch == application_remote_ui.owner_epoch &&
        application_debug_ui.epoch == next_epoch && application_remote_ui.terminal_session &&
        application_remote_ui.terminal_session == application_protocol_engine.session_id &&
        application_remote_ui.review_session == application_protocol_engine.session_id &&
        application_remote_ui.terminal_nonce == application_protocol_engine.next_session_nonce &&
        application_remote_ui.review_nonce == application_protocol_engine.next_session_nonce &&
        application_remote_ui.terminal_link && application_remote_release_link_word &&
        (application_remote_ui.terminal_link & 0x80000000UL) &&
        application_remote_ui.terminal_link == application_remote_release_link_word() &&
        application_remote_ui.review_link == application_remote_ui.terminal_link &&
        application_remote_ui.terminal_stop_generation == application_remote_ui.stop_generation &&
        application_remote_ui.terminal_code == DEBUG_UI_FAILED && application_remote_ui.terminal_id &&
        application_remote_ui.terminal_accepted && application_remote_ui.terminal_disabled &&
        application_remote_ui.terminal_mask == 0x40U &&
        (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD || operation == DEBUG_UI_OPERATION_POS_LOAD_TEST ||
         operation == DEBUG_UI_OPERATION_POS_MOVE || operation == DEBUG_UI_OPERATION_HOME) &&
        !application_debug_ui.unconfirmed_disable_mask &&
        !aethor_app_debug_ui_executor_busy() && !application_motor_runtime.discovery_active &&
        application_emergency_disable_read_index >= application_emergency_disable_batch.count &&
        application_debug_ui.fallback_frame_read_index >= application_debug_ui.fallback_frames.count &&
        !application_action.control_group_ready && application_action.frame_read_index >= application_action.frames.count &&
        application_transport_fault_read_sequence == application_transport_fault_write_sequence &&
        application_remote_release_can_idle && application_remote_release_can_idle() &&
        aethor_remote_can_defer_feedback(&application_remote_ui.review));
}

/** @brief Capabilities and submit require an empty ordinary mailbox before reserving the sole release. */
static uint8_t aethor_remote_fault_release_ready(void)
{
    return (uint8_t)(aethor_remote_fault_release_core() && !application_remote_ui.active &&
        application_remote_ui.last_external_id == application_remote_ui.terminal_id &&
        !debug_ui_mailbox_result_count(&application_debug_ui.mailbox));
}

/** @brief Recheck live platform debt and exact own reservation at execution, after the capability could have aged. */
#if AETHOR_DEBUG_UI_ALLOW_MOTION
static uint8_t aethor_remote_fault_release_admission(const DebugUiRequest *request)
{
    unsigned index;
    if (!request || !aethor_remote_fault_release_core() || !application_remote_ui.active ||
        request->identity.epoch != application_debug_ui.epoch || request->target_motor_id != 7U ||
        !request->remote_proxy || request->operation != DEBUG_UI_OPERATION_RELEASE ||
        request->identity.request_id != application_remote_ui.release_internal_id ||
        request->identity.request_id != application_remote_ui.internal_id ||
        application_remote_ui.release_terminal_id != application_remote_ui.terminal_id ||
        application_remote_ui.last_external_id != application_remote_ui.external_id ||
        debug_ui_mailbox_result_count(&application_debug_ui.mailbox) != 1U) return 0U;
    for (index = 0U; index < DEBUG_UI_RESULT_CAPACITY; ++index) {
        const DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[index];
        if (transaction->used && transaction->taken && !transaction->admission_ready && !transaction->completion_ready &&
            transaction->request.remote_proxy && transaction->request.operation == DEBUG_UI_OPERATION_RELEASE &&
            transaction->request.identity.request_id == request->identity.request_id &&
            transaction->request.identity.epoch == request->identity.epoch &&
            transaction->request.target_motor_id == 7U) return 1U;
    }
    return 0U;
}
#endif

/** @brief Clear proxy review, ownership and retained results at application initialization. */
static void aethor_app_remote_reset(void)
{
    memset(&application_remote_ui, 0, sizeof(application_remote_ui));
}

/** @brief Any STOP intent while remotely owned first latches ID7 fail-safe work, even if its later parsing fails. */
static void aethor_app_remote_stop_intent(void)
{
    if (++application_remote_ui.stop_generation == 0U) {
        ++application_remote_ui.stop_generation;
        application_remote_ui.terminal_epoch = 0U;
    }
    application_remote_ui.token = 0U;
    application_remote_ui.torque_token = 0U;
    if (application_remote_ui.owns) {
        application_debug_ui.mailbox.stop_intent_mask |= 0x40U;
        aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    }
    application_remote_ui.owns = 0U;
    application_remote_ui.lease_token = 0U;
}

/** @brief Independent remote liveness; expiry requests the existing STOP fallback and revokes stale authority. */
static void aethor_app_remote_service(uint64_t now_us)
{
    if (application_remote_ui.owns && (now_us < application_remote_ui.heartbeat_us ||
        now_us - application_remote_ui.heartbeat_us > 5000000ULL)) {
        application_remote_ui.owns = 0U; application_remote_ui.token = 0U; application_remote_ui.lease_token = 0U;
        application_debug_ui.mailbox.stop_intent_mask |= 0x40U;
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT)
            aethor_app_debug_ui_revoke(DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    }
}

/** @brief Local STOP always passes and invalidates remote review; local non-STOP cannot borrow proxy ownership. */
static DebugUiReason aethor_app_remote_local_gate(const DebugUiRequest *request)
{
    if (application_remote_ui.owns && application_remote_ui.owner_epoch &&
        application_remote_ui.owner_epoch != application_debug_ui.epoch) {
        application_remote_ui.owns = 0U; application_remote_ui.token = 0U;
    }
    if (!request) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (request->remote_proxy) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (request->operation == DEBUG_UI_OPERATION_STOP) {
        aethor_app_remote_stop_intent();
        return DEBUG_UI_REASON_NONE;
    }
    return application_remote_ui.owns ? DEBUG_UI_REASON_BUSY : DEBUG_UI_REASON_NONE;
}

/** @brief Strict decimal uint parser, rejecting signs, overflow and empty tokens. */
static uint8_t aethor_remote_uint(const TextProtocolRequest *request, const char *key, uint32_t *value)
{
    TextProtocolSpan span; size_t index; uint32_t parsed = 0U;
    if (!text_protocol_find_field(request, key, &span) || !span.length) return 0U;
    for (index = 0U; index < span.length; ++index) {
        unsigned digit = (unsigned)(span.data[index] - '0');
        if (digit > 9U || parsed > (UINT32_MAX - digit) / 10U) return 0U;
        parsed = parsed * 10U + digit;
    }
    *value = parsed; return 1U;
}

/** @brief Parse one finite strict decimal through the existing bounded parser. */
static uint8_t aethor_remote_float(const TextProtocolRequest *request, const char *key, float *value)
{
    TextProtocolSpan span;
    return (uint8_t)(text_protocol_find_field(request, key, &span) &&
        text_protocol_span_to_float(&span, value) == TEXT_PROTOCOL_STATUS_OK && isfinite(*value));
}

/** @brief Return one correlated bounded response without asynchronous USB result consumption. */
static void aethor_remote_reply(ProtocolOutputBatch *output, uint32_t id, const char *path,
    const char *detail, uint8_t failed)
{
    int written;
    memset(output, 0, sizeof(*output));
    written = snprintf(output->messages[0].data, sizeof(output->messages[0].data),
        "%s %lu ui %s schema=1 %s\r\n", failed ? "error" : "ok", (unsigned long)id, path, detail);
    if (written > 0 && (size_t)written < sizeof(output->messages[0].data)) {
        output->messages[0].length = (uint16_t)written; output->count = 1U;
    }
}

/** @brief Consume only the proxy's retained completion; LCD admissions and results remain untouched. */
static void aethor_remote_result(ProtocolOutputBatch *output, uint32_t id, uint32_t original)
{
    unsigned index; char detail[320];
    if (original && original == application_remote_ui.terminal_id) {
        aethor_remote_reply(output, id, "result", application_remote_ui.terminal_detail, 0U); return;
    }
    for (index = 0U; index < DEBUG_UI_RESULT_CAPACITY; ++index) {
        DebugUiMailboxTransaction *transaction = &application_debug_ui.mailbox.transactions[index];
        if (!transaction->used || !transaction->request.remote_proxy ||
            transaction->request.identity.request_id != application_remote_ui.internal_id ||
            original != application_remote_ui.external_id) continue;
        if (transaction->completion_ready && transaction->admission_ready) {
            const DebugUiCompletion *completion = &transaction->completion;
            if (transaction->request.operation == DEBUG_UI_OPERATION_ACQUIRE) {
                if (!transaction->admission.accepted || completion->code != DEBUG_UI_COMPLETED)
                    application_remote_ui.owns = 0U;
                else application_remote_ui.owner_epoch = application_debug_ui.epoch;
            }
            if (transaction->request.operation == DEBUG_UI_OPERATION_RELEASE && completion->code == DEBUG_UI_COMPLETED)
                application_remote_ui.owns = 0U;
            /* Diagnostic fields are append-only: a failed transaction never implies proven disablement. */
            (void)snprintf(detail, sizeof(detail), "id=%lu result=%s accepted=%u reason=%u disabled=%u stage=%u error=%u detail=%u motor=%u fb_us=%llu",
                (unsigned long)original, completion->code == DEBUG_UI_COMPLETED ? "completed" : "failed",
                transaction->admission.accepted, completion->reason, completion->disabled_confirmed,
                (unsigned int)completion->stage, (unsigned int)completion->error,
                (unsigned int)completion->detail, (unsigned int)completion->failed_motor_id,
                (unsigned long long)completion->feedback_timestamp_us);
            /* Bind read-only first-fault evidence to the original admitted trial, never to a later cleanup or release. */
            if (transaction->admission.accepted && completion->code == DEBUG_UI_FAILED &&
                completion->reason == DEBUG_UI_REASON_TORQUE_LIMIT && application_torque_fault.valid &&
                application_torque_fault.epoch == transaction->request.identity.epoch &&
                application_torque_fault.request_id == transaction->request.identity.request_id &&
                isfinite(application_torque_fault.used_before) && isfinite(application_torque_fault.absolute_nm)) {
                size_t detail_length = strlen(detail);
                (void)snprintf(detail + detail_length, sizeof(detail) - detail_length,
                    " tq_reason=%u tq_auth=%u tq_used=%.6f tq_nm=%.4f tq_limit=%.3f",
                    (unsigned int)application_torque_fault.reason, application_torque_fault.authorized_before,
                    application_torque_fault.used_before, (double)application_torque_fault.absolute_nm,
                    (double)application_torque_fault.limit_nm);
            }
            application_remote_ui.terminal_id = original;
            application_remote_ui.terminal_epoch = transaction->request.identity.epoch;
            application_remote_ui.terminal_session = application_remote_ui.request_session;
            application_remote_ui.terminal_accepted = transaction->admission.accepted;
            application_remote_ui.terminal_disabled = completion->disabled_confirmed;
            application_remote_ui.terminal_mask = completion->motor_mask;
            application_remote_ui.terminal_operation = (uint8_t)transaction->request.operation;
            application_remote_ui.terminal_code = (uint8_t)completion->code;
            application_remote_ui.terminal_nonce = application_remote_ui.request_nonce;
            application_remote_ui.terminal_link = application_remote_ui.request_link;
            application_remote_ui.terminal_stop_generation = application_remote_ui.request_stop_generation;
            memcpy(application_remote_ui.terminal_detail, detail, sizeof(detail));
            memset(transaction, 0, sizeof(*transaction)); application_remote_ui.active = 0U;
        } else (void)snprintf(detail, sizeof(detail), "id=%lu result=pending accepted=%u",
            (unsigned long)original, transaction->admission_ready && transaction->admission.accepted);
        aethor_remote_reply(output, id, "result", detail, 0U); return;
    }
    aethor_remote_reply(output, id, "result", "code=unknown_transaction", 1U);
}

/** @brief Match the immutable short operation list without accepting prefixes. */
static uint8_t aethor_remote_operation(const TextProtocolRequest *request, DebugUiOperation *operation, DebugUiMode *mode)
{
    static const char *names[] = { "acquire", "release", "pos", "configure", "hold", "lock", "pos_load", "preflight", "mode_mit", "mode_pos", "home" };
    static const DebugUiOperation operations[] = { DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_OPERATION_RELEASE,
        DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE,
        DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK, DEBUG_UI_OPERATION_POS_LOAD_TEST,
        DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT, DEBUG_UI_OPERATION_SET_MODE, DEBUG_UI_OPERATION_SET_MODE, DEBUG_UI_OPERATION_HOME };
    TextProtocolSpan span; unsigned index;
    if (!text_protocol_find_field(request, "op", &span)) return 0U;
    for (index = 0U; index < sizeof(operations) / sizeof(operations[0]); ++index)
        if (span.length == strlen(names[index]) && !memcmp(span.data, names[index], span.length)) {
            *operation = operations[index];
            if (index == 8U) *mode = DEBUG_UI_MODE_MIT;
            if (index == 9U) *mode = DEBUG_UI_MODE_POS_VEL;
            return 1U;
        }
    return 0U;
}

/** @brief Freeze a review token then submit exactly one existing guarded request; never enable or preflight implicitly. */
static uint8_t aethor_app_remote_route(const char *line, size_t length, uint64_t now_us, ProtocolOutputBatch *output)
{
    TextProtocolRequest wire; DebugUiRequest request; DebugUiReason reason = DEBUG_UI_REASON_INVALID_ARGUMENT;
    char detail[440]; uint32_t token = 0U; uint8_t caps, result, submit, heartbeat, torque, torque_reset;
    if (!output || text_protocol_parse_request(line, length, &wire) != TEXT_PROTOCOL_STATUS_OK) return 0U;
    aethor_app_remote_service(now_us);
    if (text_protocol_request_path_equals(&wire, "stop", NULL) ||
        text_protocol_request_path_equals(&wire, "arm", "stop") ||
        text_protocol_request_path_equals(&wire, "bench", "stop") ||
        text_protocol_request_path_equals(&wire, "adrc", "stop") ||
        text_protocol_request_path_equals(&wire, "disable", NULL) ||
        text_protocol_request_path_equals(&wire, "bench", "disable")) {
        aethor_app_remote_stop_intent(); return 0U;
    }
    caps = text_protocol_request_path_equals(&wire, "ui", "caps");
    result = text_protocol_request_path_equals(&wire, "ui", "result");
    submit = text_protocol_request_path_equals(&wire, "ui", "submit");
    heartbeat = text_protocol_request_path_equals(&wire, "ui", "heartbeat");
    torque = text_protocol_request_path_equals(&wire, "ui", "torque");
    torque_reset = text_protocol_request_path_equals(&wire, "ui", "torque_reset");
    if (!caps && !result && !submit && !heartbeat && !torque && !torque_reset) return 0U;
    if (!wire.has_request_id || !wire.request_id || wire.positional_count) goto rejected;
    if (torque && !wire.field_count) {
        int written;
        uint32_t blockers = aethor_app_torque_envelope_blockers(now_us);
        uint8_t can_reset;
        if (application_remote_ui.owns) blockers |= 1U;
        if (application_remote_ui.active) blockers |= 2U;
        can_reset = (uint8_t)(blockers == 0U);
        if (!application_remote_ui.sequence) application_remote_ui.sequence = application_protocol_engine.boot_id;
        if (++application_remote_ui.sequence == 0U) ++application_remote_ui.sequence;
        application_remote_ui.torque_token = can_reset ? application_remote_ui.sequence : 0U;
        application_remote_ui.torque_review_us = now_us;
        application_remote_ui.torque_boot = application_protocol_engine.boot_id;
        application_remote_ui.torque_epoch = application_debug_ui.epoch;
        application_remote_ui.torque_reference = application_debug_ui.reference_generation[6];
        application_remote_ui.torque_session = application_protocol_engine.session_id;
        application_remote_ui.torque_nonce = application_protocol_engine.next_session_nonce;
        application_remote_ui.torque_link = application_remote_release_link_word ? application_remote_release_link_word() : 0U;
        written = snprintf(detail, sizeof(detail),
            "result=policy experimental=1 motor=7 axis=output continuous=3.5 peak=%.3f window_ms=10000 budget=175.616 used=%.6f authorized=%u reason=%u can_reset=%u token=%lu boot=%lu coord=%lu reset_temp=35 evidence_ms=5000 motion_fresh_ms=100 experiment_fresh_ms=20 policy_v=2 min_mnm=3500 max_mnm=6240 peak_mnm=%lu blockers=%lu",
            (double)application_torque_envelope.peak_mnm / 1000.0,
            application_torque_envelope.exposure_nm2_s, application_torque_envelope.authorized,
            (unsigned)application_torque_envelope.last_reason, can_reset,
            (unsigned long)application_remote_ui.torque_token, (unsigned long)application_protocol_engine.boot_id,
            (unsigned long)application_debug_ui.reference_generation[6],
            (unsigned long)application_torque_envelope.peak_mnm, (unsigned long)blockers);
        if (written < 0 || (size_t)written >= sizeof(detail)) {
            application_remote_ui.torque_token = 0U;
            goto rejected;
        }
        aethor_remote_reply(output, wire.request_id, "torque", detail, 0U); return 1U;
    }
    if (torque_reset) {
        uint32_t cold = 0U;
        uint32_t peak_mnm = TORQUE_ENVELOPE_LEGACY_MNM;
        uint32_t expected_token = application_remote_ui.torque_token;
        application_remote_ui.torque_token = 0U;
        if ((wire.field_count != 2U && wire.field_count != 3U) ||
            (wire.field_count == 3U && !aethor_remote_uint(&wire, "peak_mnm", &peak_mnm)) ||
            peak_mnm < TORQUE_ENVELOPE_MIN_MNM || peak_mnm > TORQUE_ENVELOPE_MAX_MNM ||
            !aethor_remote_uint(&wire, "token", &token) || !token ||
            token != expected_token || !aethor_remote_uint(&wire, "cold", &cold) || cold != 1U ||
            now_us < application_remote_ui.torque_review_us || now_us - application_remote_ui.torque_review_us > 30000000ULL ||
            application_remote_ui.torque_boot != application_protocol_engine.boot_id ||
            application_remote_ui.torque_epoch != application_debug_ui.epoch ||
            application_remote_ui.torque_reference != application_debug_ui.reference_generation[6] ||
            !application_remote_ui.torque_session || !application_remote_ui.torque_nonce ||
            application_remote_ui.torque_session != application_protocol_engine.session_id ||
            application_remote_ui.torque_nonce != application_protocol_engine.next_session_nonce ||
            !application_remote_ui.torque_link || !application_remote_release_link_word ||
            application_remote_ui.torque_link != application_remote_release_link_word() ||
            application_remote_ui.owns || application_remote_ui.active ||
            aethor_app_torque_envelope_blockers(now_us))
            goto rejected;
        if (!torque_envelope_reset_cold_selected(&application_torque_envelope, peak_mnm)) goto rejected;
        (void)snprintf(detail, sizeof(detail),
            "result=reset accepted=1 authorized=1 experimental=1 motion_started=0 policy_v=2 peak_mnm=%lu",
            (unsigned long)application_torque_envelope.peak_mnm);
        aethor_remote_reply(output, wire.request_id, "torque_reset", detail, 0U);
        return 1U;
    }
    if (heartbeat) {
        if (wire.field_count == 1U && aethor_remote_uint(&wire, "token", &token) && token &&
            application_remote_ui.owns && token == application_remote_ui.lease_token) {
            application_remote_ui.heartbeat_us = now_us;
            aethor_remote_reply(output, wire.request_id, "heartbeat", "result=alive", 0U); return 1U;
        }
        goto rejected;
    }
    if (result && wire.field_count == 1U && aethor_remote_uint(&wire, "id", &token)) {
        aethor_remote_result(output, wire.request_id, token); return 1U;
    }
    if (caps && !wire.field_count) {
        DebugUiMotorView *motor = &application_remote_ui.review.motors[6];
        DebugUiMotorProfile load_bounds;
        (void)aethor_app_debug_ui_get_snapshot(now_us, &application_remote_ui.review);
        load_bounds = debug_ui_pos_load_bounds(motor);
        if (!application_remote_ui.sequence) application_remote_ui.sequence = application_remote_ui.review.boot_id;
        if (++application_remote_ui.sequence == 0U) ++application_remote_ui.sequence;
        application_remote_ui.token = application_remote_ui.sequence;
        application_remote_ui.review_session = application_protocol_engine.session_id;
        application_remote_ui.review_nonce = application_protocol_engine.next_session_nonce;
        application_remote_ui.review_link = application_remote_release_link_word ? application_remote_release_link_word() : 0U;
        (void)snprintf(detail, sizeof(detail),
            "token=%lu boot=%lu epoch=%lu fresh=%u mode=%u owner=%u pos=%u static=%u max_delta=%.5f max_speed=%.5f last=%lu active_id=%lu terminal_id=%lu load=%u load_delta=%.5f load_speed=%.5f flange_ratio=%.6f preflight=%u switch=%u defer=%u home=%u coord=%lu authority=%u remote_fault_release=%u",
            (unsigned long)application_remote_ui.token, (unsigned long)application_remote_ui.review.boot_id,
            (unsigned long)application_remote_ui.review.epoch, motor->feedback_valid, motor->actual_mode,
            application_remote_ui.owns ? 2U : application_remote_ui.review.authority == DEBUG_UI_AUTHORITY_REMOTE ? 0U : 1U,
            application_remote_ui.review.motion_enabled && motor->profile.pos_valid,
            application_remote_ui.review.single_static_load_enabled && motor->profile.mit_valid,
            (double)motor->profile.max_delta_rad, (double)motor->profile.max_speed_rad_s,
            (unsigned long)application_remote_ui.last_external_id,
            (unsigned long)(application_remote_ui.active ? application_remote_ui.external_id : 0U),
            (unsigned long)application_remote_ui.terminal_id,
            application_remote_ui.review.pos_load_enabled && motor->profile.pos_valid && debug_ui_pos_load_flange(motor),
            (double)load_bounds.max_delta_rad, (double)load_bounds.max_speed_rad_s,
            (double)motor->pos_load_output_per_flange,
            application_remote_ui.review.single_static_load_enabled && motor->profile.mit_valid && motor->actual_mode == DEBUG_UI_MODE_MIT,
            application_remote_ui.review.motion_enabled && application_remote_ui.review.mit_enabled && motor->profile.mit_valid && motor->profile.pos_valid,
            aethor_remote_can_defer_feedback(&application_remote_ui.review),
            application_remote_ui.review.pos_load_enabled && motor->profile.pos_valid && debug_ui_pos_load_flange(motor) &&
                application_remote_ui.review.boot_id && motor->reference_generation,
            (unsigned long)motor->reference_generation, (unsigned)application_remote_ui.review.authority,
            aethor_remote_fault_release_ready());
        aethor_remote_reply(output, wire.request_id, "caps", detail, 0U); return 1U;
    }
    memset(&request, 0, sizeof(request));
    if (!submit || wire.field_count != 8U || !aethor_remote_uint(&wire, "token", &token) ||
        !aethor_remote_operation(&wire, &request.operation, &request.requested_mode) || !aethor_remote_float(&wire, "delta", &request.delta_rad) ||
        !aethor_remote_float(&wire, "speed", &request.speed_rad_s) ||
        !aethor_remote_float(&wire, "torque", &request.torque_ff_nm) ||
        !aethor_remote_uint(&wire, "duration", &request.hold_duration_ms)) goto rejected;
    /* HOME repurposes unused gain tokens as strict decimal uint32 identities, without float conversion. */
    if (request.operation == DEBUG_UI_OPERATION_HOME) {
        if (!aethor_remote_uint(&wire, "kp", &request.home_boot_id) ||
            !aethor_remote_uint(&wire, "kd", &request.reference_generation)) goto rejected;
        reason = DEBUG_UI_REASON_OLD_EPOCH;
        if (!request.home_boot_id || request.home_boot_id != application_protocol_engine.boot_id ||
            request.home_boot_id != application_remote_ui.review.boot_id || !request.reference_generation ||
            request.reference_generation != application_remote_ui.review.motors[6].reference_generation ||
            request.reference_generation != application_debug_ui.reference_generation[6]) goto rejected;
    } else if (!aethor_remote_float(&wire, "kp", &request.kp) || !aethor_remote_float(&wire, "kd", &request.kd)) goto rejected;
    reason = DEBUG_UI_REASON_DUPLICATE;
    if (wire.request_id <= application_remote_ui.last_external_id) goto rejected;
    reason = DEBUG_UI_REASON_STALE_REQUEST;
    if (!token || token != application_remote_ui.token || now_us < application_remote_ui.review.timestamp_us ||
        now_us - application_remote_ui.review.timestamp_us > 30000000ULL ||
        application_remote_ui.review.epoch != application_debug_ui.epoch ||
        application_remote_ui.review_session != application_protocol_engine.session_id ||
        application_remote_ui.review_nonce != application_protocol_engine.next_session_nonce) goto rejected;
    application_remote_ui.token = 0U; /* An attempt consumes review, even when the runtime refuses it. */
    reason = DEBUG_UI_REASON_DISABLED;
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (aethor_app_persistence_locked()) goto rejected;
#endif
#if AETHOR_ADRC_LCD_INTEGRATED
    if (application_adrc_lcd_ownership.state != ADRC_LCD_OWNER_LCD || aethor_app_mapping_locked()) goto rejected;
#endif
    reason = DEBUG_UI_REASON_BUSY;
    if (application_remote_ui.active || debug_ui_mailbox_result_count(&application_debug_ui.mailbox) ||
        application_debug_ui.mailbox.stop_intent_mask) goto rejected;
    reason = DEBUG_UI_REASON_NOT_ARMED;
    if (request.operation == DEBUG_UI_OPERATION_ACQUIRE || request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ||
        request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        if (application_debug_ui.authority != DEBUG_UI_AUTHORITY_REMOTE || application_remote_ui.owns) goto rejected;
    } else if ((!application_remote_ui.owns || application_remote_ui.owner_epoch != application_debug_ui.epoch ||
        application_debug_ui.authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED) &&
        !(request.operation == DEBUG_UI_OPERATION_RELEASE && aethor_remote_fault_release_ready())) goto rejected;
    reason = DEBUG_UI_REASON_OUT_OF_RANGE;
    if (request.speed_rad_s <= 0.0F || request.hold_duration_ms < 1000U || request.hold_duration_ms > 60000U) goto rejected;
    if (debug_ui_is_pos_load_operation(request.operation)) {
        const DebugUiMotorView *motor = &application_remote_ui.review.motors[6];
        DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
        if (!application_remote_ui.review.pos_load_enabled || !motor->profile.pos_valid ||
            !debug_ui_pos_load_flange(motor) || motor->actual_mode != DEBUG_UI_MODE_POS_VEL ||
            request.kp != 0.0F || request.kd != 0.0F || request.torque_ff_nm != 0.0F ||
            request.hold_duration_ms % 1000U != 0U || request.speed_rad_s > bounds.max_speed_rad_s ||
            (request.operation == DEBUG_UI_OPERATION_HOME ?
                (request.hold_duration_ms != 1000U || request.delta_rad < bounds.position_min_rad ||
                 request.delta_rad > bounds.position_max_rad) :
                (request.delta_rad == 0.0F || fabsf(request.delta_rad) > bounds.max_delta_rad))) goto rejected;
        request.requested_mode = DEBUG_UI_MODE_POS_VEL;
    } else if (request.kp < 80.0F || request.kp > 200.0F || request.kd < 0.20F || request.kd > 0.40F) goto rejected;
    reason = DEBUG_UI_REASON_STALE_FEEDBACK;
    { MotorFeedbackSnapshot feedback;
      (void)motor_runtime_get_snapshot(&application_motor_runtime, now_us, MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US, &feedback);
      /* Defer only intent; unchanged LCD admission and post-TX execution gates still require actual new feedback. */
      if (request.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT && !(feedback.valid_joint_mask & 0x40U) &&
          !(aethor_remote_operation_defers_feedback(request.operation) &&
            aethor_remote_can_defer_feedback(&application_remote_ui.review))) goto rejected;
    }
    if (application_debug_ui.mailbox.remote_last_request_id == UINT32_MAX) { reason = DEBUG_UI_REASON_DUPLICATE; goto rejected; }
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = application_debug_ui.epoch;
    request.identity.request_id = application_debug_ui.mailbox.remote_last_request_id ?
        application_debug_ui.mailbox.remote_last_request_id + 1U : 0x80000001UL;
    request.remote_proxy = 1U; request.target_motor_id = 7U; request.created_at_us = now_us;
    request.reference_generation = application_remote_ui.review.motors[6].reference_generation;
    request.single_static_evidence_generation = application_remote_ui.review.motors[6].single_static_evidence_generation;
    request.static_positions_rad[6] = application_remote_ui.review.motors[6].position_rad;
    request.static_references[6] = request.reference_generation;
    reason = debug_ui_mailbox_submit(&application_debug_ui.mailbox, &request);
    if (reason != DEBUG_UI_REASON_NONE) goto rejected;
    application_remote_ui.external_id = wire.request_id; application_remote_ui.internal_id = request.identity.request_id;
    application_remote_ui.request_session = application_protocol_engine.session_id;
    application_remote_ui.request_nonce = application_protocol_engine.next_session_nonce;
    application_remote_ui.request_link = application_remote_release_link_word ? application_remote_release_link_word() : 0U;
    application_remote_ui.request_stop_generation = application_remote_ui.stop_generation;
    if (request.operation == DEBUG_UI_OPERATION_RELEASE && application_debug_ui.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT) {
        application_remote_ui.release_internal_id = request.identity.request_id;
        application_remote_ui.release_terminal_id = application_remote_ui.terminal_id;
    }
    application_remote_ui.last_external_id = wire.request_id;
    application_remote_ui.active = 1U;
    if (request.operation == DEBUG_UI_OPERATION_ACQUIRE) {
        application_remote_ui.owns = 1U; application_remote_ui.owner_epoch = 0U;
        application_remote_ui.lease_token = token; application_remote_ui.heartbeat_us = now_us;
    }
    (void)snprintf(detail, sizeof(detail), "result=queued accepted=0 lease=%lu", (unsigned long)application_remote_ui.lease_token);
    aethor_remote_reply(output, wire.request_id, "submit", detail, 0U); return 1U;
rejected:
    (void)snprintf(detail, sizeof(detail), "code=rejected reason=%u", reason);
    aethor_remote_reply(output, wire.request_id, caps ? "caps" : result ? "result" : heartbeat ? "heartbeat" :
        torque ? "torque" : torque_reset ? "torque_reset" : "submit", detail, 1U); return 1U;
}
#endif
