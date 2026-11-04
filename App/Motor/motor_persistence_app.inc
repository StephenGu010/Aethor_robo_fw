/** @file motor_persistence_app.inc
 * @brief Private bounded typed persistence App adapter; included once by aethor_app.c.
 * No production command starts this lifetime. Only fixed pure-module frames and selected DISABLE can be emitted.
 * The single snapshot stays in static storage, and failed/complete owners retain their lock.
 */
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
#define AETHOR_PERSISTENCE_HEALTH_REFRESH_US (50000ULL)
#include <float.h>
#define AETHOR_PERSISTENCE_DISABLE_LIMIT ((uint32_t)(MOTOR_PERSISTENCE_TOTAL_BUDGET_US / AETHOR_PERSISTENCE_HEALTH_REFRESH_US) + 16U)

/** @brief Small immutable owner wrapper; transport debts remain separate from pure-module pending. */
typedef struct
{
    MotorPersistenceIdentity parameter_identity;
    DebugUiStaticTxIdentity disable_identity;
    MotorPersistenceHealth health;
    S3519Ranges ranges;
    uint64_t last_us, disable_created_us, disable_submitted_us, disable_observed_us, can_idle_observed_us;
    uint32_t boot_id, local_epoch, protocol_session, owner_generation, admission_generation;
    uint32_t link_word, bound_link_word, request_id, confirmed_mode;
    DebugUiStaticTxIdentity health_identity;
    uint64_t source_fingerprint, snapshot_observed_us;
    float source_pmax, reference_native_position, native_position, continuity_tolerance;
    uint32_t disable_count;
    uint16_t disable_sof;
    MotorPersistenceError fault;
    uint8_t initialized, link_seen, locked, health_proven;
    uint8_t parameter_outstanding, parameter_submitted, parameter_completed;
    uint8_t disable_outstanding, disable_submitted, disable_actual_tx, disable_wait_rx;
    uint8_t pmax_known, mode_known, snapshot_phase, snapshot_ready, continuity_pending, continuity_checked, source_mode;
} AethorPersistenceApp;

/** @brief Exactly one boot-local pure transaction and a compact adapter; never copied onto a task stack. */
static MotorPersistenceTransaction application_persistence_transaction;
static AethorPersistenceApp application_persistence;

/** @brief Complete preserved Save evidence, independent of the overwritten all45 readback workspace. */
typedef struct
{
    MotorPersistenceItem receipt;
    CanFrame tx, ack;
    uint64_t settle_until_us;
    uint8_t valid;
} AethorPersistenceSaveJournal;
static AethorPersistenceSaveJournal application_persistence_save_journal;

/** @brief Small read-only status projection; source identity remains the original begin identity after revocation. */
typedef struct
{
    MotorPersistenceIdentity identity;
    uint64_t now_us, verified_us;
    uint32_t local_epoch, protocol_session, link_word, confirmed_pmax_raw, confirmed_mode;
    MotorPersistenceState state;
    MotorPersistenceError error, fault;
    uint8_t locked, skip_save, save_used, save_uncertain, pmax_known, mode_known, snapshot_phase, snapshot_ready;
} AethorPersistenceStatus;
/** @brief Health exports the last proven FD identity, never an in-flight replacement FD identity. */
typedef struct
{
    MotorPersistenceHealth health;
    DebugUiStaticTxIdentity proof_identity;
    float reference_native_position, native_position, continuity_tolerance;
    uint8_t health_proven, continuity_checked;
} AethorPersistenceHealthView;
/** @brief A single original item plus its published RID and raw interpretation, not a rebuilt snapshot. */
typedef struct
{
    MotorPersistenceItem item;
    uint8_t rid, is_unsigned;
} AethorPersistenceItemView;
/** @brief Private typed fixed-target operations are deliberately absent from production USB dispatch. */
static uint8_t aethor_app_persistence_prepare(uint32_t request_id, uint64_t now_us);
static uint8_t aethor_app_persistence_request_save(uint32_t request_id, uint64_t now_us);
static uint8_t aethor_app_persistence_commit_save(uint32_t request_id, uint64_t now_us);
/** @brief Private read-only projections never advance clocks, send frames or consume Save authority. */
static uint8_t aethor_app_persistence_get_status(AethorPersistenceStatus *status);
static uint8_t aethor_app_persistence_get_health(uint32_t request_id, AethorPersistenceHealthView *view);
static uint8_t aethor_app_persistence_get_item(uint32_t request_id, uint8_t phase, unsigned index, AethorPersistenceItemView *view);
static uint8_t aethor_app_persistence_get_baseline_raw(uint32_t request_id, unsigned index, uint32_t *raw);
static uint8_t aethor_app_persistence_get_save_journal(uint32_t request_id, AethorPersistenceSaveJournal *journal);

/** @brief Converts float representation without aliasing, numeric coercion or expanded thresholds. */
static uint32_t aethor_app_persistence_float_raw(float value)
{ uint32_t raw; memcpy(&raw, &value, sizeof(raw)); return raw; }
/** @brief Fingerprints all13 ordinary discovery observations with explicit fields and no padding dependence. */
static uint64_t aethor_app_persistence_discovery_fingerprint(const MotorDiscoveryResult *discovery)
{
    uint32_t fields[] = {aethor_app_persistence_float_raw(discovery->ranges.position_max_rad),
        aethor_app_persistence_float_raw(discovery->ranges.velocity_max_rad_s),
        aethor_app_persistence_float_raw(discovery->ranges.torque_max_nm),
        aethor_app_persistence_float_raw(discovery->acceleration_rad_s2),
        aethor_app_persistence_float_raw(discovery->deceleration_rad_s2),
        aethor_app_persistence_float_raw(discovery->maximum_speed_rad_s), discovery->observed_master_id,
        discovery->observed_esc_id, discovery->observed_control_mode, discovery->communication_timeout_raw,
        discovery->hardware_version, discovery->software_version, discovery->sub_version, discovery->verified_fields_mask};
    uint64_t fingerprint = 14695981039346656037ULL;
    unsigned field, byte;
    for (field = 0U; field < sizeof(fields) / sizeof(fields[0]); ++field) {
        for (byte = 0U; byte < 4U; ++byte) {
            fingerprint ^= (uint8_t)(fields[field] >> (8U * byte)); fingerprint *= 1099511628211ULL;
        }
    }
    return fingerprint;
}

/** @brief Advances authorization while keeping zero reserved as a permanent denial. */
static uint32_t aethor_app_persistence_increment(uint32_t generation)
{ ++generation; return generation ? generation : 1U; }

/** @brief Irreversibly revokes the wrapper, including after the pure module reached COMPLETE. */
static void aethor_app_persistence_abort(MotorPersistenceError fault, uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    if (!owner->locked) { return; }
    if (owner->fault == MOTOR_PERSISTENCE_ERROR_NONE) {
        owner->fault = fault;
        owner->owner_generation = aethor_app_persistence_increment(owner->owner_generation);
        owner->admission_generation = aethor_app_persistence_increment(owner->admission_generation);
    }
    owner->health_proven = owner->snapshot_ready = owner->continuity_checked = 0U;
    /* Pure abort clears its own pending flag. The independently retained original
     * parameter identity is deliberately left until its exact Platform receipt. */
    motor_persistence_tick(&application_persistence_transaction, now_us,
        owner->boot_id, owner->owner_generation, 1U);
}

/** @brief STOP invalidates maintenance without borrowing or replacing any existing STOP identity. */
static void aethor_app_persistence_preempt(uint64_t now_us)
{ aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_STOP, now_us); }

/** @brief Adds physical DISABLE intent through the existing union-preserving emergency safety path. */
static void aethor_app_persistence_disable_intent(uint8_t motor_mask)
{ application_debug_ui.mailbox.stop_intent_mask |= motor_mask; }

/** @brief Keeps a local STOP's target scope while always withdrawing the maintenance motor ID7. */
static uint8_t aethor_app_persistence_local_stop_mask(uint8_t motor_id)
{ return (uint8_t)(0x40U | (motor_id >= 1U && motor_id <= 7U ? (1U << (motor_id - 1U)) : 0U)); }

/** @brief Preserves global STOP and the existing ascending bench motor-list scope without admitting a command. */
static uint8_t aethor_app_persistence_text_stop_mask(const TextProtocolRequest *request)
{
    TextProtocolSpan targets;
    uint8_t requested_mask = 0U;
    char previous = '0';
    size_t index;
    if (text_protocol_request_path_equals(request, "adrc", "stop")) { return 0x40U; }
    if (!text_protocol_request_path_equals(request, "bench", "stop") &&
        !text_protocol_request_path_equals(request, "bench", "disable")) { return 0x7fU; }
    if (request->field_count || request->positional_count != 1U ||
        !text_protocol_get_positional(request, 0U, &targets) || !targets.length || targets.length > 13U ||
        (targets.length & 1U) == 0U) { return 0x40U; }
    for (index = 0U; index < targets.length; index += 2U) {
        char motor = targets.data[index];
        if (motor < '1' || motor > '7' || motor <= previous ||
            (index && targets.data[index - 1U] != ',')) { return 0x40U; }
        requested_mask |= (uint8_t)(1U << (unsigned)(motor - '1'));
        previous = motor;
    }
    return (uint8_t)(requested_mask | 0x40U);
}

/** @brief Requires full trusted discovery and exactly the allowed source mode/range, never a guessed mapping. */
static uint8_t aethor_app_persistence_discovery_valid(void)
{
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    const JointConfig *joint = &arm_config_get_production()->joints[6];
    return (uint8_t)(joint->esc_id == 7U && joint->master_id == 23U &&
        application_motor_runtime.discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE &&
        (application_motor_runtime.discovery.verified_joint_mask & 0x40U) &&
        discovery->verified_fields_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK &&
        discovery->observed_esc_id == 7U && discovery->observed_master_id == 23U &&
        (discovery->observed_control_mode == 1U || discovery->observed_control_mode == 2U) &&
        discovery->software_version == 808597046U && discovery->sub_version == 49U &&
        (discovery->ranges.position_max_rad == 12.5F || discovery->ranges.position_max_rad == 512.0F) &&
        discovery->ranges.velocity_max_rad_s == 200.0F && discovery->ranges.torque_max_nm == 10.0F);
}

/** @brief Extends the real mapping idle predicates with static/POS/ADRC/channel and transport debts. */
static uint8_t aethor_app_persistence_sources_idle(uint64_t now_us, uint8_t require_quiet)
{
    const AdrcAppBridge *bridge = &application_adrc_bridge;
    return (uint8_t)(aethor_app_mapping_sources_idle(0U) && !aethor_app_mapping_locked() &&
        application_controller.fault == ARM_FAULT_NONE && !application_controller.enabled && !application_controller.moving &&
        application_adrc_lcd_ownership.state == ADRC_LCD_OWNER_LCD &&
        !application_debug_ui.single_static_transport_mask && !application_debug_ui.pos_disable_pending &&
        !application_debug_ui.pos_load_cleanup_required &&
        !application_integrated_diag_requested_us && !application_integrated_probe_submitted_us &&
        application_transport_fault_read_sequence == application_transport_fault_write_sequence &&
        !application_motor_runtime.parameter_expectations.count && !application_motor_runtime.parameter_quarantine.count &&
        now_us >= application_motor_runtime.position_read.not_before_us &&
        now_us >= application_motor_runtime.idle_position_read.not_before_us &&
        application_motor_runtime.mode_switch_state != MOTOR_MODE_SWITCH_FAILED &&
        !bridge->frame_pending && !bridge->awaiting_receipt && !bridge->stop_pending &&
        !bridge->release_disable_pending && !bridge->send_failed && !bridge->simple_bridge_fault &&
        !bridge->simple_pending && !bridge->simple_guard_armed && !bridge->shaft.bridge_fault &&
        (!bridge->shaft.used || bridge->shaft.released) && !bridge->capture.state && !bridge->guard_test.state &&
        !bridge->watchdog.state && !bridge->capability_prepare_state &&
        bridge->bench.experiment.status.state == ADRC_STATE_DISABLED &&
        bridge->bench.experiment.status.fault == ADRC_FAULT_NONE &&
        !bridge->bench.gateway.active_run_request_id && !bridge->bench.gateway.command_pending &&
        !bridge->bench.gateway.stop_pending && !bridge->bench.gateway.heartbeat_pending &&
        !bridge->bench.gateway.channel_state[0] && !bridge->bench.gateway.channel_state[1] &&
        !bridge->bench.gateway.channel_state[2] &&
        bridge->capabilities.state == MOTOR_CAPABILITY_IDLE &&
        (!require_quiet || (application_integrated_can_idle && application_integrated_can_quiet_since_us &&
            application_persistence.can_idle_observed_us && now_us >= application_persistence.can_idle_observed_us &&
            now_us - application_persistence.can_idle_observed_us <= 4000ULL &&
            now_us >= application_integrated_can_quiet_since_us &&
            now_us - application_integrated_can_quiet_since_us >= 4000ULL)));
}

/** @brief Rejects unsafe actual selected feedback immediately; later good packets cannot erase the latch. */
static uint8_t aethor_app_persistence_health_safe(const MotorPersistenceHealth *health, uint64_t now_us)
{
    return (uint8_t)(health && health->valid && health->disabled && health->idle && !health->fault &&
        health->observed_us && health->observed_us <= now_us && now_us - health->observed_us <= 100000ULL &&
        isfinite(health->velocity) && fabsf(health->velocity) <= 0.05F &&
        isfinite(health->torque) && fabsf(health->torque) <= 0.15F &&
        isfinite(health->motor_temperature) && health->motor_temperature < 50.0F &&
        isfinite(health->driver_temperature) && health->driver_temperature < 50.0F);
}

/** @brief Checks each completed critical observation against its actual source or authorized target phase.
 * RX and actual-TX callbacks both run service after pure completion, including a deferred early ACK.
 * An uncompleted slot is not evidence; no register read is allowed to silently replace the frozen mapping. */
static uint8_t aethor_app_persistence_baseline_context_valid(void)
{
    static const uint8_t critical_indices[] = {7U, 8U, 10U, 13U, 14U, 21U, 22U, 23U, 36U};
    const AethorPersistenceApp *owner = &application_persistence;
    unsigned critical;
    for (critical = 0U; critical < sizeof(critical_indices) / sizeof(critical_indices[0]); ++critical) {
        unsigned index = critical_indices[critical];
        const MotorPersistenceItem *item = &application_persistence_transaction.current.items[index];
        uint32_t expected;
        if (!item->valid) { continue; }
        switch (index) {
            case 7U: expected = 23U; break;
            case 8U: expected = 7U; break;
            case 10U: expected = item->identity.phase == MOTOR_PERSISTENCE_BASELINE ? owner->source_mode : 2U; break;
            case 13U: expected = application_motor_runtime.discovery.results[6].hardware_version; break;
            case 14U: expected = 808597046U; break;
            case 21U: expected = aethor_app_persistence_float_raw(item->identity.phase == MOTOR_PERSISTENCE_BASELINE ? owner->source_pmax : 512.0F); break;
            case 22U: memcpy(&expected, &owner->ranges.velocity_max_rad_s, sizeof(expected)); break;
            case 23U: memcpy(&expected, &owner->ranges.torque_max_nm, sizeof(expected)); break;
            default: expected = 49U; break;
        }
        if (item->raw != expected) { return 0U; }
    }
    return 1U;
}

/** @brief Synchronizes only pure-confirmed exact ACK plus actual-TX outcomes, never submission/HAL acceptance. */
static void aethor_app_persistence_sync_confirmed(void)
{
    AethorPersistenceApp *owner = &application_persistence;
    const MotorPersistenceTransaction *transaction = &application_persistence_transaction;
    float pmax;
    uint8_t changed;
    if (transaction->state < MOTOR_PERSISTENCE_BASELINE_READY || transaction->state == MOTOR_PERSISTENCE_ABORTED) { return; }
    changed = (uint8_t)(owner->pmax_known != transaction->pmax_known || owner->mode_known != transaction->mode_known);
    owner->pmax_known = transaction->pmax_known; owner->mode_known = transaction->mode_known;
    if (transaction->pmax_known) {
        memcpy(&pmax, &transaction->confirmed_pmax_raw, sizeof(pmax));
        changed |= (uint8_t)(owner->ranges.position_max_rad != pmax); owner->ranges.position_max_rad = pmax;
    }
    if (transaction->mode_known) {
        changed |= (uint8_t)(owner->confirmed_mode != transaction->confirmed_mode); owner->confirmed_mode = transaction->confirmed_mode;
    }
    if (changed) { owner->health_proven = owner->continuity_checked = 0U; }
}
/** @brief Captures the original accepted AA only when both exact ACK and actual TX have settled. */
static void aethor_app_persistence_capture_save(void)
{
    const MotorPersistenceTransaction *transaction = &application_persistence_transaction;
    AethorPersistenceSaveJournal *journal = &application_persistence_save_journal;
    if (journal->valid || transaction->state != MOTOR_PERSISTENCE_SETTLE || !transaction->actual_tx ||
        !transaction->deferred || journal->ack.length != 4U) { return; }
    journal->receipt.identity = transaction->identity;
    journal->receipt.created_us = transaction->created_us; journal->receipt.submitted_us = transaction->submitted_us;
    journal->receipt.boundary_us = transaction->boundary_us;
    journal->receipt.send_completed_observed_us = transaction->send_completed_observed_us;
    journal->receipt.rx_lower_us = transaction->deferred_lower_us; journal->receipt.received_us = transaction->deferred_received_us;
    journal->receipt.valid = 1U; journal->tx = transaction->frame;
    journal->settle_until_us = transaction->settle_until_us; journal->valid = 1U;
}
/** @brief Applies the specified error bound to native feedback and vendor RID0x50 (decimal80 p_m), never RID0x32 u_off. */
static uint8_t aethor_app_persistence_continuity(void)
{
    AethorPersistenceApp *owner = &application_persistence;
    const MotorPersistenceItem *position = NULL;
    unsigned index;
    float measured;
    owner->continuity_checked = 0U;
    /* Resolve by the published wire address so a dense-index/address-base mixup fails closed. */
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        if (motor_persistence_rid(index) == 0x50U) {
            position = &application_persistence_transaction.current.items[index]; break;
        }
    }
    if (!position) { return 0U; }
    owner->continuity_tolerance = 2.0F * owner->source_pmax / 65535.0F +
        2.0F * owner->ranges.position_max_rad / 65535.0F +
        4.0F * FLT_EPSILON * fmaxf(owner->source_pmax, owner->ranges.position_max_rad) +
        0.001F * 0.01745329251994329577F;
    memcpy(&measured, &position->raw, sizeof(measured));
    owner->continuity_checked = (uint8_t)(owner->health_proven && owner->pmax_known && owner->mode_known &&
        position->valid && isfinite(owner->native_position) && isfinite(measured) &&
        fabsf(owner->native_position - owner->reference_native_position) <= owner->continuity_tolerance &&
        fabsf(measured - owner->reference_native_position) <= owner->continuity_tolerance);
    return owner->continuity_checked;
}
/** @brief A stable all45 phase awaits a fresh strict FD before publication; refreshes never renew pure budgets. */
static void aethor_app_persistence_check_snapshot(uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    const MotorPersistenceTransaction *transaction = &application_persistence_transaction;
    uint8_t phase;
    if (transaction->state != MOTOR_PERSISTENCE_BASELINE_READY && transaction->state != MOTOR_PERSISTENCE_PREPARED &&
        transaction->state != MOTOR_PERSISTENCE_SAVE_HEALTH && transaction->state != MOTOR_PERSISTENCE_COMPLETE) { return; }
    phase = transaction->current.items[44].identity.phase;
    if (phase != owner->snapshot_phase) {
        owner->snapshot_phase = phase; owner->snapshot_observed_us = now_us;
        owner->snapshot_ready = owner->health_proven = owner->continuity_checked = 0U;
        owner->continuity_pending = 1U;
    }
    if (!owner->health_proven || owner->health.observed_us <= owner->snapshot_observed_us) { return; }
    if (!aethor_app_persistence_continuity()) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, now_us); return;
    }
    owner->snapshot_ready = 1U; owner->continuity_pending = 0U;
}

/** @brief Initializes only once in the firmware lifetime; calling App init again cannot create a retry. */
static void aethor_app_persistence_init(void)
{
    if (application_persistence.initialized) { return; }
    motor_persistence_init(&application_persistence_transaction);
    memset(&application_persistence_save_journal, 0, sizeof(application_persistence_save_journal));
    application_persistence.initialized = 1U;
    application_persistence.owner_generation = application_persistence.admission_generation = 1U;
    /* Keep the typed entry private and reachable for host facade tests without a USB begin command. */
    (void)aethor_app_persistence_begin_readonly;
    (void)aethor_app_persistence_prepare; (void)aethor_app_persistence_request_save; (void)aethor_app_persistence_commit_save;
    (void)aethor_app_persistence_get_status; (void)aethor_app_persistence_get_health; (void)aethor_app_persistence_get_item;
    (void)aethor_app_persistence_get_baseline_raw; (void)aethor_app_persistence_get_save_journal;
}

/** @brief Reserves one fixed read-only owner after fresh disabled entry evidence and a real 4ms quiet sample. */
static uint8_t aethor_app_persistence_begin_readonly(uint32_t request_id, uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    const MotorObject *motor = &application_motor_runtime.bank.motors[6];
    const MotorJointFeedback *feedback = &motor->feedback;
    MotorPersistenceHealth health;
    if (!application_initialized || !owner->initialized || owner->locked || !request_id ||
        !application_protocol_engine.boot_id || !application_debug_ui.epoch ||
        !application_protocol_engine.session_active || !application_protocol_engine.session_id ||
        !owner->link_seen || !(owner->link_word & 0x80000000UL) ||
        !aethor_app_persistence_sources_idle(now_us, 1U) || !aethor_app_persistence_discovery_valid()) { return 0U; }
    memset(&health, 0, sizeof(health));
    health.boot_epoch = application_protocol_engine.boot_id;
    health.observed_us = feedback->timestamp_us;
    health.valid = motor->feedback_valid;
    health.disabled = (uint8_t)(feedback->driver_state == S3519_DRIVER_STATE_DISABLED);
    health.idle = 1U; health.fault = (uint8_t)(feedback->fault_flags != 0U);
    health.velocity = feedback->velocity_rad_s; health.torque = feedback->torque_nm;
    health.motor_temperature = feedback->rotor_temperature_c; health.driver_temperature = feedback->mos_temperature_c;
    if (!isfinite(feedback->position_rad) || !aethor_app_persistence_health_safe(&health, now_us) ||
        !motor_persistence_begin(&application_persistence_transaction, health.boot_epoch, request_id,
            owner->owner_generation, now_us)) { return 0U; }
    owner->boot_id = health.boot_epoch; owner->local_epoch = application_debug_ui.epoch;
    owner->protocol_session = application_protocol_engine.session_id;
    owner->bound_link_word = owner->link_word; owner->request_id = request_id;
    owner->confirmed_mode = application_motor_runtime.discovery.results[6].observed_control_mode;
    owner->ranges = application_motor_runtime.discovery.results[6].ranges;
    owner->source_mode = (uint8_t)owner->confirmed_mode; owner->source_pmax = owner->ranges.position_max_rad;
    owner->source_fingerprint = aethor_app_persistence_discovery_fingerprint(&application_motor_runtime.discovery.results[6]);
    owner->reference_native_position = owner->native_position = feedback->position_rad;
    owner->pmax_known = owner->mode_known = 1U;
    owner->health = health; owner->health_proven = 0U; owner->locked = 1U; owner->last_us = now_us;
    owner->admission_generation = aethor_app_persistence_increment(owner->admission_generation);
    return 1U;
}

/** @brief Checks context on every service/admission/receipt, independently of pure terminal semantics. */
static void aethor_app_persistence_service(uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[6];
    if (!owner->locked) { return; }
    if (now_us < owner->last_us) { aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CLOCK, now_us); return; }
    owner->last_us = now_us;
    if (owner->fault != MOTOR_PERSISTENCE_ERROR_NONE) { return; }
    if (application_debug_ui.mailbox.stop_intent_mask || application_debug_ui.fallback_stop_mask ||
        application_protocol_engine.stop_read_sequence != application_protocol_engine.stop_write_sequence) {
        aethor_app_persistence_preempt(now_us); return;
    }
    if (application_protocol_engine.boot_id != owner->boot_id || application_debug_ui.epoch != owner->local_epoch ||
        !application_protocol_engine.session_active || application_protocol_engine.session_id != owner->protocol_session ||
        owner->link_word != owner->bound_link_word || !aethor_app_persistence_sources_idle(now_us, 0U) ||
        !aethor_app_persistence_discovery_valid() ||
        aethor_app_persistence_discovery_fingerprint(discovery) != owner->source_fingerprint) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, now_us); return;
    }
    if (now_us >= application_persistence_transaction.total_deadline_us) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_TIMEOUT, now_us); return;
    }
    aethor_app_persistence_sync_confirmed();
    aethor_app_persistence_capture_save();
    if (!aethor_app_persistence_baseline_context_valid()) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, now_us); return;
    }
    motor_persistence_tick(&application_persistence_transaction, now_us, owner->boot_id, owner->owner_generation, 0U);
    if (application_persistence_transaction.state == MOTOR_PERSISTENCE_ABORTED) {
        aethor_app_persistence_abort(application_persistence_transaction.error, now_us); return;
    }
    if (!aethor_app_persistence_health_safe(&owner->health, now_us) ||
        (owner->disable_wait_rx && now_us - owner->disable_created_us >= MOTOR_PERSISTENCE_FRAME_BUDGET_US)) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, now_us); return;
    }
    aethor_app_persistence_check_snapshot(now_us);
}

/** @brief Exposes a nonzero ordinary-producer generation with no maintenance frame fallthrough. */
uint32_t aethor_app_persistence_admission_generation(void)
{ return application_persistence.admission_generation; }
/** @brief Final ordinary permission is denied for the full retained maintenance lifetime. */
uint8_t aethor_app_persistence_allow_ordinary(uint32_t admission_generation)
{ return (uint8_t)(application_initialized && admission_generation && !application_persistence.locked &&
    admission_generation == application_persistence.admission_generation); }
/** @brief Failed and complete transactions retain the same lock until a later explicit safe release design. */
uint8_t aethor_app_persistence_locked(void)
{ return application_persistence.locked; }
/** @brief Binds the whole CDC event word; restoration of an earlier value cannot restore permission. */
void aethor_app_persistence_observe_link(uint32_t link_state, uint64_t now_us)
{
    application_persistence.link_seen = 1U;
    application_persistence.link_word = link_state;
    aethor_app_persistence_service(now_us);
}

/** @brief Emits one independent FD or one fixed pure-module frame; refreshes never renew pure deadlines or stack snapshots. */
uint8_t aethor_app_persistence_next_transfer(uint64_t now_us, AethorPersistenceTransfer *transfer)
{
    AethorPersistenceApp *owner = &application_persistence;
    MotorPersistenceTransaction *transaction = &application_persistence_transaction;
    aethor_app_persistence_service(now_us);
    if (!transfer || !owner->locked || owner->fault || owner->parameter_outstanding || owner->disable_outstanding ||
        owner->disable_wait_rx || transaction->pending || !owner->pmax_known || !owner->mode_known) { return 0U; }
    memset(transfer, 0, sizeof(*transfer));
    if (!owner->health_proven || now_us - owner->health.observed_us >= AETHOR_PERSISTENCE_HEALTH_REFRESH_US) {
        if (owner->disable_count >= AETHOR_PERSISTENCE_DISABLE_LIMIT ||
            s3519_pack_mode_command(7U, owner->confirmed_mode == 1U ? S3519_CONTROL_MODE_MIT :
                S3519_CONTROL_MODE_POSITION_VELOCITY, S3519_MODE_COMMAND_DISABLE, &transfer->frame) != S3519_CODEC_STATUS_OK) {
            aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, now_us); return 0U;
        }
        owner->disable_identity.epoch = owner->boot_id;
        owner->disable_identity.request_id = owner->request_id;
        owner->disable_identity.sequence = ++owner->disable_count;
        owner->disable_identity.motor_id = 7U;
        owner->disable_identity.phase = DEBUG_UI_STATIC_TX_PREFLIGHT;
        owner->disable_created_us = now_us; owner->disable_submitted_us = owner->disable_observed_us = 0U;
        owner->disable_submitted = owner->disable_actual_tx = 0U;
        owner->disable_outstanding = owner->disable_wait_rx = 1U;
        transfer->kind = AETHOR_PERSISTENCE_TRANSFER_DISABLE; transfer->disable_identity = owner->disable_identity;
        return 1U;
    }
    if (!motor_persistence_next(transaction, now_us, &transfer->frame, &transfer->parameter_identity)) { return 0U; }
    if (transfer->frame.identifier != 0x7ffU || transfer->frame.data[0] != 7U || transfer->frame.data[1] != 0U ||
        !((transfer->frame.length == 4U && (transfer->frame.data[2] == 0x33U ||
             (transfer->frame.data[2] == 0xaaU && transfer->frame.data[3] == 1U))) ||
          (transfer->frame.length == 8U && transfer->frame.data[2] == 0x55U &&
             (transfer->frame.data[3] == 21U || transfer->frame.data[3] == 10U)))) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, now_us); return 0U;
    }
    owner->parameter_identity = transfer->parameter_identity;
    owner->parameter_outstanding = 1U; owner->parameter_submitted = owner->parameter_completed = 0U;
    transfer->kind = AETHOR_PERSISTENCE_TRANSFER_PARAMETER;
    return 1U;
}

/** @brief A rejected original pair aborts without requeue; accepted transport debt is not inferred cleared. */
void aethor_app_persistence_transfer_rejected(const AethorPersistenceTransfer *transfer, uint64_t now_us)
{
    if (!transfer || !application_persistence.locked) { return; }
    if ((transfer->kind == AETHOR_PERSISTENCE_TRANSFER_PARAMETER &&
         motor_persistence_identity_equal(&transfer->parameter_identity, &application_persistence.parameter_identity)) ||
        (transfer->kind == AETHOR_PERSISTENCE_TRANSFER_DISABLE &&
         debug_ui_static_tx_identity_equal(&transfer->disable_identity, &application_persistence.disable_identity))) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_TX, now_us);
    }
}
/** @brief Cancels only the independently retained original parameter identity until a real matching receipt. */
uint8_t aethor_app_persistence_cancel_identity(MotorPersistenceIdentity *identity)
{
    if (!identity || !application_persistence.fault || !application_persistence.parameter_outstanding ||
        !application_persistence.parameter_submitted) { return 0U; }
    *identity = application_persistence.parameter_identity; return 1U;
}
/** @brief Commits the exact fixed parameter pair at the trusted empty RX boundary, with current health/context. */
uint8_t aethor_app_persistence_parameter_submitted(const MotorPersistenceIdentity *identity, uint64_t boundary_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    aethor_app_persistence_service(boundary_us);
    if (owner->fault || !owner->locked || !owner->health_proven || !owner->parameter_outstanding ||
        owner->parameter_submitted || !motor_persistence_identity_equal(identity, &owner->parameter_identity) ||
        !motor_persistence_submitted(&application_persistence_transaction, identity, boundary_us, boundary_us, 1U)) { return 0U; }
    owner->parameter_submitted = 1U;
    aethor_app_persistence_sync_confirmed(); return 1U;
}
/** @brief Selected FD has its own exact identity and never calls pure parameter submission. */
uint8_t aethor_app_persistence_disable_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    aethor_app_persistence_service(boundary_us);
    if (owner->fault || !owner->locked || !owner->disable_outstanding || owner->disable_submitted ||
        boundary_us < owner->disable_created_us ||
        !debug_ui_static_tx_identity_equal(identity, &owner->disable_identity)) { return 0U; }
    owner->disable_submitted = 1U; owner->disable_submitted_us = boundary_us; return 1U;
}
/** @brief Only a real exact receipt retires parameter debt; a late receipt never reopens a revoked owner. */
void aethor_app_persistence_parameter_transmitted(const MotorPersistenceIdentity *identity,
    uint8_t succeeded, uint64_t completed_observed_us, uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    if (!owner->parameter_outstanding || !owner->parameter_submitted ||
        !motor_persistence_identity_equal(identity, &owner->parameter_identity)) { return; }
    aethor_app_persistence_service(now_us);
    owner->parameter_outstanding = 0U; owner->parameter_completed = 1U;
    if (owner->fault) { return; }
    if (!motor_persistence_report_tx(&application_persistence_transaction, identity, completed_observed_us, now_us, succeeded)) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_TX, now_us);
    }
    aethor_app_persistence_service(now_us);
}
/** @brief Consumes only the FD identity; early failures keep debt because they do not prove Platform clearance. */
uint8_t aethor_app_persistence_disable_transmitted(const DebugUiStaticTxIdentity *identity,
    uint8_t succeeded, uint16_t sof, uint64_t observed_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    if (!owner->disable_outstanding || !owner->disable_submitted ||
        !debug_ui_static_tx_identity_equal(identity, &owner->disable_identity)) { return 0U; }
    aethor_app_persistence_service(observed_us);
    if (!succeeded || observed_us < owner->disable_submitted_us) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_TX, observed_us); return 1U;
    }
    owner->disable_outstanding = 0U;
    if (!owner->fault) {
        owner->disable_actual_tx = 1U; owner->disable_sof = sof; owner->disable_observed_us = observed_us;
    }
    return 1U;
}

/** @brief Isolates exact AA always, maintenance candidates while locked, and validates every selected RX immediately. */
uint8_t aethor_app_persistence_receive(const CanFrame *frame, uint64_t arrival_lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *ordered_identity, uint16_t sof, uint8_t sof_valid)
{
    AethorPersistenceApp *owner = &application_persistence;
    S3519Feedback feedback;
    MotorPersistenceHealth health;
    if (!frame) { return 0U; }
    if (!owner->locked && frame->identifier == 23U && frame->length == 4U && frame->data[0] == 7U &&
        frame->data[1] == 0U && frame->data[2] == 0xaaU && frame->data[3] == 1U) { return 1U; }
    if (!owner->locked) { return 0U; }
    aethor_app_persistence_service(observed_us);
    /* State/ESC and temperatures are raw wire fields independent of unknown range/mode.
     * Exact parameter-shaped replies carry register bytes, not temperatures. An unsafe
     * non-disabled state remains identifiable even when a position byte equals an opcode.
     * Motion bytes stay opaque; invalid health revokes pure authorization without decoding them. */
    if (!owner->fault && (!owner->pmax_known || !owner->mode_known) && frame->identifier == 23U &&
        frame->length == 8U && (frame->data[0] & 0x0fU) == 7U &&
        ((frame->data[0] >> 4U) != S3519_DRIVER_STATE_DISABLED ||
         (!(frame->data[1] == 0U && (frame->data[2] == 0x33U || frame->data[2] == 0x55U ||
                                    frame->data[2] == 0xaaU)) &&
          (frame->data[6] >= 50U || frame->data[7] >= 50U)))) {
        memset(&health, 0, sizeof(health));
        health.boot_epoch = owner->boot_id; health.observed_us = arrival_lower_us;
        health.disabled = (uint8_t)((frame->data[0] >> 4U) == S3519_DRIVER_STATE_DISABLED);
        health.fault = (uint8_t)((frame->data[0] >> 4U) >= S3519_DRIVER_STATE_FAULT_MINIMUM);
        health.motor_temperature = (float)frame->data[7]; health.driver_temperature = (float)frame->data[6];
        (void)motor_persistence_refresh_health(&application_persistence_transaction, &health, observed_us);
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, observed_us);
        aethor_app_persistence_disable_intent(0x40U); return 1U;
    }
    /* Parameter headers encode an ESC without driver-state bits and a zero high byte.
     * Opcode-valued feedback position bytes alone cannot classify a packet. Target
     * replies on a wrong master and malformed/late selected replies remain quarantined.
     * Wire-identical disabled feedback with this full header remains indistinguishable. */
    if (frame->length >= 3U && frame->data[0] <= 0x0fU && frame->data[1] == 0U &&
        (frame->data[2] == 0x33U || frame->data[2] == 0x55U || frame->data[2] == 0xaaU) &&
        (frame->identifier == 23U || frame->data[0] == 7U)) {
        if (!owner->fault) {
            if (motor_persistence_accept(&application_persistence_transaction, frame, 1U, arrival_lower_us, observed_us) &&
                frame->length == 4U && frame->data[2] == 0xaaU) {
                application_persistence_save_journal.ack = *frame;
            }
            aethor_app_persistence_service(observed_us);
        }
        return 1U;
    }
    if (frame->identifier != 23U) { return 0U; }
    /* Submission makes mapping unknown until exact ACK and actual TX both complete.
     * Old decode context cannot manufacture safe feedback in this interval. */
    if (owner->fault || !aethor_app_persistence_discovery_valid() || !owner->pmax_known || !owner->mode_known) { return 1U; }
    if (frame->length != 8U || s3519_decode_feedback(frame, &owner->ranges, &feedback) != S3519_CODEC_STATUS_OK ||
        feedback.esc_id != 7U || !isfinite(feedback.position_rad)) {
        memset(&health, 0, sizeof(health));
        (void)motor_persistence_refresh_health(&application_persistence_transaction, &health, observed_us);
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, observed_us);
        aethor_app_persistence_disable_intent(0x40U); return 1U;
    }
    memset(&health, 0, sizeof(health));
    health.boot_epoch = owner->boot_id; health.observed_us = arrival_lower_us;
    health.valid = 1U; health.disabled = (uint8_t)(feedback.state == S3519_DRIVER_STATE_DISABLED);
    health.idle = 1U; health.fault = (uint8_t)(feedback.state >= S3519_DRIVER_STATE_FAULT_MINIMUM);
    health.velocity = feedback.velocity_rad_s; health.torque = feedback.torque_nm;
    health.motor_temperature = feedback.rotor_temperature_c; health.driver_temperature = feedback.mos_temperature_c;
    if (!aethor_app_persistence_health_safe(&health, observed_us)) {
        (void)motor_persistence_refresh_health(&application_persistence_transaction, &health, observed_us);
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_HEALTH, observed_us);
        aethor_app_persistence_disable_intent(0x40U); return 1U;
    }
    /* Equal coarse arrival/submission lower bounds do not imply the same wire SOF bin.
     * Actual TX plus trusted ordered identity and strictly later SOF prove wire order;
     * retain the original arrival lower bound and require it to advance prior health. */
    if (owner->disable_wait_rx && owner->disable_actual_tx && sof_valid &&
        debug_ui_static_tx_identity_equal(ordered_identity, &owner->disable_identity) &&
        arrival_lower_us >= owner->disable_submitted_us && arrival_lower_us > owner->health.observed_us &&
        (uint16_t)(sof - owner->disable_sof) != 0U && (uint16_t)(sof - owner->disable_sof) < 32768U) {
        owner->health = health; owner->health_proven = 1U; owner->disable_wait_rx = 0U;
        owner->health_identity = owner->disable_identity; owner->native_position = feedback.position_rad;
        (void)motor_persistence_refresh_health(&application_persistence_transaction, &health, observed_us);
        aethor_app_persistence_service(observed_us);
    }
    /* Consume selected maintenance health so the ordinary decoder cannot mutate
     * the snapshot behind this owner's immutable evidence. Other axes still pass. */
    return 1U;
}
/** @brief Applies immutable identity/context/debt/current-budget checks before any typed mutation. */
static uint8_t aethor_app_persistence_typed_ready(uint32_t request_id, uint64_t now_us)
{
    const AethorPersistenceApp *owner = &application_persistence;
    aethor_app_persistence_service(now_us);
    return (uint8_t)(request_id && owner->locked && request_id == owner->request_id && !owner->fault &&
        owner->snapshot_ready && owner->continuity_checked && owner->health_proven &&
        owner->pmax_known && owner->mode_known && !owner->parameter_outstanding && !owner->disable_outstanding &&
        !owner->disable_wait_rx && !application_persistence_transaction.pending);
}
/** @brief Starts only the pure module's fixed PMAX512/POS2 preparation after preserved baseline continuity. */
static uint8_t aethor_app_persistence_prepare(uint32_t request_id, uint64_t now_us)
{
    if (!aethor_app_persistence_typed_ready(request_id, now_us) ||
        !motor_persistence_prepare(&application_persistence_transaction, &application_persistence.health, now_us)) { return 0U; }
    application_persistence.snapshot_ready = application_persistence.continuity_checked = 0U; return 1U;
}
/** @brief Requests a new all45 verification; it cannot itself emit or authorize AA. */
static uint8_t aethor_app_persistence_request_save(uint32_t request_id, uint64_t now_us)
{
    if (!aethor_app_persistence_typed_ready(request_id, now_us) ||
        !motor_persistence_request_save(&application_persistence_transaction, &application_persistence.health, now_us)) { return 0U; }
    application_persistence.snapshot_ready = application_persistence.continuity_checked = 0U; return 1U;
}
/** @brief Consumes one strictly post-verify FD plus continuity gate; pure submission later latches Save once. */
static uint8_t aethor_app_persistence_commit_save(uint32_t request_id, uint64_t now_us)
{
    if (!aethor_app_persistence_typed_ready(request_id, now_us) ||
        !motor_persistence_authorize_save(&application_persistence_transaction, &application_persistence.health, now_us)) { return 0U; }
    application_persistence.snapshot_ready = application_persistence.continuity_checked = 0U; return 1U;
}
/** @brief Exports a small copy without servicing timers or changing a retained lock. */
static uint8_t aethor_app_persistence_get_status(AethorPersistenceStatus *status)
{
    const AethorPersistenceApp *owner = &application_persistence;
    const MotorPersistenceTransaction *transaction = &application_persistence_transaction;
    if (!status || !owner->locked) { return 0U; }
    memset(status, 0, sizeof(*status)); status->identity = transaction->identity;
    status->now_us = owner->last_us; status->verified_us = transaction->verified_us;
    status->local_epoch = owner->local_epoch; status->protocol_session = owner->protocol_session;
    status->link_word = owner->bound_link_word; status->confirmed_mode = owner->confirmed_mode;
    status->confirmed_pmax_raw = aethor_app_persistence_float_raw(owner->ranges.position_max_rad);
    status->state = transaction->state; status->error = transaction->error; status->fault = owner->fault;
    status->locked = owner->locked; status->skip_save = transaction->skip_save;
    status->save_used = transaction->save_used; status->save_uncertain = transaction->save_uncertain;
    status->pmax_known = owner->pmax_known; status->mode_known = owner->mode_known;
    status->snapshot_phase = owner->snapshot_phase; status->snapshot_ready = owner->snapshot_ready;
    return 1U;
}
/** @brief Exports the original proven FD identity and native continuity projection without refreshing health. */
static uint8_t aethor_app_persistence_get_health(uint32_t request_id, AethorPersistenceHealthView *view)
{
    const AethorPersistenceApp *owner = &application_persistence;
    if (!view || !owner->locked || !request_id || request_id != owner->request_id) { return 0U; }
    memset(view, 0, sizeof(*view)); view->health = owner->health; view->proof_identity = owner->health_identity;
    view->reference_native_position = owner->reference_native_position; view->native_position = owner->native_position;
    view->continuity_tolerance = owner->continuity_tolerance; view->health_proven = owner->health_proven;
    view->continuity_checked = owner->continuity_checked; return 1U;
}
/** @brief Exports exactly one stable current item with its original complete time and identity evidence. */
static uint8_t aethor_app_persistence_get_item(uint32_t request_id, uint8_t phase, unsigned index, AethorPersistenceItemView *view)
{
    const AethorPersistenceApp *owner = &application_persistence;
    const MotorPersistenceItem *item;
    if (!view || !request_id || request_id != owner->request_id || !owner->snapshot_ready || owner->fault ||
        index >= MOTOR_PERSISTENCE_REGISTER_COUNT || phase != owner->snapshot_phase) { return 0U; }
    item = &application_persistence_transaction.current.items[index];
    if (!item->valid || item->identity.phase != phase) { return 0U; }
    memset(view, 0, sizeof(*view)); view->item = *item;
    view->rid = motor_persistence_rid(index); view->is_unsigned = motor_persistence_is_unsigned(index); return 1U;
}
/** @brief Exports only a compact original raw configuration copy; it carries no rebuilt per-item times. */
static uint8_t aethor_app_persistence_get_baseline_raw(uint32_t request_id, unsigned index, uint32_t *raw)
{
    const AethorPersistenceApp *owner = &application_persistence;
    if (!raw || !owner->locked || !request_id || request_id != owner->request_id ||
        index >= MOTOR_PERSISTENCE_REGISTER_COUNT || !owner->snapshot_phase) { return 0U; }
    *raw = application_persistence_transaction.baseline_raw[index]; return 1U;
}
/** @brief Returns the retained original AA journal only after exact ACK plus actual TX; invalid evidence stays absent. */
static uint8_t aethor_app_persistence_get_save_journal(uint32_t request_id, AethorPersistenceSaveJournal *journal)
{
    if (!journal || !request_id || request_id != application_persistence.request_id ||
        !application_persistence.locked || !application_persistence_save_journal.valid) { return 0U; }
    *journal = application_persistence_save_journal; return 1U;
}
#endif
