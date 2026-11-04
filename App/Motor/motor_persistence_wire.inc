/** @file motor_persistence_wire.inc
 * @brief Fixed schema-1 maintenance commands; included once after the private typed adapter.
 * All entry points execute under the existing App task critical section. No arbitrary parameter,
 * automatic Save retry, enable, reset, zero or motor motion is exposed by this interface. */
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT

/** @brief One explicit release intent and the immutable successful status retained after ownership ends. */
typedef struct
{
    AethorPersistenceStatus released_status;
    uint8_t release_requested, released;
} AethorPersistenceWire;
static AethorPersistenceWire application_persistence_wire;

/** @brief Parses a complete unsigned decimal token without float conversion, signs or integer overflow. */
static uint8_t aethor_app_persistence_wire_uint(const TextProtocolSpan *span, uint32_t *value)
{
    uint32_t result = 0U;
    size_t index;
    if (!span || !value || !span->length) { return 0U; }
    for (index = 0U; index < span->length; ++index) {
        unsigned digit;
        if (span->data[index] < '0' || span->data[index] > '9') { return 0U; }
        digit = (unsigned)(span->data[index] - '0');
        if (result > (UINT32_MAX - digit) / 10U) { return 0U; }
        result = result * 10U + digit;
    }
    *value = result; return 1U;
}

/** @brief Requires exactly the published field set and nonzero original transaction identity. */
static uint8_t aethor_app_persistence_wire_fields(const TextProtocolRequest *request,
    uint8_t count, uint32_t *id, uint32_t *phase, uint32_t *index)
{
    TextProtocolSpan span;
    if (!request->has_request_id || !request->request_id || request->positional_count ||
        request->field_count != count) { return 0U; }
    if (count && (!text_protocol_find_field(request, "id", &span) ||
        !aethor_app_persistence_wire_uint(&span, id) || !*id)) { return 0U; }
    if (count == 3U && (!text_protocol_find_field(request, "phase", &span) ||
        !aethor_app_persistence_wire_uint(&span, phase) || !text_protocol_find_field(request, "index", &span) ||
        !aethor_app_persistence_wire_uint(&span, index) || *phase > UINT8_MAX ||
        *index >= MOTOR_PERSISTENCE_REGISTER_COUNT)) { return 0U; }
    return 1U;
}

/** @brief Reuses all original13 discovery validators on the already proved full snapshot, without another CAN request.
 * WAITING below selects the validator's input contract only; it is never exported as actual-TX evidence. */
static uint8_t aethor_app_persistence_wire_discovery(MotorDiscoveryResult *result)
{
    static const uint8_t rids[MOTOR_DISCOVERY_REGISTER_COUNT] = {
        S3519_REGISTER_MASTER_ID, S3519_REGISTER_ESC_ID, S3519_REGISTER_CONTROL_MODE,
        S3519_REGISTER_ACCELERATION, S3519_REGISTER_DECELERATION, S3519_REGISTER_MAXIMUM_SPEED,
        S3519_REGISTER_HARDWARE_VERSION, S3519_REGISTER_SOFTWARE_VERSION, S3519_REGISTER_SUB_VERSION,
        S3519_REGISTER_POSITION_RANGE, S3519_REGISTER_VELOCITY_RANGE, S3519_REGISTER_TORQUE_RANGE,
        S3519_REGISTER_TIMEOUT};
    MotorDiscovery validation;
    unsigned field, index;
    if (!result || motor_discovery_init(&validation, arm_config_get_production()) != MOTOR_DISCOVERY_STATUS_OK ||
        motor_discovery_begin(&validation, 0x40U) != MOTOR_DISCOVERY_STATUS_OK) { return 0U; }
    for (field = 0U; field < MOTOR_DISCOVERY_REGISTER_COUNT; ++field) {
        S3519ParameterResponse response;
        const MotorPersistenceItem *item = NULL;
        for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
            if (motor_persistence_rid(index) == rids[field]) {
                item = &application_persistence_transaction.current.items[index]; break;
            }
        }
        if (!item || !item->valid || item->identity.phase != application_persistence.snapshot_phase) { return 0U; }
        memset(&response, 0, sizeof(response)); response.esc_id = 7U;
        response.register_address = rids[field]; response.raw_value = item->raw;
        memcpy(&response.float_value, &item->raw, sizeof(response.float_value));
        validation.state = MOTOR_DISCOVERY_STATE_WAITING;
        if (motor_discovery_accept_response(&validation, 23U, &response) != MOTOR_DISCOVERY_STATUS_OK) { return 0U; }
    }
    if (validation.state != MOTOR_DISCOVERY_STATE_COMPLETE || validation.verified_joint_mask != 0x40U ||
        validation.results[6].verified_fields_mask != MOTOR_DISCOVERY_ALL_FIELDS_MASK) { return 0U; }
    *result = validation.results[6]; return 1U;
}

/** @brief Completes the explicit release only after real debt-free observations and a new four-millisecond quiet interval. */
static void aethor_app_persistence_wire_try_release(uint64_t now_us)
{
    AethorPersistenceApp *owner = &application_persistence;
    MotorDiscoveryResult validated;
    MotorJointFeedback feedback;
    MotorObject *motor = &application_motor_runtime.bank.motors[6];
    const MotorPersistenceItem *last = &application_persistence_transaction.current.items[MOTOR_PERSISTENCE_REGISTER_COUNT - 1U];
    uint64_t latest_activity;
    if (!application_persistence_wire.release_requested || application_persistence_wire.released ||
        !aethor_app_persistence_typed_ready(owner->request_id, now_us) ||
        application_persistence_transaction.state != MOTOR_PERSISTENCE_COMPLETE ||
        application_persistence_transaction.save_uncertain || owner->confirmed_mode != 2U ||
        owner->ranges.position_max_rad != 512.0F || !aethor_app_persistence_sources_idle(now_us, 1U)) { return; }
    latest_activity = owner->health.observed_us;
    if (owner->disable_submitted_us > latest_activity) { latest_activity = owner->disable_submitted_us; }
    if (owner->disable_observed_us > latest_activity) { latest_activity = owner->disable_observed_us; }
    if (last->received_us > latest_activity) { latest_activity = last->received_us; }
    if (last->send_completed_observed_us > latest_activity) { latest_activity = last->send_completed_observed_us; }
    if (!latest_activity || now_us < latest_activity || now_us - latest_activity < 4000ULL) { return; }
    if (!aethor_app_persistence_wire_discovery(&validated) || validated.observed_control_mode != 2U ||
        validated.ranges.position_max_rad != 512.0F || validated.ranges.velocity_max_rad_s != 200.0F ||
        validated.ranges.torque_max_nm != 10.0F || validated.software_version != 808597046U || validated.sub_version != 49U) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, now_us); return;
    }
    memset(&feedback, 0, sizeof(feedback));
    feedback.position_rad = owner->native_position; feedback.velocity_rad_s = owner->health.velocity;
    feedback.torque_nm = owner->health.torque; feedback.timestamp_us = owner->health.observed_us;
    feedback.mos_temperature_c = owner->health.driver_temperature;
    feedback.rotor_temperature_c = owner->health.motor_temperature; feedback.driver_state = S3519_DRIVER_STATE_DISABLED;
    if (motor_bank_update_feedback(&application_motor_runtime.bank, 23U, 7U, &feedback) != MOTOR_BANK_STATUS_OK) {
        aethor_app_persistence_abort(MOTOR_PERSISTENCE_ERROR_CONTEXT, now_us); return;
    }
    application_motor_runtime.discovery.results[6] = validated;
    application_motor_runtime.discovery.verified_joint_mask |= 0x40U;
    motor->parameter_valid_mask = validated.verified_fields_mask;
    motor->parameter_source = MOTOR_PARAMETER_SOURCE_DISCOVERED; motor->configuration_consistent = 1U;
    motor->state = MOTOR_LIFECYCLE_DISABLED; motor->target_valid = motor->at_target = 0U; motor->active_request_id = 0U;
    (void)aethor_app_persistence_get_status(&application_persistence_wire.released_status);
    owner->locked = 0U; owner->admission_generation = aethor_app_persistence_increment(owner->admission_generation);
    application_persistence_wire.released_status.locked = 0U;
    application_persistence_wire.released = 1U;
}

/** @brief Exposes only established immutable transaction status, including the explicitly released terminal projection. */
static uint8_t aethor_app_persistence_wire_status(AethorPersistenceStatus *status, uint64_t now_us)
{
    if (application_persistence_wire.released) {
        *status = application_persistence_wire.released_status;
        status->now_us = now_us; return 1U;
    }
    return aethor_app_persistence_get_status(status);
}

/** @brief Keeps the original ordinary read-only queries and heartbeat usable while motion remains excluded. */
static uint8_t aethor_app_persistence_wire_readonly(const TextProtocolRequest *request)
{
    return (uint8_t)(text_protocol_request_path_equals(request, "ping", NULL) ||
        text_protocol_request_path_equals(request, "adrc", "status") ||
        text_protocol_request_path_equals(request, "adrc", "gate") ||
        text_protocol_request_path_equals(request, "adrc", "shaft_status"));
}

/** @brief Encodes one exact raw frame word without numeric float conversion. */
static uint32_t aethor_app_persistence_wire_frame_raw(const CanFrame *frame)
{
    return (uint32_t)frame->data[0] | ((uint32_t)frame->data[1] << 8U) |
        ((uint32_t)frame->data[2] << 16U) | ((uint32_t)frame->data[3] << 24U);
}

/** @brief Handles only the fixed maintenance command family, preserving the original response capacity. */
static uint8_t aethor_app_persistence_wire_route(const TextProtocolRequest *request, uint64_t now_us,
    ProtocolOutputBatch *output, ProtocolEngineStatus *status)
{
    static const char *names[] = {"persist_begin", "persist_status", "persist_health", "persist_item",
        "persist_prepare", "persist_verify", "persist_commit", "persist_save_receipt", "persist_release"};
    unsigned operation;
    uint32_t id = 0U, phase = 0U, index = 0U;
    uint8_t accepted = 0U;
    int written = -1;
    char detail[480];
    const char *code = "not_ready_or_argument";
    for (operation = 0U; operation < sizeof(names) / sizeof(names[0]); ++operation) {
        if (text_protocol_request_path_equals(request, "adrc", names[operation])) { break; }
    }
    if (operation == sizeof(names) / sizeof(names[0])) { return 0U; }
    aethor_app_persistence_wire_try_release(now_us);
    if (operation == 0U) {
        TextProtocolSpan motor;
        if (request->has_request_id && request->request_id && !request->positional_count && request->field_count == 1U &&
            text_protocol_find_field(request, "motor", &motor) && aethor_app_persistence_wire_uint(&motor, &id) && id == 7U &&
            aethor_app_persistence_begin_readonly(request->request_id, now_us)) {
            id = request->request_id; accepted = 1U;
        }
    } else if (aethor_app_persistence_wire_fields(request, operation == 1U ? 0U : operation == 3U ? 3U : 1U,
        &id, &phase, &index)) {
        if (operation == 1U) {
            AethorPersistenceStatus view;
            if (aethor_app_persistence_wire_status(&view, now_us)) {
                written = snprintf(detail, sizeof(detail),
                    "persist_status schema=1 id=%lu boot=%lu owner=%lu link=%lu epoch=%lu session=%lu state=%u error=%u fault=%u locked=%u ready=%u snapshot_phase=%u skip_save=%u save_used=%u save_uncertain=%u pmax_known=%u mode_known=%u pmax_raw=%lu mode=%lu now_us=%llu verified_us=%llu released=%u",
                    (unsigned long)view.identity.request_id, (unsigned long)view.identity.boot_epoch,
                    (unsigned long)view.identity.owner_generation, (unsigned long)view.link_word,
                    (unsigned long)view.local_epoch, (unsigned long)view.protocol_session, (unsigned)view.state,
                    (unsigned)view.error, (unsigned)view.fault, view.locked, view.snapshot_ready, view.snapshot_phase,
                    view.skip_save, view.save_used, view.save_uncertain, view.pmax_known, view.mode_known,
                    (unsigned long)view.confirmed_pmax_raw, (unsigned long)view.confirmed_mode,
                    (unsigned long long)view.now_us, (unsigned long long)view.verified_us, application_persistence_wire.released);
            }
        } else if (operation == 2U) {
            AethorPersistenceHealthView view;
            if (aethor_app_persistence_get_health(id, &view)) {
                written = snprintf(detail, sizeof(detail),
                    "persist_health schema=1 id=%lu boot=%lu owner=%lu link=%lu epoch=%lu session=%lu now_us=%llu observed_us=%llu valid=%u disabled=%u idle=%u fault=%u health_proven=%u velocity=%.9g torque=%.9g motor_c=%.9g driver_c=%.9g reference_native=%.9g native_position=%.9g tolerance_native=%.9g continuity_valid=%u",
                    (unsigned long)id, (unsigned long)application_persistence.boot_id,
                    (unsigned long)application_persistence_transaction.identity.owner_generation,
                    (unsigned long)application_persistence.bound_link_word, (unsigned long)application_persistence.local_epoch,
                    (unsigned long)application_persistence.protocol_session, (unsigned long long)application_persistence.last_us,
                    (unsigned long long)view.health.observed_us, view.health.valid, view.health.disabled, view.health.idle,
                    view.health.fault, view.health_proven, (double)view.health.velocity, (double)view.health.torque,
                    (double)view.health.motor_temperature, (double)view.health.driver_temperature,
                    (double)view.reference_native_position, (double)view.native_position,
                    (double)view.continuity_tolerance, view.continuity_checked);
            }
        } else if (operation == 3U) {
            AethorPersistenceItemView view;
            if (aethor_app_persistence_get_item(id, (uint8_t)phase, index, &view)) {
                const MotorPersistenceItem *item = &view.item;
                written = snprintf(detail, sizeof(detail),
                    "persist_item schema=1 id=%lu index=%lu rid=%u kind=%s raw=%lu valid=%u boot_epoch=%lu request_id=%lu owner_generation=%lu phase=%u sequence=%lu created_us=%llu submitted_us=%llu boundary_us=%llu send_completed_observed_us=%llu rx_lower_us=%llu received_us=%llu",
                    (unsigned long)id, (unsigned long)index, view.rid, view.is_unsigned ? "uint32" : "float32",
                    (unsigned long)item->raw, item->valid, (unsigned long)item->identity.boot_epoch,
                    (unsigned long)item->identity.request_id, (unsigned long)item->identity.owner_generation,
                    item->identity.phase, (unsigned long)item->identity.sequence, (unsigned long long)item->created_us,
                    (unsigned long long)item->submitted_us, (unsigned long long)item->boundary_us,
                    (unsigned long long)item->send_completed_observed_us, (unsigned long long)item->rx_lower_us,
                    (unsigned long long)item->received_us);
            }
        } else if (operation >= 4U && operation <= 6U) {
            accepted = operation == 4U ? aethor_app_persistence_prepare(id, now_us) : operation == 5U ?
                aethor_app_persistence_request_save(id, now_us) : aethor_app_persistence_commit_save(id, now_us);
        } else if (operation == 7U && id == application_persistence.request_id && application_persistence.locked) {
            AethorPersistenceSaveJournal view;
            if (aethor_app_persistence_get_save_journal(id, &view)) {
                const MotorPersistenceItem *item = &view.receipt;
                written = snprintf(detail, sizeof(detail),
                    "persist_save_receipt schema=1 id=%lu valid=1 boot_epoch=%lu request_id=%lu owner_generation=%lu phase=%u sequence=%lu created_us=%llu submitted_us=%llu boundary_us=%llu send_completed_observed_us=%llu rx_lower_us=%llu received_us=%llu tx_id=%u tx_dlc=%u tx_raw=%lu ack_id=%u ack_dlc=%u ack_raw=%lu settle_until_us=%llu",
                    (unsigned long)id, (unsigned long)item->identity.boot_epoch, (unsigned long)item->identity.request_id,
                    (unsigned long)item->identity.owner_generation, item->identity.phase, (unsigned long)item->identity.sequence,
                    (unsigned long long)item->created_us, (unsigned long long)item->submitted_us,
                    (unsigned long long)item->boundary_us, (unsigned long long)item->send_completed_observed_us,
                    (unsigned long long)item->rx_lower_us, (unsigned long long)item->received_us,
                    view.tx.identifier, view.tx.length, (unsigned long)aethor_app_persistence_wire_frame_raw(&view.tx),
                    view.ack.identifier, view.ack.length, (unsigned long)aethor_app_persistence_wire_frame_raw(&view.ack),
                    (unsigned long long)view.settle_until_us);
            } else { written = snprintf(detail, sizeof(detail), "persist_save_receipt schema=1 id=%lu valid=0", (unsigned long)id); }
        } else if (operation == 8U && !application_persistence_wire.release_requested &&
            aethor_app_persistence_typed_ready(id, now_us) &&
            application_persistence_transaction.state == MOTOR_PERSISTENCE_COMPLETE &&
            !application_persistence_transaction.save_uncertain) {
            application_persistence_wire.release_requested = 1U; accepted = 1U;
        }
    }
    if (accepted) { written = snprintf(detail, sizeof(detail), "%s schema=1 id=%lu accepted=1", names[operation], (unsigned long)id); }
    if (written < 0) {
        written = snprintf(detail, sizeof(detail), "%s schema=1 code=%s", names[operation], code);
        *status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, detail);
    } else if ((size_t)written >= sizeof(detail)) {
        (void)snprintf(detail, sizeof(detail), "%s schema=1 code=output_too_small", names[operation]);
        *status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST, request->request_id, detail);
    } else { *status = aethor_app_integrated_response(output, PROTOCOL_ENGINE_STATUS_OK, request->request_id, detail); }
    return 1U;
}
#endif
