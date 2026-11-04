/** @file debug_ui_static_trace_app.inc
 * @brief Bounded passive AXI trace: permanent startup prefix, rolling cleanup tail and permanent first fault.
 * Writers are task-only. Readers copy at most 128 bytes under the shared critical hook, then format outside it.
 */
#if !AETHOR_ADRC_BENCH
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
/** @brief Bounded static trace allocation, checked below 14 KiB and explicitly assigned to AXI by the scatter file. */
static struct {
    DebugUiStaticTraceHeader header;
    DebugUiStaticTraceRecord records[DEBUG_UI_STATIC_TRACE_CAPACITY];
    uint32_t tail_next;
    uint32_t next_generation;
    uint32_t stop_epoch, stop_request;
    uint64_t fallback_after_us;
    uint64_t emergency_after_us, emergency_deadline_us;
    uint32_t emergency_result_ready, emergency_result_fault, emergency_failed_motor;
} application_static_trace __attribute__((section(".static_trace")));
typedef char StaticTraceAllocationCheck[(sizeof(application_static_trace) <= 14U * 1024U) ? 1 : -1];

/** @brief Append a fully zeroed record; trace saturation only reports gaps and never changes motor state. */
static void aethor_app_static_trace_record_identity(uint8_t event, uint8_t motor_id, const CanFrame *frame,
    uint64_t lower_us, uint64_t observed_us, uint16_t sof, uint8_t sof_valid, uint32_t fault,
    const DebugUiStaticTxIdentity *identity)
{
    DebugUiStaticTraceRecord record;
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    uint32_t slot;
    uint8_t joint_index;
    aethor_app_enter_task_critical();
    if (!header->active || header->frozen ||
        (event == STATIC_TRACE_FIRST_FAULT && header->first_fault_valid)) {
        aethor_app_exit_task_critical(); return;
    }
    memset(&record, 0, sizeof(record));
    if (!motor_id && frame) {
        motor_id = (event == STATIC_TRACE_RAW_RX || event == STATIC_TRACE_ACCEPTED_RX) ?
            (uint8_t)(frame->data[0] & 0x0FU) : (uint8_t)frame->identifier;
    }
    record.timestamp_us = lower_us;
    record.observed_us = observed_us;
    record.serial = header->total;
    record.epoch = header->epoch;
    record.request_id = header->request_id;
    record.sequence = application_action.static_identity.sequence;
    record.phase = application_action.static_identity.phase;
    if (identity) {
        record.epoch = identity->epoch; record.request_id = identity->request_id;
        record.sequence = identity->sequence; record.phase = identity->phase;
    }
    record.event = event;
    record.motor_id = motor_id;
    record.sof = sof;
    record.sof_valid = sof_valid;
    record.fault = fault;
    if (motor_id && motor_id <= ARM_JOINT_COUNT) {
        const MotorJointFeedback *feedback;
        joint_index = (uint8_t)(motor_id - 1U);
        feedback = &application_motor_runtime.bank.motors[joint_index].feedback;
        record.feedback_generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        record.target_rad = header->axes[joint_index].target_rad;
        record.kp = header->axes[joint_index].kp;
        record.kd = header->axes[joint_index].kd;
        record.torque_ff_nm = header->axes[joint_index].torque_ff_nm;
        record.position_rad = feedback->position_rad;
        record.velocity_rad_s = feedback->velocity_rad_s;
        record.torque_nm = feedback->torque_nm;
        record.driver_state = feedback->driver_state;
        if (!fault) record.fault = feedback->fault_flags;
    }
    if (frame) {
        record.can_id = frame->identifier;
        record.length = frame->length;
        memcpy(record.raw, frame->data, frame->length <= 8U ? frame->length : 8U);
    }
    if (event == STATIC_TRACE_FIRST_FAULT && !header->first_fault_valid) {
        header->first_fault = record;
        header->first_fault_valid = 1U;
    }
    if (event == STATIC_TRACE_TERMINAL) { header->terminal = record; header->terminal_valid = 1U; }
    if (header->total < 64U) slot = header->total;
    else {
        slot = 64U + application_static_trace.tail_next;
        application_static_trace.tail_next = (application_static_trace.tail_next + 1U) % 64U;
        if (header->total >= DEBUG_UI_STATIC_TRACE_CAPACITY) { ++header->overflow; ++header->gaps; }
    }
    application_static_trace.records[slot] = record;
    ++header->total;
    header->count = header->total < DEBUG_UI_STATIC_TRACE_CAPACITY ? header->total : DEBUG_UI_STATIC_TRACE_CAPACITY;
    if (event == STATIC_TRACE_TERMINAL) { header->frozen = 1U; header->active = 0U; }
    aethor_app_exit_task_critical();
}

/** @brief Record current action context; hardware receipts may instead supply their immutable original identity. */
static void aethor_app_static_trace_record(uint8_t event, uint8_t motor_id, const CanFrame *frame,
    uint64_t lower_us, uint64_t observed_us, uint16_t sof, uint8_t sof_valid, uint32_t fault)
{
    aethor_app_static_trace_record_identity(event, motor_id, frame, lower_us, observed_us, sof, sof_valid, fault, NULL);
}

/** @brief Arm before setup using the original immutable command, original ranges and current mapping. */
static void aethor_app_static_trace_begin(uint64_t timestamp_us)
{
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    uint8_t joint_index;
    if (!debug_ui_is_static_operation(application_action.command.local_operation) ||
        application_action.command.origin != DEBUG_UI_ORIGIN_LOCAL_UI) return;
    aethor_app_enter_task_critical();
    ++application_static_trace.next_generation;
    if (!application_static_trace.next_generation) ++application_static_trace.next_generation;
    memset(header, 0, sizeof(*header));
    application_static_trace.tail_next = 0U;
    application_static_trace.stop_epoch = application_static_trace.stop_request = 0U;
    application_static_trace.fallback_after_us = 0U;
    application_static_trace.emergency_after_us = application_static_trace.emergency_deadline_us = 0U;
    application_static_trace.emergency_result_ready = application_static_trace.emergency_result_fault = 0U;
    application_static_trace.emergency_failed_motor = 0U;
    header->schema = 1U;
    header->generation = application_static_trace.next_generation;
    header->record_size = sizeof(DebugUiStaticTraceRecord);
    header->header_size = sizeof(*header);
    header->active = 1U;
    header->started_us = timestamp_us;
    header->epoch = application_action.command.session_id;
    header->request_id = application_action.command.request_id;
    header->motor_mask = application_action.command.motor_mask;
    header->target_acceptance = 0U; /* UNKNOWN: the vendor feedback does not echo an MIT target. */
    header->sof_clock = 1U; /* Hardware 16-bit FDCAN bus-SOF domain, not motor sampling time. */
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index) {
        DebugUiStaticTraceAxis *axis = &header->axes[joint_index];
        const MotorDiscoveryResult *discovery = &application_motor_runtime.discovery.results[joint_index];
        DebugUiMotorProfile scoped_profile = application_action.command.local_operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            aethor_app_single_static_bounds(joint_index) : application_debug_ui.profiles[joint_index];
        const DebugUiMotorProfile *profile = &scoped_profile;
        axis->reference = application_action.command.static_reference_generations[joint_index];
        axis->evidence = application_debug_ui.single_static_evidence_generation[joint_index];
        axis->feedback_generation = application_motor_runtime.bank.motors[joint_index].feedback_generation;
        axis->esc_id = discovery->observed_esc_id;
        axis->master_id = discovery->observed_master_id;
        axis->mode = discovery->observed_control_mode;
        axis->target_rad = application_action.command.values[joint_index];
        axis->position_ratio = aethor_app_lcd_position_ratio(joint_index);
        axis->kp = application_action.command.mit_kp;
        axis->kd = application_action.command.mit_kd;
        axis->torque_ff_nm = application_action.command.mit_torque_ff_nm;
        axis->range_position_rad = discovery->ranges.position_max_rad;
        axis->range_velocity_rad_s = discovery->ranges.velocity_max_rad_s;
        axis->range_torque_nm = discovery->ranges.torque_max_nm;
        axis->minimum_rad = profile->mit_position_min_rad;
        axis->maximum_rad = profile->mit_position_max_rad;
        axis->maximum_speed_rad_s = profile->mit_max_speed_rad_s;
    }
    aethor_app_exit_task_critical();
    aethor_app_static_trace_record(STATIC_TRACE_BEGIN, 0U, NULL, timestamp_us, timestamp_us, 0U, 0U, 0U);
}

/** @brief Preserve cancellation until cleanup's STOP terminal; unrelated results cannot freeze another generation. */
static void aethor_app_static_trace_terminal(const ProtocolCommandResult *result)
{
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    uint8_t matching, stop;
    if (!result || !header->active) return;
    matching = (uint8_t)(result->origin == DEBUG_UI_ORIGIN_LOCAL_UI && result->session_id == header->epoch &&
        result->request_id == header->request_id && debug_ui_is_static_operation(result->local_operation));
    stop = (uint8_t)(result->origin == DEBUG_UI_ORIGIN_LOCAL_UI && result->local_operation == DEBUG_UI_OPERATION_STOP &&
        application_static_trace.stop_request == result->request_id &&
        application_static_trace.stop_epoch == result->session_id);
    if (!matching && !stop) return;
    if (application_static_trace.emergency_after_us) {
        if (!application_static_trace.emergency_result_ready) {
            application_static_trace.emergency_result_ready = 1U;
            application_static_trace.emergency_result_fault = ((uint32_t)result->code << 16U) | result->detail;
            application_static_trace.emergency_failed_motor = result->failed_motor_number;
        }
        return;
    }
    if (result->code == PROTOCOL_COMMAND_RESULT_CANCELLED) {
        aethor_app_static_trace_record(STATIC_TRACE_CANCELLED, result->failed_motor_number, NULL,
            result->completed_at_us, result->completed_at_us, 0U, 0U, result->detail);
        return;
    }
    aethor_app_static_trace_record(STATIC_TRACE_TERMINAL, result->failed_motor_number, NULL,
        result->completed_at_us, result->completed_at_us, 0U, 0U,
        ((uint32_t)result->code << 16U) | result->detail);
    aethor_app_enter_task_critical();
    header->frozen = 1U;
    header->active = 0U;
    aethor_app_exit_task_critical();
}

/** @brief Begin passive global-emergency collection before the command result can clear its action state. */
static void aethor_app_static_trace_emergency_begin(uint64_t timestamp_us)
{
    if (!application_static_trace.header.active || application_static_trace.emergency_after_us) return;
    aethor_app_static_trace_record(STATIC_TRACE_FIRST_FAULT, 0U, NULL, timestamp_us, timestamp_us, 0U, 0U,
        ((uint32_t)DEBUG_UI_REASON_UNHEALTHY << 16U) | (uint32_t)PROTOCOL_COMMAND_ERROR_ACTION_FAILED);
    aethor_app_enter_task_critical();
    application_static_trace.emergency_after_us = timestamp_us;
    application_static_trace.emergency_deadline_us = timestamp_us <= UINT64_MAX - AETHOR_APP_ACTION_TIMEOUT_US ?
        timestamp_us + AETHOR_APP_ACTION_TIMEOUT_US : UINT64_MAX;
    aethor_app_exit_task_critical();
}
/** @brief Observe existing emergency cleanup only; never enqueue/retry a frame or extend a control deadline. */
static void aethor_app_static_trace_emergency_progress(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    uint8_t index, safe = 1U;
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    if (!header->active || !application_static_trace.emergency_after_us ||
        !application_static_trace.emergency_result_ready) return;
    if (!snapshot || snapshot->valid_joint_mask != AETHOR_APP_ALL_JOINTS_MASK ||
        !application_emergency_disable_batch.count ||
        application_emergency_disable_read_index < application_emergency_disable_batch.count ||
        application_debug_ui.single_static_transport_mask) safe = 0U;
    for (index = 0U; safe && index < ARM_JOINT_COUNT; ++index) {
        const MotorJointFeedback *feedback = &snapshot->joints[index];
        if (feedback->timestamp_us <= application_static_trace.emergency_after_us ||
            feedback->timestamp_us > timestamp_us || feedback->driver_state != S3519_DRIVER_STATE_DISABLED ||
            feedback->fault_flags || !isfinite(feedback->velocity_rad_s) ||
            fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) safe = 0U;
    }
    if (!safe && timestamp_us < application_static_trace.emergency_deadline_us) return;
    aethor_app_enter_task_critical();
    header->cleanup_outcome = safe ? 1U : 2U;
    aethor_app_static_trace_record(STATIC_TRACE_TERMINAL, (uint8_t)application_static_trace.emergency_failed_motor,
        NULL, timestamp_us, timestamp_us, 0U, 0U, application_static_trace.emergency_result_fault);
    aethor_app_exit_task_critical();
}

/** @brief Bind the exact STOP even when startup failed before the statistical HOLD report began. */
static void aethor_app_static_trace_bind_stop(const ProtocolCommand *command)
{
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    if (!command || !header->active || command->origin != DEBUG_UI_ORIGIN_LOCAL_UI ||
        (command->motor_mask & header->motor_mask) != header->motor_mask) return;
    application_static_trace.stop_epoch = command->session_id;
    application_static_trace.stop_request = command->request_id;
}
/** @brief Retain the original scope and timestamp for a saturated startup STOP with no result identity. */
static void aethor_app_static_trace_bind_fallback(uint64_t timestamp_us)
{
    if (application_static_trace.header.active) application_static_trace.fallback_after_us = timestamp_us;
}
/** @brief Freeze a saturated startup trace only after every emitted selected emergency frame has newer safe feedback. */
static void aethor_app_static_trace_confirm_fallback(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{
    DebugUiStaticTraceHeader *header = &application_static_trace.header;
    uint8_t index, expected = 0U;
    if (!header->active || !application_static_trace.fallback_after_us || !snapshot ||
        (snapshot->valid_joint_mask & header->motor_mask) != header->motor_mask ||
        (application_debug_ui.fallback_stop_mask & header->motor_mask) != header->motor_mask) return;
    for (index = 0U; index < ARM_JOINT_COUNT; ++index)
        if (application_debug_ui.fallback_stop_mask & (1U << index)) ++expected;
    if (application_debug_ui.fallback_frames.count != expected ||
        application_debug_ui.fallback_frame_read_index != expected) return;
    for (index = 0U; index < ARM_JOINT_COUNT; ++index) {
        const MotorJointFeedback *feedback = &snapshot->joints[index];
        if (!(header->motor_mask & (1U << index))) continue;
        if (feedback->timestamp_us <= application_static_trace.fallback_after_us ||
            feedback->timestamp_us > timestamp_us || feedback->driver_state != S3519_DRIVER_STATE_DISABLED ||
            feedback->fault_flags || !isfinite(feedback->velocity_rad_s) ||
            fabsf(feedback->velocity_rad_s) > AETHOR_APP_MODE_SWITCH_MAX_SPEED_RAD_S) return;
    }
    aethor_app_static_trace_record(STATIC_TRACE_TERMINAL, 0U, NULL, timestamp_us, timestamp_us,
        0U, 0U, (uint32_t)PROTOCOL_COMMAND_RESULT_STOPPED << 16U);
    header->active = 0U; header->frozen = 1U;
}

/** @brief Parse one complete unsigned field with overflow rejection, preserving read-only query semantics. */
static uint8_t aethor_app_static_trace_unsigned(const TextProtocolRequest *request, const char *name, uint32_t *value)
{
    TextProtocolSpan span;
    size_t index;
    uint32_t parsed = 0U;
    if (!text_protocol_find_field(request, name, &span) || !span.length) return 0U;
    for (index = 0U; index < span.length; ++index) {
        uint32_t digit = (uint32_t)(span.data[index] - '0');
        if (digit > 9U || parsed > (UINT32_MAX - digit) / 10U) return 0U;
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return 1U;
}

/** @brief Read only a status prefix or one fixed chunk; no queue, authority or CAN state is changed. */
static uint8_t aethor_app_static_trace_query(const char *line, size_t length, ProtocolOutputBatch *output)
{
    TextProtocolRequest request;
    uint8_t chunk[128];
    uint32_t generation = 0U, index = 0U, part = 0U, parts = 0U, bytes = 0U, slot, offset;
    uint32_t prefix[12];
    int written;
    uint8_t status, valid = 1U;
    static const char digits[] = "0123456789abcdef";
    if (!output || text_protocol_parse_request(line, length, &request) != TEXT_PROTOCOL_STATUS_OK) return 0U;
    status = text_protocol_request_path_equals(&request, "static", "trace_status");
    if (!status && !text_protocol_request_path_equals(&request, "static", "trace")) return 0U;
    memset(output, 0, sizeof(*output));
    if (status) {
        if (!request.has_request_id || request.field_count || request.positional_count) valid = 0U;
        aethor_app_enter_task_critical();
        memcpy(prefix, &application_static_trace.header, sizeof(prefix));
        aethor_app_exit_task_critical();
        written = snprintf(output->messages[0].data, sizeof(output->messages[0].data),
            "ok %lu static trace_status valid=%u schema=1 generation=%lu active=%lu frozen=%lu count=%lu total=%lu overflow=%lu gaps=%lu record_size=96 header_size=832 first_fault=%lu terminal=%lu target_acceptance=unknown sof_clock=hardware16_bus_sof\r\n",
            (unsigned long)request.request_id, valid, (unsigned long)prefix[1], (unsigned long)prefix[6], (unsigned long)prefix[7],
            (unsigned long)prefix[2], (unsigned long)prefix[3], (unsigned long)prefix[4], (unsigned long)prefix[5],
            (unsigned long)prefix[8], (unsigned long)prefix[9]);
    } else {
        valid = (uint8_t)(request.has_request_id && request.field_count == 3U && !request.positional_count &&
            aethor_app_static_trace_unsigned(&request, "generation", &generation) &&
            aethor_app_static_trace_unsigned(&request, "index", &index) &&
            aethor_app_static_trace_unsigned(&request, "part", &part));
        aethor_app_enter_task_critical();
        valid = (uint8_t)(valid && generation && generation == application_static_trace.header.generation &&
            application_static_trace.header.frozen);
        if (valid && index == DEBUG_UI_STATIC_TRACE_HEADER_INDEX) {
            parts = (sizeof(DebugUiStaticTraceHeader) + sizeof(chunk) - 1U) / sizeof(chunk);
            if (part >= parts) valid = 0U;
            else {
                offset = part * sizeof(chunk);
                bytes = sizeof(DebugUiStaticTraceHeader) - offset;
                if (bytes > sizeof(chunk)) bytes = sizeof(chunk);
                memcpy(chunk, (const uint8_t *)&application_static_trace.header + offset, bytes);
            }
        } else if (valid && index < application_static_trace.header.count && !part) {
            parts = 1U; bytes = sizeof(DebugUiStaticTraceRecord); slot = index;
            if (index >= 64U && application_static_trace.header.total > DEBUG_UI_STATIC_TRACE_CAPACITY)
                slot = 64U + (application_static_trace.tail_next + index - 64U) % 64U;
            memcpy(chunk, &application_static_trace.records[slot], bytes);
        } else valid = 0U;
        aethor_app_exit_task_critical();
        if (!valid) bytes = 0U;
        written = snprintf(output->messages[0].data, sizeof(output->messages[0].data),
            "ok %lu static trace valid=%u generation=%lu index=%lu part=%lu parts=%lu bytes=%lu hex=",
            (unsigned long)request.request_id, valid, (unsigned long)generation, (unsigned long)index, (unsigned long)part,
            (unsigned long)parts, (unsigned long)bytes);
        if (written < 0 || (size_t)written + 2U * bytes + 3U > sizeof(output->messages[0].data)) return 1U;
        for (offset = 0U; offset < bytes; ++offset) {
            output->messages[0].data[written++] = digits[chunk[offset] >> 4U];
            output->messages[0].data[written++] = digits[chunk[offset] & 15U];
        }
        output->messages[0].data[written++] = '\r'; output->messages[0].data[written++] = '\n';
        output->messages[0].data[written] = '\0';
    }
    if (written < 0 || (size_t)written >= sizeof(output->messages[0].data)) return 1U;
    output->messages[0].length = (uint16_t)written;
    output->messages[0].priority = PROTOCOL_OUTPUT_QUERY;
    output->count = 1U;
    return 1U;
}
#else
/** @brief Default-off images allocate no trace storage. */
static void aethor_app_static_trace_begin(uint64_t timestamp_us) { (void)timestamp_us; }
/** @brief Default-off writer is a no-op and cannot create motor authority. */
static void aethor_app_static_trace_record(uint8_t event, uint8_t motor_id, const CanFrame *frame,
    uint64_t lower_us, uint64_t observed_us, uint16_t sof, uint8_t sof_valid, uint32_t fault)
{ (void)event; (void)motor_id; (void)frame; (void)lower_us; (void)observed_us; (void)sof; (void)sof_valid; (void)fault; }
/** @brief Default-off result handling allocates nothing. */
static void aethor_app_static_trace_terminal(const ProtocolCommandResult *result) { (void)result; }
/** @brief Unknown/default-off commands continue through the existing read-only protocol rejection path. */
static uint8_t aethor_app_static_trace_query(const char *line, size_t length, ProtocolOutputBatch *output)
{ (void)line; (void)length; (void)output; return 0U; }
/** @brief Default-off global emergency trace hooks preserve existing control behavior. */
static void aethor_app_static_trace_emergency_begin(uint64_t timestamp_us) { (void)timestamp_us; }
/** @brief Default-off progress has no storage, traffic or control effects. */
static void aethor_app_static_trace_emergency_progress(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{ (void)snapshot; (void)timestamp_us; }
/** @brief Default-off STOP hooks have no trace allocation or authority. */
static void aethor_app_static_trace_bind_stop(const ProtocolCommand *command) { (void)command; }
#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Default-off fallback hook has no trace allocation. */
static void aethor_app_static_trace_bind_fallback(uint64_t timestamp_us) { (void)timestamp_us; }
/** @brief Default-off fallback confirmation has no trace allocation. */
static void aethor_app_static_trace_confirm_fallback(const MotorFeedbackSnapshot *snapshot, uint64_t timestamp_us)
{ (void)snapshot; (void)timestamp_us; }
#endif
#endif

#endif
