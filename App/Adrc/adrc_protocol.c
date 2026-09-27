/** @file adrc_protocol.c
 * @brief Parses bounded ADRC requests into owner-task mailboxes without touching motor state.
 */
#include "adrc_protocol.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/** @brief Compares a parser-normalized token against an exact literal. */
static uint8_t span_equals(const TextProtocolSpan *span, const char *word)
{
    return (uint8_t)((span->length == strlen(word)) &&
        (strncmp(span->data, word, span->length) == 0));
}

/** @brief Formats one bounded response with its mandatory line terminator. */
static ProtocolEngineStatus format_output(ProtocolOutputBatch *output,
    ProtocolEngineStatus status, const char *format, ...)
{
    int length;
    va_list arguments;
    memset(output, 0, sizeof(*output));
    va_start(arguments, format);
    length = vsnprintf(output->messages[0].data, PROTOCOL_ENGINE_MESSAGE_CAPACITY - 2U,
        format, arguments);
    va_end(arguments);
    if ((length < 0) || ((unsigned int)length >= PROTOCOL_ENGINE_MESSAGE_CAPACITY - 2U))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    output->messages[0].data[length++] = '\r';
    output->messages[0].data[length++] = '\n';
    output->messages[0].data[length] = '\0';
    output->messages[0].length = (uint16_t)length;
    output->messages[0].priority = PROTOCOL_OUTPUT_HIGH_PRIORITY;
    output->count = 1U;
    return status;
}

/** @brief Reports a fixed public error without reflecting arbitrary request bytes. */
static ProtocolEngineStatus reject(ProtocolOutputBatch *output, uint32_t request_id,
    const char *reason)
{
    return format_output(output, PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
        "error %lu adrc code=%s", (unsigned long)request_id, reason);
}

/** @brief Parses a strict uint32 token without float rounding or overflow. */
static uint8_t parse_uint(const TextProtocolSpan *span, uint32_t *value)
{
    size_t index;
    uint32_t parsed = 0U;
    if (span->length == 0U) { return 0U; }
    for (index = 0U; index < span->length; ++index)
    {
        uint32_t digit;
        if ((span->data[index] < '0') || (span->data[index] > '9')) { return 0U; }
        digit = (uint32_t)(span->data[index] - '0');
        if ((parsed > UINT32_MAX / 10U) ||
            ((parsed == UINT32_MAX / 10U) && (digit > UINT32_MAX % 10U))) { return 0U; }
        parsed = parsed * 10U + digit;
    }
    *value = parsed;
    return 1U;
}

/** @brief Maps a fixed channel number to its stable command storage. */
static AdrcProtocolCommand *channel_command(AdrcProtocolGateway *gateway, uint8_t channel)
{
    if (channel == 1U) { return &gateway->stop_command; }
    if (channel == 2U) { return &gateway->heartbeat_command; }
    return &gateway->command;
}

/** @brief Validates a numeric configuration patch against immutable first-stage caps. */
static uint8_t parse_config(const TextProtocolRequest *request, AdrcProtocolCommand *command)
{
    static const char *const control_keys[] = { "b0", "wc", "wo", "target", "mode", "duration_ms" };
    static const char *const mapping_keys[] = { "pos_scale", "vel_scale", "torque_scale" };
    static const char *const limits_keys[] = { "torque_limit", "torque_slew", "reference_accel", "near_zero" };
    static const char *const identify_keys[] = { "torque", "pulse_ms" };
    const char *const *keys;
    TextProtocolSpan group;
    uint8_t key_count;
    uint8_t field_index;
    if (text_protocol_find_field(request, "group", &group) == 0U) { return 0U; }
    if (span_equals(&group, "control") != 0U)
    {
        command->group = ADRC_CONFIG_CONTROL; keys = control_keys; key_count = 6U;
    }
    else if (span_equals(&group, "mapping") != 0U)
    {
        command->group = ADRC_CONFIG_MAPPING; keys = mapping_keys; key_count = 3U;
    }
    else if (span_equals(&group, "limits") != 0U)
    {
        command->group = ADRC_CONFIG_LIMITS; keys = limits_keys; key_count = 4U;
    }
    else if (span_equals(&group, "identify") != 0U)
    {
        command->group = ADRC_CONFIG_IDENTIFY; keys = identify_keys; key_count = 2U;
    }
    else { return 0U; }
    for (field_index = 0U; field_index < request->field_count; ++field_index)
    {
        uint8_t key_index;
        const TextProtocolField *field = &request->fields[field_index];
        if (span_equals(&field->key, "group") != 0U) { continue; }
        for (key_index = 0U; key_index < key_count; ++key_index)
        {
            if (span_equals(&field->key, keys[key_index]) != 0U) { break; }
        }
        if (key_index == key_count) { return 0U; }
        if ((command->group == ADRC_CONFIG_CONTROL) && (key_index == 4U))
        {
            if (span_equals(&field->value, "identify") != 0U) { command->values[key_index] = 0.0F; }
            else if (span_equals(&field->value, "pi") != 0U) { command->values[key_index] = 1.0F; }
            else if (span_equals(&field->value, "ladrc") != 0U) { command->values[key_index] = 2.0F; }
            else { return 0U; }
        }
        else if (text_protocol_span_to_float(&field->value, &command->values[key_index]) != TEXT_PROTOCOL_STATUS_OK)
        { return 0U; }
        command->field_mask |= 1UL << key_index;
    }
    if (command->field_mask == 0U) { return 0U; }
    for (field_index = 0U; field_index < key_count; ++field_index)
    {
        float value = command->values[field_index];
        if ((command->field_mask & (1UL << field_index)) == 0U) { continue; }
        if (command->group == ADRC_CONFIG_CONTROL)
        {
            if ((field_index <= 2U) && (value <= 0.0F)) { return 0U; }
            if (((field_index == 1U) || (field_index == 2U)) && (value > 250.0F)) { return 0U; }
            if ((field_index == 3U) && ((value < -0.2F) || (value > 0.2F))) { return 0U; }
            if ((field_index == 5U) && ((value < 1.0F) || (value > 3000.0F) ||
                (value != (float)(uint32_t)value))) { return 0U; }
        }
        else if (command->group == ADRC_CONFIG_MAPPING)
        {
            if (value == 0.0F) { return 0U; }
        }
        else if (command->group == ADRC_CONFIG_IDENTIFY)
        {
            if ((field_index == 0U) && ((value < -0.05F) || (value > 0.05F) || (value == 0.0F))) { return 0U; }
            if ((field_index == 1U) && (value != 20.0F) && (value != 40.0F) && (value != 80.0F)) { return 0U; }
        }
        else
        {
            static const float hard_caps[] = { 0.1F, 0.5F, 0.3F, 0.03F };
            if ((value <= 0.0F) || (value > hard_caps[field_index])) { return 0U; }
        }
    }
    return 1U;
}

/** @brief Initializes only protocol memory; qualification remains false. */
void adrc_protocol_init(AdrcProtocolGateway *gateway)
{
    if (gateway != NULL) { memset(gateway, 0, sizeof(*gateway)); }
}

/** @brief Installs a read-only trace callback owned by the application. */
void adrc_protocol_set_trace_reader(AdrcProtocolGateway *gateway, AdrcProtocolTraceReader reader, void *context)
{
    if (gateway != NULL) { gateway->trace_reader = reader; gateway->trace_context = context; }
}

/** @brief Copies the sole control owner's published view. */
void adrc_protocol_publish_snapshot(AdrcProtocolGateway *gateway, const AdrcProtocolSnapshot *snapshot)
{
    if ((gateway != NULL) && (snapshot != NULL)) { gateway->snapshot = *snapshot; }
}

/** @brief Admits structured commands only after shared canonical cache checks have completed. */
ProtocolEngineStatus adrc_protocol_handle_request(void *context, const TextProtocolRequest *request,
    uint64_t timestamp_us, ProtocolOutputBatch *output, uint8_t *retain_request)
{
    AdrcProtocolGateway *gateway = (AdrcProtocolGateway *)context;
    AdrcProtocolCommand command;
    TextProtocolSpan field;
    uint8_t channel = 0U;
    uint32_t integer_value;
    if ((gateway == NULL) || (request == NULL) || (output == NULL) || (retain_request == NULL))
    { return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT; }
    *retain_request = 0U;
    if (request->positional_count != 0U) { return reject(output, request->request_id, "bad_argument"); }
    if ((text_protocol_request_path_equals(request, "adrc", NULL) != 0U) ||
        (text_protocol_request_path_equals(request, "adrc", "status") != 0U))
    {
        if (request->field_count != 0U) { return reject(output, request->request_id, "bad_argument"); }
        return format_output(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc status state=%u fault=%u motor=%u qualified=%u disabled=%u frozen=%u count=%lu overflow=%lu active_id=%lu last_id=%lu",
            (unsigned long)request->request_id, (unsigned int)gateway->snapshot.status.state,
            (unsigned int)gateway->snapshot.status.fault, (unsigned int)gateway->snapshot.status.axis_index + 1U,
            (unsigned int)gateway->snapshot.qualified, (unsigned int)gateway->snapshot.status.disabled_confirmed,
            (unsigned int)gateway->snapshot.trace_frozen, (unsigned long)gateway->snapshot.status.trace_count,
            (unsigned long)gateway->snapshot.status.trace_overflow_count,
            (unsigned long)gateway->active_run_request_id,
            (unsigned long)gateway->highest_admitted_request_id);
    }
    if (text_protocol_request_path_equals(request, "adrc", "trace") != 0U)
    {
        AdrcExperimentTrace record;
        if ((request->field_count != 1U) || (text_protocol_find_field(request, "index", &field) == 0U) ||
            (parse_uint(&field, &integer_value) == 0U)) { return reject(output, request->request_id, "bad_argument"); }
        if ((gateway->snapshot.status.disabled_confirmed == 0U) || (gateway->snapshot.trace_frozen == 0U) ||
            (gateway->snapshot.status.state == ADRC_STATE_RUNNING) ||
            (gateway->snapshot.status.state == ADRC_STATE_STOPPING) || (gateway->active_run_request_id != 0U) ||
            (gateway->trace_reader == NULL) ||
            (gateway->trace_reader(gateway->trace_context, integer_value, &record) == 0U))
        { return reject(output, request->request_id, "trace_unavailable"); }
        return format_output(output, PROTOCOL_ENGINE_STATUS_OK,
            "ok %lu adrc trace index=%lu t=%llu fb=%llu ref=%.6g pos=%.6g vel=%.6g raw=%.6g req=%.6g sent=%.6g z1=%.6g z2=%.6g state=%u fault=%u flags=%u seq=%lu motor=%u",
            (unsigned long)request->request_id, (unsigned long)integer_value,
            (unsigned long long)record.timestamp_us, (unsigned long long)record.feedback_us,
            (double)record.reference_rad_s, (double)record.position_rad, (double)record.velocity_rad_s,
            (double)record.torque_raw_nm, (double)record.torque_requested_nm, (double)record.torque_sent_previous_nm,
            (double)record.z1, (double)record.z2, (unsigned int)record.state, (unsigned int)record.fault,
            (unsigned int)record.flags, (unsigned long)record.sequence,
            (unsigned int)record.axis_index + 1U);
    }
    memset(&command, 0, sizeof(command));
    command.request_id = request->request_id;
    command.timestamp_us = timestamp_us;
    command.axis_index = gateway->snapshot.status.axis_index;
    if (text_protocol_request_path_equals(request, "adrc", "config") != 0U)
    {
        command.type = ADRC_PROTOCOL_CONFIG;
        if (parse_config(request, &command) == 0U) { return reject(output, request->request_id, "bad_config"); }
    }
    else if (text_protocol_request_path_equals(request, "adrc", "prepare") != 0U)
    {
        command.type = ADRC_PROTOCOL_PREPARE;
        if (request->field_count != 0U)
        {
            if ((request->field_count != 1U) || (text_protocol_find_field(request, "motor", &field) == 0U) ||
                (parse_uint(&field, &integer_value) == 0U) || (integer_value < 1U) || (integer_value > 7U))
            { return reject(output, request->request_id, "bad_argument"); }
            command.axis_index = (uint8_t)(integer_value - 1U);
        }
    }
    else
    {
        if (request->field_count != 0U) { return reject(output, request->request_id, "bad_argument"); }
        if (text_protocol_request_path_equals(request, "adrc", "run") != 0U) { command.type = ADRC_PROTOCOL_RUN; }
        else if (text_protocol_request_path_equals(request, "adrc", "stop") != 0U)
        { command.type = ADRC_PROTOCOL_STOP; channel = 1U; }
        else if (text_protocol_request_path_equals(request, "adrc", "heartbeat") != 0U)
        { command.type = ADRC_PROTOCOL_HEARTBEAT; channel = 2U; }
        else if (text_protocol_request_path_equals(request, "adrc", "clear") != 0U) { command.type = ADRC_PROTOCOL_CLEAR; }
        else { return reject(output, request->request_id, "unknown_command"); }
    }
    if (request->request_id == 0U) { return reject(output, 0U, "id_required"); }
    if (request->request_id <= gateway->highest_admitted_request_id)
    { return reject(output, request->request_id, "stale_request_id"); }
    if ((command.type == ADRC_PROTOCOL_RUN) &&
        ((gateway->snapshot.qualified == 0U) || (gateway->snapshot.status.state != ADRC_STATE_PREPARED) ||
         (gateway->snapshot.status.disabled_confirmed == 0U)))
    { return reject(output, request->request_id, "not_ready"); }
    if ((command.type == ADRC_PROTOCOL_CONFIG) &&
        ((gateway->snapshot.status.disabled_confirmed == 0U) ||
         ((gateway->snapshot.status.state != ADRC_STATE_DISABLED) &&
          (gateway->snapshot.status.state != ADRC_STATE_PREPARED))))
    { return reject(output, request->request_id, "not_disabled"); }
    if (gateway->channel_state[channel] != 0U) { return reject(output, request->request_id, "busy"); }
    if ((command.type == ADRC_PROTOCOL_STOP) && (gateway->channel_state[0] == 1U) &&
        (gateway->command.type == ADRC_PROTOCOL_RUN))
    {
        gateway->completed_result[0] = ADRC_RESULT_INVALID_STATE;
        gateway->completed_at_us[0] = timestamp_us;
        gateway->cancelled_by_stop[0] = 1U;
        gateway->channel_state[0] = 3U;
        gateway->command_pending = 0U;
    }
    gateway->cancelled_by_stop[channel] = 0U;
    *channel_command(gateway, channel) = command;
    gateway->channel_state[channel] = 1U;
    gateway->highest_admitted_request_id = request->request_id;
    if (channel == 0U) { gateway->command_pending = 1U; }
    else if (channel == 1U) { gateway->stop_pending = 1U; }
    else { gateway->heartbeat_pending = 1U; }
    if (command.type == ADRC_PROTOCOL_RUN) { gateway->active_run_request_id = request->request_id; }
    *retain_request = 1U;
    return format_output(output, PROTOCOL_ENGINE_STATUS_OK,
        "ok %lu adrc accepted=1", (unsigned long)request->request_id);
}

/** @brief Pops independent safety channels before ordinary work; occupied channels persist until DONE. */
uint8_t adrc_protocol_pop_command(AdrcProtocolGateway *gateway, AdrcProtocolCommand *command)
{
    static const uint8_t priority_order[] = { 1U, 2U, 0U };
    uint8_t priority_index;
    if ((gateway == NULL) || (command == NULL)) { return 0U; }
    for (priority_index = 0U; priority_index < 3U; ++priority_index)
    {
        uint8_t channel = priority_order[priority_index];
        if (gateway->channel_state[channel] == 1U)
        {
            *command = *channel_command(gateway, channel);
            gateway->channel_state[channel] = 2U;
            if (channel == 0U) { gateway->command_pending = 0U; }
            else if (channel == 1U) { gateway->stop_pending = 0U; }
            else { gateway->heartbeat_pending = 0U; }
            return 1U;
        }
    }
    return 0U;
}

/** @brief Retains a terminal in its reserved channel until protocol publication succeeds. */
uint8_t adrc_protocol_complete_command(AdrcProtocolGateway *gateway, uint32_t request_id,
    AdrcExperimentResult result, uint64_t timestamp_us)
{
    uint8_t channel;
    if ((gateway == NULL) || (request_id == 0U)) { return 0U; }
    for (channel = 0U; channel < 3U; ++channel)
    {
        if ((gateway->channel_state[channel] == 2U) &&
            (channel_command(gateway, channel)->request_id == request_id))
        {
            gateway->completed_result[channel] = result;
            gateway->completed_at_us[channel] = timestamp_us;
            gateway->channel_state[channel] = 3U;
            return 1U;
        }
    }
    return 0U;
}

/** @brief Atomically replaces accepted cache contents before freeing a completed mailbox. */
uint8_t adrc_protocol_pop_result_output(AdrcProtocolGateway *gateway, ProtocolEngine *engine,
    ProtocolOutputBatch *output)
{
    uint8_t channel;
    if ((gateway == NULL) || (engine == NULL) || (output == NULL)) { return 0U; }
    memset(output, 0, sizeof(*output));
    for (channel = 0U; channel < 3U; ++channel)
    {
        if (gateway->channel_state[channel] == 3U)
        {
            uint32_t request_id = channel_command(gateway, channel)->request_id;
            if (format_output(output, PROTOCOL_ENGINE_STATUS_OK,
                "done %lu adrc result=%u disabled=%u cancelled=%u", (unsigned long)request_id,
                (unsigned int)gateway->completed_result[channel],
                (unsigned int)gateway->snapshot.status.disabled_confirmed,
                (unsigned int)gateway->cancelled_by_stop[channel]) != PROTOCOL_ENGINE_STATUS_OK)
            { return 0U; }
            if (protocol_engine_complete_adrc_request(engine, request_id,
                gateway->completed_at_us[channel], output) == 0U)
            { memset(output, 0, sizeof(*output)); return 0U; }
            if (gateway->active_run_request_id == request_id) { gateway->active_run_request_id = 0U; }
            gateway->channel_state[channel] = 0U;
            return 1U;
        }
    }
    return 0U;
}
