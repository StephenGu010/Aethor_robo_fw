/**
 * @file protocol_engine.c
 * @brief Implements bounded sessions, replay protection, and core queries.
 */

#include "protocol_engine.h"

#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "build_info.h"
#include "arm_config.h"
#include "joint_motion.h"

#define PROTOCOL_ENGINE_RAD_TO_DEG (57.29577951308232F)
#define PROTOCOL_ENGINE_DEG_TO_RAD (0.017453292519943295F)
#define PROTOCOL_ENGINE_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define PROTOCOL_ENGINE_FLOAT_TOKEN_CAPACITY (32U)

/** @brief Selects one joint configuration vector for deterministic formatting. */
typedef enum
{
    PROTOCOL_CONFIG_Q_MIN = 0,
    PROTOCOL_CONFIG_Q_MAX,
    PROTOCOL_CONFIG_V_LIMIT,
    PROTOCOL_CONFIG_A_LIMIT
} ProtocolConfigVector;

/** @brief Clears an output batch before every processing attempt. */
static void protocol_engine_clear_output(ProtocolOutputBatch *output_batch)
{
    memset(output_batch, 0, sizeof(*output_batch));
}

/**
 * @brief Compares one parsed span with an exact null-terminated value.
 */
static uint8_t protocol_engine_span_equals(const AsciiProtocolSpan *span,
                                           const char *expected)
{
    size_t expected_length = strlen(expected);

    return (uint8_t)((span != NULL) &&
                     (span->length == expected_length) &&
                     (strncmp(span->data, expected, expected_length) == 0));
}

/**
 * @brief Parses one nonzero or zero uint32 field without signs or suffixes.
 */
static uint8_t protocol_engine_parse_u32(const AsciiProtocolSpan *span,
                                         uint32_t *value)
{
    uint32_t parsed_value = 0U;
    uint16_t character_index;

    if ((span == NULL) || (value == NULL) || (span->length == 0U))
    {
        return 0U;
    }
    for (character_index = 0U; character_index < span->length; ++character_index)
    {
        uint8_t digit;

        if ((span->data[character_index] < '0') ||
            (span->data[character_index] > '9'))
        {
            return 0U;
        }
        digit = (uint8_t)(span->data[character_index] - '0');
        if (parsed_value > ((UINT32_MAX - digit) / 10U))
        {
            return 0U;
        }
        parsed_value = (parsed_value * 10U) + digit;
    }
    *value = parsed_value;
    return 1U;
}

/** @brief Prevents compiler reordering across SPSC queue ownership edges. */
static void protocol_engine_compiler_barrier(void)
{
#if defined(__CC_ARM)
    __schedule_barrier();
#elif defined(__GNUC__) || defined(__clang__)
    __asm__ volatile ("" ::: "memory");
#else
    volatile uint32_t barrier_value = 0U;
    (void)barrier_value;
#endif
}

/** @brief Parses exactly seven finite comma-separated float values. */
static uint8_t protocol_engine_parse_joint_vector(
    const AsciiProtocolSpan *span,
    float values[ARM_JOINT_COUNT])
{
    uint16_t token_start = 0U;
    uint16_t character_index;
    uint8_t value_index = 0U;

    if ((span == NULL) || (values == NULL) || (span->length == 0U))
    {
        return 0U;
    }
    for (character_index = 0U; character_index <= span->length; ++character_index)
    {
        if ((character_index == span->length) ||
            (span->data[character_index] == ','))
        {
            char token[PROTOCOL_ENGINE_FLOAT_TOKEN_CAPACITY];
            char *parse_end;
            double parsed_value;
            uint16_t token_length = (uint16_t)(character_index - token_start);

            if ((value_index >= ARM_JOINT_COUNT) || (token_length == 0U) ||
                (token_length >= sizeof(token)))
            {
                return 0U;
            }
            memcpy(token, &span->data[token_start], token_length);
            token[token_length] = '\0';
            parse_end = NULL;
            parsed_value = strtod(token, &parse_end);
            if ((parse_end == token) || (*parse_end != '\0') ||
                (parsed_value != parsed_value) ||
                (parsed_value > FLT_MAX) || (parsed_value < -FLT_MAX))
            {
                return 0U;
            }
            values[value_index] = (float)parsed_value;
            ++value_index;
            token_start = (uint16_t)(character_index + 1U);
        }
    }
    return (uint8_t)(value_index == ARM_JOINT_COUNT);
}

/** @brief Parses one finite float field without accepting suffix characters. */
static uint8_t protocol_engine_parse_float(const AsciiProtocolSpan *span,
                                           float *value)
{
    char token[PROTOCOL_ENGINE_FLOAT_TOKEN_CAPACITY];
    char *parse_end;
    double parsed_value;

    if ((span == NULL) || (value == NULL) || (span->length == 0U) ||
        (span->length >= sizeof(token)))
    {
        return 0U;
    }
    memcpy(token, span->data, span->length);
    token[span->length] = '\0';
    parse_end = NULL;
    parsed_value = strtod(token, &parse_end);
    if ((parse_end == token) || (*parse_end != '\0') ||
        (parsed_value != parsed_value) ||
        (parsed_value > FLT_MAX) || (parsed_value < -FLT_MAX))
    {
        return 0U;
    }
    *value = (float)parsed_value;
    return 1U;
}

/** @brief Enqueues one normal business command into the bounded SPSC ring. */
static uint8_t protocol_engine_enqueue_command(ProtocolEngine *engine,
                                               const ProtocolCommand *command)
{
    uint8_t used_count = (uint8_t)(engine->command_write_sequence -
                                   engine->command_read_sequence);
    uint8_t slot_index;

    if (command->type == PROTOCOL_COMMAND_STOP)
    {
        if (engine->stop_write_sequence != engine->stop_read_sequence)
        {
            return 0U;
        }
        engine->stop_command = *command;
        protocol_engine_compiler_barrier();
        ++engine->stop_write_sequence;
        return 1U;
    }
    if (used_count >= PROTOCOL_ENGINE_COMMAND_CAPACITY)
    {
        return 0U;
    }
    slot_index = (uint8_t)(engine->command_write_sequence %
                           PROTOCOL_ENGINE_COMMAND_CAPACITY);
    engine->commands[slot_index] = *command;
    protocol_engine_compiler_barrier();
    ++engine->command_write_sequence;
    return 1U;
}

/**
 * @brief Encodes one response body into the next bounded output slot.
 */
static ProtocolEngineStatus protocol_engine_append_body(
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *body)
{
    ProtocolOutputMessage *message;
    size_t output_length = 0U;

    if (output_batch->count >= PROTOCOL_ENGINE_MAX_OUTPUT_COUNT)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    message = &output_batch->messages[output_batch->count];
    if (ascii_protocol_format_frame(body,
                                    strlen(body),
                                    message->data,
                                    sizeof(message->data),
                                    &output_length) != ASCII_PROTOCOL_STATUS_OK)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    message->length = (uint16_t)output_length;
    message->priority = priority;
    ++output_batch->count;
    return PROTOCOL_ENGINE_STATUS_OK;
}

/**
 * @brief Formats and appends one bounded response body.
 */
static ProtocolEngineStatus protocol_engine_append_format(
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *format,
    ...)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    int written_length;
    va_list arguments;

    va_start(arguments, format);
    written_length = vsnprintf(body, sizeof(body), format, arguments);
    va_end(arguments);
    if ((written_length < 0) || ((size_t)written_length >= sizeof(body)))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_body(output_batch, priority, body);
}

/**
 * @brief Appends formatted text to a bounded response body.
 */
static uint8_t protocol_engine_append_text(char *body,
                                           size_t body_capacity,
                                           size_t *body_length,
                                           const char *format,
                                           ...)
{
    int written_length;
    va_list arguments;

    if (*body_length >= body_capacity)
    {
        return 0U;
    }
    va_start(arguments, format);
    written_length = vsnprintf(&body[*body_length],
                               body_capacity - *body_length,
                               format,
                               arguments);
    va_end(arguments);
    if ((written_length < 0) ||
        ((size_t)written_length >= (body_capacity - *body_length)))
    {
        return 0U;
    }
    *body_length += (size_t)written_length;
    return 1U;
}

/**
 * @brief Returns one configured joint limit converted to the public degree unit.
 */
static float protocol_engine_config_value_deg(const JointConfig *joint,
                                              ProtocolConfigVector vector)
{
    switch (vector)
    {
        case PROTOCOL_CONFIG_Q_MIN:
            return joint->soft_limit_min_rad * PROTOCOL_ENGINE_RAD_TO_DEG;
        case PROTOCOL_CONFIG_Q_MAX:
            return joint->soft_limit_max_rad * PROTOCOL_ENGINE_RAD_TO_DEG;
        case PROTOCOL_CONFIG_V_LIMIT:
            return joint->max_velocity_rad_s * PROTOCOL_ENGINE_RAD_TO_DEG;
        case PROTOCOL_CONFIG_A_LIMIT:
        default:
            return joint->max_acceleration_rad_s2 * PROTOCOL_ENGINE_RAD_TO_DEG;
    }
}

/**
 * @brief Appends one seven-value configuration vector with fixed precision.
 */
static uint8_t protocol_engine_append_config_vector(
    char *body,
    size_t body_capacity,
    size_t *body_length,
    const ArmConfig *configuration,
    ProtocolConfigVector vector,
    uint8_t decimal_places)
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const char *separator = (joint_index == 0U) ? "" : ",";
        float value = protocol_engine_config_value_deg(
            &configuration->joints[joint_index],
            vector);

        if (decimal_places == 6U)
        {
            if (protocol_engine_append_text(body,
                                            body_capacity,
                                            body_length,
                                            "%s%.6f",
                                            separator,
                                            value) == 0U)
            {
                return 0U;
            }
        }
        else if (protocol_engine_append_text(body,
                                             body_capacity,
                                             body_length,
                                             "%s%.3f",
                                             separator,
                                             value) == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Appends one public seven-axis value vector with three decimals. */
static uint8_t protocol_engine_append_joint_vector(
    char *body,
    size_t body_capacity,
    size_t *body_length,
    const float values[ARM_JOINT_COUNT])
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(body,
                                        body_capacity,
                                        body_length,
                                        "%s%.3f",
                                        (joint_index == 0U) ? "" : ",",
                                        values[joint_index]) == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Returns the stable protocol text for one controller state. */
static const char *protocol_engine_arm_state_text(ArmState state)
{
    switch (state)
    {
        case ARM_STATE_BOOT:
            return "BOOT";
        case ARM_STATE_SELF_TEST:
            return "SELF_TEST";
        case ARM_STATE_UNALIGNED:
            return "UNALIGNED";
        case ARM_STATE_DISABLED:
            return "DISABLED";
        case ARM_STATE_ENABLING:
            return "ENABLING";
        case ARM_STATE_READY:
            return "READY";
        case ARM_STATE_MOVING:
            return "MOVING";
        case ARM_STATE_STOPPING:
            return "STOPPING";
        case ARM_STATE_FAULT:
        default:
            return "FAULT";
    }
}

/** @brief Returns the stable protocol text for one controller fault. */
static const char *protocol_engine_arm_fault_text(ArmFault fault)
{
    switch (fault)
    {
        case ARM_FAULT_NONE:
            return "NONE";
        case ARM_FAULT_CONFIG_INVALID:
            return "CONFIG_INVALID";
        case ARM_FAULT_CONFIG_INCOMPLETE:
            return "CONFIG_INCOMPLETE";
        case ARM_FAULT_LINK_TIMEOUT:
            return "LINK_TIMEOUT";
        default:
            return "CONFIG_INCOMPLETE";
    }
}

/** @brief Returns stable public text for one confirmed control mode. */
static const char *protocol_engine_control_mode_text(ArmControlMode mode)
{
    switch (mode)
    {
        case ARM_CONTROL_MODE_POSITION_VELOCITY:
            return "POS_VEL";
        case ARM_CONTROL_MODE_MIT:
            return "MIT";
        case ARM_CONTROL_MODE_UNKNOWN:
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Validates the SET_STREAM comma-separated capability list.
 */
static uint8_t protocol_engine_stream_fields_are_valid(
    const AsciiProtocolSpan *fields)
{
    uint16_t token_start = 0U;
    uint16_t character_index;

    if ((fields == NULL) || (fields->length == 0U) ||
        (fields->length >= sizeof(((ProtocolEngine *)0)->stream_fields)))
    {
        return 0U;
    }
    for (character_index = 0U; character_index <= fields->length; ++character_index)
    {
        if ((character_index == fields->length) ||
            (fields->data[character_index] == ','))
        {
            AsciiProtocolSpan token;

            token.data = &fields->data[token_start];
            token.length = (uint16_t)(character_index - token_start);
            if ((protocol_engine_span_equals(&token, "jpos") == 0U) &&
                (protocol_engine_span_equals(&token, "jvel") == 0U) &&
                (protocol_engine_span_equals(&token, "jtor") == 0U) &&
                (protocol_engine_span_equals(&token, "state") == 0U) &&
                (protocol_engine_span_equals(&token, "motor") == 0U) &&
                (protocol_engine_span_equals(&token, "diag") == 0U))
            {
                return 0U;
            }
            token_start = (uint16_t)(character_index + 1U);
        }
    }
    return 1U;
}

/** @brief Checks whether one exact token is present in a stored CSV list. */
static uint8_t protocol_engine_csv_contains(const char *csv,
                                            const char *expected)
{
    const char *token_start = csv;
    size_t expected_length = strlen(expected);

    while ((token_start != NULL) && (*token_start != '\0'))
    {
        const char *token_end = strchr(token_start, ',');
        size_t token_length = (token_end == NULL)
                                  ? strlen(token_start)
                                  : (size_t)(token_end - token_start);

        if ((token_length == expected_length) &&
            (strncmp(token_start, expected, expected_length) == 0))
        {
            return 1U;
        }
        token_start = (token_end == NULL) ? NULL : token_end + 1;
    }
    return 0U;
}

/**
 * @brief Finds a retained request ID while expiring records older than 60 seconds.
 */
static ProtocolRecentResult *protocol_engine_find_recent(
    ProtocolEngine *engine,
    uint32_t request_id,
    uint64_t timestamp_us)
{
    uint8_t result_index;

    for (result_index = 0U;
         result_index < PROTOCOL_ENGINE_RECENT_RESULT_CAPACITY;
         ++result_index)
    {
        ProtocolRecentResult *result = &engine->recent_results[result_index];

        if ((result->valid != 0U) &&
            (timestamp_us >= result->completed_at_us) &&
            ((timestamp_us - result->completed_at_us) >
             PROTOCOL_ENGINE_RECENT_RESULT_RETENTION_US))
        {
            result->valid = 0U;
        }
        if ((result->valid != 0U) && (result->request_id == request_id))
        {
            return result;
        }
    }
    return NULL;
}

/**
 * @brief Stores the single terminal response produced by a synchronous request.
 */
static void protocol_engine_store_recent(ProtocolEngine *engine,
                                         const AsciiProtocolRequest *request,
                                         uint32_t body_hash,
                                         uint64_t timestamp_us,
                                         const ProtocolOutputBatch *output_batch)
{
    ProtocolRecentResult *result;

    if (output_batch->count != 1U)
    {
        return;
    }
    result = &engine->recent_results[engine->recent_write_index];
    memset(result, 0, sizeof(*result));
    result->request_id = request->request_id;
    result->body_hash = body_hash;
    result->completed_at_us = timestamp_us;
    result->response_length = output_batch->messages[0].length;
    result->priority = output_batch->messages[0].priority;
    memcpy(result->response,
           output_batch->messages[0].data,
           (size_t)result->response_length + 1U);
    result->valid = 1U;
    engine->recent_write_index = (uint8_t)(
        (engine->recent_write_index + 1U) % PROTOCOL_ENGINE_RECENT_RESULT_CAPACITY);
}

/**
 * @brief Replays one retained response without re-executing its request.
 */
static void protocol_engine_replay(const ProtocolRecentResult *result,
                                   ProtocolOutputBatch *output_batch)
{
    ProtocolOutputMessage *message = &output_batch->messages[0];

    memcpy(message->data,
           result->response,
           (size_t)result->response_length + 1U);
    message->length = result->response_length;
    message->priority = result->priority;
    output_batch->count = 1U;
}

/**
 * @brief Creates a nonzero session identifier for a new HELLO transaction.
 */
static uint32_t protocol_engine_create_session(ProtocolEngine *engine,
                                               uint32_t request_id,
                                               uint64_t timestamp_us)
{
    uint32_t session_id;

    ++engine->next_session_nonce;
    session_id = engine->boot_id ^ request_id ^
                 (uint32_t)timestamp_us ^
                 (uint32_t)(timestamp_us >> 32U) ^
                 (engine->next_session_nonce * 0x9E3779B9UL);
    if (session_id == 0U)
    {
        session_id = engine->next_session_nonce;
    }
    return session_id;
}

/**
 * @brief Processes HELLO and resets all connection-scoped state.
 */
static ProtocolEngineStatus protocol_engine_handle_hello(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan client;
    AsciiProtocolSpan protocol;
    const BuildInfo *build_info = build_info_get();

    if ((ascii_protocol_find_field(request, "client", &client) == 0U) ||
        (client.length == 0U) ||
        (ascii_protocol_find_field(request, "protocol", &protocol) == 0U) ||
        (protocol_engine_span_equals(&protocol, "1") == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=HELLO",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    memset(engine->recent_results, 0, sizeof(engine->recent_results));
    engine->recent_write_index = 0U;
    engine->command_read_sequence = engine->command_write_sequence;
    engine->stop_read_sequence = engine->stop_write_sequence;
    engine->result_read_sequence = engine->result_write_sequence;
    engine->session_id = protocol_engine_create_session(engine,
                                                        request->request_id,
                                                        timestamp_us);
    engine->session_active = 1U;
    engine->watchdog_timeout_reported = 0U;
    engine->last_valid_request_at_us = timestamp_us;
    engine->next_telemetry_due_us = timestamp_us;
    engine->telemetry_sequence = 0U;
    engine->event_sequence = 0U;
    engine->last_published_state_valid = 0U;
    engine->stream_rate_hz = 50U;
    (void)strcpy(engine->stream_fields, "jpos,jvel,state,motor");
    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "RSP %lu ok product=%s controller=%s arm=%s session=%lu boot_id=%lu dof=7 protocol=%s fw=%s modes=POS_VEL,MIT stream_max_hz=100 link_watchdog_ms=1000 link_timeout_action=STOP_DISABLE",
        (unsigned long)request->request_id,
        build_info->product_name,
        build_info->controller_id,
        build_info->arm_id,
        (unsigned long)engine->session_id,
        (unsigned long)engine->boot_id,
        build_info->protocol_version,
        build_info->firmware_version);
}

/**
 * @brief Processes a session-bound HEARTBEAT request.
 */
static ProtocolEngineStatus protocol_engine_handle_heartbeat(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan session_span;
    uint32_t received_session;

    if ((ascii_protocol_find_field(request, "session", &session_span) == 0U) ||
        (protocol_engine_parse_u32(&session_span, &received_session) == 0U) ||
        (engine->session_active == 0U) ||
        (received_session != engine->session_id))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INVALID_SESSION",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_SESSION_MISMATCH;
    }

    engine->last_valid_request_at_us = timestamp_us;
    engine->watchdog_timeout_reported = 0U;
    return protocol_engine_append_format(output_batch,
                                         PROTOCOL_OUTPUT_QUERY,
                                         "RSP %lu ok state=DISABLED boot_id=%lu",
                                         (unsigned long)request->request_id,
                                         (unsigned long)engine->boot_id);
}

/**
 * @brief Processes the immutable firmware identity query.
 */
static ProtocolEngineStatus protocol_engine_handle_get_info(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    const BuildInfo *build_info = build_info_get();

    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "RSP %lu ok product=%s controller=%s arm=%s dof=7 fw=%s build=%s protocol=%s can_bitrate=1000000 transport=USB_CDC boot_id=%lu",
        (unsigned long)request->request_id,
        build_info->product_name,
        build_info->controller_id,
        build_info->arm_id,
        build_info->firmware_version,
        build_info->git_description,
        build_info->protocol_version,
        (unsigned long)engine->boot_id);
}

/** @brief Returns the current controller state without touching the CAN bus. */
static ProtocolEngineStatus protocol_engine_handle_get_state(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    uint32_t feedback_age_max_ms = 0U;
    uint8_t joint_index;

    if (engine->query_context_valid == 0U)
    {
        return protocol_engine_append_format(output_batch,
                                             PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                             "ERR %lu INTERNAL_ERROR context=state",
                                             (unsigned long)request->request_id);
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback =
            &engine->query_context.motors.joints[joint_index];
        if ((feedback->timestamp_us != 0U) &&
            (engine->query_context.timestamp_us >= feedback->timestamp_us))
        {
            uint64_t age_ms = (engine->query_context.timestamp_us -
                               feedback->timestamp_us) / 1000U;
            if (age_ms > feedback_age_max_ms)
            {
                feedback_age_max_ms = (age_ms > UINT32_MAX)
                                          ? UINT32_MAX
                                          : (uint32_t)age_ms;
            }
        }
    }
    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "RSP %lu ok state=%s aligned=%u enabled=%u moving=%u mode=%s active_request=0 fault=%s feedback_age_max_ms=%lu",
        (unsigned long)request->request_id,
        protocol_engine_arm_state_text(engine->query_context.arm.state),
        engine->query_context.arm.aligned,
        engine->query_context.arm.enabled,
        engine->query_context.arm.moving,
        protocol_engine_control_mode_text(engine->query_context.arm.control_mode),
        protocol_engine_arm_fault_text(engine->query_context.arm.fault),
        (unsigned long)feedback_age_max_ms);
}

/** @brief Returns the latest aligned joint snapshot without producing CAN traffic. */
static ProtocolEngineStatus protocol_engine_handle_get_jpos(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t body_length = 0U;
    uint8_t joint_index;

    if (engine->query_context_valid == 0U)
    {
        return protocol_engine_append_format(output_batch,
                                             PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                             "ERR %lu INTERNAL_ERROR context=jpos",
                                             (unsigned long)request->request_id);
    }
    if (protocol_engine_append_text(
            body,
            sizeof(body),
            &body_length,
            "RSP %lu ok t_us=%lu q_deg=",
            (unsigned long)request->request_id,
            (unsigned long)engine->query_context.joints.published_at_us) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "%s%.3f",
                (joint_index == 0U) ? "" : ",",
                engine->query_context.joints.position_deg[joint_index]) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body, sizeof(body), &body_length, " valid=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t valid = (uint8_t)((engine->query_context.joints.valid_joint_mask &
                                   (uint8_t)(1U << joint_index)) != 0U);
        if (protocol_engine_append_text(body,
                                        sizeof(body),
                                        &body_length,
                                        "%s%u",
                                        (joint_index == 0U) ? "" : ",",
                                        valid) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    " aligned=%u",
                                    engine->query_context.joints.aligned) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_body(output_batch, PROTOCOL_OUTPUT_QUERY, body);
}

/** @brief Returns motor validity, raw driver state, temperatures, and age. */
static ProtocolEngineStatus protocol_engine_handle_get_motors(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t body_length = 0U;
    uint8_t joint_index;

    if (engine->query_context_valid == 0U)
    {
        return protocol_engine_append_format(output_batch,
                                             PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                             "ERR %lu INTERNAL_ERROR context=motors",
                                             (unsigned long)request->request_id);
    }
    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    "RSP %lu ok status=",
                                    (unsigned long)request->request_id) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "%s%u",
                (joint_index == 0U) ? "" : ",",
                (engine->query_context.motors.valid_joint_mask &
                 (uint8_t)(1U << joint_index)) != 0U) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body, sizeof(body), &body_length, " fault=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback =
            &engine->query_context.motors.joints[joint_index];
        if (protocol_engine_append_text(body,
                                        sizeof(body),
                                        &body_length,
                                        "%s%lu",
                                        (joint_index == 0U) ? "" : ",",
                                        (unsigned long)feedback->fault_flags) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body, sizeof(body), &body_length, " mos_c=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "%s%d",
                (joint_index == 0U) ? "" : ",",
                (int)engine->query_context.motors.joints[joint_index].mos_temperature_c) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body, sizeof(body), &body_length, " rotor_c=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "%s%d",
                (joint_index == 0U) ? "" : ",",
                (int)engine->query_context.motors.joints[joint_index].rotor_temperature_c) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(body, sizeof(body), &body_length, " age_ms=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback =
            &engine->query_context.motors.joints[joint_index];
        uint32_t age_ms = UINT32_MAX;

        if ((feedback->timestamp_us != 0U) &&
            (engine->query_context.timestamp_us >= feedback->timestamp_us))
        {
            uint64_t age_value = (engine->query_context.timestamp_us -
                                  feedback->timestamp_us) / 1000U;
            age_ms = (age_value > UINT32_MAX) ? UINT32_MAX : (uint32_t)age_value;
        }
        if (protocol_engine_append_text(body,
                                        sizeof(body),
                                        &body_length,
                                        "%s%lu",
                                        (joint_index == 0U) ? "" : ",",
                                        (unsigned long)age_ms) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    return protocol_engine_append_body(output_batch, PROTOCOL_OUTPUT_QUERY, body);
}

/** @brief Returns the current bounded software diagnostic counters. */
static ProtocolEngineStatus protocol_engine_handle_get_diag(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    const DiagnosticCounters *diagnostics = &engine->query_context.diagnostics;

    if (engine->query_context_valid == 0U)
    {
        return protocol_engine_append_format(output_batch,
                                             PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                             "ERR %lu INTERNAL_ERROR context=diag",
                                             (unsigned long)request->request_id);
    }
    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "RSP %lu ok control_hz=250 service_cycles=%lu can_rx=%lu can_tx=%lu config_fail=%lu queue_hwm=%lu stack_min_words=%lu",
        (unsigned long)request->request_id,
        (unsigned long)diagnostics->service_cycles,
        (unsigned long)diagnostics->can_rx_frames,
        (unsigned long)diagnostics->can_tx_frames,
        (unsigned long)diagnostics->config_validation_failures,
        (unsigned long)diagnostics->queue_high_watermark,
        (unsigned long)diagnostics->minimum_stack_words);
}

/** @brief Returns the explicit uncommissioned configuration and deterministic hash. */
static ProtocolEngineStatus protocol_engine_handle_get_config(
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    const ArmConfig *configuration = arm_config_get_production();
    ArmConfigValidation validation;
    char canonical_map[PROTOCOL_MAX_LINE_LENGTH + 1U];
    char response_body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t canonical_length = 0U;
    size_t response_length = 0U;
    uint32_t map_hash;
    uint8_t joint_index;
    uint8_t enable_ready = arm_config_is_enable_ready(configuration, &validation);
    static const char *canonical_keys[4] = {
        ";q_min_deg=", ";q_max_deg=", ";v_limit_deg_s=", ";a_limit_deg_s2="
    };
    static const char *response_keys[4] = {
        " q_min_deg=", " q_max_deg=", " v_limit_deg_s=", " a_limit_deg_s2="
    };
    uint8_t vector_index;

    if (protocol_engine_append_text(canonical_map,
                                    sizeof(canonical_map),
                                    &canonical_length,
                                    "dof=7;direction=") == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                canonical_map,
                sizeof(canonical_map),
                &canonical_length,
                "%s%d",
                (joint_index == 0U) ? "" : ",",
                (int)configuration->joints[joint_index].direction) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    for (vector_index = 0U; vector_index < 4U; ++vector_index)
    {
        if ((protocol_engine_append_text(canonical_map,
                                         sizeof(canonical_map),
                                         &canonical_length,
                                         "%s",
                                         canonical_keys[vector_index]) == 0U) ||
            (protocol_engine_append_config_vector(
                 canonical_map,
                 sizeof(canonical_map),
                 &canonical_length,
                 configuration,
                 (ProtocolConfigVector)vector_index,
                 6U) == 0U))
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    map_hash = ascii_protocol_crc32_iso_hdlc(
        (const uint8_t *)canonical_map,
        canonical_length);

    if (protocol_engine_append_text(
            response_body,
            sizeof(response_body),
            &response_length,
            "RSP %lu ok config_rev=2026-08-12.1 verified=%u direction=",
            (unsigned long)request->request_id,
            enable_ready) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                response_body,
                sizeof(response_body),
                &response_length,
                "%s%d",
                (joint_index == 0U) ? "" : ",",
                (int)configuration->joints[joint_index].direction) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    for (vector_index = 0U; vector_index < 4U; ++vector_index)
    {
        if ((protocol_engine_append_text(response_body,
                                         sizeof(response_body),
                                         &response_length,
                                         "%s",
                                         response_keys[vector_index]) == 0U) ||
            (protocol_engine_append_config_vector(
                 response_body,
                 sizeof(response_body),
                 &response_length,
                 configuration,
                 (ProtocolConfigVector)vector_index,
                 3U) == 0U))
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
    }
    if (protocol_engine_append_text(response_body,
                                    sizeof(response_body),
                                    &response_length,
                                    " map_hash=%08lX",
                                    (unsigned long)map_hash) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_body(output_batch,
                                       PROTOCOL_OUTPUT_QUERY,
                                       response_body);
}

/** @brief Validates and applies the session-scoped telemetry configuration. */
static ProtocolEngineStatus protocol_engine_handle_set_stream(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan rate_span;
    AsciiProtocolSpan fields_span;
    uint32_t rate_hz;

    if ((ascii_protocol_find_field(request, "rate_hz", &rate_span) == 0U) ||
        (ascii_protocol_find_field(request, "fields", &fields_span) == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu MISSING_FIELD",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((protocol_engine_parse_u32(&rate_span, &rate_hz) == 0U) ||
        (rate_hz > 100U) ||
        (protocol_engine_stream_fields_are_valid(&fields_span) == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=SET_STREAM",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    engine->stream_rate_hz = (uint8_t)rate_hz;
    memcpy(engine->stream_fields, fields_span.data, fields_span.length);
    engine->stream_fields[fields_span.length] = '\0';
    engine->next_telemetry_due_us = 0U;
    return protocol_engine_append_format(output_batch,
                                         PROTOCOL_OUTPUT_QUERY,
                                         "RSP %lu ok rate_hz=%lu fields=%s",
                                         (unsigned long)request->request_id,
                                         (unsigned long)rate_hz,
                                         engine->stream_fields);
}

/** @brief Validates and queues one boot-volatile reference alignment command. */
static ProtocolEngineStatus protocol_engine_handle_align_reference(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan reference_span;
    ProtocolCommand command;
    ArmState state;

    if (ascii_protocol_find_field(request, "q_ref_deg", &reference_span) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu MISSING_FIELD field=q_ref_deg",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    memset(&command, 0, sizeof(command));
    if (protocol_engine_parse_joint_vector(&reference_span, command.values) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=q_ref_deg",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INTERNAL_ERROR context=reference",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    state = engine->query_context.arm.state;
    if (((state != ARM_STATE_UNALIGNED) && (state != ARM_STATE_DISABLED)) ||
        (engine->query_context.arm.enabled != 0U) ||
        (engine->query_context.arm.moving != 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INVALID_STATE state=%s",
                                            (unsigned long)request->request_id,
                                            protocol_engine_arm_state_text(state));
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context.motors.valid_joint_mask !=
        PROTOCOL_ENGINE_ALL_JOINTS_MASK)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu FEEDBACK_STALE valid_mask=%u",
                                            (unsigned long)request->request_id,
                                            engine->query_context.motors.valid_joint_mask);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    command.type = PROTOCOL_COMMAND_ALIGN_REFERENCE;
    command.request_id = request->request_id;
    command.session_id = engine->session_id;
    command.accepted_at_us = timestamp_us;
    command.motor_mask = PROTOCOL_ENGINE_ALL_JOINTS_MASK;
    if (protocol_engine_enqueue_command(engine, &command) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BUSY queue=command",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    return protocol_engine_append_format(output_batch,
                                         PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                         "ACK %lu accepted",
                                         (unsigned long)request->request_id);
}

/** @brief Validates and queues SET_MODE, ENABLE, STOP, DISABLE, or CLEAR_FAULT. */
static ProtocolEngineStatus protocol_engine_handle_lifecycle_action(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolCommandType command_type,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    ProtocolCommand command;
    ArmState state;

    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INTERNAL_ERROR context=state",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    memset(&command, 0, sizeof(command));
    command.type = command_type;
    command.request_id = request->request_id;
    command.session_id = engine->session_id;
    command.accepted_at_us = timestamp_us;
    command.motor_mask = PROTOCOL_ENGINE_ALL_JOINTS_MASK;
    state = engine->query_context.arm.state;

    if (command_type == PROTOCOL_COMMAND_SET_MODE)
    {
        AsciiProtocolSpan mode_span;

        if (ascii_protocol_find_field(request, "mode", &mode_span) == 0U)
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu MISSING_FIELD field=mode",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if (protocol_engine_span_equals(&mode_span, "POS_VEL") != 0U)
        {
            command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        }
        else if (protocol_engine_span_equals(&mode_span, "MIT") != 0U)
        {
            command.control_mode = ARM_CONTROL_MODE_MIT;
        }
        else
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=mode",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if ((state != ARM_STATE_DISABLED) ||
            (engine->query_context.arm.enabled != 0U) ||
            (engine->query_context.arm.moving != 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu INVALID_STATE state=%s",
                                                (unsigned long)request->request_id,
                                                protocol_engine_arm_state_text(state));
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
    }
    else if (command_type == PROTOCOL_COMMAND_ENABLE)
    {
        if ((state != ARM_STATE_DISABLED) ||
            (engine->query_context.arm.aligned == 0U) ||
            (engine->query_context.arm.control_mode == ARM_CONTROL_MODE_UNKNOWN) ||
            (engine->query_context.arm.fault != ARM_FAULT_NONE) ||
            (engine->query_context.motors.valid_joint_mask !=
             PROTOCOL_ENGINE_ALL_JOINTS_MASK))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu NOT_READY state=%s",
                                                (unsigned long)request->request_id,
                                                protocol_engine_arm_state_text(state));
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        command.control_mode = engine->query_context.arm.control_mode;
    }
    else if (command_type == PROTOCOL_COMMAND_STOP)
    {
        AsciiProtocolSpan behavior_span;

        if ((ascii_protocol_find_field(request, "behavior", &behavior_span) == 0U) ||
            (protocol_engine_span_equals(&behavior_span, "controlled") == 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=behavior",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if ((state != ARM_STATE_READY) && (state != ARM_STATE_MOVING) &&
            (state != ARM_STATE_STOPPING) && (state != ARM_STATE_FAULT))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu INVALID_STATE state=%s",
                                                (unsigned long)request->request_id,
                                                protocol_engine_arm_state_text(state));
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
    }
    else if (command_type == PROTOCOL_COMMAND_CLEAR_FAULT)
    {
        AsciiProtocolSpan scope_span;
        AsciiProtocolSpan joint_span;
        uint8_t has_scope = ascii_protocol_find_field(request,
                                                      "scope",
                                                      &scope_span);
        uint8_t has_joint = ascii_protocol_find_field(request,
                                                      "joint",
                                                      &joint_span);

        if ((has_scope != 0U) && (has_joint != 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=scope",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if (has_scope != 0U)
        {
            if (protocol_engine_span_equals(&scope_span, "all") == 0U)
            {
                (void)protocol_engine_append_format(output_batch,
                                                    PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                    "ERR %lu BAD_VALUE field=scope",
                                                    (unsigned long)request->request_id);
                return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
            }
        }
        else if (has_joint != 0U)
        {
            uint32_t joint_number;

            if ((protocol_engine_parse_u32(&joint_span, &joint_number) == 0U) ||
                (joint_number < 1U) || (joint_number > ARM_JOINT_COUNT))
            {
                (void)protocol_engine_append_format(output_batch,
                                                    PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                    "ERR %lu BAD_VALUE field=joint",
                                                    (unsigned long)request->request_id);
                return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
            }
            command.motor_mask = (uint8_t)(1U << (joint_number - 1U));
            command.scope_joint = (uint8_t)joint_number;
        }
        else
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu MISSING_FIELD field=scope",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if (((state != ARM_STATE_FAULT) && (state != ARM_STATE_DISABLED)) ||
            (engine->query_context.arm.enabled != 0U) ||
            (engine->query_context.arm.moving != 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu INVALID_STATE state=%s",
                                                (unsigned long)request->request_id,
                                                protocol_engine_arm_state_text(state));
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
    }

    if (protocol_engine_enqueue_command(engine, &command) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BUSY queue=command",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    return protocol_engine_append_format(output_batch,
                                         PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                         "ACK %lu accepted",
                                         (unsigned long)request->request_id);
}

/** @brief Validates and plans one all-axis motion before queue admission. */
static ProtocolEngineStatus protocol_engine_handle_move_joints(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan target_span;
    AsciiProtocolSpan speed_span;
    AsciiProtocolSpan mode_span;
    ProtocolCommand command;
    JointMotionPlan motion_plan;
    JointMotionMode motion_mode;
    float start_position_rad[ARM_JOINT_COUNT];
    float target_position_rad[ARM_JOINT_COUNT];
    float speed_ratio = 0.20F;
    uint8_t joint_index;

    if ((engine->configuration == NULL) || (engine->query_context_valid == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INTERNAL_ERROR context=motion",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context.arm.state == ARM_STATE_MOVING)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BUSY state=MOVING",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->active_motion_request_id != 0U)
    {
        (void)protocol_engine_append_format(
            output_batch,
            PROTOCOL_OUTPUT_HIGH_PRIORITY,
            "ERR %lu BUSY motion_id=%lu",
            (unsigned long)request->request_id,
            (unsigned long)engine->active_motion_request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((engine->query_context.arm.state != ARM_STATE_READY) ||
        (engine->query_context.arm.aligned == 0U) ||
        (engine->query_context.arm.enabled == 0U) ||
        (engine->query_context.arm.moving != 0U) ||
        (engine->query_context.arm.fault != ARM_FAULT_NONE) ||
        (engine->query_context.joints.aligned == 0U) ||
        (engine->query_context.joints.valid_joint_mask !=
         PROTOCOL_ENGINE_ALL_JOINTS_MASK) ||
        (engine->query_context.motors.valid_joint_mask !=
         PROTOCOL_ENGINE_ALL_JOINTS_MASK))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu NOT_READY state=%s",
                                            (unsigned long)request->request_id,
                                            protocol_engine_arm_state_text(
                                                engine->query_context.arm.state));
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (engine->query_context.motors.joints[joint_index].fault_flags != 0U)
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu NOT_READY joint=%u fault=%lu",
                                                (unsigned long)request->request_id,
                                                (unsigned int)(joint_index + 1U),
                                                (unsigned long)engine->query_context.motors
                                                    .joints[joint_index].fault_flags);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
    }
    if (ascii_protocol_find_field(request, "q_deg", &target_span) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu MISSING_FIELD field=q_deg",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    memset(&command, 0, sizeof(command));
    if (protocol_engine_parse_joint_vector(&target_span, command.values) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=q_deg",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((ascii_protocol_find_field(request, "speed", &speed_span) != 0U) &&
        (protocol_engine_parse_float(&speed_span, &speed_ratio) == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=speed",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    command.control_mode = engine->query_context.arm.control_mode;
    if (ascii_protocol_find_field(request, "mode", &mode_span) != 0U)
    {
        ArmControlMode requested_mode;

        if (protocol_engine_span_equals(&mode_span, "POS_VEL") != 0U)
        {
            requested_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
        }
        else if (protocol_engine_span_equals(&mode_span, "MIT") != 0U)
        {
            requested_mode = ARM_CONTROL_MODE_MIT;
        }
        else
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=mode",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if (requested_mode != command.control_mode)
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu MODE_MISMATCH",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
    }
    motion_mode = (command.control_mode == ARM_CONTROL_MODE_MIT)
                      ? JOINT_MOTION_MODE_MIT
                      : JOINT_MOTION_MODE_POSITION_VELOCITY;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        start_position_rad[joint_index] =
            engine->query_context.joints.position_deg[joint_index] *
            PROTOCOL_ENGINE_DEG_TO_RAD;
        target_position_rad[joint_index] =
            command.values[joint_index] * PROTOCOL_ENGINE_DEG_TO_RAD;
    }
    if (joint_motion_plan(engine->configuration,
                          start_position_rad,
                          target_position_rad,
                          speed_ratio,
                          motion_mode,
                          timestamp_us,
                          &motion_plan) != JOINT_MOTION_STATUS_OK)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=motion",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    command.type = PROTOCOL_COMMAND_MOVE_JOINTS;
    command.request_id = request->request_id;
    command.session_id = engine->session_id;
    command.accepted_at_us = timestamp_us;
    command.planned_duration_us = motion_plan.duration_us;
    command.speeds[0] = speed_ratio;
    command.motor_mask = PROTOCOL_ENGINE_ALL_JOINTS_MASK;
    if (protocol_engine_enqueue_command(engine, &command) == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BUSY queue=command",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    engine->active_motion_request_id = request->request_id;
    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_HIGH_PRIORITY,
        "ACK %lu accepted motion_id=%lu duration_ms=%lu mode=%s",
        (unsigned long)request->request_id,
        (unsigned long)request->request_id,
        (unsigned long)((motion_plan.duration_us + 999ULL) / 1000ULL),
        (command.control_mode == ARM_CONTROL_MODE_MIT) ? "MIT" : "POS_VEL");
}

/**
 * @brief Initializes one protocol engine for the current firmware boot.
 */
void protocol_engine_init(ProtocolEngine *engine, uint32_t boot_id)
{
    if (engine == NULL)
    {
        return;
    }
    memset(engine, 0, sizeof(*engine));
    engine->configuration = arm_config_get_production();
    engine->boot_id = (boot_id == 0U) ? 1U : boot_id;
    engine->stream_rate_hz = 50U;
    (void)strcpy(engine->stream_fields, "jpos,jvel,state,motor");
}

/**
 * @brief Selects the immutable configuration used for atomic motion admission.
 */
void protocol_engine_set_configuration(
    ProtocolEngine *engine,
    const ArmConfig *configuration)
{
    ArmConfigValidation validation;

    if ((engine == NULL) ||
        !arm_config_validate_schema(configuration, &validation))
    {
        return;
    }
    engine->configuration = configuration;
}

/**
 * @brief Copies the latest coherent application values used by query commands.
 */
void protocol_engine_update_query_context(
    ProtocolEngine *engine,
    const ProtocolQueryContext *query_context)
{
    if ((engine == NULL) || (query_context == NULL))
    {
        return;
    }
    engine->query_context = *query_context;
    engine->query_context_valid = 1U;
}

/**
 * @brief Pops the oldest accepted business command for ArmControlTask.
 */
uint8_t protocol_engine_pop_command(ProtocolEngine *engine,
                                    ProtocolCommand *command)
{
    uint8_t slot_index;

    if ((engine == NULL) || (command == NULL))
    {
        return 0U;
    }
    if (protocol_engine_pop_stop_command(engine, command) != 0U)
    {
        return 1U;
    }
    if (engine->command_read_sequence == engine->command_write_sequence)
    {
        return 0U;
    }
    protocol_engine_compiler_barrier();
    slot_index = (uint8_t)(engine->command_read_sequence %
                           PROTOCOL_ENGINE_COMMAND_CAPACITY);
    *command = engine->commands[slot_index];
    protocol_engine_compiler_barrier();
    ++engine->command_read_sequence;
    return 1U;
}

/**
 * @brief Pops only the independent highest-priority STOP slot.
 */
uint8_t protocol_engine_pop_stop_command(ProtocolEngine *engine,
                                         ProtocolCommand *command)
{
    if ((engine == NULL) || (command == NULL) ||
        (engine->stop_read_sequence == engine->stop_write_sequence))
    {
        return 0U;
    }
    protocol_engine_compiler_barrier();
    *command = engine->stop_command;
    protocol_engine_compiler_barrier();
    ++engine->stop_read_sequence;
    return 1U;
}

/**
 * @brief Cancels all accepted commands not yet taken by ArmControlTask.
 */
void protocol_engine_cancel_pending_commands(ProtocolEngine *engine)
{
    if (engine != NULL)
    {
        engine->command_read_sequence = engine->command_write_sequence;
        engine->stop_read_sequence = engine->stop_write_sequence;
        engine->active_motion_request_id = 0U;
    }
}

/**
 * @brief Submits one terminal result from ArmControlTask without formatting.
 */
uint8_t protocol_engine_submit_command_result(
    ProtocolEngine *engine,
    const ProtocolCommandResult *result)
{
    uint8_t used_count;
    uint8_t slot_index;

    if ((engine == NULL) || (result == NULL))
    {
        return 0U;
    }
    used_count = (uint8_t)(engine->result_write_sequence -
                           engine->result_read_sequence);
    if (used_count >= PROTOCOL_ENGINE_RESULT_CAPACITY)
    {
        return 0U;
    }
    slot_index = (uint8_t)(engine->result_write_sequence %
                           PROTOCOL_ENGINE_RESULT_CAPACITY);
    engine->results[slot_index] = *result;
    protocol_engine_compiler_barrier();
    ++engine->result_write_sequence;
    return 1U;
}

/** @brief Returns stable terminal text for one command result code. */
static const char *protocol_engine_result_text(ProtocolCommandResultCode code)
{
    switch (code)
    {
        case PROTOCOL_COMMAND_RESULT_COMPLETED:
            return "COMPLETED";
        case PROTOCOL_COMMAND_RESULT_STOPPED:
            return "STOPPED";
        case PROTOCOL_COMMAND_RESULT_CANCELLED:
            return "CANCELLED";
        case PROTOCOL_COMMAND_RESULT_FAILED:
        default:
            return "FAILED";
    }
}

/**
 * @brief Formats the oldest terminal result and replaces its replay cache entry.
 */
uint8_t protocol_engine_pop_result_output(
    ProtocolEngine *engine,
    ProtocolOutputBatch *output_batch)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t body_length = 0U;
    ProtocolCommandResult result;
    ProtocolRecentResult *recent_result;
    uint8_t slot_index;

    if ((engine == NULL) || (output_batch == NULL))
    {
        return 0U;
    }
    do
    {
        if (engine->result_read_sequence == engine->result_write_sequence)
        {
            return 0U;
        }
        protocol_engine_compiler_barrier();
        slot_index = (uint8_t)(engine->result_read_sequence %
                               PROTOCOL_ENGINE_RESULT_CAPACITY);
        result = engine->results[slot_index];
        protocol_engine_compiler_barrier();
        ++engine->result_read_sequence;
    } while (result.session_id != engine->session_id);

    if ((result.type == PROTOCOL_COMMAND_MOVE_JOINTS) &&
        (result.request_id == engine->active_motion_request_id))
    {
        engine->active_motion_request_id = 0U;
    }

    protocol_engine_clear_output(output_batch);
    if (result.type == PROTOCOL_COMMAND_LINK_TIMEOUT)
    {
        ++engine->event_sequence;
        if (protocol_engine_append_format(
                output_batch,
                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                "EVT %lu LINK_TIMEOUT elapsed_ms=1000 action=STOP_DISABLE",
                (unsigned long)engine->event_sequence) !=
            PROTOCOL_ENGINE_STATUS_OK)
        {
            return 0U;
        }
        return 1U;
    }
    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    "DONE %lu %s detail=%u",
                                    (unsigned long)result.request_id,
                                    protocol_engine_result_text(result.code),
                                    result.detail) == 0U)
    {
        return 0U;
    }
    if ((result.type == PROTOCOL_COMMAND_ALIGN_REFERENCE) &&
        (result.code == PROTOCOL_COMMAND_RESULT_COMPLETED))
    {
        if ((protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " q_deg=") == 0U) ||
            (protocol_engine_append_joint_vector(body,
                                                 sizeof(body),
                                                 &body_length,
                                                 result.values) == 0U) ||
            (protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " bias_deg=") == 0U) ||
            (protocol_engine_append_joint_vector(body,
                                                 sizeof(body),
                                                 &body_length,
                                                 result.auxiliary_values) == 0U))
        {
            return 0U;
        }
    }
    if (protocol_engine_append_body(output_batch,
                                    PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                    body) != PROTOCOL_ENGINE_STATUS_OK)
    {
        return 0U;
    }

    recent_result = protocol_engine_find_recent(engine,
                                                result.request_id,
                                                result.completed_at_us);
    if (recent_result != NULL)
    {
        recent_result->completed_at_us = result.completed_at_us;
        recent_result->response_length = output_batch->messages[0].length;
        recent_result->priority = output_batch->messages[0].priority;
        memcpy(recent_result->response,
               output_batch->messages[0].data,
               (size_t)recent_result->response_length + 1U);
    }
    return 1U;
}

/**
 * @brief Generates due telemetry and immediate state-change events.
 */
uint8_t protocol_engine_generate_stream_output(
    ProtocolEngine *engine,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t body_length = 0U;
    uint64_t interval_us;
    uint8_t joint_index;

    if ((engine == NULL) || (output_batch == NULL))
    {
        return 0U;
    }
    protocol_engine_clear_output(output_batch);
    if ((engine->session_active == 0U) || (engine->query_context_valid == 0U))
    {
        return 0U;
    }

    if (engine->last_published_state_valid == 0U)
    {
        engine->last_published_state = engine->query_context.arm.state;
        engine->last_published_state_valid = 1U;
    }
    else if (engine->last_published_state != engine->query_context.arm.state)
    {
        ArmState previous_state = engine->last_published_state;

        ++engine->event_sequence;
        if (protocol_engine_append_format(
                output_batch,
                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                "EVT %lu STATE_CHANGED from=%s to=%s t_us=%lu",
                (unsigned long)engine->event_sequence,
                protocol_engine_arm_state_text(previous_state),
                protocol_engine_arm_state_text(engine->query_context.arm.state),
                (unsigned long)timestamp_us) != PROTOCOL_ENGINE_STATUS_OK)
        {
            return 0U;
        }
        engine->last_published_state = engine->query_context.arm.state;
    }

    if (engine->stream_rate_hz == 0U)
    {
        return output_batch->count;
    }
    interval_us = 1000000ULL / engine->stream_rate_hz;
    if ((engine->next_telemetry_due_us != 0U) &&
        (timestamp_us < engine->next_telemetry_due_us))
    {
        return output_batch->count;
    }
    engine->next_telemetry_due_us = timestamp_us + interval_us;
    ++engine->telemetry_sequence;

    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    "TEL %lu JOINT_STATE t_us=%lu",
                                    (unsigned long)engine->telemetry_sequence,
                                    (unsigned long)engine->query_context.joints.published_at_us) == 0U)
    {
        return output_batch->count;
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "jpos") != 0U)
    {
        if ((protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " q_deg=") == 0U) ||
            (protocol_engine_append_joint_vector(
                 body,
                 sizeof(body),
                 &body_length,
                 engine->query_context.joints.position_deg) == 0U))
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "jvel") != 0U)
    {
        if ((protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " qd_deg_s=") == 0U) ||
            (protocol_engine_append_joint_vector(
                 body,
                 sizeof(body),
                 &body_length,
                 engine->query_context.joints.velocity_deg_s) == 0U))
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "jtor") != 0U)
    {
        if ((protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " tau_nm=") == 0U) ||
            (protocol_engine_append_joint_vector(
                 body,
                 sizeof(body),
                 &body_length,
                 engine->query_context.joints.torque_nm) == 0U))
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "state") != 0U)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                " arm_state=%s aligned=%u enabled=%u moving=%u",
                protocol_engine_arm_state_text(engine->query_context.arm.state),
                engine->query_context.arm.aligned,
                engine->query_context.arm.enabled,
                engine->query_context.arm.moving) == 0U)
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "motor") != 0U)
    {
        uint8_t fault_mask = 0U;

        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            if (engine->query_context.motors.joints[joint_index].fault_flags != 0U)
            {
                fault_mask |= (uint8_t)(1U << joint_index);
            }
        }
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                " motor_valid_mask=%u motor_fault_mask=%u",
                engine->query_context.motors.valid_joint_mask,
                fault_mask) == 0U)
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_csv_contains(engine->stream_fields, "diag") != 0U)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                " can_rx=%lu can_tx=%lu queue_hwm=%lu",
                (unsigned long)engine->query_context.diagnostics.can_rx_frames,
                (unsigned long)engine->query_context.diagnostics.can_tx_frames,
                (unsigned long)engine->query_context.diagnostics.queue_high_watermark) == 0U)
        {
            return output_batch->count;
        }
    }
    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    " valid_mask=%u",
                                    engine->query_context.joints.valid_joint_mask) == 0U)
    {
        return output_batch->count;
    }
    (void)protocol_engine_append_body(output_batch,
                                      PROTOCOL_OUTPUT_TELEMETRY,
                                      body);
    return output_batch->count;
}

/**
 * @brief Formats one transport-layer line overflow error without parsing.
 */
ProtocolEngineStatus protocol_engine_format_line_too_long(
    ProtocolOutputBatch *output_batch)
{
    if (output_batch == NULL)
    {
        return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT;
    }
    protocol_engine_clear_output(output_batch);
    return protocol_engine_append_format(output_batch,
                                         PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                         "ERR 0 LINE_TOO_LONG");
}

/**
 * @brief Processes one complete CRC-protected request line.
 */
ProtocolEngineStatus protocol_engine_process_line(ProtocolEngine *engine,
                                                  const char *line,
                                                  size_t length,
                                                  uint64_t timestamp_us,
                                                  ProtocolOutputBatch *output_batch)
{
    AsciiProtocolRequest request;
    AsciiProtocolStatus parse_status;
    ProtocolRecentResult *recent_result;
    ProtocolEngineStatus engine_status;
    uint32_t body_hash;

    if ((engine == NULL) || (line == NULL) || (output_batch == NULL))
    {
        return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT;
    }
    protocol_engine_clear_output(output_batch);
    parse_status = ascii_protocol_parse_request(line, length, &request);
    if (parse_status != ASCII_PROTOCOL_STATUS_OK)
    {
        const char *error_code = "BAD_FRAME";

        if (parse_status == ASCII_PROTOCOL_STATUS_BAD_CRC)
        {
            error_code = "BAD_CRC";
        }
        else if (parse_status == ASCII_PROTOCOL_STATUS_LINE_TOO_LONG)
        {
            error_code = "LINE_TOO_LONG";
        }
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu %s",
                                            (unsigned long)request.request_id,
                                            error_code);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    body_hash = ascii_protocol_crc32_iso_hdlc(
        (const uint8_t *)request.storage,
        request.body_length);
    recent_result = protocol_engine_find_recent(engine,
                                                request.request_id,
                                                timestamp_us);
    if (recent_result != NULL)
    {
        if (recent_result->body_hash != body_hash)
        {
            if (engine->session_active != 0U)
            {
                engine->last_valid_request_at_us = timestamp_us;
            }
            (void)protocol_engine_append_format(
                output_batch,
                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                "ERR %lu REQUEST_ID_CONFLICT",
                (unsigned long)request.request_id);
            return PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT;
        }
        protocol_engine_replay(recent_result, output_batch);
        if (engine->session_active != 0U)
        {
            engine->last_valid_request_at_us = timestamp_us;
        }
        return PROTOCOL_ENGINE_STATUS_REPLAYED;
    }

    if (ascii_protocol_request_operation_equals(&request, "HELLO") != 0U)
    {
        engine_status = protocol_engine_handle_hello(engine,
                                                     &request,
                                                     timestamp_us,
                                                     output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "HEARTBEAT") != 0U)
    {
        engine_status = protocol_engine_handle_heartbeat(engine,
                                                         &request,
                                                         timestamp_us,
                                                         output_batch);
    }
    else if (engine->session_active == 0U)
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu INVALID_SESSION",
                                            (unsigned long)request.request_id);
        engine_status = PROTOCOL_ENGINE_STATUS_SESSION_MISMATCH;
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_INFO") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_info(engine,
                                                        &request,
                                                        output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_CONFIG") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_config(&request, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_STATE") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_state(engine,
                                                         &request,
                                                         output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_JPOS") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_jpos(engine,
                                                        &request,
                                                        output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_MOTORS") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_motors(engine,
                                                          &request,
                                                          output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "GET_DIAG") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_get_diag(engine,
                                                        &request,
                                                        output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "SET_STREAM") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_set_stream(engine,
                                                          &request,
                                                          output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request,
                                                      "ALIGN_REFERENCE") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_align_reference(engine,
                                                               &request,
                                                               timestamp_us,
                                                               output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "SET_MODE") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_lifecycle_action(
            engine, &request, PROTOCOL_COMMAND_SET_MODE, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "ENABLE") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_lifecycle_action(
            engine, &request, PROTOCOL_COMMAND_ENABLE, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "STOP") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_lifecycle_action(
            engine, &request, PROTOCOL_COMMAND_STOP, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "DISABLE") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_lifecycle_action(
            engine, &request, PROTOCOL_COMMAND_DISABLE, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "CLEAR_FAULT") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_lifecycle_action(
            engine, &request, PROTOCOL_COMMAND_CLEAR_FAULT, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "MOVE_JOINTS") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_move_joints(engine,
                                                           &request,
                                                           timestamp_us,
                                                           output_batch);
    }
    else
    {
        engine->last_valid_request_at_us = timestamp_us;
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu UNKNOWN_OPERATION",
                                            (unsigned long)request.request_id);
        engine_status = PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    if ((output_batch->count == 1U) &&
        (engine_status != PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL))
    {
        protocol_engine_store_recent(engine,
                                     &request,
                                     body_hash,
                                     timestamp_us,
                                     output_batch);
    }
    return engine_status;
}

/**
 * @brief Detects the first 1,000 ms communication watchdog expiry per session.
 */
uint8_t protocol_engine_watchdog_expired(ProtocolEngine *engine,
                                         uint64_t timestamp_us)
{
    if ((engine == NULL) || (engine->session_active == 0U) ||
        (engine->watchdog_timeout_reported != 0U) ||
        (timestamp_us < engine->last_valid_request_at_us) ||
        ((timestamp_us - engine->last_valid_request_at_us) <
         PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US))
    {
        return 0U;
    }
    engine->watchdog_timeout_reported = 1U;
    return 1U;
}
