/**
 * @file protocol_engine.c
 * @brief Implements bounded sessions, replay protection, and core queries.
 */

#include "protocol_engine.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "build_info.h"

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
    engine->session_id = protocol_engine_create_session(engine,
                                                        request->request_id,
                                                        timestamp_us);
    engine->session_active = 1U;
    engine->watchdog_timeout_reported = 0U;
    engine->last_valid_request_at_us = timestamp_us;
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
    engine->boot_id = (boot_id == 0U) ? 1U : boot_id;
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
