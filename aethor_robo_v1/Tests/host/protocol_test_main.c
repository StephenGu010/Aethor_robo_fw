/**
 * @file protocol_test_main.c
 * @brief Host-side contract tests for aethor-arm-ascii-v1 framing.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ascii_protocol.h"
#include "protocol_engine.h"
#include "protocol_golden_vectors.h"

/**
 * @brief Verifies the published CRC-16/CCITT-FALSE check value.
 */
static void test_crc16_ccitt_false_reference_vector(void)
{
    static const uint8_t reference_text[] = "123456789";

    assert(ascii_protocol_crc16_ccitt_false(reference_text,
                                             sizeof(reference_text) - 1U) == 0x29B1U);
}

/**
 * @brief Verifies the published CRC-32/ISO-HDLC check value used by map_hash.
 */
static void test_crc32_iso_hdlc_reference_vector(void)
{
    static const uint8_t reference_text[] = "123456789";

    assert(ascii_protocol_crc32_iso_hdlc(reference_text,
                                         sizeof(reference_text) - 1U) == 0xCBF43926UL);
}

/**
 * @brief Verifies the common formatter adds an uppercase CRC and LF ending.
 */
static void test_frame_formatter(void)
{
    static const char body[] = "REQ 42 GET_JPOS";
    static const char expected_frame[] = "REQ 42 GET_JPOS *6B48\n";
    char frame[64];
    size_t frame_length = 0U;

    assert(ascii_protocol_format_frame(body,
                                       strlen(body),
                                       frame,
                                       sizeof(frame),
                                       &frame_length) == ASCII_PROTOCOL_STATUS_OK);
    assert(frame_length == sizeof(expected_frame) - 1U);
    assert(strcmp(frame, expected_frame) == 0);
    assert(ascii_protocol_format_frame(body,
                                       strlen(body),
                                       frame,
                                       sizeof(expected_frame) - 1U,
                                       &frame_length) == ASCII_PROTOCOL_STATUS_OUTPUT_TOO_SMALL);
}

/**
 * @brief Builds a complete test line from one shared Golden Frame vector.
 * @param vector Shared protocol vector.
 * @param frame Destination frame buffer.
 * @param frame_capacity Destination capacity in bytes.
 * @return Number of bytes written, excluding the null terminator.
 */
static size_t build_golden_frame(const ProtocolGoldenVector *vector,
                                 char *frame,
                                 size_t frame_capacity)
{
    const char *line_ending = (vector->use_crlf != 0U) ? "\r\n" : "\n";
    int written_length;

    assert(vector != NULL);
    assert(frame != NULL);

    written_length = snprintf(frame,
                              frame_capacity,
                              "%s *%s%s",
                              vector->body,
                              vector->crc16_text,
                              line_ending);
    assert(written_length > 0);
    assert((size_t)written_length < frame_capacity);
    return (size_t)written_length;
}

/**
 * @brief Verifies every shared Golden Frame against the firmware parser.
 */
static void test_shared_golden_frames(void)
{
    size_t vector_index;

    for (vector_index = 0U; vector_index < PROTOCOL_GOLDEN_VECTOR_COUNT; ++vector_index)
    {
        const ProtocolGoldenVector *vector = &protocol_golden_vectors[vector_index];
        char frame[PROTOCOL_MAX_LINE_LENGTH + 3U];
        AsciiProtocolRequest request;
        uint16_t expected_crc = (uint16_t)strtoul(vector->crc16_text, NULL, 16);
        size_t frame_length = build_golden_frame(vector, frame, sizeof(frame));
        AsciiProtocolStatus parse_status;

        assert(ascii_protocol_crc16_ccitt_false((const uint8_t *)vector->body,
                                                 strlen(vector->body)) == expected_crc ||
               vector->expected_status == ASCII_PROTOCOL_STATUS_BAD_CRC);

        parse_status = ascii_protocol_parse_request(frame, frame_length, &request);
        assert(parse_status == vector->expected_status);
        assert(request.request_id == vector->request_id);

        if (parse_status == ASCII_PROTOCOL_STATUS_OK)
        {
            assert(ascii_protocol_request_operation_equals(&request, vector->operation) != 0U);
            assert(request.field_count == vector->field_count);
        }
    }
}

/**
 * @brief Verifies key lookup returns exact non-owning spans into request storage.
 */
static void test_field_lookup(void)
{
    static const char frame[] =
        "REQ 43 SET_STREAM rate_hz=50 fields=jpos,jvel,state,motor *847F\n";
    AsciiProtocolRequest request;
    AsciiProtocolSpan field_value;

    assert(ascii_protocol_parse_request(frame, sizeof(frame) - 1U, &request) ==
           ASCII_PROTOCOL_STATUS_OK);
    assert(ascii_protocol_find_field(&request, "rate_hz", &field_value) != 0U);
    assert(field_value.length == 2U);
    assert(strncmp(field_value.data, "50", field_value.length) == 0);
    assert(ascii_protocol_find_field(&request, "missing", &field_value) == 0U);
}

/**
 * @brief Formats one request body into a complete CRC-protected line.
 * @param body Request body before CRC.
 * @param frame Destination line buffer.
 * @param frame_capacity Destination capacity.
 * @return Encoded line length.
 */
static size_t build_request_frame(const char *body,
                                  char *frame,
                                  size_t frame_capacity)
{
    size_t frame_length = 0U;

    assert(ascii_protocol_format_frame(body,
                                       strlen(body),
                                       frame,
                                       frame_capacity,
                                       &frame_length) == ASCII_PROTOCOL_STATUS_OK);
    return frame_length;
}

/**
 * @brief Verifies HELLO sessions, replay safety, conflicts, heartbeat, and timeout.
 */
static void test_protocol_engine_session_lifecycle(void)
{
    ProtocolEngine engine;
    ProtocolOutputBatch output_batch;
    char request_frame[256];
    char duplicate_response[PROTOCOL_ENGINE_MESSAGE_CAPACITY];
    char heartbeat_body[96];
    size_t request_length;

    protocol_engine_init(&engine, 1234U);
    request_length = build_request_frame(
        "REQ 1 HELLO client=aethor-studio-v2 protocol=1",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        1000U,
                                        &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(output_batch.count == 1U);
    assert(output_batch.messages[0].priority == PROTOCOL_OUTPUT_QUERY);
    assert(strstr(output_batch.messages[0].data, "RSP 1 ok") != NULL);
    assert(strstr(output_batch.messages[0].data, "boot_id=1234") != NULL);
    assert(strstr(output_batch.messages[0].data, "session=") != NULL);
    assert(engine.session_active != 0U);
    strcpy(duplicate_response, output_batch.messages[0].data);

    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        1100U,
                                        &output_batch) == PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(strcmp(output_batch.messages[0].data, duplicate_response) == 0);

    request_length = build_request_frame("REQ 1 GET_INFO",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        1200U,
                                        &output_batch) ==
           PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT);
    assert(strstr(output_batch.messages[0].data, "ERR 1 REQUEST_ID_CONFLICT") != NULL);

    request_length = build_request_frame("REQ 2 HEARTBEAT session=999",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        2000U,
                                        &output_batch) == PROTOCOL_ENGINE_STATUS_SESSION_MISMATCH);
    assert(strstr(output_batch.messages[0].data, "ERR 2 INVALID_SESSION") != NULL);

    assert(snprintf(heartbeat_body,
                    sizeof(heartbeat_body),
                    "REQ 3 HEARTBEAT session=%lu",
                    (unsigned long)engine.session_id) > 0);
    request_length = build_request_frame(heartbeat_body,
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        3000U,
                                        &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "RSP 3 ok") != NULL);
    assert(protocol_engine_watchdog_expired(&engine, 1002999U) == 0U);
    assert(protocol_engine_watchdog_expired(&engine, 1003000U) != 0U);
    assert(protocol_engine_watchdog_expired(&engine, 1004000U) == 0U);
}

/**
 * @brief Verifies snapshot queries and session-scoped stream configuration.
 */
static void test_protocol_engine_query_dispatch(void)
{
    ProtocolEngine engine;
    ProtocolQueryContext query_context;
    ProtocolOutputBatch output_batch;
    char request_frame[256];
    size_t request_length;

    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_FAULT;
    query_context.arm.fault = ARM_FAULT_CONFIG_INCOMPLETE;
    query_context.motors.valid_joint_mask = 0x01U;
    query_context.motors.joints[0].mos_temperature_c = 42.0F;
    query_context.motors.joints[0].rotor_temperature_c = 40.0F;
    query_context.diagnostics.service_cycles = 5U;
    query_context.timestamp_us = 9000U;

    protocol_engine_init(&engine, 4321U);
    request_length = build_request_frame(
        "REQ 10 HELLO client=test protocol=1",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine,
                                        request_frame,
                                        request_length,
                                        1000U,
                                        &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    protocol_engine_update_query_context(&engine, &query_context);

    request_length = build_request_frame("REQ 11 GET_STATE",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "state=FAULT") != NULL);
    assert(strstr(output_batch.messages[0].data, "fault=CONFIG_INCOMPLETE") != NULL);

    request_length = build_request_frame("REQ 12 GET_JPOS",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2100U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "valid=0,0,0,0,0,0,0") != NULL);
    assert(strstr(output_batch.messages[0].data, "aligned=0") != NULL);

    query_context.joints.position_deg[0] = 12.345F;
    query_context.joints.valid_joint_mask = 0x01U;
    query_context.joints.aligned = 1U;
    query_context.joints.published_at_us = 9050U;
    protocol_engine_update_query_context(&engine, &query_context);
    request_length = build_request_frame("REQ 18 GET_JPOS",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2150U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "q_deg=12.345") != NULL);
    assert(strstr(output_batch.messages[0].data, "valid=1,0,0,0,0,0,0") != NULL);
    assert(strstr(output_batch.messages[0].data, "aligned=1") != NULL);

    request_length = build_request_frame("REQ 13 GET_MOTORS",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2200U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "status=1,0,0,0,0,0,0") != NULL);
    assert(strstr(output_batch.messages[0].data, "mos_c=42,0,0,0,0,0,0") != NULL);
    assert(strstr(output_batch.messages[0].data, "age_ms=") != NULL);

    request_length = build_request_frame("REQ 14 GET_DIAG",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2300U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "service_cycles=5") != NULL);

    request_length = build_request_frame("REQ 15 GET_CONFIG",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2400U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "verified=0") != NULL);
    assert(strstr(output_batch.messages[0].data, "map_hash=E0DA65E8") != NULL);
    assert(output_batch.messages[0].length <= PROTOCOL_MAX_LINE_LENGTH);

    request_length = build_request_frame(
        "REQ 16 SET_STREAM rate_hz=100 fields=jpos,jvel,state,motor",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2500U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(engine.stream_rate_hz == 100U);
    assert(strcmp(engine.stream_fields, "jpos,jvel,state,motor") == 0);

    request_length = build_request_frame(
        "REQ 17 SET_STREAM rate_hz=101 fields=jpos",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2600U, &output_batch) == PROTOCOL_ENGINE_STATUS_BAD_REQUEST);
    assert(strstr(output_batch.messages[0].data, "BAD_VALUE") != NULL);
}

/**
 * @brief Verifies ALIGN_REFERENCE has replay-safe ACK and terminal DONE phases.
 */
static void test_protocol_engine_align_reference_lifecycle(void)
{
    ProtocolEngine engine;
    ProtocolQueryContext query_context;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    ProtocolCommandResult result;
    char request_frame[256];
    size_t request_length;

    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_UNALIGNED;
    query_context.motors.valid_joint_mask = 0x7FU;
    protocol_engine_init(&engine, 9876U);
    request_length = build_request_frame(
        "REQ 1 HELLO client=test protocol=1",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        1000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    protocol_engine_update_query_context(&engine, &query_context);

    request_length = build_request_frame(
        "REQ 20 ALIGN_REFERENCE q_ref_deg=10,-20,0,0,0,0,0",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "ACK 20 accepted") != NULL);
    assert(protocol_engine_pop_command(&engine, &command) == 1U);
    assert(command.type == PROTOCOL_COMMAND_ALIGN_REFERENCE);
    assert(command.request_id == 20U);
    assert(command.values[0] == 10.0F);
    assert(command.values[1] == -20.0F);
    assert(protocol_engine_pop_command(&engine, &command) == 0U);

    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2100U, &output_batch) ==
           PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(strstr(output_batch.messages[0].data, "ACK 20 accepted") != NULL);
    assert(protocol_engine_pop_command(&engine, &command) == 0U);

    memset(&result, 0, sizeof(result));
    result.request_id = 20U;
    result.session_id = engine.session_id;
    result.type = PROTOCOL_COMMAND_ALIGN_REFERENCE;
    result.code = PROTOCOL_COMMAND_RESULT_COMPLETED;
    result.completed_at_us = 3000U;
    result.values[0] = 10.0F;
    result.auxiliary_values[0] = 1.5F;
    assert(protocol_engine_submit_command_result(&engine, &result) == 1U);
    assert(protocol_engine_pop_result_output(&engine, &output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "DONE 20 COMPLETED") != NULL);
    assert(strstr(output_batch.messages[0].data, "q_deg=10.000") != NULL);
    assert(strstr(output_batch.messages[0].data, "bias_deg=1.500") != NULL);
    assert(protocol_engine_pop_result_output(&engine, &output_batch) == 0U);

    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        3100U, &output_batch) ==
           PROTOCOL_ENGINE_STATUS_REPLAYED);
    assert(strstr(output_batch.messages[0].data, "DONE 20 COMPLETED") != NULL);
}

/**
 * @brief Verifies rate-gated telemetry, state events, and overlong-line errors.
 */
static void test_protocol_engine_stream_generation(void)
{
    ProtocolEngine engine;
    ProtocolQueryContext query_context;
    ProtocolOutputBatch output_batch;
    char request_frame[256];
    size_t request_length;

    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_UNALIGNED;
    query_context.joints.position_deg[0] = 1.25F;
    query_context.joints.valid_joint_mask = 0x01U;
    query_context.joints.published_at_us = 1000U;
    protocol_engine_init(&engine, 3456U);
    request_length = build_request_frame(
        "REQ 1 HELLO client=test protocol=1",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        1000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    protocol_engine_update_query_context(&engine, &query_context);

    assert(protocol_engine_generate_stream_output(&engine, 1000U, &output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "TEL 1 JOINT_STATE") != NULL);
    assert(strstr(output_batch.messages[0].data, "q_deg=1.250") != NULL);
    assert(output_batch.messages[0].priority == PROTOCOL_OUTPUT_TELEMETRY);
    assert(protocol_engine_generate_stream_output(&engine, 1001U, &output_batch) == 0U);

    query_context.arm.state = ARM_STATE_DISABLED;
    protocol_engine_update_query_context(&engine, &query_context);
    assert(protocol_engine_generate_stream_output(&engine, 2000U, &output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "EVT 1 STATE_CHANGED") != NULL);
    assert(output_batch.messages[0].priority == PROTOCOL_OUTPUT_HIGH_PRIORITY);

    request_length = build_request_frame(
        "REQ 2 SET_STREAM rate_hz=100 fields=jpos",
        request_frame,
        sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        3000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(protocol_engine_generate_stream_output(&engine, 3000U, &output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "q_deg=1.250") != NULL);
    assert(strstr(output_batch.messages[0].data, "arm_state=") == NULL);
    assert(protocol_engine_generate_stream_output(&engine, 12999U, &output_batch) == 0U);
    assert(protocol_engine_generate_stream_output(&engine, 13000U, &output_batch) == 1U);

    assert(protocol_engine_format_line_too_long(&output_batch) ==
           PROTOCOL_ENGINE_STATUS_OK);
    assert(strstr(output_batch.messages[0].data, "ERR 0 LINE_TOO_LONG") != NULL);
    assert(output_batch.messages[0].priority == PROTOCOL_OUTPUT_HIGH_PRIORITY);
}

/**
 * @brief Verifies lifecycle commands enforce states and STOP bypasses a full queue.
 */
static void test_protocol_engine_lifecycle_command_admission(void)
{
    ProtocolEngine engine;
    ProtocolQueryContext query_context;
    ProtocolOutputBatch output_batch;
    ProtocolCommand command;
    char request_frame[256];
    size_t request_length;
    uint32_t request_id;

    memset(&query_context, 0, sizeof(query_context));
    query_context.arm.state = ARM_STATE_DISABLED;
    query_context.arm.aligned = 1U;
    query_context.motors.valid_joint_mask = 0x7FU;
    protocol_engine_init(&engine, 4567U);
    request_length = build_request_frame("REQ 1 HELLO client=test protocol=1",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        1000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    protocol_engine_update_query_context(&engine, &query_context);

    request_length = build_request_frame("REQ 2 SET_MODE mode=POS_VEL",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        2000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(protocol_engine_pop_command(&engine, &command) == 1U);
    assert(command.type == PROTOCOL_COMMAND_SET_MODE);
    assert(command.control_mode == ARM_CONTROL_MODE_POSITION_VELOCITY);

    query_context.arm.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    query_context.arm.state = ARM_STATE_READY;
    query_context.arm.enabled = 1U;
    protocol_engine_update_query_context(&engine, &query_context);
    for (request_id = 10U;
         request_id < (10U + PROTOCOL_ENGINE_COMMAND_CAPACITY);
         ++request_id)
    {
        char body[64];

        (void)snprintf(body, sizeof(body), "REQ %lu DISABLE", (unsigned long)request_id);
        request_length = build_request_frame(body, request_frame, sizeof(request_frame));
        assert(protocol_engine_process_line(&engine,
                                            request_frame,
                                            request_length,
                                            3000U + request_id,
                                            &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    }
    request_length = build_request_frame("REQ 30 STOP behavior=controlled",
                                         request_frame,
                                         sizeof(request_frame));
    assert(protocol_engine_process_line(&engine, request_frame, request_length,
                                        4000U, &output_batch) == PROTOCOL_ENGINE_STATUS_OK);
    assert(protocol_engine_pop_command(&engine, &command) == 1U);
    assert(command.type == PROTOCOL_COMMAND_STOP);
}

/**
 * @brief Runs all protocol contract tests.
 * @return Zero when every assertion passes.
 */
int main(void)
{
    test_crc16_ccitt_false_reference_vector();
    test_crc32_iso_hdlc_reference_vector();
    test_frame_formatter();
    test_shared_golden_frames();
    test_field_lookup();
    test_protocol_engine_session_lifecycle();
    test_protocol_engine_query_dispatch();
    test_protocol_engine_align_reference_lifecycle();
    test_protocol_engine_stream_generation();
    test_protocol_engine_lifecycle_command_admission();
    puts("PROTOCOL_TESTS_PASSED");
    return 0;
}
