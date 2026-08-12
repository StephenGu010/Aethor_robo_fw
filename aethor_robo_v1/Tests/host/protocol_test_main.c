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
    puts("PROTOCOL_TESTS_PASSED");
    return 0;
}
