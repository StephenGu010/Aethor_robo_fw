/**
 * @file ascii_protocol.h
 * @brief Allocation-free framing contract for aethor-arm-ascii-v1.
 */

#ifndef APP_PROTOCOL_ASCII_PROTOCOL_H
#define APP_PROTOCOL_ASCII_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "protocol_contract.h"

#define ASCII_PROTOCOL_MAX_FIELD_COUNT (16U)

/**
 * @brief Reports framing and structural validation outcomes.
 */
typedef enum
{
    ASCII_PROTOCOL_STATUS_OK = 0,
    ASCII_PROTOCOL_STATUS_INVALID_ARGUMENT,
    ASCII_PROTOCOL_STATUS_LINE_TOO_LONG,
    ASCII_PROTOCOL_STATUS_BAD_FRAME,
    ASCII_PROTOCOL_STATUS_BAD_CRC,
    ASCII_PROTOCOL_STATUS_BAD_REQUEST_ID,
    ASCII_PROTOCOL_STATUS_TOO_MANY_FIELDS,
    ASCII_PROTOCOL_STATUS_DUPLICATE_FIELD,
    ASCII_PROTOCOL_STATUS_OUTPUT_TOO_SMALL
} AsciiProtocolStatus;

/**
 * @brief Describes a bounded character range owned by a parsed request.
 */
typedef struct
{
    const char *data;
    uint16_t length;
} AsciiProtocolSpan;

/**
 * @brief Stores offsets for one key-value field inside request storage.
 */
typedef struct
{
    uint16_t key_offset;
    uint16_t key_length;
    uint16_t value_offset;
    uint16_t value_length;
} AsciiProtocolField;

/**
 * @brief Owns one validated request without dynamic allocation.
 */
typedef struct
{
    char storage[PROTOCOL_MAX_LINE_LENGTH + 1U];
    uint32_t request_id;
    uint16_t body_length;
    uint16_t operation_offset;
    uint16_t operation_length;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t field_count;
    AsciiProtocolField fields[ASCII_PROTOCOL_MAX_FIELD_COUNT];
} AsciiProtocolRequest;

/**
 * @brief Calculates CRC-16/CCITT-FALSE over an exact byte range.
 * @param data Input bytes; may be null only when length is zero.
 * @param length Number of input bytes.
 * @return Calculated 16-bit CRC value.
 */
uint16_t ascii_protocol_crc16_ccitt_false(const uint8_t *data, size_t length);

/**
 * @brief Calculates CRC-32/ISO-HDLC for configuration map hashes.
 * @param data Input bytes; may be null only when length is zero.
 * @param length Number of input bytes.
 * @return Calculated 32-bit CRC value.
 */
uint32_t ascii_protocol_crc32_iso_hdlc(const uint8_t *data, size_t length);

/**
 * @brief Appends the standard CRC suffix and LF ending to a frame body.
 * @param body Frame bytes before the CRC separator.
 * @param body_length Number of body bytes.
 * @param output Destination null-terminated frame buffer.
 * @param output_capacity Destination capacity in bytes.
 * @param output_length Destination encoded length excluding the null terminator.
 * @return OK or a precise size/argument error.
 */
AsciiProtocolStatus ascii_protocol_format_frame(const char *body,
                                                size_t body_length,
                                                char *output,
                                                size_t output_capacity,
                                                size_t *output_length);

/**
 * @brief Parses and validates one complete request line.
 * @param line Input line with CRC and optional LF or CRLF ending.
 * @param length Exact input length in bytes.
 * @param request Destination request, initialized even on validation errors.
 * @return Detailed framing status.
 */
AsciiProtocolStatus ascii_protocol_parse_request(const char *line,
                                                 size_t length,
                                                 AsciiProtocolRequest *request);

/**
 * @brief Compares a parsed operation against a null-terminated name.
 * @param request Parsed request.
 * @param operation Expected operation name.
 * @return One when equal, otherwise zero.
 */
uint8_t ascii_protocol_request_operation_equals(const AsciiProtocolRequest *request,
                                                const char *operation);

/**
 * @brief Finds a field value by exact key name.
 * @param request Parsed request.
 * @param key Null-terminated field key.
 * @param value Destination non-owning value span.
 * @return One when found, otherwise zero.
 */
uint8_t ascii_protocol_find_field(const AsciiProtocolRequest *request,
                                  const char *key,
                                  AsciiProtocolSpan *value);

#endif
