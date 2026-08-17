/**
 * @file ascii_protocol.c
 * @brief Implements strict allocation-free aethor-arm-ascii-v1 framing.
 */

#include "ascii_protocol.h"

#include <limits.h>
#include <string.h>

/** @brief Stores one token location while validating request grammar. */
typedef struct
{
    uint16_t offset;
    uint16_t length;
} AsciiProtocolToken;

/**
 * @brief Converts one uppercase or lowercase hexadecimal digit.
 * @param character ASCII character to convert.
 * @param nibble Destination four-bit value.
 * @return One for a valid hexadecimal digit, otherwise zero.
 */
static uint8_t ascii_protocol_parse_hex_digit(char character, uint8_t *nibble)
{
    if ((character >= '0') && (character <= '9'))
    {
        *nibble = (uint8_t)(character - '0');
        return 1U;
    }

    if ((character >= 'A') && (character <= 'F'))
    {
        *nibble = (uint8_t)(character - 'A' + 10);
        return 1U;
    }

    if ((character >= 'a') && (character <= 'f'))
    {
        *nibble = (uint8_t)(character - 'a' + 10);
        return 1U;
    }

    return 0U;
}

/**
 * @brief Parses exactly four hexadecimal characters into a CRC value.
 * @param text Four-character hexadecimal range.
 * @param parsed_crc Destination CRC value.
 * @return One when every digit is valid, otherwise zero.
 */
static uint8_t ascii_protocol_parse_crc(const char *text, uint16_t *parsed_crc)
{
    uint16_t crc_value = 0U;
    size_t digit_index;

    for (digit_index = 0U; digit_index < 4U; ++digit_index)
    {
        uint8_t nibble;

        if (ascii_protocol_parse_hex_digit(text[digit_index], &nibble) == 0U)
        {
            return 0U;
        }

        crc_value = (uint16_t)((crc_value << 4U) | nibble);
    }

    *parsed_crc = crc_value;
    return 1U;
}

/**
 * @brief Parses a bounded unsigned decimal request identifier.
 * @param text Decimal input range.
 * @param length Number of decimal characters.
 * @param request_id Destination request identifier.
 * @return One for a nonzero uint32 value, otherwise zero.
 */
static uint8_t ascii_protocol_parse_request_id(const char *text,
                                               uint16_t length,
                                               uint32_t *request_id)
{
    uint32_t parsed_value = 0U;
    uint16_t character_index;

    if (length == 0U)
    {
        return 0U;
    }

    for (character_index = 0U; character_index < length; ++character_index)
    {
        uint32_t digit;

        if ((text[character_index] < '0') || (text[character_index] > '9'))
        {
            return 0U;
        }

        digit = (uint32_t)(text[character_index] - '0');
        if (parsed_value > ((UINT32_MAX - digit) / 10U))
        {
            return 0U;
        }

        parsed_value = (parsed_value * 10U) + digit;
    }

    if (parsed_value == 0U)
    {
        return 0U;
    }

    *request_id = parsed_value;
    return 1U;
}

/**
 * @brief Recovers a trustworthy request identifier before full grammar parsing.
 * @param request Request containing the copied frame body.
 */
static void ascii_protocol_recover_request_id(AsciiProtocolRequest *request)
{
    uint16_t digit_offset = 4U;
    uint16_t digit_length = 0U;
    uint32_t recovered_request_id;

    if ((request->body_length <= digit_offset) ||
        (memcmp(request->storage, "REQ ", digit_offset) != 0))
    {
        return;
    }

    while (((uint16_t)(digit_offset + digit_length) < request->body_length) &&
           (request->storage[digit_offset + digit_length] >= '0') &&
           (request->storage[digit_offset + digit_length] <= '9'))
    {
        ++digit_length;
    }

    if (((uint16_t)(digit_offset + digit_length) < request->body_length) &&
        (request->storage[digit_offset + digit_length] == ' ') &&
        (ascii_protocol_parse_request_id(&request->storage[digit_offset],
                                         digit_length,
                                         &recovered_request_id) != 0U))
    {
        request->request_id = recovered_request_id;
    }
}

/**
 * @brief Extracts one token while enforcing single-space separators.
 * @param request Request containing the frame body.
 * @param cursor In/out parsing cursor.
 * @param token Destination token offsets.
 * @return OK when a nonempty token is extracted, otherwise BAD_FRAME.
 */
static AsciiProtocolStatus ascii_protocol_next_token(const AsciiProtocolRequest *request,
                                                     uint16_t *cursor,
                                                     AsciiProtocolToken *token)
{
    uint16_t token_start;

    if ((*cursor >= request->body_length) || (request->storage[*cursor] == ' '))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }

    token_start = *cursor;
    while ((*cursor < request->body_length) && (request->storage[*cursor] != ' '))
    {
        unsigned char character = (unsigned char)request->storage[*cursor];

        if ((character < 0x21U) || (character > 0x7EU))
        {
            return ASCII_PROTOCOL_STATUS_BAD_FRAME;
        }
        ++(*cursor);
    }

    token->offset = token_start;
    token->length = (uint16_t)(*cursor - token_start);

    if (*cursor < request->body_length)
    {
        ++(*cursor);
        if ((*cursor >= request->body_length) || (request->storage[*cursor] == ' '))
        {
            return ASCII_PROTOCOL_STATUS_BAD_FRAME;
        }
    }

    return ASCII_PROTOCOL_STATUS_OK;
}

/**
 * @brief Checks whether a token exactly equals a null-terminated string.
 * @param request Request owning the token storage.
 * @param token Token offset and length.
 * @param expected Expected null-terminated text.
 * @return One when equal, otherwise zero.
 */
static uint8_t ascii_protocol_token_equals(const AsciiProtocolRequest *request,
                                           const AsciiProtocolToken *token,
                                           const char *expected)
{
    size_t expected_length = strlen(expected);

    return (uint8_t)((expected_length == token->length) &&
                     (memcmp(&request->storage[token->offset], expected, expected_length) == 0));
}

/**
 * @brief Validates an uppercase operation token.
 * @param request Request owning the token storage.
 * @param token Operation token.
 * @return One when the token follows the operation naming contract.
 */
static uint8_t ascii_protocol_operation_is_valid(const AsciiProtocolRequest *request,
                                                 const AsciiProtocolToken *token)
{
    uint16_t character_index;

    if (token->length == 0U)
    {
        return 0U;
    }

    for (character_index = 0U; character_index < token->length; ++character_index)
    {
        char character = request->storage[token->offset + character_index];

        if (!(((character >= 'A') && (character <= 'Z')) ||
              ((character >= '0') && (character <= '9')) ||
              (character == '_')))
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief Splits and validates one lowercase key-value token.
 * @param request Request owning token storage.
 * @param token Complete key-value token.
 * @param field Destination field offsets.
 * @return OK for a valid field, otherwise BAD_FRAME.
 */
static AsciiProtocolStatus ascii_protocol_parse_field(const AsciiProtocolRequest *request,
                                                      const AsciiProtocolToken *token,
                                                      AsciiProtocolField *field)
{
    uint16_t separator_index = token->length;
    uint16_t character_index;

    for (character_index = 0U; character_index < token->length; ++character_index)
    {
        if (request->storage[token->offset + character_index] == '=')
        {
            separator_index = character_index;
            break;
        }
    }

    if ((separator_index == 0U) || (separator_index >= (uint16_t)(token->length - 1U)))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }

    for (character_index = 0U; character_index < separator_index; ++character_index)
    {
        char character = request->storage[token->offset + character_index];

        if (!(((character >= 'a') && (character <= 'z')) ||
              ((character_index > 0U) && (character >= '0') && (character <= '9')) ||
              ((character_index > 0U) && (character == '_'))))
        {
            return ASCII_PROTOCOL_STATUS_BAD_FRAME;
        }
    }

    field->key_offset = token->offset;
    field->key_length = separator_index;
    field->value_offset = (uint16_t)(token->offset + separator_index + 1U);
    field->value_length = (uint16_t)(token->length - separator_index - 1U);
    return ASCII_PROTOCOL_STATUS_OK;
}

/**
 * @brief Checks whether a newly parsed field duplicates an earlier key.
 * @param request Request containing previously accepted fields.
 * @param candidate Newly parsed field.
 * @return One when the key already exists, otherwise zero.
 */
static uint8_t ascii_protocol_field_is_duplicate(const AsciiProtocolRequest *request,
                                                 const AsciiProtocolField *candidate)
{
    uint8_t field_index;

    for (field_index = 0U; field_index < request->field_count; ++field_index)
    {
        const AsciiProtocolField *existing_field = &request->fields[field_index];

        if ((existing_field->key_length == candidate->key_length) &&
            (memcmp(&request->storage[existing_field->key_offset],
                    &request->storage[candidate->key_offset],
                    candidate->key_length) == 0))
        {
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief Calculates CRC-16/CCITT-FALSE over an exact byte range.
 * @param data Input bytes; may be null only when length is zero.
 * @param length Number of input bytes.
 * @return Calculated 16-bit CRC value.
 */
uint16_t ascii_protocol_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t byte_index;

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        uint8_t bit_index;

        crc ^= (uint16_t)((uint16_t)data[byte_index] << 8U);
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

/**
 * @brief Calculates CRC-32/ISO-HDLC for configuration map hashes.
 * @param data Input bytes; may be null only when length is zero.
 * @param length Number of input bytes.
 * @return Calculated 32-bit CRC value.
 */
uint32_t ascii_protocol_crc32_iso_hdlc(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    size_t byte_index;

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        uint8_t bit_index;

        crc ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            uint32_t reflected_mask = (uint32_t)(0UL - (crc & 1UL));
            crc = (crc >> 1U) ^ (0xEDB88320UL & reflected_mask);
        }
    }

    return crc ^ 0xFFFFFFFFUL;
}

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
                                                size_t *output_length)
{
    static const char hexadecimal_digits[] = "0123456789ABCDEF";
    size_t content_length;
    uint16_t crc;

    if (output_length != NULL)
    {
        *output_length = 0U;
    }

    if ((body == NULL) || (output == NULL) || (output_length == NULL) ||
        (body_length == 0U))
    {
        return ASCII_PROTOCOL_STATUS_INVALID_ARGUMENT;
    }

    if (body_length > (PROTOCOL_MAX_LINE_LENGTH - 6U))
    {
        return ASCII_PROTOCOL_STATUS_LINE_TOO_LONG;
    }

    content_length = body_length + 6U;
    if (output_capacity < (content_length + 2U))
    {
        return ASCII_PROTOCOL_STATUS_OUTPUT_TOO_SMALL;
    }

    memcpy(output, body, body_length);
    crc = ascii_protocol_crc16_ccitt_false((const uint8_t *)body, body_length);
    output[body_length] = ' ';
    output[body_length + 1U] = '*';
    output[body_length + 2U] = hexadecimal_digits[(crc >> 12U) & 0x0FU];
    output[body_length + 3U] = hexadecimal_digits[(crc >> 8U) & 0x0FU];
    output[body_length + 4U] = hexadecimal_digits[(crc >> 4U) & 0x0FU];
    output[body_length + 5U] = hexadecimal_digits[crc & 0x0FU];
    output[body_length + 6U] = '\n';
    output[body_length + 7U] = '\0';
    *output_length = content_length + 1U;
    return ASCII_PROTOCOL_STATUS_OK;
}

/**
 * @brief Parses and validates one complete request line.
 * @param line Input line with CRC and optional LF or CRLF ending.
 * @param length Exact input length in bytes.
 * @param request Destination request, initialized even on validation errors.
 * @return Detailed framing status.
 */
AsciiProtocolStatus ascii_protocol_parse_request(const char *line,
                                                 size_t length,
                                                 AsciiProtocolRequest *request)
{
    size_t content_length = length;
    uint16_t cursor = 0U;
    AsciiProtocolToken token;
    AsciiProtocolStatus token_status;

    if (request == NULL)
    {
        return ASCII_PROTOCOL_STATUS_INVALID_ARGUMENT;
    }
    memset(request, 0, sizeof(*request));

    if ((line == NULL) || (length == 0U))
    {
        return ASCII_PROTOCOL_STATUS_INVALID_ARGUMENT;
    }

    if ((content_length > 0U) && (line[content_length - 1U] == '\n'))
    {
        --content_length;
        if ((content_length > 0U) && (line[content_length - 1U] == '\r'))
        {
            --content_length;
        }
    }

    if (content_length > PROTOCOL_MAX_LINE_LENGTH)
    {
        return ASCII_PROTOCOL_STATUS_LINE_TOO_LONG;
    }

    if ((content_length < 7U) ||
        (line[content_length - 6U] != ' ') ||
        (line[content_length - 5U] != '*'))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }

    memcpy(request->storage, line, content_length);
    request->storage[content_length] = '\0';
    request->body_length = (uint16_t)(content_length - 6U);

    if ((request->body_length == 0U) ||
        (ascii_protocol_parse_crc(&request->storage[content_length - 4U],
                                  &request->received_crc) == 0U))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }

    request->calculated_crc = ascii_protocol_crc16_ccitt_false(
        (const uint8_t *)request->storage,
        request->body_length);
    ascii_protocol_recover_request_id(request);

    if (request->calculated_crc != request->received_crc)
    {
        return ASCII_PROTOCOL_STATUS_BAD_CRC;
    }

    token_status = ascii_protocol_next_token(request, &cursor, &token);
    if ((token_status != ASCII_PROTOCOL_STATUS_OK) ||
        (ascii_protocol_token_equals(request, &token, "REQ") == 0U))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }

    token_status = ascii_protocol_next_token(request, &cursor, &token);
    if (token_status != ASCII_PROTOCOL_STATUS_OK)
    {
        return token_status;
    }
    if (ascii_protocol_parse_request_id(&request->storage[token.offset],
                                        token.length,
                                        &request->request_id) == 0U)
    {
        request->request_id = 0U;
        return ASCII_PROTOCOL_STATUS_BAD_REQUEST_ID;
    }

    token_status = ascii_protocol_next_token(request, &cursor, &token);
    if ((token_status != ASCII_PROTOCOL_STATUS_OK) ||
        (ascii_protocol_operation_is_valid(request, &token) == 0U))
    {
        return ASCII_PROTOCOL_STATUS_BAD_FRAME;
    }
    request->operation_offset = token.offset;
    request->operation_length = token.length;

    while (cursor < request->body_length)
    {
        AsciiProtocolField parsed_field;

        if (request->field_count >= ASCII_PROTOCOL_MAX_FIELD_COUNT)
        {
            return ASCII_PROTOCOL_STATUS_TOO_MANY_FIELDS;
        }

        token_status = ascii_protocol_next_token(request, &cursor, &token);
        if (token_status != ASCII_PROTOCOL_STATUS_OK)
        {
            return token_status;
        }

        token_status = ascii_protocol_parse_field(request, &token, &parsed_field);
        if (token_status != ASCII_PROTOCOL_STATUS_OK)
        {
            return token_status;
        }

        if (ascii_protocol_field_is_duplicate(request, &parsed_field) != 0U)
        {
            return ASCII_PROTOCOL_STATUS_DUPLICATE_FIELD;
        }

        request->fields[request->field_count] = parsed_field;
        ++request->field_count;
    }

    return ASCII_PROTOCOL_STATUS_OK;
}

/**
 * @brief Compares a parsed operation against a null-terminated name.
 * @param request Parsed request.
 * @param operation Expected operation name.
 * @return One when equal, otherwise zero.
 */
uint8_t ascii_protocol_request_operation_equals(const AsciiProtocolRequest *request,
                                                const char *operation)
{
    size_t operation_length;

    if ((request == NULL) || (operation == NULL))
    {
        return 0U;
    }

    operation_length = strlen(operation);
    return (uint8_t)((operation_length == request->operation_length) &&
                     (memcmp(&request->storage[request->operation_offset],
                             operation,
                             operation_length) == 0));
}

/**
 * @brief Finds a field value by exact key name.
 * @param request Parsed request.
 * @param key Null-terminated field key.
 * @param value Destination non-owning value span.
 * @return One when found, otherwise zero.
 */
uint8_t ascii_protocol_find_field(const AsciiProtocolRequest *request,
                                  const char *key,
                                  AsciiProtocolSpan *value)
{
    size_t key_length;
    uint8_t field_index;

    if ((request == NULL) || (key == NULL) || (value == NULL))
    {
        return 0U;
    }

    value->data = NULL;
    value->length = 0U;
    key_length = strlen(key);

    for (field_index = 0U; field_index < request->field_count; ++field_index)
    {
        const AsciiProtocolField *field = &request->fields[field_index];

        if ((key_length == field->key_length) &&
            (memcmp(&request->storage[field->key_offset], key, key_length) == 0))
        {
            value->data = &request->storage[field->value_offset];
            value->length = field->value_length;
            return 1U;
        }
    }

    return 0U;
}
