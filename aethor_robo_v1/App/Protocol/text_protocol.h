/**
 * @file text_protocol.h
 * @brief Defines the bounded aethor-text-v1 request parser contract.
 */

#ifndef APP_PROTOCOL_TEXT_PROTOCOL_H
#define APP_PROTOCOL_TEXT_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH (160U)
#define TEXT_PROTOCOL_MAX_COMMAND_WORDS (2U)
#define TEXT_PROTOCOL_MAX_POSITIONAL_COUNT (2U)
#define TEXT_PROTOCOL_MAX_FIELD_COUNT (8U)

/**
 * @brief Reports deterministic text parsing and conversion outcomes.
 */
typedef enum
{
    TEXT_PROTOCOL_STATUS_OK = 0,
    TEXT_PROTOCOL_STATUS_INVALID_ARGUMENT,
    TEXT_PROTOCOL_STATUS_EMPTY_LINE,
    TEXT_PROTOCOL_STATUS_LINE_TOO_LONG,
    TEXT_PROTOCOL_STATUS_BAD_CHARACTER,
    TEXT_PROTOCOL_STATUS_BAD_REQUEST_ID,
    TEXT_PROTOCOL_STATUS_BAD_COMMAND,
    TEXT_PROTOCOL_STATUS_BAD_FIELD,
    TEXT_PROTOCOL_STATUS_DUPLICATE_FIELD,
    TEXT_PROTOCOL_STATUS_TOO_MANY_ARGUMENTS,
    TEXT_PROTOCOL_STATUS_BAD_NUMBER
} TextProtocolStatus;

/**
 * @brief References one immutable token inside request-owned storage.
 */
typedef struct
{
    const char *data;
    size_t length;
} TextProtocolSpan;

/**
 * @brief References one validated key and its value inside request storage.
 */
typedef struct
{
    TextProtocolSpan key;
    TextProtocolSpan value;
} TextProtocolField;

/**
 * @brief Owns one parsed request and every token referenced by its spans.
 */
typedef struct
{
    char storage[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 1U];
    uint32_t request_id;
    uint8_t has_request_id;
    TextProtocolSpan command_words[TEXT_PROTOCOL_MAX_COMMAND_WORDS];
    TextProtocolSpan positionals[TEXT_PROTOCOL_MAX_POSITIONAL_COUNT];
    TextProtocolField fields[TEXT_PROTOCOL_MAX_FIELD_COUNT];
    uint8_t command_word_count;
    uint8_t positional_count;
    uint8_t field_count;
} TextProtocolRequest;

/**
 * @brief Parses one optional-ID aethor-text-v1 request line.
 * @param line Input bytes with an optional LF or CRLF terminator.
 * @param length Number of input bytes available at line.
 * @param request Destination request initialized on every non-null call.
 * @return Detailed parse status without allocating memory.
 */
TextProtocolStatus text_protocol_parse_request(const char *line,
                                               size_t length,
                                               TextProtocolRequest *request);

/**
 * @brief Compares a parsed one-word or two-word command path case-insensitively.
 * @param request Parsed request.
 * @param first_word Required first command word.
 * @param second_word Optional second word; null selects a one-word command.
 * @return Nonzero when the complete command path matches.
 */
uint8_t text_protocol_request_path_equals(const TextProtocolRequest *request,
                                          const char *first_word,
                                          const char *second_word);

/**
 * @brief Returns one positional token by zero-based index.
 * @param request Parsed request.
 * @param index Zero-based positional index.
 * @param value Destination token span.
 * @return Nonzero when the requested positional exists.
 */
uint8_t text_protocol_get_positional(const TextProtocolRequest *request,
                                     uint8_t index,
                                     TextProtocolSpan *value);

/**
 * @brief Finds one exact field name in a parsed request.
 * @param request Parsed request.
 * @param key Lowercase field name to find.
 * @param value Destination value span.
 * @return Nonzero when the field exists.
 */
uint8_t text_protocol_find_field(const TextProtocolRequest *request,
                                 const char *key,
                                 TextProtocolSpan *value);

/**
 * @brief Converts a strict decimal span without accepting exponent or NaN syntax.
 * @param span Decimal token to convert.
 * @param value Destination finite float.
 * @return OK for one complete finite decimal, otherwise BAD_NUMBER.
 */
TextProtocolStatus text_protocol_span_to_float(const TextProtocolSpan *span,
                                               float *value);

#endif
