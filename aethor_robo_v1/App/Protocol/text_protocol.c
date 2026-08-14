/**
 * @file text_protocol.c
 * @brief Implements the bounded aethor-text-v1 request parser.
 */

#include "text_protocol.h"

#include <errno.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>

#define TEXT_PROTOCOL_MAX_TOKEN_COUNT                                      \
    (1U + TEXT_PROTOCOL_MAX_COMMAND_WORDS +                               \
     TEXT_PROTOCOL_MAX_POSITIONAL_COUNT + TEXT_PROTOCOL_MAX_FIELD_COUNT)
#define TEXT_PROTOCOL_FLOAT_BUFFER_CAPACITY (48U)

/**
 * @brief Converts one ASCII letter to lowercase and preserves other bytes.
 * @param character Character to normalize.
 * @return Lowercase ASCII value or the original byte.
 */
static char text_protocol_ascii_lower(char character)
{
    if ((character >= 'A') && (character <= 'Z'))
    {
        return (char)(character + ('a' - 'A'));
    }
    return character;
}

/**
 * @brief Compares one span with one null-terminated word without locale rules.
 * @param span Span to compare.
 * @param word Expected word.
 * @return Nonzero when both values match case-insensitively.
 */
static uint8_t text_protocol_span_equals_word(const TextProtocolSpan *span,
                                              const char *word)
{
    size_t index;
    size_t word_length;

    if ((span == NULL) || (word == NULL))
    {
        return 0U;
    }
    word_length = strlen(word);
    if (span->length != word_length)
    {
        return 0U;
    }
    for (index = 0U; index < span->length; ++index)
    {
        if (text_protocol_ascii_lower(span->data[index]) !=
            text_protocol_ascii_lower(word[index]))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Validates an ASCII command or field-key word.
 * @param data Token bytes.
 * @param length Token length.
 * @return Nonzero when the token follows the identifier grammar.
 */
static uint8_t text_protocol_is_identifier(const char *data, size_t length)
{
    size_t index;

    if ((data == NULL) || (length == 0U) ||
        !(((data[0] >= 'A') && (data[0] <= 'Z')) ||
          ((data[0] >= 'a') && (data[0] <= 'z'))))
    {
        return 0U;
    }
    for (index = 1U; index < length; ++index)
    {
        const char character = data[index];

        if (!(((character >= 'A') && (character <= 'Z')) ||
              ((character >= 'a') && (character <= 'z')) ||
              ((character >= '0') && (character <= '9')) ||
              (character == '_')))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Lowercases one identifier span in request-owned storage.
 * @param span Mutable span whose backing storage belongs to the request.
 */
static void text_protocol_lowercase_span(TextProtocolSpan *span)
{
    size_t index;
    char *mutable_data;

    if ((span == NULL) || (span->data == NULL))
    {
        return;
    }
    mutable_data = (char *)span->data;
    for (index = 0U; index < span->length; ++index)
    {
        mutable_data[index] = text_protocol_ascii_lower(mutable_data[index]);
    }
}

/**
 * @brief Parses one complete decimal uint32 request ID with overflow checking.
 * @param span Candidate decimal token.
 * @param request_id Destination integer.
 * @return Nonzero when the token is a valid uint32 decimal.
 */
static uint8_t text_protocol_parse_request_id(const TextProtocolSpan *span,
                                              uint32_t *request_id)
{
    size_t index;
    uint32_t value = 0U;

    if ((span == NULL) || (request_id == NULL) || (span->length == 0U))
    {
        return 0U;
    }
    for (index = 0U; index < span->length; ++index)
    {
        uint32_t digit;

        if ((span->data[index] < '0') || (span->data[index] > '9'))
        {
            return 0U;
        }
        digit = (uint32_t)(span->data[index] - '0');
        if ((value > (UINT32_MAX / 10U)) ||
            ((value == (UINT32_MAX / 10U)) &&
             (digit > (UINT32_MAX % 10U))))
        {
            return 0U;
        }
        value = (value * 10U) + digit;
    }
    *request_id = value;
    return 1U;
}

/**
 * @brief Determines whether a first command word has a one-word path.
 * @param first_word Parsed first command word.
 * @return Nonzero for help, hello, or ping.
 */
static uint8_t text_protocol_is_single_word_command(
    const TextProtocolSpan *first_word)
{
    return (uint8_t)((text_protocol_span_equals_word(first_word, "help") != 0U) ||
                     (text_protocol_span_equals_word(first_word, "hello") != 0U) ||
                     (text_protocol_span_equals_word(first_word, "ping") != 0U));
}

/**
 * @brief Splits normalized request storage into non-empty space-delimited spans.
 * @param storage Mutable request-owned storage.
 * @param length Number of bytes before the null terminator.
 * @param tokens Destination token array.
 * @param token_count Destination token count.
 * @return OK or TOO_MANY_ARGUMENTS.
 */
static TextProtocolStatus text_protocol_tokenize(
    char *storage,
    size_t length,
    TextProtocolSpan tokens[TEXT_PROTOCOL_MAX_TOKEN_COUNT],
    uint8_t *token_count)
{
    size_t index = 0U;
    uint8_t count = 0U;

    while (index < length)
    {
        size_t token_start;

        while ((index < length) && (storage[index] == ' '))
        {
            storage[index] = '\0';
            ++index;
        }
        if (index >= length)
        {
            break;
        }
        if (count >= TEXT_PROTOCOL_MAX_TOKEN_COUNT)
        {
            return TEXT_PROTOCOL_STATUS_TOO_MANY_ARGUMENTS;
        }
        token_start = index;
        while ((index < length) && (storage[index] != ' '))
        {
            ++index;
        }
        tokens[count].data = &storage[token_start];
        tokens[count].length = index - token_start;
        ++count;
    }
    *token_count = count;
    return TEXT_PROTOCOL_STATUS_OK;
}

/**
 * @brief Parses one field token and rejects invalid or duplicate keys.
 * @param request Request receiving the field.
 * @param token Candidate key=value token.
 * @return Detailed field status.
 */
static TextProtocolStatus text_protocol_append_field(
    TextProtocolRequest *request,
    const TextProtocolSpan *token)
{
    size_t separator_index = 0U;
    uint8_t field_index;
    TextProtocolField *field;

    while ((separator_index < token->length) &&
           (token->data[separator_index] != '='))
    {
        ++separator_index;
    }
    if ((separator_index == 0U) ||
        (separator_index >= (token->length - 1U)) ||
        (memchr(&token->data[separator_index + 1U],
                '=',
                token->length - separator_index - 1U) != NULL))
    {
        return TEXT_PROTOCOL_STATUS_BAD_FIELD;
    }
    if (request->field_count >= TEXT_PROTOCOL_MAX_FIELD_COUNT)
    {
        return TEXT_PROTOCOL_STATUS_TOO_MANY_ARGUMENTS;
    }
    field = &request->fields[request->field_count];
    field->key.data = token->data;
    field->key.length = separator_index;
    field->value.data = &token->data[separator_index + 1U];
    field->value.length = token->length - separator_index - 1U;
    if (text_protocol_is_identifier(field->key.data, field->key.length) == 0U)
    {
        return TEXT_PROTOCOL_STATUS_BAD_FIELD;
    }
    text_protocol_lowercase_span(&field->key);
    for (field_index = 0U; field_index < request->field_count; ++field_index)
    {
        if ((request->fields[field_index].key.length == field->key.length) &&
            (strncmp(request->fields[field_index].key.data,
                     field->key.data,
                     field->key.length) == 0))
        {
            return TEXT_PROTOCOL_STATUS_DUPLICATE_FIELD;
        }
    }
    ++request->field_count;
    return TEXT_PROTOCOL_STATUS_OK;
}

/**
 * @brief Parses one optional-ID aethor-text-v1 request line.
 * @param line Input bytes with an optional LF or CRLF terminator.
 * @param length Number of input bytes available at line.
 * @param request Destination request initialized on every non-null call.
 * @return Detailed parse status without allocating memory.
 */
TextProtocolStatus text_protocol_parse_request(const char *line,
                                               size_t length,
                                               TextProtocolRequest *request)
{
    TextProtocolSpan tokens[TEXT_PROTOCOL_MAX_TOKEN_COUNT];
    uint8_t token_count = 0U;
    uint8_t token_index = 0U;
    size_t content_length = length;
    size_t index;
    TextProtocolStatus status;
    uint8_t named_fields_started = 0U;

    if ((line == NULL) || (request == NULL))
    {
        return TEXT_PROTOCOL_STATUS_INVALID_ARGUMENT;
    }
    memset(request, 0, sizeof(*request));
    if ((content_length > 0U) && (line[content_length - 1U] == '\n'))
    {
        --content_length;
    }
    if ((content_length > 0U) && (line[content_length - 1U] == '\r'))
    {
        --content_length;
    }
    if (content_length > TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH)
    {
        return TEXT_PROTOCOL_STATUS_LINE_TOO_LONG;
    }
    for (index = 0U; index < content_length; ++index)
    {
        const uint8_t character = (uint8_t)line[index];

        if ((character < 0x20U) || (character > 0x7EU))
        {
            return TEXT_PROTOCOL_STATUS_BAD_CHARACTER;
        }
    }
    memcpy(request->storage, line, content_length);
    request->storage[content_length] = '\0';
    status = text_protocol_tokenize(request->storage,
                                    content_length,
                                    tokens,
                                    &token_count);
    if (status != TEXT_PROTOCOL_STATUS_OK)
    {
        return status;
    }
    if (token_count == 0U)
    {
        return TEXT_PROTOCOL_STATUS_EMPTY_LINE;
    }

    if ((tokens[0].data[0] >= '0') && (tokens[0].data[0] <= '9'))
    {
        if (text_protocol_parse_request_id(&tokens[0], &request->request_id) == 0U)
        {
            return TEXT_PROTOCOL_STATUS_BAD_REQUEST_ID;
        }
        request->has_request_id = (uint8_t)(request->request_id != 0U);
        token_index = 1U;
    }
    if (token_index >= token_count)
    {
        return TEXT_PROTOCOL_STATUS_BAD_COMMAND;
    }
    if (text_protocol_is_identifier(tokens[token_index].data,
                                    tokens[token_index].length) == 0U)
    {
        return TEXT_PROTOCOL_STATUS_BAD_COMMAND;
    }
    request->command_words[0] = tokens[token_index];
    text_protocol_lowercase_span(&request->command_words[0]);
    request->command_word_count = 1U;
    ++token_index;

    if ((text_protocol_is_single_word_command(&request->command_words[0]) == 0U) &&
        (token_index < token_count) &&
        (memchr(tokens[token_index].data, '=', tokens[token_index].length) == NULL))
    {
        if (text_protocol_is_identifier(tokens[token_index].data,
                                        tokens[token_index].length) == 0U)
        {
            return TEXT_PROTOCOL_STATUS_BAD_COMMAND;
        }
        request->command_words[1] = tokens[token_index];
        text_protocol_lowercase_span(&request->command_words[1]);
        request->command_word_count = 2U;
        ++token_index;
    }

    while (token_index < token_count)
    {
        const TextProtocolSpan *token = &tokens[token_index];

        if (memchr(token->data, '=', token->length) != NULL)
        {
            named_fields_started = 1U;
            status = text_protocol_append_field(request, token);
            if (status != TEXT_PROTOCOL_STATUS_OK)
            {
                return status;
            }
        }
        else
        {
            if (named_fields_started != 0U)
            {
                return TEXT_PROTOCOL_STATUS_BAD_FIELD;
            }
            if (request->positional_count >= TEXT_PROTOCOL_MAX_POSITIONAL_COUNT)
            {
                return TEXT_PROTOCOL_STATUS_TOO_MANY_ARGUMENTS;
            }
            request->positionals[request->positional_count] = *token;
            ++request->positional_count;
        }
        ++token_index;
    }
    return TEXT_PROTOCOL_STATUS_OK;
}

/**
 * @brief Compares a parsed one-word or two-word command path case-insensitively.
 * @param request Parsed request.
 * @param first_word Required first command word.
 * @param second_word Optional second word; null selects a one-word command.
 * @return Nonzero when the complete command path matches.
 */
uint8_t text_protocol_request_path_equals(const TextProtocolRequest *request,
                                          const char *first_word,
                                          const char *second_word)
{
    const uint8_t expected_word_count = (second_word == NULL) ? 1U : 2U;

    if ((request == NULL) || (first_word == NULL) ||
        (request->command_word_count != expected_word_count) ||
        (text_protocol_span_equals_word(&request->command_words[0], first_word) == 0U))
    {
        return 0U;
    }
    if ((second_word != NULL) &&
        (text_protocol_span_equals_word(&request->command_words[1], second_word) == 0U))
    {
        return 0U;
    }
    return 1U;
}

/**
 * @brief Returns one positional token by zero-based index.
 * @param request Parsed request.
 * @param index Zero-based positional index.
 * @param value Destination token span.
 * @return Nonzero when the requested positional exists.
 */
uint8_t text_protocol_get_positional(const TextProtocolRequest *request,
                                     uint8_t index,
                                     TextProtocolSpan *value)
{
    if ((request == NULL) || (value == NULL) ||
        (index >= request->positional_count))
    {
        return 0U;
    }
    *value = request->positionals[index];
    return 1U;
}

/**
 * @brief Finds one exact field name in a parsed request.
 * @param request Parsed request.
 * @param key Lowercase field name to find.
 * @param value Destination value span.
 * @return Nonzero when the field exists.
 */
uint8_t text_protocol_find_field(const TextProtocolRequest *request,
                                 const char *key,
                                 TextProtocolSpan *value)
{
    uint8_t field_index;

    if ((request == NULL) || (key == NULL) || (value == NULL))
    {
        return 0U;
    }
    for (field_index = 0U; field_index < request->field_count; ++field_index)
    {
        if (text_protocol_span_equals_word(&request->fields[field_index].key,
                                           key) != 0U)
        {
            *value = request->fields[field_index].value;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief Converts a strict decimal span without accepting exponent or NaN syntax.
 * @param span Decimal token to convert.
 * @param value Destination finite float.
 * @return OK for one complete finite decimal, otherwise BAD_NUMBER.
 */
TextProtocolStatus text_protocol_span_to_float(const TextProtocolSpan *span,
                                               float *value)
{
    char buffer[TEXT_PROTOCOL_FLOAT_BUFFER_CAPACITY];
    char *conversion_end;
    size_t index = 0U;
    uint8_t digit_seen = 0U;
    uint8_t decimal_point_seen = 0U;
    float converted_value;

    if ((span == NULL) || (value == NULL) || (span->data == NULL) ||
        (span->length == 0U) ||
        (span->length >= TEXT_PROTOCOL_FLOAT_BUFFER_CAPACITY))
    {
        return TEXT_PROTOCOL_STATUS_BAD_NUMBER;
    }
    if ((span->data[index] == '+') || (span->data[index] == '-'))
    {
        ++index;
    }
    for (; index < span->length; ++index)
    {
        const char character = span->data[index];

        if ((character >= '0') && (character <= '9'))
        {
            digit_seen = 1U;
        }
        else if ((character == '.') && (decimal_point_seen == 0U))
        {
            decimal_point_seen = 1U;
        }
        else
        {
            return TEXT_PROTOCOL_STATUS_BAD_NUMBER;
        }
    }
    if (digit_seen == 0U)
    {
        return TEXT_PROTOCOL_STATUS_BAD_NUMBER;
    }
    memcpy(buffer, span->data, span->length);
    buffer[span->length] = '\0';
    errno = 0;
    converted_value = strtof(buffer, &conversion_end);
    if ((errno == ERANGE) || (conversion_end != &buffer[span->length]) ||
        (converted_value != converted_value) ||
        (converted_value > FLT_MAX) || (converted_value < -FLT_MAX))
    {
        return TEXT_PROTOCOL_STATUS_BAD_NUMBER;
    }
    *value = converted_value;
    return TEXT_PROTOCOL_STATUS_OK;
}
