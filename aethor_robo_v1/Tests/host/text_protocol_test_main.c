/**
 * @file text_protocol_test_main.c
 * @brief Specifies the bounded aethor-text-v1 line codec contract.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "text_protocol.h"

/**
 * @brief Verifies a manual query receives request ID zero and a two-word path.
 */
static void test_text_protocol_parses_manual_query(void)
{
    static const char line[] = "show state\r\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 0U);
    assert(request.has_request_id == 0U);
    assert(text_protocol_request_path_equals(&request, "show", "state") != 0U);
    assert(request.positional_count == 0U);
    assert(request.field_count == 0U);
}

/**
 * @brief Verifies a numbered arm command exposes positional and named values.
 */
static void test_text_protocol_parses_numbered_arm_move(void)
{
    static const char line[] =
        "42 arm move 0,-15,30,0,20,0,5 speed=5\n";
    TextProtocolRequest request;
    TextProtocolSpan position_list;
    TextProtocolSpan speed;
    float speed_value = 0.0F;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 42U);
    assert(request.has_request_id != 0U);
    assert(text_protocol_request_path_equals(&request, "arm", "move") != 0U);
    assert(text_protocol_get_positional(&request, 0U, &position_list) != 0U);
    assert(position_list.length == strlen("0,-15,30,0,20,0,5"));
    assert(strncmp(position_list.data,
                   "0,-15,30,0,20,0,5",
                   position_list.length) == 0);
    assert(text_protocol_find_field(&request, "speed", &speed) != 0U);
    assert(text_protocol_span_to_float(&speed, &speed_value) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(speed_value == 5.0F);
}

/**
 * @brief Verifies command words are case-insensitive and output storage is normalized.
 */
static void test_text_protocol_normalizes_command_case_and_spaces(void)
{
    static const char line[] = "  7   ShOw   MoToR   1   \n";
    TextProtocolRequest request;
    TextProtocolSpan motor_index;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 7U);
    assert(text_protocol_request_path_equals(&request, "show", "motor") != 0U);
    assert(text_protocol_get_positional(&request, 0U, &motor_index) != 0U);
    assert(motor_index.length == 1U);
    assert(motor_index.data[0] == '1');
}

/**
 * @brief Verifies a single-word command path does not consume an argument as a verb.
 */
static void test_text_protocol_parses_single_word_help_topic(void)
{
    static const char line[] = "help arm\n";
    TextProtocolRequest request;
    TextProtocolSpan topic;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(text_protocol_request_path_equals(&request, "help", NULL) != 0U);
    assert(text_protocol_get_positional(&request, 0U, &topic) != 0U);
    assert(topic.length == 3U);
    assert(strncmp(topic.data, "arm", topic.length) == 0);
}

/**
 * @brief Verifies duplicate fields are rejected instead of silently overwritten.
 */
static void test_text_protocol_rejects_duplicate_fields(void)
{
    static const char line[] = "8 stream joints rate=20 rate=30\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_DUPLICATE_FIELD);
}

/**
 * @brief Verifies control characters other than accepted line endings are rejected.
 */
static void test_text_protocol_rejects_tab(void)
{
    static const char line[] = "show\tstate\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_BAD_CHARACTER);
}

/**
 * @brief Verifies request lines longer than the public bound are rejected.
 */
static void test_text_protocol_rejects_overlength_line(void)
{
    char line[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 3U];
    TextProtocolRequest request;

    memset(line, 'a', sizeof(line));
    line[sizeof(line) - 2U] = '\n';
    line[sizeof(line) - 1U] = '\0';
    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_LINE_TOO_LONG);
}

/**
 * @brief Verifies request IDs must fit in uint32 and zero stays manual-only.
 */
static void test_text_protocol_validates_request_id(void)
{
    static const char overflow_line[] = "4294967296 show state\n";
    static const char zero_line[] = "0 show state\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(overflow_line,
                                       sizeof(overflow_line) - 1U,
                                       &request) ==
           TEXT_PROTOCOL_STATUS_BAD_REQUEST_ID);
    assert(text_protocol_parse_request(zero_line,
                                       sizeof(zero_line) - 1U,
                                       &request) == TEXT_PROTOCOL_STATUS_OK);
    assert(request.request_id == 0U);
    assert(request.has_request_id == 0U);
}

/**
 * @brief Verifies decimal parsing rejects scientific notation and non-finite tokens.
 */
static void test_text_protocol_float_conversion_is_strict(void)
{
    static const char scientific_text[] = "1e2";
    static const char nan_text[] = "nan";
    static const char signed_decimal_text[] = "-3.25";
    static const char positive_fraction_text[] = "+.5";
    static const char trailing_decimal_text[] = "5.";
    static const char decimal_point_only_text[] = ".";
    TextProtocolSpan scientific = {scientific_text, sizeof(scientific_text) - 1U};
    TextProtocolSpan nan_value = {nan_text, sizeof(nan_text) - 1U};
    TextProtocolSpan signed_decimal = {
        signed_decimal_text,
        sizeof(signed_decimal_text) - 1U
    };
    TextProtocolSpan positive_fraction = {
        positive_fraction_text,
        sizeof(positive_fraction_text) - 1U
    };
    TextProtocolSpan trailing_decimal = {
        trailing_decimal_text,
        sizeof(trailing_decimal_text) - 1U
    };
    TextProtocolSpan decimal_point_only = {
        decimal_point_only_text,
        sizeof(decimal_point_only_text) - 1U
    };
    char maximum_token[64];
    char overlength_token[65];
    TextProtocolSpan maximum_length_decimal;
    TextProtocolSpan overlength_decimal;
    float value = 0.0F;

    memset(maximum_token, '0', sizeof(maximum_token));
    maximum_token[62] = '1';
    maximum_token[63] = '\0';
    maximum_length_decimal.data = maximum_token;
    maximum_length_decimal.length = 63U;
    memset(overlength_token, '0', sizeof(overlength_token));
    overlength_token[63] = '1';
    overlength_token[64] = '\0';
    overlength_decimal.data = overlength_token;
    overlength_decimal.length = 64U;

    assert(text_protocol_span_to_float(&scientific, &value) ==
           TEXT_PROTOCOL_STATUS_BAD_NUMBER);
    assert(text_protocol_span_to_float(&nan_value, &value) ==
           TEXT_PROTOCOL_STATUS_BAD_NUMBER);
    assert(text_protocol_span_to_float(&signed_decimal, &value) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(value == -3.25F);
    assert(text_protocol_span_to_float(&positive_fraction, &value) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(value == 0.5F);
    assert(text_protocol_span_to_float(&trailing_decimal, &value) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(value == 5.0F);
    assert(text_protocol_span_to_float(&decimal_point_only, &value) ==
           TEXT_PROTOCOL_STATUS_BAD_NUMBER);
    assert(text_protocol_span_to_float(&maximum_length_decimal, &value) ==
           TEXT_PROTOCOL_STATUS_OK);
    assert(value == 1.0F);
    assert(text_protocol_span_to_float(&overlength_decimal, &value) ==
           TEXT_PROTOCOL_STATUS_BAD_NUMBER);
}

/**
 * @brief Verifies the parser refuses more than the fixed positional capacity.
 */
static void test_text_protocol_rejects_too_many_positionals(void)
{
    static const char line[] = "7 show motor 1 extra unexpected\n";
    TextProtocolRequest request;

    assert(text_protocol_parse_request(line, sizeof(line) - 1U, &request) ==
           TEXT_PROTOCOL_STATUS_TOO_MANY_ARGUMENTS);
}

/**
 * @brief Runs the complete text codec contract suite.
 * @return Zero when every assertion passes.
 */
int main(void)
{
    test_text_protocol_parses_manual_query();
    test_text_protocol_parses_numbered_arm_move();
    test_text_protocol_normalizes_command_case_and_spaces();
    test_text_protocol_parses_single_word_help_topic();
    test_text_protocol_rejects_duplicate_fields();
    test_text_protocol_rejects_tab();
    test_text_protocol_rejects_overlength_line();
    test_text_protocol_validates_request_id();
    test_text_protocol_float_conversion_is_strict();
    test_text_protocol_rejects_too_many_positionals();
    puts("TEXT_PROTOCOL_TESTS_PASSED");
    return 0;
}
