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
#include "app_profile.h"
#include "board_config.h"
#include "joint_motion.h"
#include "s3519_codec.h"

#define PROTOCOL_ENGINE_RAD_TO_DEG (57.29577951308232F)
#define PROTOCOL_ENGINE_DEG_TO_RAD (0.017453292519943295F)
#define PROTOCOL_ENGINE_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define PROTOCOL_ENGINE_FLOAT_TOKEN_CAPACITY (32U)
#define PROTOCOL_ENGINE_TEXT_STREAM_OFF (0U)
#define PROTOCOL_ENGINE_TEXT_STREAM_JOINTS (1U)
#define PROTOCOL_ENGINE_TEXT_STREAM_MOTORS (2U)

#ifndef AETHOR_BUILD_DATE_YYYYMMDD
#define AETHOR_BUILD_DATE_YYYYMMDD "unknown"
#endif

/** @brief Selects one joint configuration vector for deterministic formatting. */
typedef enum
{
    PROTOCOL_CONFIG_Q_MIN = 0,
    PROTOCOL_CONFIG_Q_MAX,
    PROTOCOL_CONFIG_V_LIMIT,
    PROTOCOL_CONFIG_A_LIMIT
} ProtocolConfigVector;

/** @brief Creates a nonzero connection-scope identifier for internal ownership. */
static uint32_t protocol_engine_create_session(ProtocolEngine *engine,
                                               uint32_t request_id,
                                               uint64_t timestamp_us);

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

#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
/** @brief Reports whether a parsed request contains one named field. */
static uint8_t protocol_engine_request_has_field(
    const AsciiProtocolRequest *request,
    const char *key)
{
    AsciiProtocolSpan ignored_value;

    return ascii_protocol_find_field(request, key, &ignored_value);
}
#endif

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

#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
/** @brief Parses a unique ascending comma list of motor numbers. */
static uint8_t protocol_engine_parse_motor_mask(const AsciiProtocolSpan *span,
                                                uint8_t *motor_mask)
{
    uint16_t token_start = 0U;
    uint16_t character_index;
    uint8_t parsed_mask = 0U;
    uint32_t previous_motor_number = 0U;

    if ((span == NULL) || (motor_mask == NULL) || (span->length == 0U))
    {
        return 0U;
    }
    for (character_index = 0U; character_index <= span->length; ++character_index)
    {
        if ((character_index == span->length) ||
            (span->data[character_index] == ','))
        {
            AsciiProtocolSpan token;
            uint32_t motor_number;
            uint8_t motor_bit;

            token.data = &span->data[token_start];
            token.length = (uint16_t)(character_index - token_start);
            if ((protocol_engine_parse_u32(&token, &motor_number) == 0U) ||
                (motor_number < 1U) || (motor_number > ARM_JOINT_COUNT) ||
                (motor_number <= previous_motor_number))
            {
                return 0U;
            }
            motor_bit = (uint8_t)(1U << (motor_number - 1U));
            if ((parsed_mask & motor_bit) != 0U)
            {
                return 0U;
            }
            parsed_mask |= motor_bit;
            previous_motor_number = motor_number;
            token_start = (uint16_t)(character_index + 1U);
        }
    }
    *motor_mask = parsed_mask;
    return (uint8_t)(parsed_mask != 0U);
}

/** @brief Parses one float per selected motor into joint-indexed storage. */
static uint8_t protocol_engine_parse_selected_motor_values(
    const AsciiProtocolSpan *span,
    uint8_t motor_mask,
    float values[ARM_JOINT_COUNT])
{
    uint16_t token_start = 0U;
    uint16_t character_index;
    uint8_t next_joint_index = 0U;
    uint8_t parsed_count = 0U;

    if ((span == NULL) || (values == NULL) || (motor_mask == 0U) ||
        (span->length == 0U))
    {
        return 0U;
    }
    memset(values, 0, sizeof(float) * ARM_JOINT_COUNT);
    for (character_index = 0U; character_index <= span->length; ++character_index)
    {
        if ((character_index == span->length) ||
            (span->data[character_index] == ','))
        {
            AsciiProtocolSpan token;
            float parsed_value;

            while ((next_joint_index < ARM_JOINT_COUNT) &&
                   ((motor_mask & (uint8_t)(1U << next_joint_index)) == 0U))
            {
                ++next_joint_index;
            }
            if (next_joint_index >= ARM_JOINT_COUNT)
            {
                return 0U;
            }
            token.data = &span->data[token_start];
            token.length = (uint16_t)(character_index - token_start);
            if (protocol_engine_parse_float(&token, &parsed_value) == 0U)
            {
                return 0U;
            }
            values[next_joint_index] = parsed_value;
            ++next_joint_index;
            ++parsed_count;
            token_start = (uint16_t)(character_index + 1U);
        }
    }
    for (next_joint_index = 0U;
         next_joint_index < ARM_JOINT_COUNT;
         ++next_joint_index)
    {
        if ((motor_mask & (uint8_t)(1U << next_joint_index)) != 0U)
        {
            --parsed_count;
        }
    }
    return (uint8_t)(parsed_count == 0U);
}
#endif

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
 * @brief Appends one bounded LF-terminated aethor-text-v1 response.
 */
static ProtocolEngineStatus protocol_engine_append_text_format(
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *format,
    ...)
{
    ProtocolOutputMessage *message;
    int written_length;
    va_list arguments;

    if ((output_batch == NULL) || (format == NULL) ||
        (output_batch->count >= PROTOCOL_ENGINE_MAX_OUTPUT_COUNT))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    message = &output_batch->messages[output_batch->count];
    va_start(arguments, format);
    written_length = vsnprintf(message->data,
                               TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH,
                               format,
                               arguments);
    va_end(arguments);
    if ((written_length < 0) ||
        ((size_t)written_length >= TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH) ||
        (((size_t)written_length + 1U) > TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH))
    {
        message->data[0] = '\0';
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    message->data[written_length] = '\n';
    message->data[written_length + 1] = '\0';
    message->length = (uint16_t)((size_t)written_length + 1U);
    message->priority = priority;
    ++output_batch->count;
    return PROTOCOL_ENGINE_STATUS_OK;
}

/**
 * @brief Appends one prebuilt bounded body as an LF-terminated text response.
 */
static ProtocolEngineStatus protocol_engine_append_text_body(
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *body)
{
    ProtocolOutputMessage *message;
    size_t body_length;

    if ((output_batch == NULL) || (body == NULL) ||
        (output_batch->count >= PROTOCOL_ENGINE_MAX_OUTPUT_COUNT))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    body_length = strlen(body);
    if ((body_length + 1U) > TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    message = &output_batch->messages[output_batch->count];
    memcpy(message->data, body, body_length);
    message->data[body_length] = '\n';
    message->data[body_length + 1U] = '\0';
    message->length = (uint16_t)(body_length + 1U);
    message->priority = priority;
    ++output_batch->count;
    return PROTOCOL_ENGINE_STATUS_OK;
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
 * @brief Appends one finite fixed-point value while trimming redundant zeros.
 */
static uint8_t protocol_engine_append_compact_float(char *body,
                                                    size_t body_capacity,
                                                    size_t *body_length,
                                                    float value)
{
    char token[PROTOCOL_ENGINE_FLOAT_TOKEN_CAPACITY];
    size_t token_length;
    int written_length;

    if ((value > -0.0005F) && (value < 0.0005F))
    {
        value = 0.0F;
    }
    written_length = snprintf(token, sizeof(token), "%.3f", value);
    if ((written_length < 0) || ((size_t)written_length >= sizeof(token)))
    {
        return 0U;
    }
    token_length = (size_t)written_length;
    while ((token_length > 0U) && (token[token_length - 1U] == '0'))
    {
        --token_length;
    }
    if ((token_length > 0U) && (token[token_length - 1U] == '.'))
    {
        --token_length;
    }
    token[token_length] = '\0';
    return protocol_engine_append_text(body,
                                       body_capacity,
                                       body_length,
                                       "%s",
                                       token);
}

/** @brief Appends seven compact public joint values separated by commas. */
static uint8_t protocol_engine_append_text_joint_vector(
    char *body,
    size_t body_capacity,
    size_t *body_length,
    const float values[ARM_JOINT_COUNT])
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (((joint_index != 0U) &&
             (protocol_engine_append_text(body,
                                          body_capacity,
                                          body_length,
                                          ",") == 0U)) ||
            (protocol_engine_append_compact_float(body,
                                                  body_capacity,
                                                  body_length,
                                                  values[joint_index]) == 0U))
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Parses one complete unsigned decimal text-protocol span. */
static uint8_t protocol_engine_parse_text_u32(const TextProtocolSpan *span,
                                              uint32_t *value)
{
    uint32_t parsed_value = 0U;
    size_t character_index;

    if ((span == NULL) || (value == NULL) || (span->length == 0U))
    {
        return 0U;
    }
    for (character_index = 0U; character_index < span->length; ++character_index)
    {
        uint32_t digit;

        if ((span->data[character_index] < '0') ||
            (span->data[character_index] > '9'))
        {
            return 0U;
        }
        digit = (uint32_t)(span->data[character_index] - '0');
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

/** @brief Returns lowercase aethor-text-v1 text for one controller state. */
static const char *protocol_engine_text_arm_state(ArmState state)
{
    switch (state)
    {
        case ARM_STATE_BOOT:
            return "boot";
        case ARM_STATE_SELF_TEST:
            return "self_test";
        case ARM_STATE_UNALIGNED:
            return "unaligned";
        case ARM_STATE_DISABLED:
            return "disabled";
        case ARM_STATE_ENABLING:
            return "enabling";
        case ARM_STATE_READY:
            return "ready";
        case ARM_STATE_MOVING:
            return "moving";
        case ARM_STATE_STOPPING:
            return "stopping";
        case ARM_STATE_FAULT:
        default:
            return "fault";
    }
}

/** @brief Returns lowercase aethor-text-v1 text for one controller fault. */
static const char *protocol_engine_text_arm_fault(ArmFault fault)
{
    switch (fault)
    {
        case ARM_FAULT_NONE:
            return "none";
        case ARM_FAULT_CONFIG_INVALID:
            return "config_invalid";
        case ARM_FAULT_CONFIG_INCOMPLETE:
            return "config_incomplete";
        case ARM_FAULT_LINK_TIMEOUT:
            return "link_timeout";
        case ARM_FAULT_MOTION_CONTROL:
            return "motion_control";
        case ARM_FAULT_TRANSPORT:
            return "transport";
        case ARM_FAULT_CONTROL_DEADLINE:
            return "control_deadline";
        case ARM_FAULT_DRIVER:
            return "driver";
        case ARM_FAULT_FEEDBACK_STALE:
        default:
            return "feedback_stale";
    }
}

/**
 * @brief Derives readable aggregate motor masks from the current snapshot.
 */
static void protocol_engine_text_motor_masks(
    const ProtocolQueryContext *query_context,
    uint8_t *enabled_mask,
    uint8_t *moving_mask,
    uint8_t *holding_mask,
    uint8_t *fault_mask)
{
    uint8_t joint_index;

    *enabled_mask = 0U;
    *moving_mask = 0U;
    *holding_mask = 0U;
    *fault_mask = 0U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const uint8_t joint_bit = (uint8_t)(1U << joint_index);
        const MotorJointFeedback *feedback =
            &query_context->motors.joints[joint_index];

        if ((query_context->motors.valid_joint_mask & joint_bit) == 0U)
        {
            continue;
        }
        if ((feedback->driver_state >= S3519_DRIVER_STATE_FAULT_MINIMUM) ||
            (feedback->fault_flags != 0U))
        {
            *fault_mask |= joint_bit;
        }
        if (feedback->driver_state == S3519_DRIVER_STATE_ENABLED)
        {
            *enabled_mask |= joint_bit;
            if ((feedback->velocity_rad_s > 0.001F) ||
                (feedback->velocity_rad_s < -0.001F))
            {
                *moving_mask |= joint_bit;
            }
            else
            {
                *holding_mask |= joint_bit;
            }
        }
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
 * @brief Stores one nonzero aethor-text-v1 request result for exact replay.
 */
static void protocol_engine_store_text_recent(
    ProtocolEngine *engine,
    uint32_t request_id,
    uint32_t body_hash,
    uint64_t timestamp_us,
    const ProtocolOutputBatch *output_batch)
{
    ProtocolRecentResult *result;

    if ((request_id == 0U) || (output_batch->count != 1U))
    {
        return;
    }
    result = &engine->recent_results[engine->recent_write_index];
    memset(result, 0, sizeof(*result));
    result->request_id = request_id;
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
 * @brief Appends one parsed token to a single-space canonical request body.
 */
static uint8_t protocol_engine_append_canonical_span(
    char canonical[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 1U],
    size_t *canonical_length,
    const TextProtocolSpan *span,
    uint8_t prepend_space)
{
    size_t required_length;

    if ((canonical == NULL) || (canonical_length == NULL) || (span == NULL))
    {
        return 0U;
    }
    required_length = *canonical_length + span->length +
                      ((prepend_space != 0U) ? 1U : 0U);
    if (required_length > TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH)
    {
        return 0U;
    }
    if (prepend_space != 0U)
    {
        canonical[*canonical_length] = ' ';
        ++(*canonical_length);
    }
    memcpy(&canonical[*canonical_length], span->data, span->length);
    *canonical_length += span->length;
    canonical[*canonical_length] = '\0';
    return 1U;
}

/**
 * @brief Builds the normalized command body used for request-ID conflict checks.
 */
static uint8_t protocol_engine_build_text_canonical_body(
    const TextProtocolRequest *request,
    char canonical[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 1U],
    size_t *canonical_length)
{
    uint8_t index;

    if ((request == NULL) || (canonical == NULL) || (canonical_length == NULL))
    {
        return 0U;
    }
    canonical[0] = '\0';
    *canonical_length = 0U;
    for (index = 0U; index < request->command_word_count; ++index)
    {
        if (protocol_engine_append_canonical_span(
                canonical,
                canonical_length,
                &request->command_words[index],
                (uint8_t)(*canonical_length != 0U)) == 0U)
        {
            return 0U;
        }
    }
    for (index = 0U; index < request->positional_count; ++index)
    {
        if (protocol_engine_append_canonical_span(canonical,
                                                  canonical_length,
                                                  &request->positionals[index],
                                                  1U) == 0U)
        {
            return 0U;
        }
    }
    for (index = 0U; index < request->field_count; ++index)
    {
        const TextProtocolField *field = &request->fields[index];
        static const TextProtocolSpan separator = {"=", 1U};

        if ((protocol_engine_append_canonical_span(canonical,
                                                   canonical_length,
                                                   &field->key,
                                                   1U) == 0U) ||
            (protocol_engine_append_canonical_span(canonical,
                                                   canonical_length,
                                                   &separator,
                                                   0U) == 0U) ||
            (protocol_engine_append_canonical_span(canonical,
                                                   canonical_length,
                                                   &field->value,
                                                   0U) == 0U))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Formats one error using the parsed command path as its operation name.
 */
static ProtocolEngineStatus protocol_engine_append_text_command_error(
    ProtocolOutputBatch *output_batch,
    const TextProtocolRequest *request,
    const char *error_code)
{
    if (request->command_word_count == 2U)
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_HIGH_PRIORITY,
            "error %lu %.*s %.*s code=%s",
            (unsigned long)request->request_id,
            (int)request->command_words[0].length,
            request->command_words[0].data,
            (int)request->command_words[1].length,
            request->command_words[1].data,
            error_code);
    }
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_HIGH_PRIORITY,
        "error %lu %.*s code=%s",
        (unsigned long)request->request_id,
        (int)request->command_words[0].length,
        request->command_words[0].data,
        error_code);
}

/**
 * @brief Starts one aethor-text-v1 connection scope and returns device identity.
 */
static ProtocolEngineStatus protocol_engine_handle_text_hello(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    const BuildInfo *build_info = build_info_get();
#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
    static const char profile[] = "bench";
#else
    static const char profile[] = "arm";
#endif

    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        return protocol_engine_append_text_command_error(output_batch,
                                                         request,
                                                         "bad_argument");
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
    engine->stream_rate_hz = 0U;
    engine->text_stream_kind = PROTOCOL_ENGINE_TEXT_STREAM_OFF;
    engine->text_protocol_active = 1U;
    engine->next_telemetry_due_us = timestamp_us;
    engine->next_motor_telemetry_due_us = timestamp_us;
    engine->telemetry_sequence = 0U;
    engine->event_sequence = 0U;
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "ok %lu hello protocol=aethor-text-v1 fw=%s profile=%s dof=%u boot=%lu watchdog_ms=%lu",
        (unsigned long)request->request_id,
        build_info->firmware_version,
        profile,
        (unsigned int)ARM_JOINT_COUNT,
        (unsigned long)engine->boot_id,
        (unsigned long)(PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US / 1000ULL));
}

/** @brief Returns the minimal controller state used as a manual keepalive. */
static ProtocolEngineStatus protocol_engine_handle_text_ping(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    ArmState state = ARM_STATE_BOOT;
    uint8_t enabled_mask = 0U;

    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        return protocol_engine_append_text_command_error(output_batch,
                                                         request,
                                                         "bad_argument");
    }
    if (engine->query_context_valid != 0U)
    {
        state = engine->query_context.arm.state;
        if (engine->query_context.arm.enabled != 0U)
        {
            enabled_mask = PROTOCOL_ENGINE_ALL_JOINTS_MASK;
        }
    }
    if (engine->session_active != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine->watchdog_timeout_reported = 0U;
    }
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "ok %lu ping state=%s enabled=%02x boot=%lu",
        (unsigned long)request->request_id,
        protocol_engine_text_arm_state(state),
        (unsigned int)enabled_mask,
        (unsigned long)engine->boot_id);
}

/** @brief Returns immutable board, transport, motor, and build identity. */
static ProtocolEngineStatus protocol_engine_handle_text_show_info(
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "ok %lu show info mcu=%s transport=usb_cdc can=classic_1m motor=s3519 driver=dm3520 build=%s",
        (unsigned long)request->request_id,
        BOARD_MCU_PART_NUMBER,
        AETHOR_BUILD_DATE_YYYYMMDD);
}

/** @brief Returns the concise public controller state snapshot. */
static ProtocolEngineStatus protocol_engine_handle_text_show_state(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    const ArmSnapshot *arm;
    uint8_t enabled_mask;

    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "unavailable");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    arm = &engine->query_context.arm;
    enabled_mask = (arm->enabled != 0U) ? PROTOCOL_ENGINE_ALL_JOINTS_MASK : 0U;
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "ok %lu show state state=%s aligned=%u enabled=%02x moving=%u active=%lu fault=%s",
        (unsigned long)request->request_id,
        protocol_engine_text_arm_state(arm->state),
        (unsigned int)arm->aligned,
        (unsigned int)enabled_mask,
        (unsigned int)arm->moving,
        (unsigned long)engine->active_motion_request_id,
        protocol_engine_text_arm_fault(arm->fault));
}

/** @brief Appends one concise joint snapshot for queries or data streaming. */
static ProtocolEngineStatus protocol_engine_append_text_joints_snapshot(
    ProtocolEngine *engine,
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *prefix,
    uint8_t include_state)
{
    char body[TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH];
    size_t body_length = 0U;

    if ((protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     "%s t_ms=%lu q=",
                                     prefix,
                                     (unsigned long)(engine->query_context.joints
                                                         .published_at_us /
                                                     1000ULL)) == 0U) ||
        (protocol_engine_append_text_joint_vector(
             body,
             sizeof(body),
             &body_length,
             engine->query_context.joints.position_deg) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " qd=") == 0U) ||
        (protocol_engine_append_text_joint_vector(
             body,
             sizeof(body),
             &body_length,
             engine->query_context.joints.velocity_deg_s) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " valid=%02x",
                                     (unsigned int)engine->query_context.joints
                                         .valid_joint_mask) == 0U))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    if ((include_state != 0U) &&
        (protocol_engine_append_text(
             body,
             sizeof(body),
             &body_length,
             " state=%s",
             protocol_engine_text_arm_state(engine->query_context.arm.state)) == 0U))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_text_body(output_batch, priority, body);
}

/** @brief Returns the current seven-axis public joint snapshot. */
static ProtocolEngineStatus protocol_engine_handle_text_show_joints(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    char prefix[48];
    int written_length;

    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "unavailable");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    written_length = snprintf(prefix,
                              sizeof(prefix),
                              "ok %lu show joints",
                              (unsigned long)request->request_id);
    if ((written_length < 0) || ((size_t)written_length >= sizeof(prefix)))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_text_joints_snapshot(engine,
                                                       output_batch,
                                                       PROTOCOL_OUTPUT_QUERY,
                                                       prefix,
                                                       0U);
}

/** @brief Appends aggregate motor masks for a query or stream sample. */
static ProtocolEngineStatus protocol_engine_append_text_motor_summary(
    ProtocolEngine *engine,
    ProtocolOutputBatch *output_batch,
    ProtocolOutputPriority priority,
    const char *prefix)
{
    uint8_t enabled_mask;
    uint8_t moving_mask;
    uint8_t holding_mask;
    uint8_t fault_mask;

    protocol_engine_text_motor_masks(&engine->query_context,
                                     &enabled_mask,
                                     &moving_mask,
                                     &holding_mask,
                                     &fault_mask);
    return protocol_engine_append_text_format(
        output_batch,
        priority,
        "%s present=%02x enabled=%02x moving=%02x holding=%02x stale=00 fault=%02x",
        prefix,
        (unsigned int)engine->query_context.motors.valid_joint_mask,
        (unsigned int)enabled_mask,
        (unsigned int)moving_mask,
        (unsigned int)holding_mask,
        (unsigned int)fault_mask);
}

/** @brief Returns concise aggregate motor status masks. */
static ProtocolEngineStatus protocol_engine_handle_text_show_motors(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    char prefix[48];
    int written_length;

    if ((request->positional_count != 0U) || (request->field_count != 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "unavailable");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    written_length = snprintf(prefix,
                              sizeof(prefix),
                              "ok %lu show motors",
                              (unsigned long)request->request_id);
    if ((written_length < 0) || ((size_t)written_length >= sizeof(prefix)))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_text_motor_summary(engine,
                                                     output_batch,
                                                     PROTOCOL_OUTPUT_QUERY,
                                                     prefix);
}

/** @brief Returns a stable lowercase lifecycle label for one motor sample. */
static const char *protocol_engine_text_motor_state(
    const MotorJointFeedback *feedback,
    uint8_t valid)
{
    if (valid == 0U)
    {
        return "absent";
    }
    if ((feedback->driver_state >= S3519_DRIVER_STATE_FAULT_MINIMUM) ||
        (feedback->fault_flags != 0U))
    {
        return "fault";
    }
    if (feedback->driver_state != S3519_DRIVER_STATE_ENABLED)
    {
        return "disabled";
    }
    if ((feedback->velocity_rad_s > 0.001F) ||
        (feedback->velocity_rad_s < -0.001F))
    {
        return "moving";
    }
    return "holding";
}

/** @brief Returns one detailed motor feedback sample without guessing parameters. */
static ProtocolEngineStatus protocol_engine_handle_text_show_motor(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    TextProtocolSpan joint_span;
    const JointConfig *joint_configuration;
    const MotorJointFeedback *feedback;
    char body[TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH];
    size_t body_length = 0U;
    uint32_t joint_number;
    uint32_t age_ms = UINT32_MAX;
    uint8_t joint_bit;
    uint8_t valid;

    if ((request->positional_count != 1U) || (request->field_count != 0U) ||
        (text_protocol_get_positional(request, 0U, &joint_span) == 0U) ||
        (protocol_engine_parse_text_u32(&joint_span, &joint_number) == 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((joint_number < 1U) || (joint_number > ARM_JOINT_COUNT))
    {
        (void)protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_HIGH_PRIORITY,
            "error %lu show motor code=out_of_range field=joint",
            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (engine->query_context_valid == 0U)
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "unavailable");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    joint_configuration = &engine->configuration->joints[joint_number - 1U];
    feedback = &engine->query_context.motors.joints[joint_number - 1U];
    joint_bit = (uint8_t)(1U << (joint_number - 1U));
    valid = (uint8_t)((engine->query_context.motors.valid_joint_mask & joint_bit) != 0U);
    if ((feedback->timestamp_us != 0U) && (timestamp_us >= feedback->timestamp_us))
    {
        const uint64_t age_value = (timestamp_us - feedback->timestamp_us) / 1000ULL;

        age_ms = (age_value > UINT32_MAX) ? UINT32_MAX : (uint32_t)age_value;
    }
    if (protocol_engine_append_text(
            body,
            sizeof(body),
            &body_length,
            "ok %lu show motor joint=%lu esc=%02x master=%02x state=%s pos_deg=",
            (unsigned long)request->request_id,
            (unsigned long)joint_number,
            (unsigned int)joint_configuration->esc_id,
            (unsigned int)joint_configuration->master_id,
            protocol_engine_text_motor_state(feedback, valid)) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    if ((protocol_engine_append_compact_float(body,
                                              sizeof(body),
                                              &body_length,
                                              feedback->position_rad *
                                                  PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " speed_deg_s=") == 0U) ||
        (protocol_engine_append_compact_float(body,
                                              sizeof(body),
                                              &body_length,
                                              feedback->velocity_rad_s *
                                                  PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " torque_nm=") == 0U) ||
        (protocol_engine_append_compact_float(body,
                                              sizeof(body),
                                              &body_length,
                                              feedback->torque_nm) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " mos_c=") == 0U) ||
        (protocol_engine_append_compact_float(body,
                                              sizeof(body),
                                              &body_length,
                                              feedback->mos_temperature_c) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " rotor_c=") == 0U) ||
        (protocol_engine_append_compact_float(body,
                                              sizeof(body),
                                              &body_length,
                                              feedback->rotor_temperature_c) == 0U) ||
        (protocol_engine_append_text(body,
                                     sizeof(body),
                                     &body_length,
                                     " fault=%lu age_ms=%lu",
                                     (unsigned long)feedback->fault_flags,
                                     (unsigned long)age_ms) == 0U))
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
    }
    return protocol_engine_append_text_body(output_batch,
                                            PROTOCOL_OUTPUT_QUERY,
                                            body);
}

/** @brief Computes a deterministic checksum over the configured CAN map. */
static uint32_t protocol_engine_text_config_map_hash(
    const ArmConfig *configuration)
{
    char canonical_map[128];
    size_t canonical_length = 0U;
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint = &configuration->joints[joint_index];

        if (protocol_engine_append_text(canonical_map,
                                        sizeof(canonical_map),
                                        &canonical_length,
                                        "%s%u:%u:%u",
                                        (joint_index == 0U) ? "" : ";",
                                        (unsigned int)joint->joint_index,
                                        (unsigned int)joint->esc_id,
                                        (unsigned int)joint->master_id) == 0U)
        {
            return 0U;
        }
    }
    return ascii_protocol_crc32_iso_hdlc((const uint8_t *)canonical_map,
                                         canonical_length);
}

/** @brief Appends one configuration value or an explicit unknown marker. */
static uint8_t protocol_engine_append_text_config_float(
    char *body,
    size_t body_capacity,
    size_t *body_length,
    const char *key,
    uint8_t verified,
    float value)
{
    if (protocol_engine_append_text(body,
                                    body_capacity,
                                    body_length,
                                    " %s=",
                                    key) == 0U)
    {
        return 0U;
    }
    if (verified == 0U)
    {
        return protocol_engine_append_text(body,
                                           body_capacity,
                                           body_length,
                                           "?");
    }
    return protocol_engine_append_compact_float(body,
                                                body_capacity,
                                                body_length,
                                                value);
}

/** @brief Returns the configuration summary or one joint's verified fields. */
static ProtocolEngineStatus protocol_engine_handle_text_show_config(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    ArmConfigValidation validation;
    TextProtocolSpan joint_span;
    uint32_t joint_number;
    uint8_t enable_ready;

    if ((request->field_count != 0U) || (request->positional_count > 1U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    enable_ready = arm_config_is_enable_ready(engine->configuration, &validation);
    if (request->positional_count == 0U)
    {
        uint8_t verified_joint_mask = 0U;
        uint8_t joint_index;

        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            if ((engine->configuration->joints[joint_index].verified_fields &
                 ARM_JOINT_REQUIRED_ENABLE_FIELDS) ==
                ARM_JOINT_REQUIRED_ENABLE_FIELDS)
            {
                verified_joint_mask |= (uint8_t)(1U << joint_index);
            }
        }
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show config map=%08lx required=%02lx verified=%02x enable_ready=%u",
            (unsigned long)request->request_id,
            (unsigned long)protocol_engine_text_config_map_hash(engine->configuration),
            (unsigned long)ARM_JOINT_REQUIRED_ENABLE_FIELDS,
            (unsigned int)verified_joint_mask,
            (unsigned int)enable_ready);
    }
    if ((text_protocol_get_positional(request, 0U, &joint_span) == 0U) ||
        (protocol_engine_parse_text_u32(&joint_span, &joint_number) == 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((joint_number < 1U) || (joint_number > ARM_JOINT_COUNT))
    {
        (void)protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_HIGH_PRIORITY,
            "error %lu show config code=out_of_range field=joint",
            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    {
        const JointConfig *joint = &engine->configuration->joints[joint_number - 1U];
        char body[TEXT_PROTOCOL_MAX_RESPONSE_LINE_LENGTH];
        size_t body_length = 0U;
        const uint32_t verified = joint->verified_fields;

        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "ok %lu show config joint=%lu esc=%02x master=%02x dir=",
                (unsigned long)request->request_id,
                (unsigned long)joint_number,
                (unsigned int)joint->esc_id,
                (unsigned int)joint->master_id) == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
        if ((verified & ARM_JOINT_VERIFIED_DIRECTION) != 0U)
        {
            if (protocol_engine_append_text(body,
                                            sizeof(body),
                                            &body_length,
                                            "%d",
                                            (int)joint->direction) == 0U)
            {
                return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
            }
        }
        else if (protocol_engine_append_text(body,
                                             sizeof(body),
                                             &body_length,
                                             "?") == 0U)
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
        if ((protocol_engine_append_text_config_float(
                 body, sizeof(body), &body_length, "gear",
                 (uint8_t)((verified & ARM_JOINT_VERIFIED_GEAR_RATIO) != 0U),
                 joint->gear_ratio) == 0U) ||
            (protocol_engine_append_text_config_float(
                 body, sizeof(body), &body_length, "min_deg",
                 (uint8_t)((verified & ARM_JOINT_VERIFIED_LIMITS) != 0U),
                 joint->soft_limit_min_rad * PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
            (protocol_engine_append_text_config_float(
                 body, sizeof(body), &body_length, "max_deg",
                 (uint8_t)((verified & ARM_JOINT_VERIFIED_LIMITS) != 0U),
                 joint->soft_limit_max_rad * PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
            (protocol_engine_append_text_config_float(
                 body, sizeof(body), &body_length, "vmax_deg_s",
                 (uint8_t)((verified & ARM_JOINT_VERIFIED_MAX_VELOCITY) != 0U),
                 joint->max_velocity_rad_s * PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
            (protocol_engine_append_text_config_float(
                 body, sizeof(body), &body_length, "amax_deg_s2",
                 (uint8_t)((verified & ARM_JOINT_VERIFIED_MAX_ACCELERATION) != 0U),
                 joint->max_acceleration_rad_s2 * PROTOCOL_ENGINE_RAD_TO_DEG) == 0U) ||
            (protocol_engine_append_text(body,
                                         sizeof(body),
                                         &body_length,
                                         " verified=%02lx",
                                         (unsigned long)(verified & 0xFFUL)) == 0U))
        {
            return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
        }
        return protocol_engine_append_text_body(output_batch,
                                                PROTOCOL_OUTPUT_QUERY,
                                                body);
    }
}

/** @brief Returns the minimal or section-specific diagnostic snapshot. */
static ProtocolEngineStatus protocol_engine_handle_text_show_diag(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    ProtocolOutputBatch *output_batch)
{
    const DiagnosticCounters *diagnostics;
    TextProtocolSpan section;

    if ((request->field_count != 0U) || (request->positional_count > 1U) ||
        (engine->query_context_valid == 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    diagnostics = &engine->query_context.diagnostics;
    if (request->positional_count == 0U)
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show diag loop_max_us=%lu deadline_miss=%lu can_error=%lu usb_drop=%lu fault=%s",
            (unsigned long)request->request_id,
            (unsigned long)diagnostics->control_period_max_us,
            (unsigned long)diagnostics->control_deadline_miss_count,
            (unsigned long)diagnostics->can_tx_error_count,
            (unsigned long)diagnostics->usb_telemetry_drop_count,
            protocol_engine_text_arm_fault(engine->query_context.arm.fault));
    }
    (void)text_protocol_get_positional(request, 0U, &section);
    if ((section.length == 3U) && (strncmp(section.data, "can", 3U) == 0))
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show diag can rx=%lu tx=%lu drop=%lu error=%lu busoff=%lu queue_hwm=%lu",
            (unsigned long)request->request_id,
            (unsigned long)diagnostics->can_rx_frames,
            (unsigned long)diagnostics->can_tx_frames,
            (unsigned long)diagnostics->can_rx_overflow_count,
            (unsigned long)diagnostics->can_tx_error_count,
            (unsigned long)diagnostics->can_bus_off_count,
            (unsigned long)diagnostics->can_tx_queue_high_watermark);
    }
    if ((section.length == 6U) && (strncmp(section.data, "motion", 6U) == 0))
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show diag motion active=%lu predicted_ms=%lu elapsed_ms=%lu max_error_deg=%.3f",
            (unsigned long)request->request_id,
            (unsigned long)engine->active_motion_request_id,
            (unsigned long)((engine->active_motion_planned_duration_us + 999ULL) /
                            1000ULL),
            (unsigned long)engine->last_motion_actual_duration_ms,
            (double)engine->last_motion_max_following_error_mdeg / 1000.0);
    }
    if ((section.length == 3U) && (strncmp(section.data, "usb", 3U) == 0))
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show diag usb rx_lines=? bad_lines=%lu rx_overflow=%lu tx_drop_data=%lu busy_max_ms=?",
            (unsigned long)request->request_id,
            (unsigned long)engine->bad_frame_count,
            (unsigned long)diagnostics->usb_rx_overflow_count,
            (unsigned long)diagnostics->usb_telemetry_drop_count);
    }
    if ((section.length == 4U) && (strncmp(section.data, "rtos", 4U) == 0))
    {
        return protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_QUERY,
            "ok %lu show diag rtos control_stack=? can_stack=? protocol_stack=? usb_stack=? heap_min=%lu",
            (unsigned long)request->request_id,
            (unsigned long)diagnostics->minimum_heap_bytes);
    }
    (void)protocol_engine_append_text_command_error(output_batch,
                                                    request,
                                                    "bad_argument");
    return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
}

/** @brief Validates and applies one fixed-format text telemetry stream. */
static ProtocolEngineStatus protocol_engine_handle_text_stream(
    ProtocolEngine *engine,
    const TextProtocolRequest *request,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    TextProtocolSpan rate_span;
    uint32_t rate_hz;
    uint32_t maximum_rate;

    if (text_protocol_request_path_equals(request, "stream", "off") != 0U)
    {
        if ((request->positional_count != 0U) || (request->field_count != 0U))
        {
            (void)protocol_engine_append_text_command_error(output_batch,
                                                            request,
                                                            "bad_argument");
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        engine->stream_rate_hz = 0U;
        engine->text_stream_kind = PROTOCOL_ENGINE_TEXT_STREAM_OFF;
        return protocol_engine_append_text_format(output_batch,
                                                  PROTOCOL_OUTPUT_QUERY,
                                                  "ok %lu stream off",
                                                  (unsigned long)request->request_id);
    }
    if ((request->positional_count != 0U) || (request->field_count != 1U) ||
        (text_protocol_find_field(request, "rate", &rate_span) == 0U) ||
        (protocol_engine_parse_text_u32(&rate_span, &rate_hz) == 0U))
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "bad_argument");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (text_protocol_request_path_equals(request, "stream", "joints") != 0U)
    {
        engine->text_stream_kind = PROTOCOL_ENGINE_TEXT_STREAM_JOINTS;
        maximum_rate = 50U;
    }
    else if (text_protocol_request_path_equals(request, "stream", "motors") != 0U)
    {
        engine->text_stream_kind = PROTOCOL_ENGINE_TEXT_STREAM_MOTORS;
        maximum_rate = 10U;
    }
    else
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        request,
                                                        "unknown_command");
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if ((rate_hz < 1U) || (rate_hz > maximum_rate))
    {
        engine->text_stream_kind = PROTOCOL_ENGINE_TEXT_STREAM_OFF;
        (void)protocol_engine_append_text_format(
            output_batch,
            PROTOCOL_OUTPUT_HIGH_PRIORITY,
            "error %lu stream %.*s code=out_of_range field=rate",
            (unsigned long)request->request_id,
            (int)request->command_words[1].length,
            request->command_words[1].data);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    engine->stream_rate_hz = (uint8_t)rate_hz;
    engine->next_telemetry_due_us = timestamp_us;
    return protocol_engine_append_text_format(
        output_batch,
        PROTOCOL_OUTPUT_QUERY,
        "ok %lu stream %.*s rate=%lu",
        (unsigned long)request->request_id,
        (int)request->command_words[1].length,
        request->command_words[1].data,
        (unsigned long)rate_hz);
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
    engine->next_motor_telemetry_due_us = timestamp_us;
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
        "RSP %lu ok state=%s aligned=%u enabled=%u moving=%u mode=%s active_request=%lu fault=%s feedback_age_max_ms=%lu",
        (unsigned long)request->request_id,
        protocol_engine_arm_state_text(engine->query_context.arm.state),
        engine->query_context.arm.aligned,
        engine->query_context.arm.enabled,
        engine->query_context.arm.moving,
        protocol_engine_control_mode_text(engine->query_context.arm.control_mode),
        (unsigned long)engine->active_motion_request_id,
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
    if (protocol_engine_append_text(
            body,
            sizeof(body),
            &body_length,
            " startup=%u,%u,%u,%u",
            engine->query_context.motor_identity_verified_mask,
            engine->query_context.motor_mode_verified_mask,
            engine->query_context.motor_ranges_verified_mask,
            engine->query_context.motor_version_verified_mask) == 0U)
    {
        return PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL;
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
        "RSP %lu ok control_hz=250 service_cycles=%lu p_us=%lu,%lu,%lu miss=%lu,%lu can=%lu,%lu,%lu,%lu,%lu,%lu,%lu skew_us=%lu usb=%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu parse=%lu,%lu motion=%lu,%lu,%lu,%lu event_ov=%lu config_fail=%lu stack=%lu heap=%lu",
        (unsigned long)request->request_id,
        (unsigned long)diagnostics->service_cycles,
        (unsigned long)diagnostics->control_period_last_us,
        (unsigned long)diagnostics->control_period_min_us,
        (unsigned long)diagnostics->control_period_max_us,
        (unsigned long)diagnostics->control_deadline_miss_count,
        (unsigned long)diagnostics->control_consecutive_miss_count,
        (unsigned long)diagnostics->can_rx_frames,
        (unsigned long)diagnostics->can_tx_frames,
        (unsigned long)diagnostics->can_rx_overflow_count,
        (unsigned long)diagnostics->can_tx_error_count,
        (unsigned long)diagnostics->can_bus_off_count,
        (unsigned long)diagnostics->can_tx_queue_high_watermark,
        (unsigned long)diagnostics->control_group_reject_count,
        (unsigned long)diagnostics->control_group_skew_max_us,
        (unsigned long)diagnostics->usb_rx_bytes,
        (unsigned long)diagnostics->usb_rx_overflow_count,
        (unsigned long)diagnostics->usb_overlong_line_count,
        (unsigned long)diagnostics->usb_high_queue_high_watermark,
        (unsigned long)diagnostics->usb_query_queue_high_watermark,
        (unsigned long)diagnostics->usb_telemetry_queue_high_watermark,
        (unsigned long)diagnostics->usb_telemetry_drop_count,
        (unsigned long)diagnostics->usb_high_queue_full_count,
        (unsigned long)diagnostics->usb_transmit_busy_count,
        (unsigned long)diagnostics->usb_transmit_error_count,
        (unsigned long)engine->bad_frame_count,
        (unsigned long)engine->bad_crc_count,
        (unsigned long)engine->active_motion_request_id,
        (unsigned long)((engine->active_motion_planned_duration_us + 999ULL) /
                        1000ULL),
        (unsigned long)engine->last_motion_actual_duration_ms,
        (unsigned long)engine->last_motion_max_following_error_mdeg,
        (unsigned long)diagnostics->event_overwrite_count,
        (unsigned long)diagnostics->config_validation_failures,
        (unsigned long)diagnostics->minimum_stack_words,
        (unsigned long)diagnostics->minimum_heap_bytes);
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
    engine->next_motor_telemetry_due_us = 0U;
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
        command.control_mode = engine->query_context.arm.control_mode;
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
    engine->active_motion_accepted_at_us = timestamp_us;
    engine->active_motion_planned_duration_us = motion_plan.duration_us;
    return protocol_engine_append_format(
        output_batch,
        PROTOCOL_OUTPUT_HIGH_PRIORITY,
        "ACK %lu accepted motion_id=%lu duration_ms=%lu mode=%s",
        (unsigned long)request->request_id,
        (unsigned long)request->request_id,
        (unsigned long)((motion_plan.duration_us + 999ULL) / 1000ULL),
        (command.control_mode == ARM_CONTROL_MODE_MIT) ? "MIT" : "POS_VEL");
}

#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
/** @brief Validates one explicit-motor bench command without implicit selection. */
static ProtocolEngineStatus protocol_engine_handle_bench_action(
    ProtocolEngine *engine,
    const AsciiProtocolRequest *request,
    ProtocolCommandType command_type,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    AsciiProtocolSpan motors_span;
    ProtocolCommand command;
    uint8_t joint_index;

    memset(&command, 0, sizeof(command));
    if ((ascii_protocol_find_field(request, "motors", &motors_span) == 0U) ||
        (protocol_engine_parse_motor_mask(&motors_span, &command.motor_mask) == 0U))
    {
        (void)protocol_engine_append_format(output_batch,
                                            PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                            "ERR %lu BAD_VALUE field=motors",
                                            (unsigned long)request->request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (command_type == PROTOCOL_COMMAND_MOVE_RELATIVE)
    {
        AsciiProtocolSpan delta_span;
        AsciiProtocolSpan speed_span;

        if ((ascii_protocol_find_field(request, "delta_deg", &delta_span) == 0U) ||
            (protocol_engine_parse_selected_motor_values(&delta_span,
                                                         command.motor_mask,
                                                         command.values) == 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=delta_deg",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        if ((ascii_protocol_find_field(request, "speed_deg_s", &speed_span) == 0U) ||
            (protocol_engine_parse_selected_motor_values(&speed_span,
                                                         command.motor_mask,
                                                         command.speeds) == 0U))
        {
            (void)protocol_engine_append_format(output_batch,
                                                PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                "ERR %lu BAD_VALUE field=speed_deg_s",
                                                (unsigned long)request->request_id);
            return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
        }
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            if ((command.motor_mask & (uint8_t)(1U << joint_index)) == 0U)
            {
                continue;
            }
            if ((command.values[joint_index] < -3.0F) ||
                (command.values[joint_index] > 3.0F))
            {
                (void)protocol_engine_append_format(
                    output_batch,
                    PROTOCOL_OUTPUT_HIGH_PRIORITY,
                    "ERR %lu BAD_VALUE field=delta_deg",
                    (unsigned long)request->request_id);
                return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
            }
            if ((command.speeds[joint_index] <= 0.0F) ||
                (command.speeds[joint_index] > 3.0F))
            {
                (void)protocol_engine_append_format(
                    output_batch,
                    PROTOCOL_OUTPUT_HIGH_PRIORITY,
                    "ERR %lu BAD_VALUE field=speed_deg_s",
                    (unsigned long)request->request_id);
                return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
            }
        }
    }
    command.type = command_type;
    command.request_id = request->request_id;
    command.session_id = engine->session_id;
    command.accepted_at_us = timestamp_us;
    command.control_mode = ARM_CONTROL_MODE_POSITION_VELOCITY;
    command.bench_relative_scope = 1U;
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
                                         "ACK %lu accepted motors=%u",
                                         (unsigned long)request->request_id,
                                         (unsigned int)command.motor_mask);
}
#endif

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
        uint64_t actual_duration_us =
            (result.completed_at_us >= engine->active_motion_accepted_at_us)
                ? (result.completed_at_us - engine->active_motion_accepted_at_us)
                : 0U;
        float maximum_error_deg = result.auxiliary_values[0];

        engine->last_motion_actual_duration_ms =
            (actual_duration_us > ((uint64_t)UINT32_MAX * 1000ULL))
                ? UINT32_MAX
                : (uint32_t)((actual_duration_us + 999ULL) / 1000ULL);
        engine->last_motion_max_following_error_mdeg =
            ((maximum_error_deg >= 0.0F) &&
             (maximum_error_deg < ((float)UINT32_MAX / 1000.0F)))
                ? (uint32_t)(maximum_error_deg * 1000.0F + 0.5F)
                : UINT32_MAX;
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
 * @brief Appends one compact 10 Hz motor-state snapshot from the query context.
 */
static uint8_t protocol_engine_append_motor_telemetry(
    ProtocolEngine *engine,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    char body[PROTOCOL_MAX_LINE_LENGTH + 1U];
    size_t body_length = 0U;
    uint8_t fault_mask = 0U;
    uint8_t joint_index;

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
            "TEL %lu MOTOR_STATE t_us=%lu valid_mask=%u fault_mask=%u driver=",
            (unsigned long)engine->telemetry_sequence,
            (unsigned long)timestamp_us,
            engine->query_context.motors.valid_joint_mask,
            fault_mask) == 0U)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if (protocol_engine_append_text(
                body,
                sizeof(body),
                &body_length,
                "%s%u",
                (joint_index == 0U) ? "" : ",",
                engine->query_context.motors.joints[joint_index].driver_state) == 0U)
        {
            return 0U;
        }
    }
    if (protocol_engine_append_text(body,
                                    sizeof(body),
                                    &body_length,
                                    " age_ms=") == 0U)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorJointFeedback *feedback =
            &engine->query_context.motors.joints[joint_index];
        uint32_t age_ms = UINT32_MAX;

        if ((feedback->timestamp_us != 0U) &&
            (timestamp_us >= feedback->timestamp_us))
        {
            uint64_t age_value = (timestamp_us - feedback->timestamp_us) / 1000U;
            age_ms = (age_value > UINT32_MAX) ? UINT32_MAX : (uint32_t)age_value;
        }
        if (protocol_engine_append_text(body,
                                        sizeof(body),
                                        &body_length,
                                        "%s%lu",
                                        (joint_index == 0U) ? "" : ",",
                                        (unsigned long)age_ms) == 0U)
        {
            return 0U;
        }
    }
    if (protocol_engine_append_text(
            body,
            sizeof(body),
            &body_length,
            " arm_state=%s",
            protocol_engine_arm_state_text(engine->query_context.arm.state)) == 0U)
    {
        return 0U;
    }
    return (uint8_t)(protocol_engine_append_body(output_batch,
                                                  PROTOCOL_OUTPUT_TELEMETRY,
                                                  body) ==
                     PROTOCOL_ENGINE_STATUS_OK);
}

/** @brief Generates one due fixed-format aethor-text-v1 data sample. */
static uint8_t protocol_engine_generate_text_stream_output(
    ProtocolEngine *engine,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    uint64_t interval_us;

    if ((engine->query_context_valid == 0U) ||
        (engine->stream_rate_hz == 0U) ||
        (engine->text_stream_kind == PROTOCOL_ENGINE_TEXT_STREAM_OFF) ||
        ((engine->next_telemetry_due_us != 0U) &&
         (timestamp_us < engine->next_telemetry_due_us)))
    {
        return 0U;
    }
    interval_us = 1000000ULL / engine->stream_rate_hz;
    engine->next_telemetry_due_us = timestamp_us + interval_us;
    if (engine->text_stream_kind == PROTOCOL_ENGINE_TEXT_STREAM_JOINTS)
    {
        return (uint8_t)(protocol_engine_append_text_joints_snapshot(
                             engine,
                             output_batch,
                             PROTOCOL_OUTPUT_TELEMETRY,
                             "data 501 joints",
                             1U) == PROTOCOL_ENGINE_STATUS_OK);
    }
    if (engine->text_stream_kind == PROTOCOL_ENGINE_TEXT_STREAM_MOTORS)
    {
        uint8_t enabled_mask;
        uint8_t moving_mask;
        uint8_t holding_mask;
        uint8_t fault_mask;

        protocol_engine_text_motor_masks(&engine->query_context,
                                         &enabled_mask,
                                         &moving_mask,
                                         &holding_mask,
                                         &fault_mask);
        (void)holding_mask;
        return (uint8_t)(protocol_engine_append_text_format(
                             output_batch,
                             PROTOCOL_OUTPUT_TELEMETRY,
                             "data 502 motors present=%02x enabled=%02x moving=%02x stale=00 fault=%02x",
                             (unsigned int)engine->query_context.motors
                                 .valid_joint_mask,
                             (unsigned int)enabled_mask,
                             (unsigned int)moving_mask,
                             (unsigned int)fault_mask) ==
                         PROTOCOL_ENGINE_STATUS_OK);
    }
    return 0U;
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
    uint8_t joint_telemetry_due;
    uint8_t motor_telemetry_due;
    uint8_t state_changed = 0U;
    uint8_t joint_index;

    if ((engine == NULL) || (output_batch == NULL))
    {
        return 0U;
    }
    protocol_engine_clear_output(output_batch);
    if (engine->text_protocol_active != 0U)
    {
        return protocol_engine_generate_text_stream_output(engine,
                                                           timestamp_us,
                                                           output_batch);
    }
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
        state_changed = 1U;
    }

    if (engine->stream_rate_hz == 0U)
    {
        return output_batch->count;
    }
    interval_us = 1000000ULL / engine->stream_rate_hz;
    joint_telemetry_due = (uint8_t)((engine->next_telemetry_due_us == 0U) ||
                                    (timestamp_us >= engine->next_telemetry_due_us));
    motor_telemetry_due = (uint8_t)(
        (protocol_engine_csv_contains(engine->stream_fields, "motor") != 0U) &&
        ((state_changed != 0U) || (engine->next_motor_telemetry_due_us == 0U) ||
         (timestamp_us >= engine->next_motor_telemetry_due_us)));
    if ((joint_telemetry_due == 0U) && (motor_telemetry_due == 0U))
    {
        return output_batch->count;
    }
    if (joint_telemetry_due != 0U)
    {
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
    }
    if ((motor_telemetry_due != 0U) &&
        (output_batch->count < PROTOCOL_ENGINE_MAX_OUTPUT_COUNT))
    {
        engine->next_motor_telemetry_due_us = timestamp_us + 100000ULL;
        ++engine->telemetry_sequence;
        (void)protocol_engine_append_motor_telemetry(engine,
                                                     timestamp_us,
                                                     output_batch);
    }
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
            if (engine->bad_crc_count < UINT32_MAX)
            {
                ++engine->bad_crc_count;
            }
        }
        else if (parse_status == ASCII_PROTOCOL_STATUS_LINE_TOO_LONG)
        {
            error_code = "LINE_TOO_LONG";
        }
        else if (engine->bad_frame_count < UINT32_MAX)
        {
            ++engine->bad_frame_count;
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
#if (AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE)
    else if (ascii_protocol_request_operation_equals(&request, "INIT_MOTORS") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_INIT_MOTORS, timestamp_us, output_batch);
    }
    else if (ascii_protocol_request_operation_equals(&request, "MOVE_REL") != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_MOVE_RELATIVE, timestamp_us, output_batch);
    }
    else if ((protocol_engine_request_has_field(&request, "motors") != 0U) &&
             (ascii_protocol_request_operation_equals(&request, "ENABLE") != 0U))
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_ENABLE, timestamp_us, output_batch);
    }
    else if ((protocol_engine_request_has_field(&request, "motors") != 0U) &&
             (ascii_protocol_request_operation_equals(&request, "STOP") != 0U))
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_STOP, timestamp_us, output_batch);
    }
    else if ((protocol_engine_request_has_field(&request, "motors") != 0U) &&
             (ascii_protocol_request_operation_equals(&request, "DISABLE") != 0U))
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_DISABLE, timestamp_us, output_batch);
    }
    else if ((protocol_engine_request_has_field(&request, "motors") != 0U) &&
             (ascii_protocol_request_operation_equals(&request, "CLEAR_FAULT") != 0U))
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine_status = protocol_engine_handle_bench_action(
            engine, &request, PROTOCOL_COMMAND_CLEAR_FAULT, timestamp_us, output_batch);
    }
#endif
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
 * @brief Processes one complete aethor-text-v1 request line without wire CRC.
 */
ProtocolEngineStatus protocol_engine_process_text_line(
    ProtocolEngine *engine,
    const char *line,
    size_t length,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch)
{
    TextProtocolRequest request;
    TextProtocolStatus parse_status;
    ProtocolRecentResult *recent_result;
    ProtocolEngineStatus engine_status;
    char canonical[TEXT_PROTOCOL_MAX_REQUEST_LINE_LENGTH + 1U];
    size_t canonical_length = 0U;
    uint32_t body_hash;

    if ((engine == NULL) || (line == NULL) || (output_batch == NULL))
    {
        return PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT;
    }
    protocol_engine_clear_output(output_batch);
    parse_status = text_protocol_parse_request(line, length, &request);
    if (parse_status != TEXT_PROTOCOL_STATUS_OK)
    {
        const char *error_code =
            (parse_status == TEXT_PROTOCOL_STATUS_LINE_TOO_LONG)
                ? "line_too_long"
                : "bad_line";

        if (engine->bad_frame_count < UINT32_MAX)
        {
            ++engine->bad_frame_count;
        }
        (void)protocol_engine_append_text_format(output_batch,
                                                 PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                 "error 0 parse code=%s",
                                                 error_code);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    if (protocol_engine_build_text_canonical_body(&request,
                                                   canonical,
                                                   &canonical_length) == 0U)
    {
        (void)protocol_engine_append_text_format(output_batch,
                                                 PROTOCOL_OUTPUT_HIGH_PRIORITY,
                                                 "error %lu parse code=bad_line",
                                                 (unsigned long)request.request_id);
        return PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }
    engine->text_protocol_active = 1U;
    if (engine->session_active != 0U)
    {
        engine->last_valid_request_at_us = timestamp_us;
        engine->watchdog_timeout_reported = 0U;
    }
    body_hash = ascii_protocol_crc32_iso_hdlc((const uint8_t *)canonical,
                                              canonical_length);
    recent_result = (request.has_request_id != 0U)
                        ? protocol_engine_find_recent(engine,
                                                      request.request_id,
                                                      timestamp_us)
                        : NULL;
    if (recent_result != NULL)
    {
        if (recent_result->body_hash != body_hash)
        {
            (void)protocol_engine_append_text_command_error(output_batch,
                                                             &request,
                                                             "request_conflict");
            return PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT;
        }
        protocol_engine_replay(recent_result, output_batch);
        if (engine->session_active != 0U)
        {
            engine->last_valid_request_at_us = timestamp_us;
            engine->watchdog_timeout_reported = 0U;
        }
        return PROTOCOL_ENGINE_STATUS_REPLAYED;
    }

    if (text_protocol_request_path_equals(&request, "hello", NULL) != 0U)
    {
        engine_status = protocol_engine_handle_text_hello(engine,
                                                          &request,
                                                          timestamp_us,
                                                          output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "ping", NULL) != 0U)
    {
        engine_status = protocol_engine_handle_text_ping(engine,
                                                         &request,
                                                         timestamp_us,
                                                         output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "info") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_info(&request,
                                                              output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "state") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_state(engine,
                                                               &request,
                                                               output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "joints") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_joints(engine,
                                                                &request,
                                                                output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "motors") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_motors(engine,
                                                                &request,
                                                                output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "motor") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_motor(engine,
                                                               &request,
                                                               timestamp_us,
                                                               output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "config") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_config(engine,
                                                                &request,
                                                                output_batch);
    }
    else if (text_protocol_request_path_equals(&request, "show", "diag") != 0U)
    {
        engine_status = protocol_engine_handle_text_show_diag(engine,
                                                              &request,
                                                              output_batch);
    }
    else if ((text_protocol_request_path_equals(&request, "stream", "off") != 0U) ||
             (text_protocol_request_path_equals(&request, "stream", "joints") != 0U) ||
             (text_protocol_request_path_equals(&request, "stream", "motors") != 0U))
    {
        engine_status = protocol_engine_handle_text_stream(engine,
                                                           &request,
                                                           timestamp_us,
                                                           output_batch);
    }
    else
    {
        (void)protocol_engine_append_text_command_error(output_batch,
                                                        &request,
                                                        "unknown_command");
        engine_status = PROTOCOL_ENGINE_STATUS_BAD_REQUEST;
    }

    if ((request.has_request_id != 0U) && (output_batch->count == 1U) &&
        (engine_status != PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL))
    {
        protocol_engine_store_text_recent(engine,
                                          request.request_id,
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
