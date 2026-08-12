/**
 * @file s3519_codec.c
 * @brief Implements verified S3519 Classic CAN layouts without HAL dependencies.
 */

#include "s3519_codec.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Checks whether an ESC ID fits the S3519 feedback identifier nibble.
 * @param esc_id Motor receive identifier.
 * @return One when supported, otherwise zero.
 */
static uint8_t s3519_esc_id_is_valid(uint8_t esc_id)
{
    return (uint8_t)((esc_id >= S3519_MINIMUM_ESC_ID) &&
                     (esc_id <= S3519_MAXIMUM_ESC_ID));
}

/**
 * @brief Checks whether a control-mode CAN identifier offset is supported.
 * @param control_mode Requested control mode.
 * @return One when supported, otherwise zero.
 */
static uint8_t s3519_control_mode_is_valid(S3519ControlMode control_mode)
{
    return (uint8_t)((control_mode == S3519_CONTROL_MODE_MIT) ||
                     (control_mode == S3519_CONTROL_MODE_POSITION_VELOCITY));
}

/**
 * @brief Checks whether one special mode command byte is recognized.
 * @param command Requested special command.
 * @return One when supported, otherwise zero.
 */
static uint8_t s3519_mode_command_is_valid(S3519ModeCommand command)
{
    return (uint8_t)((command == S3519_MODE_COMMAND_CLEAR_ERROR) ||
                     (command == S3519_MODE_COMMAND_ENABLE) ||
                     (command == S3519_MODE_COMMAND_DISABLE) ||
                     (command == S3519_MODE_COMMAND_SAVE_ZERO));
}

/**
 * @brief Checks whether a register belongs to the non-persistent discovery set.
 * @param register_address Register address.
 * @return One when supported, otherwise zero.
 */
static uint8_t s3519_register_is_valid(S3519Register register_address)
{
    return (uint8_t)((register_address == S3519_REGISTER_ACCELERATION) ||
                     (register_address == S3519_REGISTER_DECELERATION) ||
                     (register_address == S3519_REGISTER_MAXIMUM_SPEED) ||
                     (register_address == S3519_REGISTER_MASTER_ID) ||
                     (register_address == S3519_REGISTER_ESC_ID) ||
                     (register_address == S3519_REGISTER_CONTROL_MODE) ||
                     (register_address == S3519_REGISTER_HARDWARE_VERSION) ||
                     (register_address == S3519_REGISTER_SOFTWARE_VERSION) ||
                     (register_address == S3519_REGISTER_POSITION_RANGE) ||
                     (register_address == S3519_REGISTER_VELOCITY_RANGE) ||
                     (register_address == S3519_REGISTER_TORQUE_RANGE) ||
                     (register_address == S3519_REGISTER_SUB_VERSION));
}

/**
 * @brief Checks whether discovered mapping ranges are finite and positive.
 * @param ranges Runtime range values.
 * @return One when usable, otherwise zero.
 */
static uint8_t s3519_ranges_are_valid(const S3519Ranges *ranges)
{
    return (uint8_t)((ranges != NULL) &&
                     isfinite(ranges->position_max_rad) &&
                     (ranges->position_max_rad > 0.0F) &&
                     isfinite(ranges->velocity_max_rad_s) &&
                     (ranges->velocity_max_rad_s > 0.0F) &&
                     isfinite(ranges->torque_max_nm) &&
                     (ranges->torque_max_nm > 0.0F));
}

/**
 * @brief Writes one IEEE-754 float in the vendor little-endian payload order.
 * @param value Floating-point value.
 * @param destination Four-byte payload destination.
 */
static void s3519_write_float_little_endian(float value, uint8_t *destination)
{
    uint32_t raw_value;

    memcpy(&raw_value, &value, sizeof(raw_value));
    destination[0] = (uint8_t)(raw_value & 0xFFU);
    destination[1] = (uint8_t)((raw_value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((raw_value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((raw_value >> 24U) & 0xFFU);
}

/**
 * @brief Reads one little-endian uint32 payload value.
 * @param source Four-byte payload source.
 * @return Decoded value.
 */
static uint32_t s3519_read_u32_little_endian(const uint8_t *source)
{
    return ((uint32_t)source[0]) |
           ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) |
           ((uint32_t)source[3] << 24U);
}

/**
 * @brief Maps a bounded float to an unsigned fixed-width protocol integer.
 * @param value Input value already validated inside the range.
 * @param minimum Minimum physical value.
 * @param maximum Maximum physical value.
 * @param bits Encoded bit width.
 * @return Truncated vendor-compatible unsigned value.
 */
static uint32_t s3519_float_to_unsigned(float value,
                                        float minimum,
                                        float maximum,
                                        uint8_t bits)
{
    uint32_t encoded_maximum = (1UL << bits) - 1UL;
    return (uint32_t)((value - minimum) * (float)encoded_maximum /
                      (maximum - minimum));
}

/**
 * @brief Maps an unsigned protocol integer to a physical range.
 * @param value Encoded value.
 * @param minimum Minimum physical value.
 * @param maximum Maximum physical value.
 * @param bits Encoded bit width.
 * @return Decoded physical value.
 */
static float s3519_unsigned_to_float(uint32_t value,
                                    float minimum,
                                    float maximum,
                                    uint8_t bits)
{
    uint32_t encoded_maximum = (1UL << bits) - 1UL;
    return ((float)value * (maximum - minimum) / (float)encoded_maximum) + minimum;
}

/**
 * @brief Packs one position-velocity command using little-endian float values.
 * @param esc_id Target motor receive identifier.
 * @param position_rad Target output position in radians.
 * @param velocity_rad_s Nonnegative output velocity limit.
 * @param frame Destination Classic CAN frame.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_pack_position_velocity(uint8_t esc_id,
                                              float position_rad,
                                              float velocity_rad_s,
                                              CanFrame *frame)
{
    uint8_t payload[8];

    if (frame == NULL)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (s3519_esc_id_is_valid(esc_id) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ID;
    }
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) ||
        (velocity_rad_s < 0.0F))
    {
        return S3519_CODEC_STATUS_INVALID_RANGE;
    }

    s3519_write_float_little_endian(position_rad, &payload[0]);
    s3519_write_float_little_endian(velocity_rad_s, &payload[4]);
    if (can_frame_init(frame,
                       (uint16_t)(S3519_CONTROL_MODE_POSITION_VELOCITY + esc_id),
                       payload,
                       sizeof(payload)) != CAN_FRAME_STATUS_OK)
    {
        return S3519_CODEC_STATUS_INVALID_FRAME;
    }
    return S3519_CODEC_STATUS_OK;
}

/**
 * @brief Packs enable, disable, clear-error, or save-zero for one control mode.
 * @param esc_id Target motor receive identifier.
 * @param control_mode Active motor control mode.
 * @param command Special mode command.
 * @param frame Destination Classic CAN frame.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_pack_mode_command(uint8_t esc_id,
                                        S3519ControlMode control_mode,
                                        S3519ModeCommand command,
                                        CanFrame *frame)
{
    uint8_t payload[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U
    };

    if (frame == NULL)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (s3519_esc_id_is_valid(esc_id) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ID;
    }
    if (s3519_control_mode_is_valid(control_mode) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_MODE;
    }
    if (s3519_mode_command_is_valid(command) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }

    payload[7] = (uint8_t)command;
    if (can_frame_init(frame,
                       (uint16_t)((uint16_t)control_mode + esc_id),
                       payload,
                       sizeof(payload)) != CAN_FRAME_STATUS_OK)
    {
        return S3519_CODEC_STATUS_INVALID_FRAME;
    }
    return S3519_CODEC_STATUS_OK;
}

/**
 * @brief Packs one volatile discovery register read.
 * @param esc_id Target motor receive identifier.
 * @param register_address Register to read.
 * @param frame Destination Classic CAN frame.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_pack_parameter_read(uint8_t esc_id,
                                          S3519Register register_address,
                                          CanFrame *frame)
{
    uint8_t payload[8] = {0U};

    if (frame == NULL)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (s3519_esc_id_is_valid(esc_id) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ID;
    }
    if (s3519_register_is_valid(register_address) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }

    payload[0] = esc_id;
    payload[2] = 0x33U;
    payload[3] = (uint8_t)register_address;
    if (can_frame_init(frame,
                       S3519_PARAMETER_COMMAND_IDENTIFIER,
                       payload,
                       sizeof(payload)) != CAN_FRAME_STATUS_OK)
    {
        return S3519_CODEC_STATUS_INVALID_FRAME;
    }
    return S3519_CODEC_STATUS_OK;
}

/**
 * @brief Packs one MIT command using discovered motor ranges.
 * @param esc_id Target motor receive identifier.
 * @param ranges Discovered PMAX, VMAX, and TMAX values.
 * @param position_rad Target output position.
 * @param velocity_rad_s Target output velocity.
 * @param kp Position gain in the vendor range zero through 500.
 * @param kd Damping gain in the vendor range zero through five.
 * @param torque_nm Feed-forward torque.
 * @param frame Destination Classic CAN frame.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_pack_mit(uint8_t esc_id,
                               const S3519Ranges *ranges,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm,
                               CanFrame *frame)
{
    uint8_t payload[8];
    uint32_t position_encoded;
    uint32_t velocity_encoded;
    uint32_t kp_encoded;
    uint32_t kd_encoded;
    uint32_t torque_encoded;

    if (frame == NULL)
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (s3519_esc_id_is_valid(esc_id) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ID;
    }
    if ((s3519_ranges_are_valid(ranges) == 0U) ||
        !isfinite(position_rad) || !isfinite(velocity_rad_s) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(torque_nm) ||
        (fabsf(position_rad) > ranges->position_max_rad) ||
        (fabsf(velocity_rad_s) > ranges->velocity_max_rad_s) ||
        (kp < S3519_KP_MIN) || (kp > S3519_KP_MAX) ||
        (kd < S3519_KD_MIN) || (kd > S3519_KD_MAX) ||
        (fabsf(torque_nm) > ranges->torque_max_nm))
    {
        return S3519_CODEC_STATUS_INVALID_RANGE;
    }

    position_encoded = s3519_float_to_unsigned(position_rad,
                                                -ranges->position_max_rad,
                                                ranges->position_max_rad,
                                                16U);
    velocity_encoded = s3519_float_to_unsigned(velocity_rad_s,
                                                -ranges->velocity_max_rad_s,
                                                ranges->velocity_max_rad_s,
                                                12U);
    kp_encoded = s3519_float_to_unsigned(kp, S3519_KP_MIN, S3519_KP_MAX, 12U);
    kd_encoded = s3519_float_to_unsigned(kd, S3519_KD_MIN, S3519_KD_MAX, 12U);
    torque_encoded = s3519_float_to_unsigned(torque_nm,
                                             -ranges->torque_max_nm,
                                             ranges->torque_max_nm,
                                             12U);

    payload[0] = (uint8_t)(position_encoded >> 8U);
    payload[1] = (uint8_t)position_encoded;
    payload[2] = (uint8_t)(velocity_encoded >> 4U);
    payload[3] = (uint8_t)(((velocity_encoded & 0x0FU) << 4U) |
                           (kp_encoded >> 8U));
    payload[4] = (uint8_t)kp_encoded;
    payload[5] = (uint8_t)(kd_encoded >> 4U);
    payload[6] = (uint8_t)(((kd_encoded & 0x0FU) << 4U) |
                           (torque_encoded >> 8U));
    payload[7] = (uint8_t)torque_encoded;

    if (can_frame_init(frame, esc_id, payload, sizeof(payload)) != CAN_FRAME_STATUS_OK)
    {
        return S3519_CODEC_STATUS_INVALID_FRAME;
    }
    return S3519_CODEC_STATUS_OK;
}

/**
 * @brief Decodes one eight-byte S3519 control feedback frame.
 * @param frame Received frame already filtered by Master ID.
 * @param ranges Discovered PMAX, VMAX, and TMAX values.
 * @param feedback Destination decoded feedback.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_decode_feedback(const CanFrame *frame,
                                       const S3519Ranges *ranges,
                                       S3519Feedback *feedback)
{
    uint32_t position_encoded;
    uint32_t velocity_encoded;
    uint32_t torque_encoded;

    if ((frame == NULL) || (feedback == NULL))
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (frame->length != 8U)
    {
        return S3519_CODEC_STATUS_INVALID_LENGTH;
    }
    if (s3519_ranges_are_valid(ranges) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_RANGE;
    }

    feedback->esc_id = (uint8_t)(frame->data[0] & 0x0FU);
    feedback->state = (uint8_t)(frame->data[0] >> 4U);
    if (s3519_esc_id_is_valid(feedback->esc_id) == 0U)
    {
        return S3519_CODEC_STATUS_INVALID_ID;
    }

    position_encoded = ((uint32_t)frame->data[1] << 8U) | frame->data[2];
    velocity_encoded = ((uint32_t)frame->data[3] << 4U) | (frame->data[4] >> 4U);
    torque_encoded = (((uint32_t)frame->data[4] & 0x0FU) << 8U) | frame->data[5];
    feedback->position_rad = s3519_unsigned_to_float(position_encoded,
                                                     -ranges->position_max_rad,
                                                     ranges->position_max_rad,
                                                     16U);
    feedback->velocity_rad_s = s3519_unsigned_to_float(velocity_encoded,
                                                       -ranges->velocity_max_rad_s,
                                                       ranges->velocity_max_rad_s,
                                                       12U);
    feedback->torque_nm = s3519_unsigned_to_float(torque_encoded,
                                                  -ranges->torque_max_nm,
                                                  ranges->torque_max_nm,
                                                  12U);
    feedback->mos_temperature_c = (float)frame->data[6];
    feedback->rotor_temperature_c = (float)frame->data[7];
    return S3519_CODEC_STATUS_OK;
}

/**
 * @brief Decodes one eight-byte S3519 register response.
 * @param frame Received parameter response.
 * @param response Destination decoded response.
 * @return Detailed codec status.
 */
S3519CodecStatus s3519_decode_parameter_response(const CanFrame *frame,
                                                 S3519ParameterResponse *response)
{
    uint32_t raw_value;

    if ((frame == NULL) || (response == NULL))
    {
        return S3519_CODEC_STATUS_INVALID_ARGUMENT;
    }
    if (frame->length != 8U)
    {
        return S3519_CODEC_STATUS_INVALID_LENGTH;
    }
    if ((frame->data[2] != 0x33U) && (frame->data[2] != 0x55U))
    {
        return S3519_CODEC_STATUS_INVALID_FRAME;
    }

    response->esc_id = (uint16_t)frame->data[0] |
                       ((uint16_t)(frame->data[1] & 0x07U) << 8U);
    response->register_address = frame->data[3];
    raw_value = s3519_read_u32_little_endian(&frame->data[4]);
    response->raw_value = raw_value;
    memcpy(&response->float_value, &raw_value, sizeof(response->float_value));
    return S3519_CODEC_STATUS_OK;
}
