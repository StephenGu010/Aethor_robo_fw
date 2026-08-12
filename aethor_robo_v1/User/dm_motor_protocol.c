/**
 * @file dm_motor_protocol.c
 * @brief Implements S3519 position-speed commands, mode commands, and feedback decoding.
 */

#include "dm_motor_protocol.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Tests whether a motor identifier can be represented in the feedback ID nibble.
 * @param motor_id Motor identifier to validate.
 * @return Non-zero for a supported identifier; otherwise zero.
 */
static uint8_t dm_motor_id_is_valid(uint8_t motor_id)
{
    return (uint8_t)(motor_id >= DM_MOTOR_MINIMUM_ID && motor_id <= DM_MOTOR_MAXIMUM_ID);
}

/**
 * @brief Writes one float to four little-endian payload bytes.
 * @param value Floating-point value to encode.
 * @param destination Four-byte destination.
 */
static void dm_motor_write_float_little_endian(float value, uint8_t *destination)
{
    uint32_t raw_value;

    memcpy(&raw_value, &value, sizeof(raw_value));
    destination[0] = (uint8_t)(raw_value & 0xFFU);
    destination[1] = (uint8_t)((raw_value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((raw_value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((raw_value >> 24U) & 0xFFU);
}

/**
 * @brief Reads a little-endian 32-bit value from four payload bytes.
 * @param source Four-byte source.
 * @return Decoded unsigned value.
 */
static uint32_t dm_motor_read_u32_little_endian(const uint8_t *source)
{
    return ((uint32_t)source[0]) |
           ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) |
           ((uint32_t)source[3] << 24U);
}

/**
 * @brief Writes one unsigned 32-bit value to four little-endian payload bytes.
 * @param value Unsigned value to encode.
 * @param destination Four-byte destination.
 */
static void dm_motor_write_u32_little_endian(uint32_t value, uint8_t *destination)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

/**
 * @brief Maps an unsigned protocol value back to a symmetric floating-point range.
 * @param value Encoded unsigned value.
 * @param maximum Positive end of the symmetric range.
 * @param bits Number of encoded bits.
 * @return Decoded floating-point value.
 */
static float dm_motor_unsigned_to_symmetric_float(uint32_t value, float maximum, uint8_t bits)
{
    const uint32_t encoded_maximum = (1UL << bits) - 1UL;
    return ((float)value * (2.0f * maximum) / (float)encoded_maximum) - maximum;
}

/**
 * @brief Packs an S3519 position-speed command using output-shaft SI units.
 * @param motor_id Motor CAN identifier in the range 1 through 15.
 * @param output_position_rad Desired output-shaft position in radians.
 * @param output_velocity_rad_s Desired output-shaft velocity limit in radians per second.
 * @param frame Destination classic CAN frame.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_pack_position_velocity(uint8_t motor_id,
                                               float output_position_rad,
                                               float output_velocity_rad_s,
                                               FdcanClassicFrame *frame)
{
    uint8_t payload[8];

    if (frame == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(output_position_rad) || !isfinite(output_velocity_rad_s))
    {
        return DM_MOTOR_STATUS_INVALID_RANGE;
    }
    if (dm_motor_id_is_valid(motor_id) == 0U)
    {
        return DM_MOTOR_STATUS_INVALID_ID;
    }
    if (output_velocity_rad_s < 0.0f)
    {
        return DM_MOTOR_STATUS_INVALID_RANGE;
    }

    dm_motor_write_float_little_endian(output_position_rad, &payload[0]);
    dm_motor_write_float_little_endian(output_velocity_rad_s, &payload[4]);
    if (fdcan_classic_frame_init(frame,
                                 (uint16_t)(DM_MOTOR_POSITION_MODE_OFFSET + motor_id),
                                 payload,
                                 sizeof(payload)) != FDCAN_CLASSIC_STATUS_OK)
    {
        return DM_MOTOR_STATUS_INVALID_FRAME;
    }

    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Packs an S3519 enable, disable, clear-error, or save-zero command.
 * @param motor_id Motor CAN identifier in the range 1 through 15.
 * @param command Special mode command byte.
 * @param frame Destination classic CAN frame.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_pack_mode_command(uint8_t motor_id,
                                         DmMotorModeCommand command,
                                         FdcanClassicFrame *frame)
{
    uint8_t payload[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x00U};

    if (frame == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (dm_motor_id_is_valid(motor_id) == 0U)
    {
        return DM_MOTOR_STATUS_INVALID_ID;
    }
    if (command != DM_MOTOR_MODE_COMMAND_CLEAR_ERROR &&
        command != DM_MOTOR_MODE_COMMAND_ENABLE &&
        command != DM_MOTOR_MODE_COMMAND_DISABLE &&
        command != DM_MOTOR_MODE_COMMAND_SAVE_ZERO)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }

    payload[7] = (uint8_t)command;
    if (fdcan_classic_frame_init(frame,
                                 (uint16_t)(DM_MOTOR_POSITION_MODE_OFFSET + motor_id),
                                 payload,
                                 sizeof(payload)) != FDCAN_CLASSIC_STATUS_OK)
    {
        return DM_MOTOR_STATUS_INVALID_FRAME;
    }

    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Packs a non-persistent motor parameter read command.
 * @param motor_id Motor CAN identifier in the range 1 through 15.
 * @param register_address Register to read.
 * @param frame Destination classic CAN frame.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_pack_parameter_read(uint8_t motor_id,
                                            DmMotorRegister register_address,
                                            FdcanClassicFrame *frame)
{
    uint8_t payload[8] = {0U};

    if (frame == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (dm_motor_id_is_valid(motor_id) == 0U)
    {
        return DM_MOTOR_STATUS_INVALID_ID;
    }

    payload[0] = motor_id;
    payload[1] = 0U;
    payload[2] = 0x33U;
    payload[3] = (uint8_t)register_address;
    if (fdcan_classic_frame_init(frame, DM_MOTOR_PARAMETER_COMMAND_ID, payload, sizeof(payload)) !=
        FDCAN_CLASSIC_STATUS_OK)
    {
        return DM_MOTOR_STATUS_INVALID_FRAME;
    }

    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Packs a non-persistent unsigned motor parameter write command.
 * @param motor_id Motor CAN identifier in the range 1 through 15.
 * @param register_address Register to write.
 * @param value Unsigned 32-bit register value.
 * @param frame Destination classic CAN frame.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_pack_parameter_write_u32(uint8_t motor_id,
                                                DmMotorRegister register_address,
                                                uint32_t value,
                                                FdcanClassicFrame *frame)
{
    uint8_t payload[8] = {0U};

    if (frame == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (dm_motor_id_is_valid(motor_id) == 0U)
    {
        return DM_MOTOR_STATUS_INVALID_ID;
    }

    payload[0] = motor_id;
    payload[1] = 0U;
    payload[2] = 0x55U;
    payload[3] = (uint8_t)register_address;
    dm_motor_write_u32_little_endian(value, &payload[4]);
    if (fdcan_classic_frame_init(frame,
                                 DM_MOTOR_PARAMETER_COMMAND_ID,
                                 payload,
                                 sizeof(payload)) != FDCAN_CLASSIC_STATUS_OK)
    {
        return DM_MOTOR_STATUS_INVALID_FRAME;
    }

    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Decodes one eight-byte S3519 status feedback frame.
 * @param frame Received classic CAN frame.
 * @param ranges Runtime PMAX, VMAX, and TMAX mapping ranges.
 * @param feedback Destination feedback structure.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_decode_feedback(const FdcanClassicFrame *frame,
                                        const DmMotorRanges *ranges,
                                        DmMotorFeedback *feedback)
{
    uint32_t encoded_position;
    uint32_t encoded_velocity;
    uint32_t encoded_torque;

    if (frame == NULL || ranges == NULL || feedback == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (frame->length != 8U)
    {
        return DM_MOTOR_STATUS_INVALID_LENGTH;
    }
    if (!isfinite(ranges->position_max_rad) || ranges->position_max_rad <= 0.0f ||
        !isfinite(ranges->velocity_max_rad_s) || ranges->velocity_max_rad_s <= 0.0f ||
        !isfinite(ranges->torque_max_nm) || ranges->torque_max_nm <= 0.0f)
    {
        return DM_MOTOR_STATUS_INVALID_RANGE;
    }

    feedback->motor_id = frame->data[0] & 0x0FU;
    feedback->state = frame->data[0] >> 4U;
    if (dm_motor_id_is_valid(feedback->motor_id) == 0U)
    {
        return DM_MOTOR_STATUS_INVALID_ID;
    }

    encoded_position = ((uint32_t)frame->data[1] << 8U) | frame->data[2];
    encoded_velocity = ((uint32_t)frame->data[3] << 4U) | (frame->data[4] >> 4U);
    encoded_torque = (((uint32_t)frame->data[4] & 0x0FU) << 8U) | frame->data[5];
    feedback->position_rad = dm_motor_unsigned_to_symmetric_float(encoded_position,
                                                                   ranges->position_max_rad,
                                                                   16U);
    feedback->velocity_rad_s = dm_motor_unsigned_to_symmetric_float(encoded_velocity,
                                                                     ranges->velocity_max_rad_s,
                                                                     12U);
    feedback->torque_nm = dm_motor_unsigned_to_symmetric_float(encoded_torque,
                                                                ranges->torque_max_nm,
                                                                12U);
    feedback->mos_temperature_c = (float)frame->data[6];
    feedback->rotor_temperature_c = (float)frame->data[7];
    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Decodes one eight-byte response to a motor parameter read.
 * @param frame Received classic CAN frame.
 * @param response Destination parameter response.
 * @return Protocol status describing success or the validation failure.
 */
DmMotorStatus dm_motor_decode_parameter_response(const FdcanClassicFrame *frame,
                                                  DmMotorParameterResponse *response)
{
    uint32_t raw_value;

    if (frame == NULL || response == NULL)
    {
        return DM_MOTOR_STATUS_INVALID_ARGUMENT;
    }
    if (frame->length != 8U)
    {
        return DM_MOTOR_STATUS_INVALID_LENGTH;
    }
    if ((frame->data[2] != 0x33U) && (frame->data[2] != 0x55U))
    {
        return DM_MOTOR_STATUS_INVALID_FRAME;
    }

    response->motor_id = (uint16_t)frame->data[0] | ((uint16_t)(frame->data[1] & 0x07U) << 8U);
    response->register_address = frame->data[3];
    raw_value = dm_motor_read_u32_little_endian(&frame->data[4]);
    response->raw_value = raw_value;
    memcpy(&response->float_value, &raw_value, sizeof(response->float_value));
    return DM_MOTOR_STATUS_OK;
}

/**
 * @brief Derives motor-rotor angle from the S3519 output-shaft angle for telemetry only.
 * @param output_angle Output-shaft angle in any angular unit.
 * @return Motor-rotor angle in the same angular unit.
 */
float dm_motor_output_to_rotor_angle(float output_angle)
{
    return output_angle * DM_MOTOR_GEAR_RATIO;
}
