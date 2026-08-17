/**
 * @file dm_motor_protocol.h
 * @brief Hardware-independent S3519 classic CAN protocol definitions and codecs.
 */

#ifndef DM_MOTOR_PROTOCOL_H
#define DM_MOTOR_PROTOCOL_H

#include <stdint.h>

#include "fdcan_classic_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DM_MOTOR_POSITION_MODE_OFFSET 0x100U
#define DM_MOTOR_PARAMETER_COMMAND_ID 0x7FFU
#define DM_MOTOR_DEFAULT_MASTER_ID 0x000U
#define DM_MOTOR_GEAR_RATIO 19.2f
#define DM_MOTOR_MINIMUM_ID 1U
#define DM_MOTOR_MAXIMUM_ID 15U

/** @brief Supported non-persistent motor register addresses used during discovery. */
typedef enum
{
    DM_MOTOR_REGISTER_MASTER_ID = 0x07U,
    DM_MOTOR_REGISTER_RECEIVE_ID = 0x08U,
    DM_MOTOR_REGISTER_CONTROL_MODE = 0x0AU,
    DM_MOTOR_REGISTER_POSITION_RANGE = 0x15U,
    DM_MOTOR_REGISTER_VELOCITY_RANGE = 0x16U,
    DM_MOTOR_REGISTER_TORQUE_RANGE = 0x17U
} DmMotorRegister;

/** @brief Result codes returned by S3519 protocol functions. */
typedef enum
{
    DM_MOTOR_STATUS_OK = 0,
    DM_MOTOR_STATUS_INVALID_ARGUMENT,
    DM_MOTOR_STATUS_INVALID_ID,
    DM_MOTOR_STATUS_INVALID_LENGTH,
    DM_MOTOR_STATUS_INVALID_RANGE,
    DM_MOTOR_STATUS_INVALID_FRAME
} DmMotorStatus;

/** @brief Special mode command encoded in the final byte of an eight-byte control frame. */
typedef enum
{
    DM_MOTOR_MODE_COMMAND_CLEAR_ERROR = 0xFBU,
    DM_MOTOR_MODE_COMMAND_ENABLE = 0xFCU,
    DM_MOTOR_MODE_COMMAND_DISABLE = 0xFDU,
    DM_MOTOR_MODE_COMMAND_SAVE_ZERO = 0xFEU
} DmMotorModeCommand;

/** @brief Runtime mapping ranges read from the motor before feedback decoding. */
typedef struct
{
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
} DmMotorRanges;

/** @brief Decoded motor feedback expressed in output-shaft units. */
typedef struct
{
    uint8_t motor_id;
    uint8_t state;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
} DmMotorFeedback;

/** @brief Decoded response to a non-persistent motor register read. */
typedef struct
{
    uint16_t motor_id;
    uint8_t register_address;
    uint32_t raw_value;
    float float_value;
} DmMotorParameterResponse;

DmMotorStatus dm_motor_pack_position_velocity(uint8_t motor_id,
                                               float output_position_rad,
                                               float output_velocity_rad_s,
                                               FdcanClassicFrame *frame);
DmMotorStatus dm_motor_pack_mode_command(uint8_t motor_id,
                                         DmMotorModeCommand command,
                                         FdcanClassicFrame *frame);
DmMotorStatus dm_motor_pack_parameter_read(uint8_t motor_id,
                                            DmMotorRegister register_address,
                                            FdcanClassicFrame *frame);
DmMotorStatus dm_motor_pack_parameter_write_u32(uint8_t motor_id,
                                                DmMotorRegister register_address,
                                                uint32_t value,
                                                FdcanClassicFrame *frame);
DmMotorStatus dm_motor_decode_feedback(const FdcanClassicFrame *frame,
                                        const DmMotorRanges *ranges,
                                        DmMotorFeedback *feedback);
DmMotorStatus dm_motor_decode_parameter_response(const FdcanClassicFrame *frame,
                                                  DmMotorParameterResponse *response);
float dm_motor_output_to_rotor_angle(float output_angle);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_PROTOCOL_H */
