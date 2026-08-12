/**
 * @file s3519_codec.h
 * @brief Defines HAL-independent S3519 Classic CAN command and feedback codecs.
 */

#ifndef APP_MOTOR_S3519_CODEC_H
#define APP_MOTOR_S3519_CODEC_H

#include <stdint.h>

#include "can_frame.h"

#define S3519_PARAMETER_COMMAND_IDENTIFIER (0x7FFU)
#define S3519_MINIMUM_ESC_ID (1U)
#define S3519_MAXIMUM_ESC_ID (15U)
#define S3519_KP_MIN (0.0F)
#define S3519_KP_MAX (500.0F)
#define S3519_KD_MIN (0.0F)
#define S3519_KD_MAX (5.0F)

/**
 * @brief Identifies S3519 CAN identifier offsets for supported control modes.
 */
typedef enum
{
    S3519_CONTROL_MODE_MIT = 0x000U,
    S3519_CONTROL_MODE_POSITION_VELOCITY = 0x100U
} S3519ControlMode;

/**
 * @brief Identifies special commands carried in the final control byte.
 */
typedef enum
{
    S3519_MODE_COMMAND_CLEAR_ERROR = 0xFBU,
    S3519_MODE_COMMAND_ENABLE = 0xFCU,
    S3519_MODE_COMMAND_DISABLE = 0xFDU,
    S3519_MODE_COMMAND_SAVE_ZERO = 0xFEU
} S3519ModeCommand;

/**
 * @brief Identifies the discovery registers required by the PRD.
 */
typedef enum
{
    S3519_REGISTER_ACCELERATION = 0x04U,
    S3519_REGISTER_DECELERATION = 0x05U,
    S3519_REGISTER_MAXIMUM_SPEED = 0x06U,
    S3519_REGISTER_MASTER_ID = 0x07U,
    S3519_REGISTER_ESC_ID = 0x08U,
    S3519_REGISTER_CONTROL_MODE = 0x0AU,
    S3519_REGISTER_HARDWARE_VERSION = 0x0DU,
    S3519_REGISTER_SOFTWARE_VERSION = 0x0EU,
    S3519_REGISTER_POSITION_RANGE = 0x15U,
    S3519_REGISTER_VELOCITY_RANGE = 0x16U,
    S3519_REGISTER_TORQUE_RANGE = 0x17U,
    S3519_REGISTER_SUB_VERSION = 0x24U
} S3519Register;

/**
 * @brief Reports S3519 validation and encoding outcomes.
 */
typedef enum
{
    S3519_CODEC_STATUS_OK = 0,
    S3519_CODEC_STATUS_INVALID_ARGUMENT,
    S3519_CODEC_STATUS_INVALID_ID,
    S3519_CODEC_STATUS_INVALID_MODE,
    S3519_CODEC_STATUS_INVALID_RANGE,
    S3519_CODEC_STATUS_INVALID_LENGTH,
    S3519_CODEC_STATUS_INVALID_FRAME
} S3519CodecStatus;

/**
 * @brief Stores runtime PMAX, VMAX, and TMAX values read from one motor.
 */
typedef struct
{
    float position_max_rad;
    float velocity_max_rad_s;
    float torque_max_nm;
} S3519Ranges;

/**
 * @brief Stores decoded S3519 control feedback.
 */
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    uint8_t esc_id;
    uint8_t state;
} S3519Feedback;

/**
 * @brief Stores one decoded non-persistent register response.
 */
typedef struct
{
    uint32_t raw_value;
    float float_value;
    uint16_t esc_id;
    uint8_t register_address;
} S3519ParameterResponse;

/**
 * @brief Packs one position-velocity command using little-endian float values.
 */
S3519CodecStatus s3519_pack_position_velocity(uint8_t esc_id,
                                              float position_rad,
                                              float velocity_rad_s,
                                              CanFrame *frame);

/**
 * @brief Packs enable, disable, clear-error, or save-zero for one control mode.
 */
S3519CodecStatus s3519_pack_mode_command(uint8_t esc_id,
                                        S3519ControlMode control_mode,
                                        S3519ModeCommand command,
                                        CanFrame *frame);

/**
 * @brief Packs one volatile discovery register read.
 */
S3519CodecStatus s3519_pack_parameter_read(uint8_t esc_id,
                                          S3519Register register_address,
                                          CanFrame *frame);

/**
 * @brief Packs one MIT command using discovered motor ranges.
 */
S3519CodecStatus s3519_pack_mit(uint8_t esc_id,
                               const S3519Ranges *ranges,
                               float position_rad,
                               float velocity_rad_s,
                               float kp,
                               float kd,
                               float torque_nm,
                               CanFrame *frame);

/**
 * @brief Decodes one eight-byte S3519 control feedback frame.
 */
S3519CodecStatus s3519_decode_feedback(const CanFrame *frame,
                                       const S3519Ranges *ranges,
                                       S3519Feedback *feedback);

/**
 * @brief Decodes one eight-byte S3519 register response.
 */
S3519CodecStatus s3519_decode_parameter_response(const CanFrame *frame,
                                                 S3519ParameterResponse *response);

#endif
