/**
 * @file can_frame.h
 * @brief Defines a HAL-independent standard Classic CAN data frame.
 */

#ifndef APP_MOTOR_CAN_FRAME_H
#define APP_MOTOR_CAN_FRAME_H

#include <stdint.h>

#define CAN_STANDARD_MAX_IDENTIFIER (0x7FFU)
#define CAN_CLASSIC_MAX_DATA_LENGTH (8U)

/**
 * @brief Reports Classic CAN frame validation outcomes.
 */
typedef enum
{
    CAN_FRAME_STATUS_OK = 0,
    CAN_FRAME_STATUS_INVALID_ARGUMENT,
    CAN_FRAME_STATUS_INVALID_IDENTIFIER,
    CAN_FRAME_STATUS_INVALID_LENGTH
} CanFrameStatus;

/**
 * @brief Owns one standard 11-bit Classic CAN data frame.
 */
typedef struct
{
    uint16_t identifier;
    uint8_t length;
    uint8_t data[CAN_CLASSIC_MAX_DATA_LENGTH];
} CanFrame;

/**
 * @brief Initializes a validated Classic CAN frame.
 * @param frame Destination frame.
 * @param identifier Standard 11-bit identifier.
 * @param data Payload bytes, or null only for a zero-length payload.
 * @param length Payload length from zero through eight.
 * @return Detailed validation status.
 */
CanFrameStatus can_frame_init(CanFrame *frame,
                              uint16_t identifier,
                              const uint8_t *data,
                              uint8_t length);

#endif
