/**
 * @file can_frame.c
 * @brief Implements HAL-independent Classic CAN frame validation.
 */

#include "can_frame.h"

#include <stddef.h>
#include <string.h>

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
                              uint8_t length)
{
    if ((frame == NULL) || ((data == NULL) && (length != 0U)))
    {
        return CAN_FRAME_STATUS_INVALID_ARGUMENT;
    }

    if (identifier > CAN_STANDARD_MAX_IDENTIFIER)
    {
        return CAN_FRAME_STATUS_INVALID_IDENTIFIER;
    }

    if (length > CAN_CLASSIC_MAX_DATA_LENGTH)
    {
        return CAN_FRAME_STATUS_INVALID_LENGTH;
    }

    frame->identifier = identifier;
    frame->length = length;
    memset(frame->data, 0, sizeof(frame->data));
    if (length != 0U)
    {
        memcpy(frame->data, data, length);
    }
    return CAN_FRAME_STATUS_OK;
}
