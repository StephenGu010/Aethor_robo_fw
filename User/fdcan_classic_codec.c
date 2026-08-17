/**
 * @file fdcan_classic_codec.c
 * @brief Implements strict classic CAN frame validation without HAL dependencies.
 */

#include "fdcan_classic_codec.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Converts a classic CAN payload length to its DLC value.
 * @param length Payload length in bytes.
 * @param dlc Destination for the DLC value.
 * @return Codec status describing success or the validation failure.
 */
FdcanClassicStatus fdcan_classic_length_to_dlc(uint8_t length, uint8_t *dlc)
{
    if (dlc == NULL)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_ARGUMENT;
    }
    if (length > FDCAN_CLASSIC_MAX_DATA_LENGTH)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_LENGTH;
    }

    *dlc = length;
    return FDCAN_CLASSIC_STATUS_OK;
}

/**
 * @brief Converts a DLC value to a classic CAN payload length.
 * @param dlc DLC value from an FDCAN receive header.
 * @param length Destination for the decoded byte length.
 * @return Codec status describing success or the validation failure.
 */
FdcanClassicStatus fdcan_classic_dlc_to_length(uint8_t dlc, uint8_t *length)
{
    if (length == NULL)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_ARGUMENT;
    }
    if (dlc > FDCAN_CLASSIC_MAX_DATA_LENGTH)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_DLC;
    }

    *length = dlc;
    return FDCAN_CLASSIC_STATUS_OK;
}

/**
 * @brief Initializes a validated classic CAN data frame.
 * @param frame Destination frame.
 * @param identifier Standard 11-bit CAN identifier.
 * @param data Payload bytes, or NULL only when length is zero.
 * @param length Payload length in bytes.
 * @return Codec status describing success or the validation failure.
 */
FdcanClassicStatus fdcan_classic_frame_init(FdcanClassicFrame *frame,
                                             uint16_t identifier,
                                             const uint8_t *data,
                                             uint8_t length)
{
    if (frame == NULL || (length > 0U && data == NULL))
    {
        return FDCAN_CLASSIC_STATUS_INVALID_ARGUMENT;
    }
    if (identifier > FDCAN_CLASSIC_MAX_IDENTIFIER)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_ID;
    }
    if (length > FDCAN_CLASSIC_MAX_DATA_LENGTH)
    {
        return FDCAN_CLASSIC_STATUS_INVALID_LENGTH;
    }

    frame->identifier = identifier;
    frame->length = length;
    memset(frame->data, 0, sizeof(frame->data));
    if (length > 0U)
    {
        memcpy(frame->data, data, length);
    }

    return FDCAN_CLASSIC_STATUS_OK;
}
