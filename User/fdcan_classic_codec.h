/**
 * @file fdcan_classic_codec.h
 * @brief Hardware-independent validation and representation for classic CAN frames.
 */

#ifndef FDCAN_CLASSIC_CODEC_H
#define FDCAN_CLASSIC_CODEC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FDCAN_CLASSIC_MAX_IDENTIFIER 0x7FFU
#define FDCAN_CLASSIC_MAX_DATA_LENGTH 8U

/** @brief Result codes returned by classic CAN codec functions. */
typedef enum
{
    FDCAN_CLASSIC_STATUS_OK = 0,
    FDCAN_CLASSIC_STATUS_INVALID_ARGUMENT,
    FDCAN_CLASSIC_STATUS_INVALID_ID,
    FDCAN_CLASSIC_STATUS_INVALID_LENGTH,
    FDCAN_CLASSIC_STATUS_INVALID_DLC
} FdcanClassicStatus;

/** @brief Hardware-independent representation of one 11-bit classic CAN data frame. */
typedef struct
{
    uint16_t identifier;
    uint8_t length;
    uint8_t data[FDCAN_CLASSIC_MAX_DATA_LENGTH];
} FdcanClassicFrame;

FdcanClassicStatus fdcan_classic_length_to_dlc(uint8_t length, uint8_t *dlc);
FdcanClassicStatus fdcan_classic_dlc_to_length(uint8_t dlc, uint8_t *length);
FdcanClassicStatus fdcan_classic_frame_init(FdcanClassicFrame *frame,
                                             uint16_t identifier,
                                             const uint8_t *data,
                                             uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_CLASSIC_CODEC_H */
