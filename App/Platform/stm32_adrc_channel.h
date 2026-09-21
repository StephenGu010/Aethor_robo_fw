/** @file stm32_adrc_channel.h
 * @brief Single-owner dedicated FDCAN buffer for expiring ADRC commands and transmit receipts.
 */
#ifndef APP_PLATFORM_STM32_ADRC_CHANNEL_H
#define APP_PLATFORM_STM32_ADRC_CHANNEL_H
#include "can_frame.h"

/** @brief Identifies the one outstanding operation; torque receipts alone feed the observer. */
typedef enum { ADRC_CAN_ENABLE = 0, ADRC_CAN_TORQUE, ADRC_CAN_DISABLE } AdrcCanKind;
/** @brief Reports bus transmission, never mere queue acceptance or motor execution. */
typedef struct
{
    float decoded_torque_nm;
    AdrcCanKind kind;
    uint8_t available;
    uint8_t transmitted;
} AdrcCanReceipt;

/** @brief Clears software bookkeeping before FDCAN starts; does not issue an actuator command. */
void stm32_adrc_channel_init(void);
/** @brief Poll once at the next control tick; an unfinished request is cancelled and reported failed. */
AdrcCanReceipt stm32_adrc_channel_collect(void);
/** @brief Submits at most one frame in dedicated buffer 0; never appends to the legacy FIFO. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float decoded_torque_nm);
#endif
