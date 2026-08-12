/**
 * @file can_rx_inbox.h
 * @brief Defines a bounded single-producer CAN ISR inbox.
 */

#ifndef APP_PLATFORM_CAN_RX_INBOX_H
#define APP_PLATFORM_CAN_RX_INBOX_H

#include <stdint.h>

#include "can_frame.h"

#define CAN_RX_INBOX_CAPACITY (16U)

/**
 * @brief Reports non-blocking CAN inbox outcomes.
 */
typedef enum
{
    CAN_RX_INBOX_STATUS_OK = 0,
    CAN_RX_INBOX_STATUS_EMPTY,
    CAN_RX_INBOX_STATUS_FULL,
    CAN_RX_INBOX_STATUS_INVALID_ARGUMENT
} CanRxInboxStatus;

/**
 * @brief Owns ISR-produced CAN frames and latched bus health.
 */
typedef struct
{
    CanFrame frames[CAN_RX_INBOX_CAPACITY + 1U];
    volatile uint32_t received_frame_count;
    volatile uint32_t dropped_frame_count;
    volatile uint32_t bus_off_event_count;
    volatile uint8_t write_index;
    volatile uint8_t read_index;
    volatile uint8_t bus_off_latched;
    uint8_t initialized;
} CanRxInbox;

/**
 * @brief Initializes an empty CAN receive inbox.
 */
void can_rx_inbox_init(CanRxInbox *inbox);

/**
 * @brief Copies one validated frame from ISR context without blocking.
 */
CanRxInboxStatus can_rx_inbox_push_isr(CanRxInbox *inbox, const CanFrame *frame);

/**
 * @brief Pops one frame from task context.
 */
CanRxInboxStatus can_rx_inbox_pop(CanRxInbox *inbox, CanFrame *frame);

/**
 * @brief Latches Bus-Off from ISR context until an explicit platform restart.
 */
void can_rx_inbox_latch_bus_off_isr(CanRxInbox *inbox);

#endif
