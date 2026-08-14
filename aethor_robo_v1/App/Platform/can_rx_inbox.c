/**
 * @file can_rx_inbox.c
 * @brief Implements a bounded single-producer, single-consumer CAN inbox.
 */

#include "can_rx_inbox.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Checks whether a frame remains a valid standard Classic CAN frame.
 */
static uint8_t can_rx_inbox_frame_is_valid(const CanFrame *frame)
{
    return (uint8_t)((frame != NULL) &&
                     (frame->identifier <= CAN_STANDARD_MAX_IDENTIFIER) &&
                     (frame->length <= CAN_CLASSIC_MAX_DATA_LENGTH));
}

/**
 * @brief Initializes an empty CAN receive inbox.
 * @param inbox Destination inbox.
 */
void can_rx_inbox_init(CanRxInbox *inbox)
{
    if (inbox == NULL)
    {
        return;
    }

    memset(inbox, 0, sizeof(*inbox));
    inbox->initialized = 1U;
}

/**
 * @brief Copies one validated frame from ISR context without blocking.
 * @param inbox Initialized inbox.
 * @param frame Received frame to copy.
 * @return OK, FULL, or an argument error.
 */
CanRxInboxStatus can_rx_inbox_push_isr(CanRxInbox *inbox, const CanFrame *frame)
{
    uint8_t write_index;
    uint8_t next_write_index;

    if ((inbox == NULL) || (inbox->initialized == 0U) ||
        (can_rx_inbox_frame_is_valid(frame) == 0U))
    {
        return CAN_RX_INBOX_STATUS_INVALID_ARGUMENT;
    }

    write_index = inbox->write_index;
    next_write_index = (uint8_t)((write_index + 1U) % (CAN_RX_INBOX_CAPACITY + 1U));
    if (next_write_index == inbox->read_index)
    {
        ++inbox->dropped_frame_count;
        return CAN_RX_INBOX_STATUS_FULL;
    }

    inbox->frames[write_index] = *frame;
    inbox->write_index = next_write_index;
    ++inbox->received_frame_count;
    return CAN_RX_INBOX_STATUS_OK;
}

/**
 * @brief Pops one frame from task context.
 * @param inbox Initialized inbox.
 * @param frame Destination frame.
 * @return OK, EMPTY, or an argument error.
 */
CanRxInboxStatus can_rx_inbox_pop(CanRxInbox *inbox, CanFrame *frame)
{
    uint8_t read_index;

    if ((inbox == NULL) || (inbox->initialized == 0U) || (frame == NULL))
    {
        return CAN_RX_INBOX_STATUS_INVALID_ARGUMENT;
    }

    read_index = inbox->read_index;
    if (read_index == inbox->write_index)
    {
        return CAN_RX_INBOX_STATUS_EMPTY;
    }

    *frame = inbox->frames[read_index];
    inbox->read_index = (uint8_t)((read_index + 1U) % (CAN_RX_INBOX_CAPACITY + 1U));
    return CAN_RX_INBOX_STATUS_OK;
}

/**
 * @brief Latches Bus-Off from ISR context until an explicit platform restart.
 * @param inbox Initialized inbox.
 */
void can_rx_inbox_latch_bus_off_isr(CanRxInbox *inbox)
{
    if ((inbox == NULL) || (inbox->initialized == 0U))
    {
        return;
    }

    inbox->bus_off_latched = 1U;
    ++inbox->bus_off_event_count;
}
