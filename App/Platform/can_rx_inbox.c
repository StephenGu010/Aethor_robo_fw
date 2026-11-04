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
CanRxInboxStatus can_rx_inbox_push_isr_at(CanRxInbox *inbox, const CanFrame *frame,
    uint64_t arrival_lower_bound_us)
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
    inbox->arrival_lower_bound_us[write_index] = arrival_lower_bound_us;
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
CanRxInboxStatus can_rx_inbox_pop_at(CanRxInbox *inbox, CanFrame *frame,
    uint64_t *arrival_lower_bound_us)
{
    uint8_t read_index;

    if ((inbox == NULL) || (inbox->initialized == 0U) || (frame == NULL) ||
        (arrival_lower_bound_us == NULL))
    {
        return CAN_RX_INBOX_STATUS_INVALID_ARGUMENT;
    }

    read_index = inbox->read_index;
    if (read_index == inbox->write_index)
    {
        return CAN_RX_INBOX_STATUS_EMPTY;
    }

    *frame = inbox->frames[read_index];
    *arrival_lower_bound_us = inbox->arrival_lower_bound_us[read_index];
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

/** @brief Legacy producers explicitly publish unknown arrival timing. */
CanRxInboxStatus can_rx_inbox_push_isr(CanRxInbox *inbox, const CanFrame *frame)
{
    return can_rx_inbox_push_isr_at(inbox, frame, 0ULL);
}

/** @brief Legacy consumers discard timing without relabelling arrival time. */
CanRxInboxStatus can_rx_inbox_pop(CanRxInbox *inbox, CanFrame *frame)
{
    uint64_t ignored_lower_bound_us;
    return can_rx_inbox_pop_at(inbox, frame, &ignored_lower_bound_us);
}

/** @brief Keeps older safe bounds across backlog; invalid clock observations become unknown. */
uint64_t can_rx_inbox_observe_lower_bound(uint64_t previous_us, uint64_t before_observation_us,
    uint32_t fifo0_count, uint32_t fifo1_count, uint32_t pending_tx)
{
    if (before_observation_us == 0ULL || before_observation_us < previous_us) { return 0ULL; }
    if (fifo0_count != 0U || fifo1_count != 0U || pending_tx != 0U) { return previous_us; }
    return before_observation_us;
}
