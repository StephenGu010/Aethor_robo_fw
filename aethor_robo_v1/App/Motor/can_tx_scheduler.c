/**
 * @file can_tx_scheduler.c
 * @brief Implements bounded emergency-first and round-robin CAN scheduling.
 */

#include "can_tx_scheduler.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Checks whether a copied frame remains a valid standard Classic CAN frame.
 * @param frame Frame to inspect.
 * @return One when valid, otherwise zero.
 */
static uint8_t can_tx_scheduler_frame_is_valid(const CanFrame *frame)
{
    return (uint8_t)((frame != NULL) &&
                     (frame->identifier <= CAN_STANDARD_MAX_IDENTIFIER) &&
                     (frame->length <= CAN_CLASSIC_MAX_DATA_LENGTH));
}

/**
 * @brief Initializes an empty scheduler.
 * @param scheduler Scheduler storage.
 */
void can_tx_scheduler_init(CanTxScheduler *scheduler)
{
    if (scheduler == NULL)
    {
        return;
    }

    memset(scheduler, 0, sizeof(*scheduler));
    scheduler->initialized = 1U;
}

/**
 * @brief Queues one non-droppable emergency frame in FIFO order.
 * @param scheduler Initialized scheduler.
 * @param frame Frame to copy.
 * @return OK or an explicit capacity/argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_emergency(CanTxScheduler *scheduler,
                                                       const CanFrame *frame)
{
    uint8_t write_index;

    if ((scheduler == NULL) || (scheduler->initialized == 0U) ||
        (can_tx_scheduler_frame_is_valid(frame) == 0U))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }

    if (scheduler->emergency_count >= CAN_TX_EMERGENCY_QUEUE_CAPACITY)
    {
        ++scheduler->emergency_queue_full_count;
        return CAN_TX_SCHEDULER_STATUS_EMERGENCY_QUEUE_FULL;
    }

    write_index = (uint8_t)((scheduler->emergency_head + scheduler->emergency_count) %
                            CAN_TX_EMERGENCY_QUEUE_CAPACITY);
    scheduler->emergency_frames[write_index] = *frame;
    ++scheduler->emergency_count;
    if (scheduler->emergency_count > scheduler->emergency_high_watermark)
    {
        scheduler->emergency_high_watermark = scheduler->emergency_count;
    }
    return CAN_TX_SCHEDULER_STATUS_OK;
}

/**
 * @brief Stores the latest control frame for one joint, replacing older pending data.
 * @param scheduler Initialized scheduler.
 * @param joint_index Zero-based joint index.
 * @param frame Frame to copy.
 * @return OK, REPLACED, or an argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_joint(CanTxScheduler *scheduler,
                                                   uint8_t joint_index,
                                                   const CanFrame *frame)
{
    uint8_t joint_bit;
    CanTxSchedulerStatus result = CAN_TX_SCHEDULER_STATUS_OK;

    if ((scheduler == NULL) || (scheduler->initialized == 0U) ||
        (can_tx_scheduler_frame_is_valid(frame) == 0U))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }

    if (joint_index >= ARM_JOINT_COUNT)
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_JOINT;
    }

    joint_bit = (uint8_t)(1U << joint_index);
    if ((scheduler->pending_joint_mask & joint_bit) != 0U)
    {
        ++scheduler->coalesced_joint_frame_count;
        result = CAN_TX_SCHEDULER_STATUS_REPLACED;
    }

    scheduler->joint_frames[joint_index] = *frame;
    scheduler->pending_joint_mask |= joint_bit;
    return result;
}

/**
 * @brief Pops the highest-priority pending frame without blocking.
 * @param scheduler Initialized scheduler.
 * @param frame Destination frame.
 * @param priority Destination priority classification.
 * @return OK or EMPTY/argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_pop(CanTxScheduler *scheduler,
                                          CanFrame *frame,
                                          CanTxPriority *priority)
{
    uint8_t scan_offset;

    if ((scheduler == NULL) || (scheduler->initialized == 0U) ||
        (frame == NULL) || (priority == NULL))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }

    if (scheduler->emergency_count != 0U)
    {
        *frame = scheduler->emergency_frames[scheduler->emergency_head];
        scheduler->emergency_head = (uint8_t)((scheduler->emergency_head + 1U) %
                                              CAN_TX_EMERGENCY_QUEUE_CAPACITY);
        --scheduler->emergency_count;
        *priority = CAN_TX_PRIORITY_EMERGENCY;
        return CAN_TX_SCHEDULER_STATUS_OK;
    }

    for (scan_offset = 0U; scan_offset < ARM_JOINT_COUNT; ++scan_offset)
    {
        uint8_t joint_index = (uint8_t)((scheduler->next_joint_index + scan_offset) %
                                        ARM_JOINT_COUNT);
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((scheduler->pending_joint_mask & joint_bit) != 0U)
        {
            *frame = scheduler->joint_frames[joint_index];
            scheduler->pending_joint_mask &= (uint8_t)~joint_bit;
            scheduler->next_joint_index = (uint8_t)((joint_index + 1U) % ARM_JOINT_COUNT);
            *priority = CAN_TX_PRIORITY_JOINT_CONTROL;
            return CAN_TX_SCHEDULER_STATUS_OK;
        }
    }

    return CAN_TX_SCHEDULER_STATUS_EMPTY;
}
