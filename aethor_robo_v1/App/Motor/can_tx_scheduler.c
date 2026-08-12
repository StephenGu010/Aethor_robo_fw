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
 * @brief Checks whether a public priority value is recognized.
 * @param priority Priority to inspect.
 * @return One when valid, otherwise zero.
 */
static uint8_t can_tx_scheduler_priority_is_valid(CanTxPriority priority)
{
    return (uint8_t)(priority <= CAN_TX_PRIORITY_PARAMETER);
}

/**
 * @brief Appends one already validated entry without a capacity check.
 * @param scheduler Scheduler with at least one free slot.
 * @param priority Entry priority.
 * @param frame Frame to copy.
 */
static void can_tx_scheduler_append(CanTxScheduler *scheduler,
                                    CanTxPriority priority,
                                    const CanFrame *frame)
{
    CanTxSchedulerEntry *entry = &scheduler->entries[scheduler->count];

    entry->frame = *frame;
    entry->priority = priority;
    entry->sequence = scheduler->next_sequence;
    ++scheduler->next_sequence;
    ++scheduler->count;
    if (scheduler->count > scheduler->high_watermark)
    {
        scheduler->high_watermark = scheduler->count;
    }
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
 * @brief Queues one frame with a stable priority classification.
 * @param scheduler Initialized scheduler.
 * @param priority Scheduling priority.
 * @param frame Frame to copy.
 * @return OK or an explicit capacity/argument/priority error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit(CanTxScheduler *scheduler,
                                             CanTxPriority priority,
                                             const CanFrame *frame)
{
    if ((scheduler == NULL) || (scheduler->initialized == 0U) ||
        (can_tx_scheduler_frame_is_valid(frame) == 0U))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }
    if (can_tx_scheduler_priority_is_valid(priority) == 0U)
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_PRIORITY;
    }
    if (scheduler->count >= CAN_TX_SCHEDULER_CAPACITY)
    {
        ++scheduler->full_count;
        return CAN_TX_SCHEDULER_STATUS_FULL;
    }

    can_tx_scheduler_append(scheduler, priority, frame);
    return CAN_TX_SCHEDULER_STATUS_OK;
}

/**
 * @brief Atomically queues one complete J1 through J7 control group.
 * @param scheduler Initialized scheduler.
 * @param frames Ordered control frames.
 * @param frame_count Must equal ARM_JOINT_COUNT.
 * @return OK or GROUP_REJECTED without a partial enqueue.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_control_group(CanTxScheduler *scheduler,
                                                           const CanFrame *frames,
                                                           uint8_t frame_count)
{
    uint8_t frame_index;

    if ((scheduler == NULL) || (scheduler->initialized == 0U) || (frames == NULL) ||
        (frame_count != ARM_JOINT_COUNT))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }
    for (frame_index = 0U; frame_index < frame_count; ++frame_index)
    {
        if (can_tx_scheduler_frame_is_valid(&frames[frame_index]) == 0U)
        {
            return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
        }
    }
    if ((uint8_t)(CAN_TX_SCHEDULER_CAPACITY - scheduler->count) < frame_count)
    {
        ++scheduler->atomic_group_reject_count;
        return CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED;
    }

    for (frame_index = 0U; frame_index < frame_count; ++frame_index)
    {
        can_tx_scheduler_append(scheduler,
                                CAN_TX_PRIORITY_JOINT_CONTROL,
                                &frames[frame_index]);
    }
    return CAN_TX_SCHEDULER_STATUS_OK;
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
    uint8_t entry_index;
    uint8_t selected_index = 0U;
    uint8_t compact_index;

    if ((scheduler == NULL) || (scheduler->initialized == 0U) ||
        (frame == NULL) || (priority == NULL))
    {
        return CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT;
    }

    if (scheduler->count == 0U)
    {
        return CAN_TX_SCHEDULER_STATUS_EMPTY;
    }

    for (entry_index = 1U; entry_index < scheduler->count; ++entry_index)
    {
        const CanTxSchedulerEntry *candidate = &scheduler->entries[entry_index];
        const CanTxSchedulerEntry *selected = &scheduler->entries[selected_index];

        if ((candidate->priority < selected->priority) ||
            ((candidate->priority == selected->priority) &&
             (candidate->sequence < selected->sequence)))
        {
            selected_index = entry_index;
        }
    }

    *frame = scheduler->entries[selected_index].frame;
    *priority = scheduler->entries[selected_index].priority;
    for (compact_index = selected_index;
         compact_index < (uint8_t)(scheduler->count - 1U);
         ++compact_index)
    {
        scheduler->entries[compact_index] = scheduler->entries[compact_index + 1U];
    }
    --scheduler->count;
    return CAN_TX_SCHEDULER_STATUS_OK;
}
