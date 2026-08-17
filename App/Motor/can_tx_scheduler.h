/**
 * @file can_tx_scheduler.h
 * @brief Defines a bounded priority scheduler for seven-axis CAN transmit traffic.
 */

#ifndef APP_MOTOR_CAN_TX_SCHEDULER_H
#define APP_MOTOR_CAN_TX_SCHEDULER_H

#include <stdint.h>

#include "arm_config.h"
#include "can_frame.h"

#define CAN_TX_SCHEDULER_CAPACITY (32U)

/**
 * @brief Identifies the source priority of a scheduled frame.
 */
typedef enum
{
    CAN_TX_PRIORITY_EMERGENCY = 0,
    CAN_TX_PRIORITY_JOINT_CONTROL,
    CAN_TX_PRIORITY_FEEDBACK_QUERY,
    CAN_TX_PRIORITY_PARAMETER
} CanTxPriority;

/**
 * @brief Reports bounded scheduler outcomes.
 */
typedef enum
{
    CAN_TX_SCHEDULER_STATUS_OK = 0,
    CAN_TX_SCHEDULER_STATUS_EMPTY,
    CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT,
    CAN_TX_SCHEDULER_STATUS_INVALID_PRIORITY,
    CAN_TX_SCHEDULER_STATUS_FULL,
    CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED
} CanTxSchedulerStatus;

/**
 * @brief Owns one queued frame and its scheduling priority.
 */
typedef struct
{
    CanFrame frame;
    CanTxPriority priority;
    uint32_t sequence;
} CanTxSchedulerEntry;

/**
 * @brief Owns a fixed-capacity stable-priority CAN transmit ring.
 */
typedef struct
{
    CanTxSchedulerEntry entries[CAN_TX_SCHEDULER_CAPACITY];
    uint32_t next_sequence;
    uint32_t full_count;
    uint32_t atomic_group_reject_count;
    uint32_t emergency_eviction_count;
    uint8_t count;
    uint8_t high_watermark;
    uint8_t initialized;
} CanTxScheduler;

/**
 * @brief Initializes an empty scheduler.
 * @param scheduler Scheduler storage.
 */
void can_tx_scheduler_init(CanTxScheduler *scheduler);

/**
 * @brief Queues one frame with a stable priority classification.
 * @param scheduler Initialized scheduler.
 * @param priority Scheduling priority.
 * @param frame Frame to copy.
 * @return OK or an explicit capacity/argument/priority error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit(CanTxScheduler *scheduler,
                                             CanTxPriority priority,
                                             const CanFrame *frame);

/**
 * @brief Atomically queues one complete J1 through J7 control group.
 * @param scheduler Initialized scheduler.
 * @param frames Ordered control frames.
 * @param frame_count Must equal ARM_JOINT_COUNT.
 * @return OK or GROUP_REJECTED without a partial enqueue.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_control_group(CanTxScheduler *scheduler,
                                                           const CanFrame *frames,
                                                           uint8_t frame_count);

/**
 * @brief Pops the highest-priority pending frame without blocking.
 * @param scheduler Initialized scheduler.
 * @param frame Destination frame.
 * @param priority Destination priority classification.
 * @return OK or EMPTY/argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_pop(CanTxScheduler *scheduler,
                                          CanFrame *frame,
                                          CanTxPriority *priority);

#endif
