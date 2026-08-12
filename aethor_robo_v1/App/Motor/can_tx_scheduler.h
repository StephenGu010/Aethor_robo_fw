/**
 * @file can_tx_scheduler.h
 * @brief Defines a bounded priority scheduler for seven-axis CAN transmit traffic.
 */

#ifndef APP_MOTOR_CAN_TX_SCHEDULER_H
#define APP_MOTOR_CAN_TX_SCHEDULER_H

#include <stdint.h>

#include "arm_config.h"
#include "can_frame.h"

#define CAN_TX_EMERGENCY_QUEUE_CAPACITY (8U)

/**
 * @brief Identifies the source priority of a scheduled frame.
 */
typedef enum
{
    CAN_TX_PRIORITY_EMERGENCY = 0,
    CAN_TX_PRIORITY_JOINT_CONTROL
} CanTxPriority;

/**
 * @brief Reports bounded scheduler outcomes.
 */
typedef enum
{
    CAN_TX_SCHEDULER_STATUS_OK = 0,
    CAN_TX_SCHEDULER_STATUS_REPLACED,
    CAN_TX_SCHEDULER_STATUS_EMPTY,
    CAN_TX_SCHEDULER_STATUS_INVALID_ARGUMENT,
    CAN_TX_SCHEDULER_STATUS_INVALID_JOINT,
    CAN_TX_SCHEDULER_STATUS_EMERGENCY_QUEUE_FULL
} CanTxSchedulerStatus;

/**
 * @brief Owns emergency FIFO traffic and one coalescing slot per joint.
 */
typedef struct
{
    CanFrame emergency_frames[CAN_TX_EMERGENCY_QUEUE_CAPACITY];
    CanFrame joint_frames[ARM_JOINT_COUNT];
    uint32_t coalesced_joint_frame_count;
    uint32_t emergency_queue_full_count;
    uint8_t emergency_head;
    uint8_t emergency_count;
    uint8_t emergency_high_watermark;
    uint8_t pending_joint_mask;
    uint8_t next_joint_index;
    uint8_t initialized;
} CanTxScheduler;

/**
 * @brief Initializes an empty scheduler.
 * @param scheduler Scheduler storage.
 */
void can_tx_scheduler_init(CanTxScheduler *scheduler);

/**
 * @brief Queues one non-droppable emergency frame in FIFO order.
 * @param scheduler Initialized scheduler.
 * @param frame Frame to copy.
 * @return OK or an explicit capacity/argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_emergency(CanTxScheduler *scheduler,
                                                       const CanFrame *frame);

/**
 * @brief Stores the latest control frame for one joint, replacing older pending data.
 * @param scheduler Initialized scheduler.
 * @param joint_index Zero-based joint index.
 * @param frame Frame to copy.
 * @return OK, REPLACED, or an argument error.
 */
CanTxSchedulerStatus can_tx_scheduler_submit_joint(CanTxScheduler *scheduler,
                                                   uint8_t joint_index,
                                                   const CanFrame *frame);

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
