/**
 * @file motor_types.h
 * @brief Defines motor feedback values without exposing control operations.
 */

#ifndef APP_MOTOR_MOTOR_TYPES_H
#define APP_MOTOR_MOTOR_TYPES_H

#include <stdint.h>

#include "arm_config.h"

/**
 * @brief Describes receive-side bus health for future motor adapters.
 */
typedef enum
{
    MOTOR_BUS_NOT_INITIALIZED = 0,
    MOTOR_BUS_ACTIVE,
    MOTOR_BUS_FAULT
} MotorBusState;

/**
 * @brief Stores one decoded motor feedback sample as a value object.
 */
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint32_t fault_flags;
    uint64_t timestamp_us;
} MotorJointFeedback;

/**
 * @brief Stores an atomic seven-axis motor feedback snapshot.
 */
typedef struct
{
    MotorJointFeedback joints[ARM_JOINT_COUNT];
    MotorBusState bus_state;
    uint64_t published_at_us;
    uint32_t generation;
    uint8_t valid_joint_mask;
} MotorFeedbackSnapshot;

#endif
