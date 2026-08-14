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
 * @brief Describes the independently tracked lifecycle of one motor axis.
 */
typedef enum
{
    MOTOR_LIFECYCLE_ABSENT = 0,
    MOTOR_LIFECYCLE_DISCOVERING,
    MOTOR_LIFECYCLE_DISABLED,
    MOTOR_LIFECYCLE_ENABLED,
    MOTOR_LIFECYCLE_MOVING,
    MOTOR_LIFECYCLE_HOLDING,
    MOTOR_LIFECYCLE_FAULT
} MotorLifecycleState;

/**
 * @brief Identifies where the active motor parameters were obtained.
 */
typedef enum
{
    MOTOR_PARAMETER_SOURCE_UNKNOWN = 0,
    MOTOR_PARAMETER_SOURCE_STATIC_CONFIG,
    MOTOR_PARAMETER_SOURCE_DISCOVERED
} MotorParameterSource;

/**
 * @brief Stores one decoded motor feedback sample as a value object.
 */
typedef struct
{
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    uint32_t fault_flags;
    uint64_t timestamp_us;
    uint8_t driver_state;
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
