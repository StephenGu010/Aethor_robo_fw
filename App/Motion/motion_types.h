/**
 * @file motion_types.h
 * @brief Defines hardware-independent seven-axis motion value objects.
 */

#ifndef APP_MOTION_MOTION_TYPES_H
#define APP_MOTION_MOTION_TYPES_H

#include <stdint.h>

#include "arm_config.h"

/**
 * @brief Stores a future joint-space target without exposing a planner API.
 */
typedef struct
{
    float position_rad[ARM_JOINT_COUNT];
    float velocity_rad_s[ARM_JOINT_COUNT];
    uint64_t execute_at_us;
    uint8_t valid_joint_mask;
} MotionTarget;

/**
 * @brief Stores a value snapshot of future motion state.
 */
typedef struct
{
    float commanded_position_rad[ARM_JOINT_COUNT];
    float commanded_velocity_rad_s[ARM_JOINT_COUNT];
    uint64_t timestamp_us;
    uint8_t valid_joint_mask;
} MotionSnapshot;

#endif
