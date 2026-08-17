/**
 * @file joint_motion.h
 * @brief Defines hardware-independent seven-axis synchronized motion math.
 */

#ifndef APP_MOTION_JOINT_MOTION_H
#define APP_MOTION_JOINT_MOTION_H

#include <stdint.h>

#include "arm_config.h"

#define JOINT_MOTION_MINIMUM_DURATION_US (4000ULL)
#define JOINT_MOTION_SPEED_RATIO_MIN (0.01F)
#define JOINT_MOTION_SPEED_RATIO_MAX (1.00F)

/** @brief Selects the planned motor control strategy. */
typedef enum
{
    JOINT_MOTION_MODE_POSITION_VELOCITY = 0,
    JOINT_MOTION_MODE_MIT
} JointMotionMode;

/** @brief Reports deterministic plan, sample, and completion outcomes. */
typedef enum
{
    JOINT_MOTION_STATUS_OK = 0,
    JOINT_MOTION_STATUS_INVALID_ARGUMENT,
    JOINT_MOTION_STATUS_CONFIG_INCOMPLETE,
    JOINT_MOTION_STATUS_NONFINITE_VALUE,
    JOINT_MOTION_STATUS_TARGET_OUT_OF_RANGE,
    JOINT_MOTION_STATUS_SPEED_RATIO_OUT_OF_RANGE,
    JOINT_MOTION_STATUS_FEEDBACK_INCOMPLETE
} JointMotionStatus;

/** @brief Owns one immutable seven-axis synchronized motion plan. */
typedef struct
{
    float start_position_rad[ARM_JOINT_COUNT];
    float target_position_rad[ARM_JOINT_COUNT];
    float command_velocity_rad_s[ARM_JOINT_COUNT];
    uint64_t start_time_us;
    uint64_t duration_us;
    float speed_ratio;
    JointMotionMode mode;
} JointMotionPlan;

/** @brief Owns one shared-duration constant-deceleration controlled stop. */
typedef struct
{
    float start_position_rad[ARM_JOINT_COUNT];
    float start_velocity_rad_s[ARM_JOINT_COUNT];
    float hold_position_rad[ARM_JOINT_COUNT];
    uint64_t start_time_us;
    uint64_t duration_us;
} JointControlledStopPlan;

/** @brief Stores one synchronized position, velocity, and acceleration sample. */
typedef struct
{
    float position_rad[ARM_JOINT_COUNT];
    float velocity_rad_s[ARM_JOINT_COUNT];
    float acceleration_rad_s2[ARM_JOINT_COUNT];
    uint64_t timestamp_us;
    uint8_t trajectory_complete;
} JointMotionSample;

/** @brief Tracks a continuous all-axis in-tolerance settling window. */
typedef struct
{
    uint64_t settle_started_at_us;
    float maximum_position_error_rad;
    uint8_t settle_started;
    uint8_t completed;
} JointMotionCompletion;

/**
 * @brief Atomically validates and creates a synchronized seven-axis plan.
 */
JointMotionStatus joint_motion_plan(
    const ArmConfig *configuration,
    const float start_position_rad[ARM_JOINT_COUNT],
    const float target_position_rad[ARM_JOINT_COUNT],
    float speed_ratio,
    JointMotionMode mode,
    uint64_t start_time_us,
    JointMotionPlan *plan);

/**
 * @brief Samples the shared POS_VEL endpoint or MIT quintic time scaling.
 */
JointMotionStatus joint_motion_sample(const JointMotionPlan *plan,
                                      uint64_t timestamp_us,
                                      JointMotionSample *sample);

/**
 * @brief Plans one shared-duration constant-deceleration all-axis stop.
 */
JointMotionStatus joint_motion_plan_controlled_stop(
    const ArmConfig *configuration,
    const float start_position_rad[ARM_JOINT_COUNT],
    const float start_velocity_rad_s[ARM_JOINT_COUNT],
    float acceleration_ratio,
    uint64_t start_time_us,
    JointControlledStopPlan *plan);

/**
 * @brief Samples position, velocity, and acceleration for a controlled stop.
 */
JointMotionStatus joint_motion_sample_controlled_stop(
    const JointControlledStopPlan *plan,
    uint64_t timestamp_us,
    JointMotionSample *sample);

/** @brief Resets one continuous-settle completion tracker. */
void joint_motion_completion_init(JointMotionCompletion *completion);

/**
 * @brief Updates all-axis completion using fresh feedback and configured tolerances.
 */
JointMotionStatus joint_motion_update_completion(
    const JointMotionPlan *plan,
    const ArmConfig *configuration,
    const float feedback_position_rad[ARM_JOINT_COUNT],
    const float feedback_velocity_rad_s[ARM_JOINT_COUNT],
    uint8_t valid_joint_mask,
    uint64_t timestamp_us,
    uint64_t settle_duration_us,
    JointMotionCompletion *completion);

#endif
