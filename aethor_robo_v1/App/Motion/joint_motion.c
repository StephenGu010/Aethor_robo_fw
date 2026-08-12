/**
 * @file joint_motion.c
 * @brief Implements synchronized POS_VEL and quintic MIT motion mathematics.
 */

#include "joint_motion.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define JOINT_MOTION_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define JOINT_MOTION_QUINTIC_MAX_VELOCITY_FACTOR (1.875F)
#define JOINT_MOTION_QUINTIC_MAX_ACCELERATION_FACTOR (5.773502692F)

/** @brief Checks a float without relying on target-specific isfinite macros. */
static uint8_t joint_motion_float_is_finite(float value)
{
    return (uint8_t)((value == value) && (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

/** @brief Converts a positive duration in seconds to a bounded microsecond count. */
static uint64_t joint_motion_seconds_to_microseconds(float seconds)
{
    double microseconds = ceil((double)seconds * 1000000.0);

    if (microseconds < (double)JOINT_MOTION_MINIMUM_DURATION_US)
    {
        return JOINT_MOTION_MINIMUM_DURATION_US;
    }
    if (microseconds > (double)UINT64_MAX)
    {
        return UINT64_MAX;
    }
    return (uint64_t)microseconds;
}

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
    JointMotionPlan *plan)
{
    ArmConfigValidation validation;
    float duration_seconds = 0.0F;
    uint8_t joint_index;

    if ((configuration == NULL) || (start_position_rad == NULL) ||
        (target_position_rad == NULL) || (plan == NULL) ||
        ((mode != JOINT_MOTION_MODE_POSITION_VELOCITY) &&
         (mode != JOINT_MOTION_MODE_MIT)))
    {
        return JOINT_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (!arm_config_is_enable_ready(configuration, &validation))
    {
        return JOINT_MOTION_STATUS_CONFIG_INCOMPLETE;
    }
    if (!joint_motion_float_is_finite(speed_ratio) ||
        (speed_ratio < JOINT_MOTION_SPEED_RATIO_MIN) ||
        (speed_ratio > JOINT_MOTION_SPEED_RATIO_MAX))
    {
        return JOINT_MOTION_STATUS_SPEED_RATIO_OUT_OF_RANGE;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint = &configuration->joints[joint_index];
        float distance;
        float velocity_duration;
        float acceleration_duration = 0.0F;
        float joint_duration;

        if (!joint_motion_float_is_finite(start_position_rad[joint_index]) ||
            !joint_motion_float_is_finite(target_position_rad[joint_index]))
        {
            return JOINT_MOTION_STATUS_NONFINITE_VALUE;
        }
        if ((target_position_rad[joint_index] < joint->soft_limit_min_rad) ||
            (target_position_rad[joint_index] > joint->soft_limit_max_rad))
        {
            return JOINT_MOTION_STATUS_TARGET_OUT_OF_RANGE;
        }
        distance = fabsf(target_position_rad[joint_index] -
                         start_position_rad[joint_index]);
        velocity_duration = distance /
                            (speed_ratio * joint->max_velocity_rad_s);
        if (mode == JOINT_MOTION_MODE_MIT)
        {
            velocity_duration *= JOINT_MOTION_QUINTIC_MAX_VELOCITY_FACTOR;
            acceleration_duration = sqrtf(
                JOINT_MOTION_QUINTIC_MAX_ACCELERATION_FACTOR * distance /
                (speed_ratio * joint->max_acceleration_rad_s2));
        }
        joint_duration = (velocity_duration > acceleration_duration)
                             ? velocity_duration
                             : acceleration_duration;
        if (joint_duration > duration_seconds)
        {
            duration_seconds = joint_duration;
        }
    }

    memset(plan, 0, sizeof(*plan));
    plan->start_time_us = start_time_us;
    plan->duration_us = joint_motion_seconds_to_microseconds(duration_seconds);
    plan->speed_ratio = speed_ratio;
    plan->mode = mode;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        float duration = (float)plan->duration_us / 1000000.0F;

        plan->start_position_rad[joint_index] = start_position_rad[joint_index];
        plan->target_position_rad[joint_index] = target_position_rad[joint_index];
        plan->command_velocity_rad_s[joint_index] =
            (target_position_rad[joint_index] - start_position_rad[joint_index]) /
            duration;
    }
    return JOINT_MOTION_STATUS_OK;
}

/**
 * @brief Samples the shared POS_VEL endpoint or MIT quintic time scaling.
 */
JointMotionStatus joint_motion_sample(const JointMotionPlan *plan,
                                      uint64_t timestamp_us,
                                      JointMotionSample *sample)
{
    float normalized_time;
    float scale;
    float scale_velocity;
    float scale_acceleration;
    float duration_seconds;
    uint8_t joint_index;

    if ((plan == NULL) || (sample == NULL) || (plan->duration_us == 0U))
    {
        return JOINT_MOTION_STATUS_INVALID_ARGUMENT;
    }
    memset(sample, 0, sizeof(*sample));
    sample->timestamp_us = timestamp_us;
    duration_seconds = (float)plan->duration_us / 1000000.0F;

    if (timestamp_us <= plan->start_time_us)
    {
        normalized_time = 0.0F;
    }
    else if ((timestamp_us - plan->start_time_us) >= plan->duration_us)
    {
        normalized_time = 1.0F;
        sample->trajectory_complete = 1U;
    }
    else
    {
        normalized_time = (float)(timestamp_us - plan->start_time_us) /
                          (float)plan->duration_us;
    }

    if (plan->mode == JOINT_MOTION_MODE_POSITION_VELOCITY)
    {
        scale = 1.0F;
        scale_velocity = 0.0F;
        scale_acceleration = 0.0F;
    }
    else if ((normalized_time <= 0.0F) || (normalized_time >= 1.0F))
    {
        scale = normalized_time;
        scale_velocity = 0.0F;
        scale_acceleration = 0.0F;
    }
    else
    {
        float u2 = normalized_time * normalized_time;
        float u3 = u2 * normalized_time;
        float u4 = u3 * normalized_time;
        float u5 = u4 * normalized_time;

        scale = (10.0F * u3) - (15.0F * u4) + (6.0F * u5);
        scale_velocity = ((30.0F * u2) - (60.0F * u3) + (30.0F * u4)) /
                         duration_seconds;
        scale_acceleration = ((60.0F * normalized_time) - (180.0F * u2) +
                              (120.0F * u3)) /
                             (duration_seconds * duration_seconds);
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        float distance = plan->target_position_rad[joint_index] -
                         plan->start_position_rad[joint_index];

        sample->position_rad[joint_index] =
            plan->start_position_rad[joint_index] + (distance * scale);
        sample->velocity_rad_s[joint_index] = distance * scale_velocity;
        sample->acceleration_rad_s2[joint_index] = distance * scale_acceleration;
    }
    return JOINT_MOTION_STATUS_OK;
}

/** @brief Resets one continuous-settle completion tracker. */
void joint_motion_completion_init(JointMotionCompletion *completion)
{
    if (completion != NULL)
    {
        memset(completion, 0, sizeof(*completion));
    }
}

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
    JointMotionCompletion *completion)
{
    float maximum_error = 0.0F;
    uint8_t all_within_tolerance = 1U;
    uint8_t joint_index;

    if ((plan == NULL) || (configuration == NULL) ||
        (feedback_position_rad == NULL) || (feedback_velocity_rad_s == NULL) ||
        (completion == NULL))
    {
        return JOINT_MOTION_STATUS_INVALID_ARGUMENT;
    }
    if (valid_joint_mask != JOINT_MOTION_ALL_JOINTS_MASK)
    {
        completion->settle_started = 0U;
        completion->completed = 0U;
        return JOINT_MOTION_STATUS_FEEDBACK_INCOMPLETE;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        float position_error = fabsf(plan->target_position_rad[joint_index] -
                                     feedback_position_rad[joint_index]);
        float velocity_error = fabsf(feedback_velocity_rad_s[joint_index]);

        if (position_error > maximum_error)
        {
            maximum_error = position_error;
        }
        if ((position_error >
             configuration->joints[joint_index].position_tolerance_rad) ||
            (velocity_error >
             configuration->joints[joint_index].velocity_tolerance_rad_s))
        {
            all_within_tolerance = 0U;
        }
    }
    completion->maximum_position_error_rad = maximum_error;
    if (all_within_tolerance == 0U)
    {
        completion->settle_started = 0U;
        completion->completed = 0U;
        return JOINT_MOTION_STATUS_OK;
    }
    if (completion->settle_started == 0U)
    {
        completion->settle_started = 1U;
        completion->settle_started_at_us = timestamp_us;
        return JOINT_MOTION_STATUS_OK;
    }
    if ((timestamp_us >= completion->settle_started_at_us) &&
        ((timestamp_us - completion->settle_started_at_us) >= settle_duration_us))
    {
        completion->completed = 1U;
    }
    return JOINT_MOTION_STATUS_OK;
}
