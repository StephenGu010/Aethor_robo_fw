/**
  ******************************************************************************
  * @file    sync_trajectory.c
  * @brief   Seven-joint synchronized trapezoidal trajectory implementation.
  ******************************************************************************
  */

#include "sync_trajectory.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/**
  * @brief Check whether all trajectory input arrays contain valid values.
  * @param start_positions Initial joint positions in degrees.
  * @param target_positions Target joint positions in degrees.
  * @param maximum_velocities Maximum joint velocities in degrees per second.
  * @param maximum_accelerations Maximum joint accelerations in degrees per second squared.
  * @retval 1 when every input value is finite and every limit is positive, otherwise 0.
  */
static int sync_trajectory_inputs_are_valid(
    const float start_positions[SYNC_TRAJECTORY_JOINT_COUNT],
    const float target_positions[SYNC_TRAJECTORY_JOINT_COUNT],
    const float maximum_velocities[SYNC_TRAJECTORY_JOINT_COUNT],
    const float maximum_accelerations[SYNC_TRAJECTORY_JOINT_COUNT])
{
    uint32_t joint_index;

    for (joint_index = 0U; joint_index < SYNC_TRAJECTORY_JOINT_COUNT; ++joint_index)
    {
        if ((!isfinite(start_positions[joint_index])) ||
            (!isfinite(target_positions[joint_index])) ||
            (!isfinite(maximum_velocities[joint_index])) ||
            (!isfinite(maximum_accelerations[joint_index])) ||
            (maximum_velocities[joint_index] <= 0.0F) ||
            (maximum_accelerations[joint_index] <= 0.0F))
        {
            return 0;
        }
    }

    return 1;
}

/**
  * @brief Start a synchronized seven-joint trapezoidal trajectory.
  * @param trajectory Trajectory state to initialize.
  * @param start_positions Initial joint positions in degrees.
  * @param target_positions Target joint positions in degrees.
  * @param maximum_velocities Maximum joint velocities in degrees per second.
  * @param maximum_accelerations Maximum joint accelerations in degrees per second squared.
  * @param speed_percent Global speed scaling percentage from 1 to 100.
  * @retval Explicit trajectory status code.
  */
SyncTrajectoryStatus sync_trajectory_start(
    SyncTrajectory *trajectory,
    const float start_positions[SYNC_TRAJECTORY_JOINT_COUNT],
    const float target_positions[SYNC_TRAJECTORY_JOINT_COUNT],
    const float maximum_velocities[SYNC_TRAJECTORY_JOINT_COUNT],
    const float maximum_accelerations[SYNC_TRAJECTORY_JOINT_COUNT],
    float speed_percent)
{
    float normalized_velocity_limit = INFINITY;
    float normalized_acceleration_limit = INFINITY;
    float speed_scale;
    uint32_t joint_index;
    int has_motion = 0;

    if ((trajectory == NULL) || (start_positions == NULL) ||
        (target_positions == NULL) || (maximum_velocities == NULL) ||
        (maximum_accelerations == NULL))
    {
        return SYNC_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }

    if ((!isfinite(speed_percent)) || (speed_percent < 1.0F) ||
        (speed_percent > 100.0F))
    {
        return SYNC_TRAJECTORY_STATUS_INVALID_SPEED_PERCENT;
    }

    if (!sync_trajectory_inputs_are_valid(start_positions,
                                           target_positions,
                                           maximum_velocities,
                                           maximum_accelerations))
    {
        return SYNC_TRAJECTORY_STATUS_INVALID_LIMIT;
    }

    memset(trajectory, 0, sizeof(*trajectory));
    speed_scale = speed_percent / 100.0F;

    for (joint_index = 0U; joint_index < SYNC_TRAJECTORY_JOINT_COUNT; ++joint_index)
    {
        float displacement_magnitude;
        float joint_normalized_velocity;
        float joint_normalized_acceleration;

        trajectory->start_position[joint_index] = start_positions[joint_index];
        trajectory->delta_position[joint_index] =
            target_positions[joint_index] - start_positions[joint_index];
        displacement_magnitude = fabsf(trajectory->delta_position[joint_index]);

        if (displacement_magnitude <= SYNC_TRAJECTORY_POSITION_EPSILON)
        {
            continue;
        }

        has_motion = 1;
        joint_normalized_velocity =
            (maximum_velocities[joint_index] * speed_scale) / displacement_magnitude;
        joint_normalized_acceleration =
            (maximum_accelerations[joint_index] * speed_scale) / displacement_magnitude;

        if (joint_normalized_velocity < normalized_velocity_limit)
        {
            normalized_velocity_limit = joint_normalized_velocity;
        }
        if (joint_normalized_acceleration < normalized_acceleration_limit)
        {
            normalized_acceleration_limit = joint_normalized_acceleration;
        }
    }

    if (!has_motion)
    {
        trajectory->progress = 1.0F;
        trajectory->active = 0U;
        return SYNC_TRAJECTORY_STATUS_OK;
    }

    assert(isfinite(normalized_velocity_limit));
    assert(isfinite(normalized_acceleration_limit));
    assert(normalized_velocity_limit > 0.0F);
    assert(normalized_acceleration_limit > 0.0F);

    if ((normalized_velocity_limit * normalized_velocity_limit) >=
        normalized_acceleration_limit)
    {
        trajectory->progress_acceleration = normalized_acceleration_limit;
        trajectory->peak_progress_velocity = sqrtf(normalized_acceleration_limit);
        trajectory->acceleration_seconds =
            trajectory->peak_progress_velocity / normalized_acceleration_limit;
        trajectory->cruise_seconds = 0.0F;
    }
    else
    {
        trajectory->progress_acceleration = normalized_acceleration_limit;
        trajectory->peak_progress_velocity = normalized_velocity_limit;
        trajectory->acceleration_seconds =
            normalized_velocity_limit / normalized_acceleration_limit;
        trajectory->cruise_seconds =
            (1.0F - ((normalized_velocity_limit * normalized_velocity_limit) /
                     normalized_acceleration_limit)) /
            normalized_velocity_limit;
    }

    trajectory->total_seconds =
        (2.0F * trajectory->acceleration_seconds) + trajectory->cruise_seconds;
    trajectory->active = 1U;
    return SYNC_TRAJECTORY_STATUS_OK;
}

/**
  * @brief Advance a synchronized trajectory and calculate all joint setpoints.
  * @param trajectory Active trajectory state.
  * @param time_step_seconds Elapsed time since the previous update in seconds.
  * @param output_positions Calculated joint positions in degrees.
  * @param output_velocities Calculated joint velocities in degrees per second.
  * @param is_complete Receives 1 when the trajectory is complete, otherwise 0.
  * @retval Explicit trajectory status code.
  */
SyncTrajectoryStatus sync_trajectory_step(
    SyncTrajectory *trajectory,
    float time_step_seconds,
    float output_positions[SYNC_TRAJECTORY_JOINT_COUNT],
    float output_velocities[SYNC_TRAJECTORY_JOINT_COUNT],
    uint8_t *is_complete)
{
    float progress_velocity = 0.0F;
    uint32_t joint_index;

    if ((trajectory == NULL) || (output_positions == NULL) ||
        (output_velocities == NULL) || (is_complete == NULL))
    {
        return SYNC_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }

    if ((!isfinite(time_step_seconds)) || (time_step_seconds <= 0.0F))
    {
        return SYNC_TRAJECTORY_STATUS_INVALID_TIME_STEP;
    }

    if (trajectory->active != 0U)
    {
        trajectory->elapsed_seconds += time_step_seconds;

        if (trajectory->elapsed_seconds < trajectory->acceleration_seconds)
        {
            trajectory->progress =
                0.5F * trajectory->progress_acceleration *
                trajectory->elapsed_seconds * trajectory->elapsed_seconds;
            progress_velocity =
                trajectory->progress_acceleration * trajectory->elapsed_seconds;
        }
        else if (trajectory->elapsed_seconds <
                 (trajectory->acceleration_seconds + trajectory->cruise_seconds))
        {
            float cruise_elapsed_time =
                trajectory->elapsed_seconds - trajectory->acceleration_seconds;
            float acceleration_distance =
                0.5F * trajectory->progress_acceleration *
                trajectory->acceleration_seconds * trajectory->acceleration_seconds;

            trajectory->progress = acceleration_distance +
                                   (trajectory->peak_progress_velocity *
                                    cruise_elapsed_time);
            progress_velocity = trajectory->peak_progress_velocity;
        }
        else if (trajectory->elapsed_seconds < trajectory->total_seconds)
        {
            float remaining_time =
                trajectory->total_seconds - trajectory->elapsed_seconds;

            trajectory->progress =
                1.0F - (0.5F * trajectory->progress_acceleration *
                        remaining_time * remaining_time);
            progress_velocity =
                trajectory->progress_acceleration * remaining_time;
        }
        else
        {
            trajectory->elapsed_seconds = trajectory->total_seconds;
            trajectory->progress = 1.0F;
            trajectory->active = 0U;
        }
    }
    else
    {
        trajectory->progress = 1.0F;
    }

    for (joint_index = 0U; joint_index < SYNC_TRAJECTORY_JOINT_COUNT; ++joint_index)
    {
        output_positions[joint_index] =
            trajectory->start_position[joint_index] +
            (trajectory->delta_position[joint_index] * trajectory->progress);
        output_velocities[joint_index] =
            trajectory->delta_position[joint_index] * progress_velocity;
    }

    *is_complete = (trajectory->active == 0U) ? 1U : 0U;
    return SYNC_TRAJECTORY_STATUS_OK;
}
