/**
 * @file sync_trajectory.h
 * @brief Common-progress trapezoidal trajectory generator for seven synchronized joints.
 */

#ifndef SYNC_TRAJECTORY_H
#define SYNC_TRAJECTORY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SYNC_TRAJECTORY_JOINT_COUNT 7U
#define SYNC_TRAJECTORY_POSITION_EPSILON 0.000001F

/** @brief Result codes returned by synchronized trajectory functions. */
typedef enum
{
    SYNC_TRAJECTORY_STATUS_OK = 0,
    SYNC_TRAJECTORY_STATUS_INVALID_ARGUMENT,
    SYNC_TRAJECTORY_STATUS_INVALID_LIMIT,
    SYNC_TRAJECTORY_STATUS_INVALID_SPEED_PERCENT,
    SYNC_TRAJECTORY_STATUS_INVALID_TIME_STEP
} SyncTrajectoryStatus;

/** @brief Complete state of one seven-axis common-progress trajectory. */
typedef struct
{
    float start_position[SYNC_TRAJECTORY_JOINT_COUNT];
    float delta_position[SYNC_TRAJECTORY_JOINT_COUNT];
    float elapsed_seconds;
    float acceleration_seconds;
    float cruise_seconds;
    float total_seconds;
    float progress;
    float progress_acceleration;
    float peak_progress_velocity;
    uint8_t active;
} SyncTrajectory;

SyncTrajectoryStatus sync_trajectory_start(SyncTrajectory *trajectory,
                                            const float start_position[SYNC_TRAJECTORY_JOINT_COUNT],
                                            const float target_position[SYNC_TRAJECTORY_JOINT_COUNT],
                                            const float maximum_velocity[SYNC_TRAJECTORY_JOINT_COUNT],
                                            const float maximum_acceleration[SYNC_TRAJECTORY_JOINT_COUNT],
                                            float speed_percent);
SyncTrajectoryStatus sync_trajectory_step(SyncTrajectory *trajectory,
                                           float delta_time_seconds,
                                           float position[SYNC_TRAJECTORY_JOINT_COUNT],
                                           float velocity[SYNC_TRAJECTORY_JOINT_COUNT],
                                           uint8_t *complete);

#ifdef __cplusplus
}
#endif

#endif /* SYNC_TRAJECTORY_H */
