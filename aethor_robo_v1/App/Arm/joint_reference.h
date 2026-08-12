/**
 * @file joint_reference.h
 * @brief Defines volatile RAM reference alignment and coherent joint snapshots.
 */

#ifndef APP_ARM_JOINT_REFERENCE_H
#define APP_ARM_JOINT_REFERENCE_H

#include <stdint.h>

#include "arm_config.h"
#include "motor_types.h"

/** @brief Reports deterministic reference alignment and snapshot outcomes. */
typedef enum
{
    JOINT_REFERENCE_STATUS_OK = 0,
    JOINT_REFERENCE_STATUS_INVALID_ARGUMENT,
    JOINT_REFERENCE_STATUS_CONFIG_INCOMPLETE,
    JOINT_REFERENCE_STATUS_FEEDBACK_INCOMPLETE,
    JOINT_REFERENCE_STATUS_REFERENCE_OUT_OF_RANGE,
    JOINT_REFERENCE_STATUS_NOT_ALIGNED,
    JOINT_REFERENCE_STATUS_SNAPSHOT_BUSY
} JointReferenceStatus;

/** @brief Stores one generation-consistent public seven-axis joint snapshot. */
typedef struct
{
    float position_deg[ARM_JOINT_COUNT];
    float velocity_deg_s[ARM_JOINT_COUNT];
    float torque_nm[ARM_JOINT_COUNT];
    uint64_t published_at_us;
    uint32_t motor_generation;
    uint32_t generation;
    uint8_t valid_joint_mask;
    uint8_t aligned;
} JointStateSnapshot;

/** @brief Owns boot-volatile joint offsets and the latest coherent snapshot. */
typedef struct
{
    const ArmConfig *configuration;
    JointStateSnapshot snapshot;
    float bias_rad[ARM_JOINT_COUNT];
    volatile uint32_t publication_sequence;
    uint8_t aligned;
    uint8_t initialized;
} JointReference;

/**
 * @brief Initializes and invalidates all boot-volatile reference data.
 * @param reference Destination reference domain.
 * @param configuration Immutable seven-axis mapping.
 * @return OK or an argument/configuration error.
 */
JointReferenceStatus joint_reference_init(JointReference *reference,
                                          const ArmConfig *configuration);

/**
 * @brief Aligns fresh raw motor positions to a supplied seven-axis pose.
 * @param reference Initialized reference domain.
 * @param motor_snapshot Same-generation fresh motor feedback.
 * @param reference_degrees Mechanical reference pose in joint degrees.
 * @param timestamp_us Alignment timestamp.
 * @return OK or a precise validation error.
 */
JointReferenceStatus joint_reference_align(
    JointReference *reference,
    const MotorFeedbackSnapshot *motor_snapshot,
    const float reference_degrees[ARM_JOINT_COUNT],
    uint64_t timestamp_us);

/**
 * @brief Converts one motor feedback generation into public joint coordinates.
 * @param reference Initialized reference domain.
 * @param motor_snapshot Same-generation motor feedback.
 * @param timestamp_us Publication timestamp.
 * @return OK, NOT_ALIGNED, or a validation error.
 */
JointReferenceStatus joint_reference_publish(
    JointReference *reference,
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us);

/**
 * @brief Copies the latest generation-consistent joint snapshot.
 * @param reference Initialized reference domain.
 * @param snapshot Destination snapshot.
 * @return OK, SNAPSHOT_BUSY, or INVALID_ARGUMENT.
 */
JointReferenceStatus joint_reference_get_snapshot(
    const JointReference *reference,
    JointStateSnapshot *snapshot);

#endif
