/**
 * @file joint_reference.c
 * @brief Implements boot-volatile motor-to-joint reference alignment.
 */

#include "joint_reference.h"

#include <float.h>
#include <stddef.h>
#include <string.h>

#define JOINT_REFERENCE_DEG_TO_RAD (0.017453292519943295F)
#define JOINT_REFERENCE_RAD_TO_DEG (57.29577951308232F)
#define JOINT_REFERENCE_ALL_JOINTS_MASK ((uint8_t)0x7FU)
#define JOINT_REFERENCE_SNAPSHOT_MAX_ATTEMPTS (3U)
#define JOINT_REFERENCE_REQUIRED_CONFIG_FIELDS                              \
    ((uint32_t)(ARM_JOINT_VERIFIED_DIRECTION | ARM_JOINT_VERIFIED_LIMITS | \
                ARM_JOINT_VERIFIED_GEAR_RATIO))

/** @brief Prevents compiler reordering across single-writer seqlock edges. */
static void joint_reference_compiler_barrier(void)
{
#if defined(__CC_ARM)
    __schedule_barrier();
#elif defined(__GNUC__) || defined(__clang__)
    __asm__ volatile ("" ::: "memory");
#else
    volatile uint32_t barrier_value = 0U;
    (void)barrier_value;
#endif
}

/** @brief Checks one float without relying on platform-specific isfinite. */
static uint8_t joint_reference_float_is_finite(float value)
{
    return (uint8_t)((value == value) && (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

/** @brief Checks that mapping fields required for joint conversion are verified. */
static uint8_t joint_reference_configuration_is_ready(
    const ArmConfig *configuration)
{
    ArmConfigValidation validation;
    uint8_t joint_index;

    if (!arm_config_validate_schema(configuration, &validation))
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((configuration->joints[joint_index].verified_fields &
             JOINT_REFERENCE_REQUIRED_CONFIG_FIELDS) !=
            JOINT_REFERENCE_REQUIRED_CONFIG_FIELDS)
        {
            return 0U;
        }
    }
    return 1U;
}

/** @brief Publishes one fully assembled snapshot through the bounded seqlock. */
static void joint_reference_commit_snapshot(
    JointReference *reference,
    const JointStateSnapshot *snapshot)
{
    ++reference->publication_sequence;
    joint_reference_compiler_barrier();
    reference->snapshot = *snapshot;
    joint_reference_compiler_barrier();
    ++reference->publication_sequence;
}

/**
 * @brief Initializes and invalidates all boot-volatile reference data.
 */
JointReferenceStatus joint_reference_init(JointReference *reference,
                                          const ArmConfig *configuration)
{
    if ((reference == NULL) || (configuration == NULL))
    {
        return JOINT_REFERENCE_STATUS_INVALID_ARGUMENT;
    }

    memset(reference, 0, sizeof(*reference));
    reference->configuration = configuration;
    reference->initialized = 1U;
    if (joint_reference_configuration_is_ready(configuration) == 0U)
    {
        return JOINT_REFERENCE_STATUS_CONFIG_INCOMPLETE;
    }
    return JOINT_REFERENCE_STATUS_OK;
}

/**
 * @brief Converts one motor feedback generation into public joint coordinates.
 */
JointReferenceStatus joint_reference_publish(
    JointReference *reference,
    const MotorFeedbackSnapshot *motor_snapshot,
    uint64_t timestamp_us)
{
    JointStateSnapshot snapshot;
    uint8_t joint_index;

    if ((reference == NULL) || (motor_snapshot == NULL) ||
        (reference->initialized == 0U))
    {
        return JOINT_REFERENCE_STATUS_INVALID_ARGUMENT;
    }

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.published_at_us = timestamp_us;
    snapshot.motor_generation = motor_snapshot->generation;
    snapshot.generation = reference->snapshot.generation + 1U;
    snapshot.aligned = reference->aligned;

    if (reference->aligned != 0U)
    {
        for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
        {
            const JointConfig *joint =
                &reference->configuration->joints[joint_index];
            const MotorJointFeedback *feedback =
                &motor_snapshot->joints[joint_index];
            float direction = (float)joint->direction;

            snapshot.position_deg[joint_index] =
                ((direction * feedback->position_rad / joint->gear_ratio) +
                 reference->bias_rad[joint_index]) * JOINT_REFERENCE_RAD_TO_DEG;
            snapshot.velocity_deg_s[joint_index] =
                direction * feedback->velocity_rad_s / joint->gear_ratio *
                JOINT_REFERENCE_RAD_TO_DEG;
            snapshot.torque_nm[joint_index] = feedback->torque_nm;
        }
        snapshot.valid_joint_mask = motor_snapshot->valid_joint_mask;
    }

    joint_reference_commit_snapshot(reference, &snapshot);
    return (reference->aligned != 0U) ? JOINT_REFERENCE_STATUS_OK
                                     : JOINT_REFERENCE_STATUS_NOT_ALIGNED;
}

/**
 * @brief Aligns fresh raw motor positions to a supplied seven-axis pose.
 */
JointReferenceStatus joint_reference_align(
    JointReference *reference,
    const MotorFeedbackSnapshot *motor_snapshot,
    const float reference_degrees[ARM_JOINT_COUNT],
    uint64_t timestamp_us)
{
    uint8_t joint_index;

    if ((reference == NULL) || (motor_snapshot == NULL) ||
        (reference_degrees == NULL) || (reference->initialized == 0U))
    {
        return JOINT_REFERENCE_STATUS_INVALID_ARGUMENT;
    }
    if (joint_reference_configuration_is_ready(reference->configuration) == 0U)
    {
        return JOINT_REFERENCE_STATUS_CONFIG_INCOMPLETE;
    }
    if (motor_snapshot->valid_joint_mask != JOINT_REFERENCE_ALL_JOINTS_MASK)
    {
        return JOINT_REFERENCE_STATUS_FEEDBACK_INCOMPLETE;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint =
            &reference->configuration->joints[joint_index];
        float reference_rad;

        if (joint_reference_float_is_finite(reference_degrees[joint_index]) == 0U)
        {
            return JOINT_REFERENCE_STATUS_REFERENCE_OUT_OF_RANGE;
        }
        reference_rad = reference_degrees[joint_index] *
                        JOINT_REFERENCE_DEG_TO_RAD;
        if ((reference_rad < joint->soft_limit_min_rad) ||
            (reference_rad > joint->soft_limit_max_rad))
        {
            return JOINT_REFERENCE_STATUS_REFERENCE_OUT_OF_RANGE;
        }
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint =
            &reference->configuration->joints[joint_index];
        float reference_rad = reference_degrees[joint_index] *
                              JOINT_REFERENCE_DEG_TO_RAD;

        reference->bias_rad[joint_index] =
            reference_rad - ((float)joint->direction *
                             motor_snapshot->joints[joint_index].position_rad /
                             joint->gear_ratio);
    }
    reference->aligned = 1U;
    return joint_reference_publish(reference, motor_snapshot, timestamp_us);
}

/**
 * @brief Copies the latest generation-consistent joint snapshot.
 */
JointReferenceStatus joint_reference_get_snapshot(
    const JointReference *reference,
    JointStateSnapshot *snapshot)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint8_t attempt_index;

    if ((reference == NULL) || (snapshot == NULL) ||
        (reference->initialized == 0U))
    {
        return JOINT_REFERENCE_STATUS_INVALID_ARGUMENT;
    }

    for (attempt_index = 0U;
         attempt_index < JOINT_REFERENCE_SNAPSHOT_MAX_ATTEMPTS;
         ++attempt_index)
    {
        sequence_before = reference->publication_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        joint_reference_compiler_barrier();
        *snapshot = reference->snapshot;
        joint_reference_compiler_barrier();
        sequence_after = reference->publication_sequence;
        if ((sequence_before == sequence_after) &&
            ((sequence_after & 1U) == 0U))
        {
            return JOINT_REFERENCE_STATUS_OK;
        }
    }
    return JOINT_REFERENCE_STATUS_SNAPSHOT_BUSY;
}

/**
 * @brief Copies the current boot-volatile joint bias in public degree units.
 */
JointReferenceStatus joint_reference_get_bias_degrees(
    const JointReference *reference,
    float bias_degrees[ARM_JOINT_COUNT])
{
    uint8_t joint_index;

    if ((reference == NULL) || (bias_degrees == NULL) ||
        (reference->initialized == 0U))
    {
        return JOINT_REFERENCE_STATUS_INVALID_ARGUMENT;
    }
    if (reference->aligned == 0U)
    {
        return JOINT_REFERENCE_STATUS_NOT_ALIGNED;
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        bias_degrees[joint_index] =
            reference->bias_rad[joint_index] * JOINT_REFERENCE_RAD_TO_DEG;
    }
    return JOINT_REFERENCE_STATUS_OK;
}
