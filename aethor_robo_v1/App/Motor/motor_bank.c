/**
 * @file motor_bank.c
 * @brief Implements seven independent motor identities and feedback snapshots.
 */

#include "motor_bank.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Checks that all floating-point feedback values are finite.
 * @param feedback Decoded feedback sample.
 * @return One when valid, otherwise zero.
 */
static uint8_t motor_bank_feedback_is_finite(const MotorJointFeedback *feedback)
{
    return (uint8_t)(isfinite(feedback->position_rad) &&
                     isfinite(feedback->velocity_rad_s) &&
                     isfinite(feedback->torque_nm));
}

/**
 * @brief Finds one joint by its frozen Master ID.
 * @param bank Initialized motor bank.
 * @param master_id Standard feedback identifier.
 * @param joint_index Destination zero-based joint index.
 * @return One when found, otherwise zero.
 */
static uint8_t motor_bank_find_master_id(const MotorBank *bank,
                                         uint16_t master_id,
                                         uint8_t *joint_index)
{
    uint8_t index;

    for (index = 0U; index < ARM_JOINT_COUNT; ++index)
    {
        if (bank->motors[index].master_id == master_id)
        {
            *joint_index = index;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief Initializes seven independent motor objects from the immutable mapping.
 * @param bank Destination motor bank.
 * @param configuration Valid seven-axis configuration.
 * @return OK or a precise argument/configuration error.
 */
MotorBankStatus motor_bank_init(MotorBank *bank, const ArmConfig *configuration)
{
    ArmConfigValidation validation;
    uint8_t joint_index;

    if ((bank == NULL) || (configuration == NULL))
    {
        return MOTOR_BANK_STATUS_INVALID_ARGUMENT;
    }

    memset(bank, 0, sizeof(*bank));
    if (!arm_config_validate_schema(configuration, &validation))
    {
        return MOTOR_BANK_STATUS_CONFIG_INVALID;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        bank->motors[joint_index].joint_index = joint_index;
        bank->motors[joint_index].esc_id = configuration->joints[joint_index].esc_id;
        bank->motors[joint_index].master_id = configuration->joints[joint_index].master_id;
    }

    bank->bus_state = MOTOR_BUS_NOT_INITIALIZED;
    bank->initialized = 1U;
    return MOTOR_BANK_STATUS_OK;
}

/**
 * @brief Applies one identity-checked, monotonically newer feedback sample.
 * @param bank Initialized motor bank.
 * @param master_id Received standard CAN identifier.
 * @param esc_id Motor identifier decoded from feedback byte D0.
 * @param feedback Decoded SI-unit feedback sample.
 * @return Detailed identity or sample status.
 */
MotorBankStatus motor_bank_update_feedback(MotorBank *bank,
                                           uint16_t master_id,
                                           uint16_t esc_id,
                                           const MotorJointFeedback *feedback)
{
    uint8_t joint_index;
    MotorObject *motor;

    if ((bank == NULL) || (bank->initialized == 0U) || (feedback == NULL))
    {
        return MOTOR_BANK_STATUS_INVALID_ARGUMENT;
    }

    if (motor_bank_feedback_is_finite(feedback) == 0U)
    {
        return MOTOR_BANK_STATUS_INVALID_SAMPLE;
    }

    if (motor_bank_find_master_id(bank, master_id, &joint_index) == 0U)
    {
        return MOTOR_BANK_STATUS_ID_UNKNOWN;
    }

    motor = &bank->motors[joint_index];
    if (motor->esc_id != esc_id)
    {
        return MOTOR_BANK_STATUS_ID_MISMATCH;
    }

    if ((motor->feedback_valid != 0U) &&
        (feedback->timestamp_us <= motor->feedback.timestamp_us))
    {
        return MOTOR_BANK_STATUS_STALE_SAMPLE;
    }

    motor->feedback = *feedback;
    motor->feedback_valid = 1U;
    bank->valid_joint_mask |= (uint8_t)(1U << joint_index);
    bank->bus_state = MOTOR_BUS_ACTIVE;
    ++bank->generation;
    return MOTOR_BANK_STATUS_OK;
}

/**
 * @brief Copies one coherent seven-axis snapshot and applies freshness masking.
 * @param bank Initialized motor bank.
 * @param timestamp_us Snapshot publication time.
 * @param stale_after_us Maximum accepted feedback age.
 * @param snapshot Destination snapshot.
 * @return OK or an argument error.
 */
MotorBankStatus motor_bank_get_snapshot(const MotorBank *bank,
                                        uint64_t timestamp_us,
                                        uint64_t stale_after_us,
                                        MotorFeedbackSnapshot *snapshot)
{
    uint8_t joint_index;

    if ((bank == NULL) || (bank->initialized == 0U) || (snapshot == NULL))
    {
        return MOTOR_BANK_STATUS_INVALID_ARGUMENT;
    }

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->bus_state = bank->bus_state;
    snapshot->published_at_us = timestamp_us;
    snapshot->generation = bank->generation;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorObject *motor = &bank->motors[joint_index];

        snapshot->joints[joint_index] = motor->feedback;
        if ((motor->feedback_valid != 0U) &&
            (timestamp_us >= motor->feedback.timestamp_us) &&
            ((timestamp_us - motor->feedback.timestamp_us) <= stale_after_us))
        {
            snapshot->valid_joint_mask |= (uint8_t)(1U << joint_index);
        }
    }
    return MOTOR_BANK_STATUS_OK;
}
