/**
 * @file motor_bank.h
 * @brief Defines seven independent motor objects and coherent feedback snapshots.
 */

#ifndef APP_MOTOR_MOTOR_BANK_H
#define APP_MOTOR_MOTOR_BANK_H

#include <stdint.h>

#include "arm_config.h"
#include "motor_types.h"

/**
 * @brief Reports motor mapping and feedback update outcomes.
 */
typedef enum
{
    MOTOR_BANK_STATUS_OK = 0,
    MOTOR_BANK_STATUS_INVALID_ARGUMENT,
    MOTOR_BANK_STATUS_CONFIG_INVALID,
    MOTOR_BANK_STATUS_ID_UNKNOWN,
    MOTOR_BANK_STATUS_ID_MISMATCH,
    MOTOR_BANK_STATUS_INVALID_SAMPLE,
    MOTOR_BANK_STATUS_STALE_SAMPLE
} MotorBankStatus;

/**
 * @brief Owns runtime identity and feedback for one physical joint motor.
 */
typedef struct
{
    MotorJointFeedback feedback;
    float target_position_rad;
    float target_velocity_rad_s;
    uint32_t active_request_id;
    uint16_t esc_id;
    uint16_t master_id;
    uint16_t parameter_valid_mask;
    MotorLifecycleState state;
    MotorParameterSource parameter_source;
    uint8_t joint_index;
    uint8_t feedback_valid;
    uint8_t target_valid;
    uint8_t at_target;
    uint8_t configuration_consistent;
} MotorObject;

/**
 * @brief Owns the fixed seven-motor runtime bank.
 */
typedef struct
{
    MotorObject motors[ARM_JOINT_COUNT];
    MotorBusState bus_state;
    uint32_t generation;
    uint8_t valid_joint_mask;
    uint8_t initialized;
} MotorBank;

/**
 * @brief Initializes seven independent motor objects from the immutable mapping.
 * @param bank Destination motor bank.
 * @param configuration Valid seven-axis configuration.
 * @return OK or a precise argument/configuration error.
 */
MotorBankStatus motor_bank_init(MotorBank *bank, const ArmConfig *configuration);

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
                                           const MotorJointFeedback *feedback);

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
                                        MotorFeedbackSnapshot *snapshot);

#endif
