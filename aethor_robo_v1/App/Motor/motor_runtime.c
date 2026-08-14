/**
 * @file motor_runtime.c
 * @brief Implements bounded seven-motor discovery and feedback routing.
 */

#include "motor_runtime.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "s3519_codec.h"

/**
 * @brief Finds a joint by the exact configured feedback Master ID.
 * @param runtime Initialized motor runtime.
 * @param master_id Received standard identifier.
 * @param joint_index Destination zero-based joint index.
 * @return One when the mapping exists, otherwise zero.
 */
static uint8_t motor_runtime_find_joint(const MotorRuntime *runtime,
                                        uint16_t master_id,
                                        uint8_t *joint_index)
{
    uint8_t candidate_index;

    for (candidate_index = 0U;
         candidate_index < ARM_JOINT_COUNT;
         ++candidate_index)
    {
        if (runtime->configuration->joints[candidate_index].master_id == master_id)
        {
            *joint_index = candidate_index;
            return 1U;
        }
    }
    return 0U;
}

/** @brief Clears one expected or quarantined parameter response tuple. */
static void motor_runtime_clear_parameter_signature(
    MotorParameterResponseSignature *signature)
{
    if (signature != NULL)
    {
        memset(signature, 0, sizeof(*signature));
    }
}

/** @brief Records the exact response tuple for one emitted parameter request. */
static void motor_runtime_record_parameter_expectation(
    MotorRuntime *runtime,
    MotorParameterResponseSignature *signature,
    uint8_t joint_index,
    const CanFrame *request,
    uint64_t timestamp_us)
{
    motor_runtime_clear_parameter_signature(signature);
    signature->timestamp_us = timestamp_us;
    signature->identifier =
        runtime->configuration->joints[joint_index].master_id;
    signature->joint_index = joint_index;
    signature->esc_id = request->data[0];
    signature->opcode = request->data[2];
    signature->register_address = request->data[3];
    signature->valid = 1U;
}

/** @brief Matches one frame against every field of a parameter response tuple. */
static uint8_t motor_runtime_frame_matches_parameter_signature(
    const MotorParameterResponseSignature *signature,
    const CanFrame *frame)
{
    return (uint8_t)((signature->valid != 0U) &&
                     (frame->length == CAN_CLASSIC_MAX_DATA_LENGTH) &&
                     (frame->identifier == signature->identifier) &&
                     (frame->data[0] == signature->esc_id) &&
                     (frame->data[2] == signature->opcode) &&
                     (frame->data[3] == signature->register_address));
}

/**
 * @brief Keeps new requests behind one indistinguishable aborted response.
 * @return One while the quarantine window remains active, otherwise zero.
 */
static uint8_t motor_runtime_parameter_quarantine_blocks(
    MotorRuntime *runtime,
    uint64_t timestamp_us)
{
    MotorParameterResponseSignature *quarantine =
        &runtime->parameter_quarantine;

    if (quarantine->valid == 0U)
    {
        return 0U;
    }
    if ((timestamp_us >= quarantine->timestamp_us) &&
        ((timestamp_us - quarantine->timestamp_us) <
         MOTOR_DISCOVERY_REQUEST_TIMEOUT_US))
    {
        return 1U;
    }
    motor_runtime_clear_parameter_signature(quarantine);
    return 0U;
}

/** @brief Advances the mode-switch cursor past every unselected motor. */
static void motor_runtime_skip_unselected_mode_joints(MotorRuntime *runtime)
{
    while ((runtime->mode_switch_joint_index < ARM_JOINT_COUNT) &&
           ((runtime->mode_switch_joint_mask &
             (uint8_t)(1U << runtime->mode_switch_joint_index)) == 0U))
    {
        ++runtime->mode_switch_joint_index;
    }
}

/**
 * @brief Checks that every selected motor retains verified non-mode discovery data.
 * @param runtime Initialized runtime owning discovery results.
 * @param motor_mask Nonzero selected J1-J7 mask.
 * @return One when every selected result is complete, otherwise zero.
 */
static uint8_t motor_runtime_selected_discovery_is_ready(
    const MotorRuntime *runtime,
    uint8_t motor_mask)
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if ((runtime->discovery.results[joint_index].verified_fields_mask &
             (uint16_t)(MOTOR_DISCOVERY_ALL_FIELDS_MASK &
                        (uint16_t)~MOTOR_DISCOVERY_MODE_FIELDS_MASK)) !=
            (uint16_t)(MOTOR_DISCOVERY_ALL_FIELDS_MASK &
                       (uint16_t)~MOTOR_DISCOVERY_MODE_FIELDS_MASK))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Maps a discovery state-machine result to the public runtime result.
 * @param discovery_status Discovery result to map.
 * @return Equivalent runtime result.
 */
static MotorRuntimeStatus motor_runtime_map_discovery_status(
    MotorDiscoveryStatus discovery_status)
{
    switch (discovery_status)
    {
        case MOTOR_DISCOVERY_STATUS_OK:
            return MOTOR_RUNTIME_STATUS_OK;
        case MOTOR_DISCOVERY_STATUS_FRAME_READY:
            return MOTOR_RUNTIME_STATUS_FRAME_READY;
        case MOTOR_DISCOVERY_STATUS_WAITING:
            return MOTOR_RUNTIME_STATUS_WAITING;
        case MOTOR_DISCOVERY_STATUS_COMPLETE:
            return MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE;
        case MOTOR_DISCOVERY_STATUS_INVALID_ARGUMENT:
            return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
        default:
            return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
    }
}

/**
 * @brief Checks whether a frame can be the outstanding discovery response.
 * @param runtime Initialized runtime.
 * @param frame Received Classic CAN frame.
 * @return One for a parameter response signature while discovery waits.
 */
static uint8_t motor_runtime_is_parameter_response(const MotorRuntime *runtime,
                                                   const CanFrame *frame)
{
    return (uint8_t)((runtime->discovery.state == MOTOR_DISCOVERY_STATE_WAITING) &&
                     (frame->length == CAN_CLASSIC_MAX_DATA_LENGTH) &&
                     ((frame->data[2] == 0x33U) || (frame->data[2] == 0x55U)));
}

/** @brief Checks whether a frame is the pending control-mode readback response. */
static uint8_t motor_runtime_is_mode_readback_response(
    const MotorRuntime *runtime,
    const CanFrame *frame)
{
    uint8_t joint_index = runtime->mode_switch_joint_index;

    return (uint8_t)((runtime->mode_switch_state ==
                      MOTOR_MODE_SWITCH_READ_WAITING) &&
                     (joint_index < ARM_JOINT_COUNT) &&
                     (frame->length == CAN_CLASSIC_MAX_DATA_LENGTH) &&
                     (frame->identifier ==
                      runtime->configuration->joints[joint_index].master_id) &&
                     (frame->data[0] ==
                      (uint8_t)runtime->configuration->joints[joint_index]
                          .esc_id) &&
                     (frame->data[2] == 0x33U) &&
                     (frame->data[3] == S3519_REGISTER_CONTROL_MODE));
}

/** @brief Validates and consumes one pending control-mode register readback. */
static MotorRuntimeStatus motor_runtime_accept_mode_readback(
    MotorRuntime *runtime,
    const CanFrame *frame)
{
    S3519ParameterResponse response;
    uint8_t joint_index = runtime->mode_switch_joint_index;
    uint32_t expected_mode =
        (runtime->requested_control_mode == S3519_CONTROL_MODE_MIT) ? 1U : 2U;

    if ((joint_index >= ARM_JOINT_COUNT) ||
        (s3519_decode_parameter_response(frame, &response) !=
         S3519_CODEC_STATUS_OK) ||
        (frame->identifier != runtime->configuration->joints[joint_index].master_id) ||
        (response.esc_id != runtime->configuration->joints[joint_index].esc_id) ||
        (response.register_address != S3519_REGISTER_CONTROL_MODE) ||
        (response.raw_value != expected_mode))
    {
        runtime->mode_switch_state = MOTOR_MODE_SWITCH_FAILED;
        return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
    }

    runtime->discovery.results[joint_index].observed_control_mode = expected_mode;
    runtime->discovery.results[joint_index].verified_fields_mask |=
        MOTOR_DISCOVERY_MODE_FIELDS_MASK;
    if ((runtime->discovery.results[joint_index].verified_fields_mask &
         MOTOR_DISCOVERY_ALL_FIELDS_MASK) == MOTOR_DISCOVERY_ALL_FIELDS_MASK)
    {
        runtime->discovery.verified_joint_mask |=
            (uint8_t)(1U << joint_index);
    }
    ++runtime->mode_switch_joint_index;
    motor_runtime_skip_unselected_mode_joints(runtime);
    runtime->mode_switch_attempt_count = 0U;
    runtime->mode_switch_state =
        (runtime->mode_switch_joint_index >= ARM_JOINT_COUNT)
            ? MOTOR_MODE_SWITCH_COMPLETE
            : MOTOR_MODE_SWITCH_READ_READY;
    return (runtime->mode_switch_state == MOTOR_MODE_SWITCH_COMPLETE)
               ? MOTOR_RUNTIME_STATUS_ACTION_COMPLETE
               : MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Decodes and applies one outstanding read-only discovery response.
 * @param runtime Initialized runtime.
 * @param frame Received parameter frame.
 * @return OK or a precise codec/discovery error.
 */
static MotorRuntimeStatus motor_runtime_accept_parameter_response(
    MotorRuntime *runtime,
    const CanFrame *frame)
{
    S3519ParameterResponse response;
    MotorDiscoveryStatus discovery_status;
    uint8_t response_joint_index = runtime->discovery.current_joint_index;
    MotorObject *motor;

    if (response_joint_index >= ARM_JOINT_COUNT)
    {
        ++runtime->rejected_parameter_response_count;
        return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
    }
    motor = &runtime->bank.motors[response_joint_index];

    if (s3519_decode_parameter_response(frame, &response) != S3519_CODEC_STATUS_OK)
    {
        ++runtime->rejected_parameter_response_count;
        return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
    }

    motor->state = MOTOR_LIFECYCLE_DISCOVERING;
    discovery_status = motor_discovery_accept_response(&runtime->discovery,
                                                        frame->identifier,
                                                        &response);
    if (discovery_status != MOTOR_DISCOVERY_STATUS_OK)
    {
        if (runtime->discovery.state == MOTOR_DISCOVERY_STATE_FAILED)
        {
            runtime->discovery_active = 0U;
        }
        ++runtime->rejected_parameter_response_count;
        return motor_runtime_map_discovery_status(discovery_status);
    }

    motor->parameter_valid_mask =
        runtime->discovery.results[response_joint_index].verified_fields_mask;
    motor->parameter_source = MOTOR_PARAMETER_SOURCE_DISCOVERED;
    if (motor->parameter_valid_mask == MOTOR_DISCOVERY_ALL_FIELDS_MASK)
    {
        motor->configuration_consistent = 1U;
        motor->state = MOTOR_LIFECYCLE_DISABLED;
    }
    ++runtime->accepted_parameter_response_count;
    if (runtime->discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE)
    {
        runtime->discovery_active = 0U;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Decodes identity-checked control feedback using discovered ranges.
 * @param runtime Initialized runtime.
 * @param frame Received control feedback frame.
 * @param timestamp_us Timestamp attached to the decoded sample.
 * @return OK or a precise range, codec, identity, or freshness error.
 */
static MotorRuntimeStatus motor_runtime_accept_control_feedback(
    MotorRuntime *runtime,
    const CanFrame *frame,
    uint64_t timestamp_us)
{
    uint8_t joint_index;
    S3519Feedback decoded_feedback;
    MotorJointFeedback bank_feedback;
    S3519CodecStatus codec_status;
    MotorBankStatus bank_status;

    if (motor_runtime_find_joint(runtime, frame->identifier, &joint_index) == 0U)
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_ID_MISMATCH;
    }

    codec_status = s3519_decode_feedback(
        frame,
        &runtime->discovery.results[joint_index].ranges,
        &decoded_feedback);
    if (codec_status == S3519_CODEC_STATUS_INVALID_RANGE)
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE;
    }
    if (codec_status != S3519_CODEC_STATUS_OK)
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
    }

    memset(&bank_feedback, 0, sizeof(bank_feedback));
    bank_feedback.position_rad = decoded_feedback.position_rad;
    bank_feedback.velocity_rad_s = decoded_feedback.velocity_rad_s;
    bank_feedback.torque_nm = decoded_feedback.torque_nm;
    bank_feedback.mos_temperature_c = decoded_feedback.mos_temperature_c;
    bank_feedback.rotor_temperature_c = decoded_feedback.rotor_temperature_c;
    bank_feedback.fault_flags =
        (decoded_feedback.state >= S3519_DRIVER_STATE_FAULT_MINIMUM)
            ? decoded_feedback.state
            : 0U;
    bank_feedback.timestamp_us = timestamp_us;
    bank_feedback.driver_state = decoded_feedback.state;

    bank_status = motor_bank_update_feedback(&runtime->bank,
                                             frame->identifier,
                                             decoded_feedback.esc_id,
                                             &bank_feedback);
    if ((bank_status == MOTOR_BANK_STATUS_ID_MISMATCH) ||
        (bank_status == MOTOR_BANK_STATUS_ID_UNKNOWN))
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_ID_MISMATCH;
    }
    if (bank_status == MOTOR_BANK_STATUS_STALE_SAMPLE)
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_STALE_FEEDBACK;
    }
    if (bank_status != MOTOR_BANK_STATUS_OK)
    {
        ++runtime->rejected_feedback_count;
        return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
    }

    if (decoded_feedback.state >= S3519_DRIVER_STATE_FAULT_MINIMUM)
    {
        runtime->bank.motors[joint_index].state = MOTOR_LIFECYCLE_FAULT;
    }
    else if (decoded_feedback.state == S3519_DRIVER_STATE_ENABLED)
    {
        runtime->bank.motors[joint_index].state = MOTOR_LIFECYCLE_ENABLED;
    }
    else
    {
        runtime->bank.motors[joint_index].state = MOTOR_LIFECYCLE_DISABLED;
    }
    ++runtime->accepted_feedback_count;
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Initializes seven motor identities and read-only discovery state.
 */
MotorRuntimeStatus motor_runtime_init(MotorRuntime *runtime,
                                      const ArmConfig *configuration)
{
    if ((runtime == NULL) || (configuration == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }

    memset(runtime, 0, sizeof(*runtime));
    if ((motor_bank_init(&runtime->bank, configuration) != MOTOR_BANK_STATUS_OK) ||
        (motor_discovery_init(&runtime->discovery, configuration) !=
         MOTOR_DISCOVERY_STATUS_OK))
    {
        return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
    }

    runtime->configuration = configuration;
    runtime->initialized = 1U;
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Starts a fresh discovery pass for an explicit bench motor subset.
 */
MotorRuntimeStatus motor_runtime_begin_discovery(MotorRuntime *runtime,
                                                 uint8_t target_joint_mask)
{
    uint8_t joint_index;

    if ((runtime == NULL) || (target_joint_mask == 0U) ||
        ((target_joint_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    if (motor_discovery_begin(&runtime->discovery, target_joint_mask) !=
        MOTOR_DISCOVERY_STATUS_OK)
    {
        return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
    }
    motor_runtime_clear_parameter_signature(
        &runtime->discovery_parameter_expectation);
    runtime->discovery_active = 1U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((target_joint_mask & joint_bit) != 0U)
        {
            MotorObject *motor = &runtime->bank.motors[joint_index];

            motor->state = MOTOR_LIFECYCLE_DISCOVERING;
            motor->parameter_valid_mask = 0U;
            motor->parameter_source = MOTOR_PARAMETER_SOURCE_UNKNOWN;
            motor->configuration_consistent = 0U;
            motor->target_valid = 0U;
            motor->at_target = 0U;
        }
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Produces at most one bounded read-only discovery frame.
 */
MotorRuntimeStatus motor_runtime_next_discovery_frame(MotorRuntime *runtime,
                                                      uint64_t timestamp_us,
                                                      CanFrame *frame)
{
    MotorDiscoveryStatus discovery_status;
    uint8_t request_joint_index;

    if ((runtime == NULL) || (frame == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    if ((runtime->discovery_active != 0U) &&
        (motor_runtime_parameter_quarantine_blocks(runtime, timestamp_us) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_WAITING;
    }
    request_joint_index = runtime->discovery.current_joint_index;
    discovery_status = motor_discovery_next_request(&runtime->discovery,
                                                     timestamp_us,
                                                     frame);
    if ((discovery_status == MOTOR_DISCOVERY_STATUS_FRAME_READY) &&
        (request_joint_index < ARM_JOINT_COUNT))
    {
        motor_runtime_record_parameter_expectation(
            runtime,
            &runtime->discovery_parameter_expectation,
            request_joint_index,
            frame,
            timestamp_us);
    }
    if ((runtime->discovery.state == MOTOR_DISCOVERY_STATE_COMPLETE) ||
        (runtime->discovery.state == MOTOR_DISCOVERY_STATE_FAILED))
    {
        runtime->discovery_active = 0U;
    }
    return motor_runtime_map_discovery_status(discovery_status);
}

/**
 * @brief Aborts active parameter sequences without revalidating selected mode fields.
 */
MotorRuntimeStatus motor_runtime_abort_active_parameter_sequences(
    MotorRuntime *runtime,
    uint64_t timestamp_us)
{
    uint8_t mode_switch_is_active;
    uint8_t parameter_sequence_is_active;

    if (runtime == NULL)
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    mode_switch_is_active =
        (uint8_t)((runtime->mode_switch_state == MOTOR_MODE_SWITCH_WRITING) ||
                  (runtime->mode_switch_state == MOTOR_MODE_SWITCH_READ_READY) ||
                  (runtime->mode_switch_state ==
                   MOTOR_MODE_SWITCH_READ_WAITING));
    parameter_sequence_is_active =
        (uint8_t)((runtime->discovery_active != 0U) ||
                  (mode_switch_is_active != 0U));
    if ((parameter_sequence_is_active != 0U) &&
        ((runtime->mode_parameter_expectation.valid != 0U) ||
         (runtime->discovery_parameter_expectation.valid != 0U)))
    {
        runtime->parameter_quarantine =
            (runtime->mode_parameter_expectation.valid != 0U)
                ? runtime->mode_parameter_expectation
                : runtime->discovery_parameter_expectation;
        runtime->parameter_quarantine.timestamp_us = timestamp_us;
        motor_runtime_clear_parameter_signature(
            &runtime->mode_parameter_expectation);
        motor_runtime_clear_parameter_signature(
            &runtime->discovery_parameter_expectation);
    }
    if (runtime->discovery_active != 0U)
    {
        runtime->discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
        runtime->discovery.target_joint_mask = 0U;
        runtime->discovery.current_joint_index = 0U;
        runtime->discovery.current_register_index = 0U;
        runtime->discovery.attempt_count = 0U;
        runtime->discovery.request_sent_at_us = 0U;
        runtime->discovery_active = 0U;
    }
    if (mode_switch_is_active != 0U)
    {
        runtime->mode_switch_state = MOTOR_MODE_SWITCH_IDLE;
        runtime->mode_switch_joint_mask = 0U;
        runtime->mode_switch_joint_index = 0U;
        runtime->mode_switch_attempt_count = 0U;
        runtime->mode_request_sent_at_us = 0U;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Routes one validated Classic CAN frame to discovery or feedback decode.
 */
MotorRuntimeStatus motor_runtime_accept_frame(MotorRuntime *runtime,
                                              const CanFrame *frame,
                                              uint64_t timestamp_us)
{
    if ((runtime == NULL) || (frame == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    if ((motor_runtime_frame_matches_parameter_signature(
             &runtime->mode_parameter_expectation,
             frame) != 0U) &&
        (motor_runtime_is_mode_readback_response(runtime, frame) != 0U))
    {
        motor_runtime_clear_parameter_signature(
            &runtime->mode_parameter_expectation);
        return motor_runtime_accept_mode_readback(runtime, frame);
    }
    if ((motor_runtime_frame_matches_parameter_signature(
             &runtime->mode_parameter_expectation,
             frame) != 0U) &&
        ((runtime->mode_switch_state == MOTOR_MODE_SWITCH_WRITING) ||
         (runtime->mode_switch_state == MOTOR_MODE_SWITCH_READ_READY) ||
         (runtime->mode_switch_state == MOTOR_MODE_SWITCH_READ_WAITING)) &&
        (frame->data[2] == 0x55U))
    {
        motor_runtime_clear_parameter_signature(
            &runtime->mode_parameter_expectation);
        return MOTOR_RUNTIME_STATUS_OK;
    }

    if ((motor_runtime_frame_matches_parameter_signature(
             &runtime->discovery_parameter_expectation,
             frame) != 0U) &&
        (motor_runtime_is_parameter_response(runtime, frame) != 0U))
    {
        motor_runtime_clear_parameter_signature(
            &runtime->discovery_parameter_expectation);
        return motor_runtime_accept_parameter_response(runtime, frame);
    }
    if ((motor_runtime_parameter_quarantine_blocks(runtime,
                                                   timestamp_us) != 0U) &&
        (motor_runtime_frame_matches_parameter_signature(
             &runtime->parameter_quarantine,
             frame) != 0U))
    {
        motor_runtime_clear_parameter_signature(&runtime->parameter_quarantine);
        return MOTOR_RUNTIME_STATUS_OK;
    }
    return motor_runtime_accept_control_feedback(runtime, frame, timestamp_us);
}

/**
 * @brief Copies one coherent seven-axis feedback snapshot.
 */
MotorRuntimeStatus motor_runtime_get_snapshot(const MotorRuntime *runtime,
                                              uint64_t timestamp_us,
                                              uint64_t stale_after_us,
                                              MotorFeedbackSnapshot *snapshot)
{
    if ((runtime == NULL) || (snapshot == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    if (motor_bank_get_snapshot(&runtime->bank,
                                timestamp_us,
                                stale_after_us,
                                snapshot) != MOTOR_BANK_STATUS_OK)
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/** @brief Appends one validated disable command to an emergency batch. */
static MotorRuntimeStatus motor_runtime_append_disable(
    MotorEmergencyFrameBatch *batch,
    uint8_t esc_id,
    S3519ControlMode mode)
{
    if ((batch->count >= MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES) ||
        (s3519_pack_mode_command(esc_id,
                                  mode,
                                  S3519_MODE_COMMAND_DISABLE,
                                  &batch->frames[batch->count]) !=
         S3519_CODEC_STATUS_OK))
    {
        return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
    }
    ++batch->count;
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Builds fail-safe disable frames, covering both identifiers if mode is unknown.
 */
MotorRuntimeStatus motor_runtime_build_emergency_disable(
    const MotorRuntime *runtime,
    MotorEmergencyFrameBatch *batch)
{
    return motor_runtime_build_emergency_disable_subset(runtime,
                                                        0x7FU,
                                                        batch);
}

/**
 * @brief Builds fail-safe disable frames for only the selected motors.
 */
MotorRuntimeStatus motor_runtime_build_emergency_disable_subset(
    const MotorRuntime *runtime,
    uint8_t motor_mask,
    MotorEmergencyFrameBatch *batch)
{
    MotorEmergencyFrameBatch validated_batch;
    uint8_t joint_index;

    if (batch != NULL)
    {
        memset(batch, 0, sizeof(*batch));
    }
    if ((runtime == NULL) || (batch == NULL) || (motor_mask == 0U) ||
        ((motor_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    memset(&validated_batch, 0, sizeof(validated_batch));
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const MotorDiscoveryResult *discovery_result;
        uint8_t esc_id = (uint8_t)runtime->configuration->joints[joint_index].esc_id;
        uint32_t observed_mode;
        uint8_t mode_is_verified;

        if ((motor_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        discovery_result = &runtime->discovery.results[joint_index];
        observed_mode = discovery_result->observed_control_mode;
        mode_is_verified =
            (uint8_t)((discovery_result->verified_fields_mask &
                       MOTOR_DISCOVERY_MODE_FIELDS_MASK) ==
                      MOTOR_DISCOVERY_MODE_FIELDS_MASK);

        if ((mode_is_verified != 0U) && (observed_mode == 1U))
        {
            if (motor_runtime_append_disable(&validated_batch,
                                             esc_id,
                                             S3519_CONTROL_MODE_MIT) !=
                MOTOR_RUNTIME_STATUS_OK)
            {
                return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
            }
        }
        else if ((mode_is_verified != 0U) && (observed_mode == 2U))
        {
            if (motor_runtime_append_disable(
                    &validated_batch,
                    esc_id,
                    S3519_CONTROL_MODE_POSITION_VELOCITY) !=
                MOTOR_RUNTIME_STATUS_OK)
            {
                return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
            }
        }
        else
        {
            if ((motor_runtime_append_disable(&validated_batch,
                                              esc_id,
                                              S3519_CONTROL_MODE_MIT) !=
                 MOTOR_RUNTIME_STATUS_OK) ||
                (motor_runtime_append_disable(
                     &validated_batch,
                     esc_id,
                     S3519_CONTROL_MODE_POSITION_VELOCITY) !=
                 MOTOR_RUNTIME_STATUS_OK))
            {
                return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
            }
        }
    }
    memcpy(batch, &validated_batch, sizeof(*batch));
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Encodes one ordered all-or-nothing J1-J7 motor control group.
 */
MotorRuntimeStatus motor_runtime_build_control_group(
    const MotorRuntime *runtime,
    S3519ControlMode control_mode,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_velocity_rad_s[ARM_JOINT_COUNT],
    CanFrame frames[ARM_JOINT_COUNT])
{
    CanFrame validated_frames[ARM_JOINT_COUNT];
    uint8_t joint_index;

    if ((runtime == NULL) || (motor_position_rad == NULL) ||
        (motor_velocity_rad_s == NULL) || (frames == NULL) ||
        ((control_mode != S3519_CONTROL_MODE_POSITION_VELOCITY) &&
         (control_mode != S3519_CONTROL_MODE_MIT)))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    memset(validated_frames, 0, sizeof(validated_frames));
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint = &runtime->configuration->joints[joint_index];
        S3519CodecStatus codec_status;

        if (control_mode == S3519_CONTROL_MODE_POSITION_VELOCITY)
        {
            codec_status = s3519_pack_position_velocity(
                (uint8_t)joint->esc_id,
                motor_position_rad[joint_index],
                fabsf(motor_velocity_rad_s[joint_index]),
                &validated_frames[joint_index]);
        }
        else
        {
            codec_status = s3519_pack_mit(
                (uint8_t)joint->esc_id,
                &runtime->discovery.results[joint_index].ranges,
                motor_position_rad[joint_index],
                motor_velocity_rad_s[joint_index],
                joint->mit_kp,
                joint->mit_kd,
                0.0F,
                &validated_frames[joint_index]);
        }
        if (codec_status != S3519_CODEC_STATUS_OK)
        {
            return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
        }
    }
    memcpy(frames, validated_frames, sizeof(validated_frames));
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Gets complete discovered POS_VEL limits for one motor.
 */
MotorRuntimeStatus motor_runtime_get_position_velocity_limits(
    const MotorRuntime *runtime,
    uint8_t joint_index,
    MotorPositionVelocityLimits *limits)
{
    const MotorDiscoveryResult *discovery_result;
    uint8_t joint_bit;

    if ((runtime == NULL) || (limits == NULL) ||
        (joint_index >= ARM_JOINT_COUNT))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    memset(limits, 0, sizeof(*limits));
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    joint_bit = (uint8_t)(1U << joint_index);
    discovery_result = &runtime->discovery.results[joint_index];
    if (((runtime->discovery.verified_joint_mask & joint_bit) == 0U) ||
        ((discovery_result->verified_fields_mask &
          MOTOR_DISCOVERY_ALL_FIELDS_MASK) != MOTOR_DISCOVERY_ALL_FIELDS_MASK) ||
        !isfinite(discovery_result->ranges.position_max_rad) ||
        !isfinite(discovery_result->ranges.velocity_max_rad_s) ||
        !isfinite(discovery_result->maximum_speed_rad_s) ||
        (discovery_result->ranges.position_max_rad <= 0.0F) ||
        (discovery_result->ranges.velocity_max_rad_s <= 0.0F) ||
        (discovery_result->maximum_speed_rad_s <= 0.0F))
    {
        return MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE;
    }

    limits->position_max_rad = discovery_result->ranges.position_max_rad;
    limits->velocity_mapping_max_rad_s =
        discovery_result->ranges.velocity_max_rad_s;
    limits->maximum_speed_rad_s = discovery_result->maximum_speed_rad_s;
    limits->move_speed_limit_rad_s =
        fminf(limits->velocity_mapping_max_rad_s,
              limits->maximum_speed_rad_s);
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Validates finite POS_VEL values against one discovered motor contract.
 * @param runtime Initialized runtime owning current discovered limits.
 * @param joint_index Zero-based target joint index.
 * @param position_rad Requested motor position.
 * @param speed_rad_s Requested nonnegative or positive speed limit.
 * @param require_positive_speed One for move commands, zero for HOLD-capable encoding.
 * @return OK or a discovery, position, or speed range error.
 */
static MotorRuntimeStatus motor_runtime_validate_position_velocity_values(
    const MotorRuntime *runtime,
    uint8_t joint_index,
    float position_rad,
    float speed_rad_s,
    uint8_t require_positive_speed)
{
    MotorPositionVelocityLimits limits;
    MotorRuntimeStatus runtime_status =
        motor_runtime_get_position_velocity_limits(runtime,
                                                   joint_index,
                                                   &limits);

    if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
    {
        return runtime_status;
    }
    if (!isfinite(position_rad) ||
        (fabsf(position_rad) > limits.position_max_rad))
    {
        return MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE;
    }
    if (!isfinite(speed_rad_s) ||
        ((require_positive_speed != 0U) && (speed_rad_s <= 0.0F)) ||
        ((require_positive_speed == 0U) && (speed_rad_s < 0.0F)) ||
        (speed_rad_s > limits.move_speed_limit_rad_s))
    {
        return MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Validates a selected fault-free positive-speed POS_VEL move.
 */
MotorRuntimeStatus motor_runtime_validate_position_velocity_move_subset(
    const MotorRuntime *runtime,
    const MotorFeedbackSnapshot *feedback_snapshot,
    uint8_t motor_mask,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_speed_rad_s[ARM_JOINT_COUNT],
    uint8_t *failed_joint_index)
{
    uint8_t joint_index;

    if ((runtime == NULL) || (feedback_snapshot == NULL) ||
        (motor_position_rad == NULL) || (motor_speed_rad_s == NULL) ||
        (failed_joint_index == NULL) || (motor_mask == 0U) ||
        ((motor_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        MotorRuntimeStatus runtime_status;

        if ((motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        if ((feedback_snapshot->valid_joint_mask & joint_bit) == 0U)
        {
            *failed_joint_index = joint_index;
            return MOTOR_RUNTIME_STATUS_STALE_FEEDBACK;
        }
        if (feedback_snapshot->joints[joint_index].fault_flags != 0U)
        {
            *failed_joint_index = joint_index;
            return MOTOR_RUNTIME_STATUS_FAULT_PRESENT;
        }
        runtime_status = motor_runtime_validate_position_velocity_values(
            runtime,
            joint_index,
            motor_position_rad[joint_index],
            motor_speed_rad_s[joint_index],
            1U);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
        {
            *failed_joint_index = joint_index;
            return runtime_status;
        }
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Encodes selected POS_VEL targets without altering unselected motors.
 */
MotorRuntimeStatus motor_runtime_build_position_velocity_subset(
    const MotorRuntime *runtime,
    uint8_t motor_mask,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_velocity_rad_s[ARM_JOINT_COUNT],
    MotorEmergencyFrameBatch *batch)
{
    MotorEmergencyFrameBatch validated_batch;
    uint8_t joint_index;

    if (batch != NULL)
    {
        memset(batch, 0, sizeof(*batch));
    }
    if ((runtime == NULL) || (motor_position_rad == NULL) ||
        (motor_velocity_rad_s == NULL) || (batch == NULL) ||
        (motor_mask == 0U) || ((motor_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    memset(&validated_batch, 0, sizeof(validated_batch));
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        MotorRuntimeStatus runtime_status;

        if ((motor_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        runtime_status = motor_runtime_validate_position_velocity_values(
            runtime,
            joint_index,
            motor_position_rad[joint_index],
            motor_velocity_rad_s[joint_index],
            0U);
        if (runtime_status != MOTOR_RUNTIME_STATUS_OK)
        {
            return runtime_status;
        }
        if (s3519_pack_position_velocity(
                (uint8_t)runtime->configuration->joints[joint_index].esc_id,
                motor_position_rad[joint_index],
                motor_velocity_rad_s[joint_index],
                &validated_batch.frames[validated_batch.count]) !=
            S3519_CODEC_STATUS_OK)
        {
            return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
        }
        ++validated_batch.count;
    }
    memcpy(batch, &validated_batch, sizeof(*batch));
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Starts a seven-motor volatile control-mode write/readback operation.
 */
MotorRuntimeStatus motor_runtime_begin_control_mode_switch(
    MotorRuntime *runtime,
    S3519ControlMode control_mode)
{
    return motor_runtime_begin_control_mode_switch_mask(runtime,
                                                        control_mode,
                                                        (uint8_t)0x7FU);
}

/**
 * @brief Starts a volatile control-mode write/readback for a selected subset.
 */
MotorRuntimeStatus motor_runtime_begin_control_mode_switch_mask(
    MotorRuntime *runtime,
    S3519ControlMode control_mode,
    uint8_t motor_mask)
{
    uint8_t joint_index;

    if (runtime == NULL)
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    if ((control_mode != S3519_CONTROL_MODE_MIT) &&
        (control_mode != S3519_CONTROL_MODE_POSITION_VELOCITY))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if ((motor_mask == 0U) || ((motor_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (motor_runtime_selected_discovery_is_ready(runtime, motor_mask) == 0U)
    {
        return MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR;
    }
    if ((runtime->mode_switch_state != MOTOR_MODE_SWITCH_IDLE) &&
        (runtime->mode_switch_state != MOTOR_MODE_SWITCH_COMPLETE) &&
        (runtime->mode_switch_state != MOTOR_MODE_SWITCH_FAILED))
    {
        return MOTOR_RUNTIME_STATUS_WAITING;
    }
    runtime->requested_control_mode = control_mode;
    runtime->mode_switch_joint_mask = motor_mask;
    runtime->mode_switch_joint_index = 0U;
    motor_runtime_skip_unselected_mode_joints(runtime);
    runtime->mode_switch_attempt_count = 0U;
    runtime->mode_request_sent_at_us = 0U;
    runtime->mode_switch_state = MOTOR_MODE_SWITCH_WRITING;
    motor_runtime_clear_parameter_signature(
        &runtime->mode_parameter_expectation);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);

        if ((motor_mask & joint_bit) == 0U)
        {
            continue;
        }
        runtime->discovery.results[joint_index].verified_fields_mask &=
            (uint16_t)~MOTOR_DISCOVERY_MODE_FIELDS_MASK;
        runtime->discovery.verified_joint_mask &= (uint8_t)~joint_bit;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}

/**
 * @brief Produces the next mode write or readback request frame.
 */
MotorRuntimeStatus motor_runtime_next_control_mode_frame(
    MotorRuntime *runtime,
    uint64_t timestamp_us,
    CanFrame *frame)
{
    uint8_t esc_id;
    uint8_t request_joint_index;

    if ((runtime == NULL) || (frame == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->mode_switch_state == MOTOR_MODE_SWITCH_COMPLETE)
    {
        return MOTOR_RUNTIME_STATUS_ACTION_COMPLETE;
    }
    if (runtime->mode_switch_state == MOTOR_MODE_SWITCH_FAILED)
    {
        return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
    }
    if (motor_runtime_parameter_quarantine_blocks(runtime, timestamp_us) != 0U)
    {
        return MOTOR_RUNTIME_STATUS_WAITING;
    }
    if (runtime->mode_switch_state == MOTOR_MODE_SWITCH_READ_WAITING)
    {
        if ((timestamp_us >= runtime->mode_request_sent_at_us) &&
            ((timestamp_us - runtime->mode_request_sent_at_us) <
             MOTOR_DISCOVERY_REQUEST_TIMEOUT_US))
        {
            return MOTOR_RUNTIME_STATUS_WAITING;
        }
        ++runtime->mode_switch_attempt_count;
        if (runtime->mode_switch_attempt_count >= MOTOR_DISCOVERY_MAX_ATTEMPTS)
        {
            runtime->mode_switch_state = MOTOR_MODE_SWITCH_FAILED;
            return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
        }
        runtime->mode_switch_state = MOTOR_MODE_SWITCH_READ_READY;
    }
    if (runtime->mode_switch_joint_index >= ARM_JOINT_COUNT)
    {
        runtime->mode_switch_state = MOTOR_MODE_SWITCH_COMPLETE;
        return MOTOR_RUNTIME_STATUS_ACTION_COMPLETE;
    }
    esc_id = (uint8_t)runtime->configuration->joints[
        runtime->mode_switch_joint_index].esc_id;
    request_joint_index = runtime->mode_switch_joint_index;
    if (runtime->mode_switch_state == MOTOR_MODE_SWITCH_WRITING)
    {
        uint32_t mode_value =
            (runtime->requested_control_mode == S3519_CONTROL_MODE_MIT) ? 1U : 2U;

        if (s3519_pack_control_mode_write(esc_id, mode_value, frame) !=
            S3519_CODEC_STATUS_OK)
        {
            runtime->mode_switch_state = MOTOR_MODE_SWITCH_FAILED;
            return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
        }
        ++runtime->mode_switch_joint_index;
        motor_runtime_skip_unselected_mode_joints(runtime);
        if (runtime->mode_switch_joint_index >= ARM_JOINT_COUNT)
        {
            runtime->mode_switch_joint_index = 0U;
            motor_runtime_skip_unselected_mode_joints(runtime);
            runtime->mode_switch_state = MOTOR_MODE_SWITCH_READ_READY;
        }
        motor_runtime_record_parameter_expectation(
            runtime,
            &runtime->mode_parameter_expectation,
            request_joint_index,
            frame,
            timestamp_us);
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    if (runtime->mode_switch_state == MOTOR_MODE_SWITCH_READ_READY)
    {
        if (s3519_pack_parameter_read(esc_id,
                                      S3519_REGISTER_CONTROL_MODE,
                                      frame) != S3519_CODEC_STATUS_OK)
        {
            runtime->mode_switch_state = MOTOR_MODE_SWITCH_FAILED;
            return MOTOR_RUNTIME_STATUS_ACTION_FAILED;
        }
        runtime->mode_request_sent_at_us = timestamp_us;
        runtime->mode_switch_state = MOTOR_MODE_SWITCH_READ_WAITING;
        motor_runtime_record_parameter_expectation(
            runtime,
            &runtime->mode_parameter_expectation,
            request_joint_index,
            frame,
            timestamp_us);
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    return MOTOR_RUNTIME_STATUS_WAITING;
}

/**
 * @brief Builds one enable/disable/clear command for each selected motor.
 */
MotorRuntimeStatus motor_runtime_build_mode_command_batch(
    const MotorRuntime *runtime,
    S3519ControlMode control_mode,
    S3519ModeCommand command,
    uint8_t motor_mask,
    MotorEmergencyFrameBatch *batch)
{
    uint8_t joint_index;

    if ((runtime == NULL) || (batch == NULL) || (motor_mask == 0U) ||
        ((motor_mask & (uint8_t)~0x7FU) != 0U))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }
    memset(batch, 0, sizeof(*batch));
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((motor_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        if ((batch->count >= MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES) ||
            (s3519_pack_mode_command(
                 (uint8_t)runtime->configuration->joints[joint_index].esc_id,
                 control_mode,
                 command,
                 &batch->frames[batch->count]) != S3519_CODEC_STATUS_OK))
        {
            return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
        }
        ++batch->count;
    }
    return MOTOR_RUNTIME_STATUS_OK;
}
