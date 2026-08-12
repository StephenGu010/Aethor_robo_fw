/**
 * @file motor_runtime.c
 * @brief Implements bounded seven-motor discovery and feedback routing.
 */

#include "motor_runtime.h"

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

    if (s3519_decode_parameter_response(frame, &response) != S3519_CODEC_STATUS_OK)
    {
        ++runtime->rejected_parameter_response_count;
        return MOTOR_RUNTIME_STATUS_CODEC_ERROR;
    }

    discovery_status = motor_discovery_accept_response(&runtime->discovery,
                                                        frame->identifier,
                                                        &response);
    if (discovery_status != MOTOR_DISCOVERY_STATUS_OK)
    {
        ++runtime->rejected_parameter_response_count;
        return motor_runtime_map_discovery_status(discovery_status);
    }

    ++runtime->accepted_parameter_response_count;
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
    bank_feedback.fault_flags = decoded_feedback.state;
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
 * @brief Produces at most one bounded read-only discovery frame.
 */
MotorRuntimeStatus motor_runtime_next_discovery_frame(MotorRuntime *runtime,
                                                      uint64_t timestamp_us,
                                                      CanFrame *frame)
{
    if ((runtime == NULL) || (frame == NULL))
    {
        return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT;
    }
    if (runtime->initialized == 0U)
    {
        return MOTOR_RUNTIME_STATUS_NOT_INITIALIZED;
    }

    return motor_runtime_map_discovery_status(
        motor_discovery_next_request(&runtime->discovery, timestamp_us, frame));
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

    if (motor_runtime_is_parameter_response(runtime, frame) != 0U)
    {
        return motor_runtime_accept_parameter_response(runtime, frame);
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
