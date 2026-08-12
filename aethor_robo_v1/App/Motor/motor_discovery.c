/**
 * @file motor_discovery.c
 * @brief Implements bounded read-only identity, mode, and range discovery.
 */

#include "motor_discovery.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const S3519Register discovery_registers[MOTOR_DISCOVERY_REGISTER_COUNT] = {
    S3519_REGISTER_MASTER_ID,
    S3519_REGISTER_ESC_ID,
    S3519_REGISTER_CONTROL_MODE,
    S3519_REGISTER_POSITION_RANGE,
    S3519_REGISTER_VELOCITY_RANGE,
    S3519_REGISTER_TORQUE_RANGE
};

/**
 * @brief Returns the immutable mapping for the current discovery joint.
 * @param discovery Initialized discovery state.
 * @return Current joint configuration.
 */
static const JointConfig *motor_discovery_current_configuration(
    const MotorDiscovery *discovery)
{
    return &discovery->configuration->joints[discovery->current_joint_index];
}

/**
 * @brief Latches a terminal discovery failure.
 * @param discovery Discovery state to update.
 * @param status Failure status returned to the caller.
 * @return The supplied failure status.
 */
static MotorDiscoveryStatus motor_discovery_fail(MotorDiscovery *discovery,
                                                 MotorDiscoveryStatus status)
{
    discovery->state = MOTOR_DISCOVERY_STATE_FAILED;
    return status;
}

/**
 * @brief Stores one validated response in the current joint result.
 * @param discovery Discovery state and results.
 * @param response Decoded register response.
 * @return OK or a fail-safe mismatch/value error.
 */
static MotorDiscoveryStatus motor_discovery_store_response(
    MotorDiscovery *discovery,
    const S3519ParameterResponse *response)
{
    const JointConfig *joint_configuration =
        motor_discovery_current_configuration(discovery);
    MotorDiscoveryResult *result = &discovery->results[discovery->current_joint_index];
    S3519Register register_address = discovery_registers[discovery->current_register_index];
    uint8_t field_bit = (uint8_t)(1U << discovery->current_register_index);

    switch (register_address)
    {
        case S3519_REGISTER_MASTER_ID:
            result->observed_master_id = response->raw_value;
            if (response->raw_value != joint_configuration->master_id)
            {
                return motor_discovery_fail(discovery,
                                            MOTOR_DISCOVERY_STATUS_CONFIG_MISMATCH);
            }
            break;

        case S3519_REGISTER_ESC_ID:
            result->observed_esc_id = response->raw_value;
            if (response->raw_value != joint_configuration->esc_id)
            {
                return motor_discovery_fail(discovery,
                                            MOTOR_DISCOVERY_STATUS_CONFIG_MISMATCH);
            }
            break;

        case S3519_REGISTER_CONTROL_MODE:
            result->observed_control_mode = response->raw_value;
            if ((response->raw_value != 1U) && (response->raw_value != 2U))
            {
                return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_BAD_VALUE);
            }
            break;

        case S3519_REGISTER_POSITION_RANGE:
            if (!isfinite(response->float_value) || (response->float_value <= 0.0F))
            {
                return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_BAD_VALUE);
            }
            result->ranges.position_max_rad = response->float_value;
            break;

        case S3519_REGISTER_VELOCITY_RANGE:
            if (!isfinite(response->float_value) || (response->float_value <= 0.0F))
            {
                return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_BAD_VALUE);
            }
            result->ranges.velocity_max_rad_s = response->float_value;
            break;

        case S3519_REGISTER_TORQUE_RANGE:
            if (!isfinite(response->float_value) || (response->float_value <= 0.0F))
            {
                return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_BAD_VALUE);
            }
            result->ranges.torque_max_nm = response->float_value;
            break;

        default:
            return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_BAD_VALUE);
    }

    result->verified_fields_mask |= field_bit;
    return MOTOR_DISCOVERY_STATUS_OK;
}

/**
 * @brief Advances to the next register or marks a joint/all discovery complete.
 * @param discovery Discovery state to advance.
 */
static void motor_discovery_advance(MotorDiscovery *discovery)
{
    ++discovery->current_register_index;
    discovery->attempt_count = 0U;

    if (discovery->current_register_index < MOTOR_DISCOVERY_REGISTER_COUNT)
    {
        discovery->state = MOTOR_DISCOVERY_STATE_READY;
        return;
    }

    if (discovery->results[discovery->current_joint_index].verified_fields_mask ==
        MOTOR_DISCOVERY_ALL_FIELDS_MASK)
    {
        discovery->verified_joint_mask |=
            (uint8_t)(1U << discovery->current_joint_index);
    }

    ++discovery->current_joint_index;
    discovery->current_register_index = 0U;
    if (discovery->current_joint_index >= ARM_JOINT_COUNT)
    {
        discovery->state = MOTOR_DISCOVERY_STATE_COMPLETE;
    }
    else
    {
        discovery->state = MOTOR_DISCOVERY_STATE_READY;
    }
}

/**
 * @brief Initializes read-only discovery from the frozen joint mapping.
 * @param discovery Destination discovery state.
 * @param configuration Seven-axis mapping.
 * @return OK or an argument/configuration error.
 */
MotorDiscoveryStatus motor_discovery_init(MotorDiscovery *discovery,
                                          const ArmConfig *configuration)
{
    ArmConfigValidation validation;

    if ((discovery == NULL) || (configuration == NULL))
    {
        return MOTOR_DISCOVERY_STATUS_INVALID_ARGUMENT;
    }

    memset(discovery, 0, sizeof(*discovery));
    if (!arm_config_validate_schema(configuration, &validation))
    {
        discovery->state = MOTOR_DISCOVERY_STATE_FAILED;
        return MOTOR_DISCOVERY_STATUS_CONFIG_MISMATCH;
    }

    discovery->configuration = configuration;
    discovery->state = MOTOR_DISCOVERY_STATE_READY;
    discovery->initialized = 1U;
    return MOTOR_DISCOVERY_STATUS_OK;
}

/**
 * @brief Emits the next read request or reports a bounded wait/terminal state.
 * @param discovery Initialized discovery state.
 * @param timestamp_us Current monotonic timestamp.
 * @param frame Destination request frame.
 * @return FRAME_READY, WAITING, COMPLETE, or a terminal error.
 */
MotorDiscoveryStatus motor_discovery_next_request(MotorDiscovery *discovery,
                                                  uint64_t timestamp_us,
                                                  CanFrame *frame)
{
    const JointConfig *joint_configuration;
    S3519CodecStatus codec_status;

    if ((discovery == NULL) || (discovery->initialized == 0U) || (frame == NULL))
    {
        return MOTOR_DISCOVERY_STATUS_INVALID_ARGUMENT;
    }
    if (discovery->state == MOTOR_DISCOVERY_STATE_COMPLETE)
    {
        return MOTOR_DISCOVERY_STATUS_COMPLETE;
    }
    if (discovery->state == MOTOR_DISCOVERY_STATE_FAILED)
    {
        return MOTOR_DISCOVERY_STATUS_INVALID_STATE;
    }

    if (discovery->state == MOTOR_DISCOVERY_STATE_WAITING)
    {
        if ((timestamp_us >= discovery->request_sent_at_us) &&
            ((timestamp_us - discovery->request_sent_at_us) <
             MOTOR_DISCOVERY_REQUEST_TIMEOUT_US))
        {
            return MOTOR_DISCOVERY_STATUS_WAITING;
        }

        if (discovery->attempt_count >= MOTOR_DISCOVERY_MAX_ATTEMPTS)
        {
            return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_TIMEOUT);
        }
    }

    joint_configuration = motor_discovery_current_configuration(discovery);
    codec_status = s3519_pack_parameter_read(
        (uint8_t)joint_configuration->esc_id,
        discovery_registers[discovery->current_register_index],
        frame);
    if (codec_status != S3519_CODEC_STATUS_OK)
    {
        return motor_discovery_fail(discovery, MOTOR_DISCOVERY_STATUS_CODEC_ERROR);
    }

    ++discovery->attempt_count;
    discovery->request_sent_at_us = timestamp_us;
    discovery->state = MOTOR_DISCOVERY_STATE_WAITING;
    return MOTOR_DISCOVERY_STATUS_FRAME_READY;
}

/**
 * @brief Validates and consumes the response for the only outstanding request.
 * @param discovery Initialized discovery state.
 * @param received_master_id Standard CAN identifier carrying the response.
 * @param response Decoded response value.
 * @return OK or a latched fail-safe error.
 */
MotorDiscoveryStatus motor_discovery_accept_response(
    MotorDiscovery *discovery,
    uint16_t received_master_id,
    const S3519ParameterResponse *response)
{
    const JointConfig *joint_configuration;
    S3519Register expected_register;
    MotorDiscoveryStatus store_status;

    if ((discovery == NULL) || (discovery->initialized == 0U) || (response == NULL))
    {
        return MOTOR_DISCOVERY_STATUS_INVALID_ARGUMENT;
    }
    if (discovery->state != MOTOR_DISCOVERY_STATE_WAITING)
    {
        return MOTOR_DISCOVERY_STATUS_INVALID_STATE;
    }

    joint_configuration = motor_discovery_current_configuration(discovery);
    expected_register = discovery_registers[discovery->current_register_index];
    if ((received_master_id != joint_configuration->master_id) ||
        (response->esc_id != joint_configuration->esc_id) ||
        (response->register_address != (uint8_t)expected_register))
    {
        return MOTOR_DISCOVERY_STATUS_UNEXPECTED_RESPONSE;
    }

    store_status = motor_discovery_store_response(discovery, response);
    if (store_status != MOTOR_DISCOVERY_STATUS_OK)
    {
        return store_status;
    }

    motor_discovery_advance(discovery);
    return MOTOR_DISCOVERY_STATUS_OK;
}
