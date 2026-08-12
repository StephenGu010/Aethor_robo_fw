/**
 * @file motor_discovery.h
 * @brief Defines bounded read-only discovery for seven S3519 motors.
 */

#ifndef APP_MOTOR_MOTOR_DISCOVERY_H
#define APP_MOTOR_MOTOR_DISCOVERY_H

#include <stdint.h>

#include "arm_config.h"
#include "s3519_codec.h"

#define MOTOR_DISCOVERY_REGISTER_COUNT (12U)
#define MOTOR_DISCOVERY_REQUEST_TIMEOUT_US (20000ULL)
#define MOTOR_DISCOVERY_MAX_ATTEMPTS (4U)
#define MOTOR_DISCOVERY_ALL_FIELDS_MASK (0x0FFFU)

/**
 * @brief Identifies the progress of the read-only discovery process.
 */
typedef enum
{
    MOTOR_DISCOVERY_STATE_READY = 0,
    MOTOR_DISCOVERY_STATE_WAITING,
    MOTOR_DISCOVERY_STATE_COMPLETE,
    MOTOR_DISCOVERY_STATE_FAILED
} MotorDiscoveryState;

/**
 * @brief Reports discovery requests, waits, completion, and fail-safe errors.
 */
typedef enum
{
    MOTOR_DISCOVERY_STATUS_OK = 0,
    MOTOR_DISCOVERY_STATUS_FRAME_READY,
    MOTOR_DISCOVERY_STATUS_WAITING,
    MOTOR_DISCOVERY_STATUS_COMPLETE,
    MOTOR_DISCOVERY_STATUS_INVALID_ARGUMENT,
    MOTOR_DISCOVERY_STATUS_INVALID_STATE,
    MOTOR_DISCOVERY_STATUS_UNEXPECTED_RESPONSE,
    MOTOR_DISCOVERY_STATUS_CONFIG_MISMATCH,
    MOTOR_DISCOVERY_STATUS_BAD_VALUE,
    MOTOR_DISCOVERY_STATUS_TIMEOUT,
    MOTOR_DISCOVERY_STATUS_CODEC_ERROR
} MotorDiscoveryStatus;

/**
 * @brief Stores the six verified runtime values for one motor.
 */
typedef struct
{
    S3519Ranges ranges;
    float acceleration_rad_s2;
    float deceleration_rad_s2;
    float maximum_speed_rad_s;
    uint32_t observed_master_id;
    uint32_t observed_esc_id;
    uint32_t observed_control_mode;
    uint32_t hardware_version;
    uint32_t software_version;
    uint32_t sub_version;
    uint16_t verified_fields_mask;
} MotorDiscoveryResult;

/**
 * @brief Owns deterministic one-request-at-a-time discovery state.
 */
typedef struct
{
    const ArmConfig *configuration;
    MotorDiscoveryResult results[ARM_JOINT_COUNT];
    MotorDiscoveryState state;
    uint64_t request_sent_at_us;
    uint8_t verified_joint_mask;
    uint8_t target_joint_mask;
    uint8_t current_joint_index;
    uint8_t current_register_index;
    uint8_t attempt_count;
    uint8_t initialized;
} MotorDiscovery;

/**
 * @brief Initializes read-only discovery from the frozen joint mapping.
 */
MotorDiscoveryStatus motor_discovery_init(MotorDiscovery *discovery,
                                          const ArmConfig *configuration);

/**
 * @brief Starts a fresh bounded discovery pass for an explicit motor subset.
 * @param discovery Initialized discovery domain.
 * @param target_joint_mask Nonzero J1-J7 bit mask.
 * @return OK or an argument/state error.
 */
MotorDiscoveryStatus motor_discovery_begin(MotorDiscovery *discovery,
                                           uint8_t target_joint_mask);

/**
 * @brief Emits the next read request or reports a bounded wait/terminal state.
 */
MotorDiscoveryStatus motor_discovery_next_request(MotorDiscovery *discovery,
                                                  uint64_t timestamp_us,
                                                  CanFrame *frame);

/**
 * @brief Validates and consumes the response for the only outstanding request.
 */
MotorDiscoveryStatus motor_discovery_accept_response(
    MotorDiscovery *discovery,
    uint16_t received_master_id,
    const S3519ParameterResponse *response);

#endif
