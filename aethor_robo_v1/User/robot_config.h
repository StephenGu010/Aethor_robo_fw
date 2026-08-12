/**
 * @file robot_config.h
 * @brief Static seven-axis DH, CAN, joint-limit, and external-drivetrain configuration.
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_JOINT_COUNT 7U
#define ROBOT_ALL_JOINTS_MASK 0x7FU

/** @brief One standard Denavit-Hartenberg row expressed in degrees and millimeters. */
typedef struct
{
    float theta_offset_degrees;
    float d_millimeters;
    float a_millimeters;
    float alpha_degrees;
} RobotDhParameter;

/** @brief One joint-to-motor mapping and its independently commissioned limits. */
typedef struct
{
    uint8_t motor_id;
    uint16_t master_id;
    int8_t direction;
    float joint_zero_degrees;
    float external_reduction_ratio;
    float minimum_degrees;
    float maximum_degrees;
    float maximum_velocity_degrees_s;
    float maximum_acceleration_degrees_s2;
    uint8_t commissioned;
} RobotJointParameter;

/** @brief Complete immutable configuration consumed by the joint-control layer. */
typedef struct
{
    RobotDhParameter dh[ROBOT_JOINT_COUNT];
    RobotJointParameter joint[ROBOT_JOINT_COUNT];
    uint8_t active_joint_mask;
    uint8_t dh_parameters_valid;
} RobotConfiguration;

/** @brief Configuration validation and conversion results. */
typedef enum
{
    ROBOT_CONFIG_STATUS_OK = 0,
    ROBOT_CONFIG_STATUS_INVALID_ARGUMENT,
    ROBOT_CONFIG_STATUS_INVALID_MASK,
    ROBOT_CONFIG_STATUS_INVALID_IDENTIFIER,
    ROBOT_CONFIG_STATUS_DUPLICATE_IDENTIFIER,
    ROBOT_CONFIG_STATUS_INVALID_DIRECTION,
    ROBOT_CONFIG_STATUS_INVALID_REDUCTION,
    ROBOT_CONFIG_STATUS_INVALID_LIMIT,
    ROBOT_CONFIG_STATUS_INVALID_DH
} RobotConfigStatus;

const RobotConfiguration *robot_config_get(void);
RobotConfigStatus robot_config_validate(const RobotConfiguration *configuration);
uint8_t robot_config_active_profiles_are_commissioned(
    const RobotConfiguration *configuration);
RobotConfigStatus robot_config_joint_to_motor_position_rad(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float joint_degrees,
    float *motor_position_rad);
RobotConfigStatus robot_config_motor_to_joint_position_degrees(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float motor_position_rad,
    float *joint_degrees);
RobotConfigStatus robot_config_joint_to_motor_velocity_rad_s(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float joint_velocity_degrees_s,
    float *motor_velocity_rad_s);
RobotConfigStatus robot_config_motor_to_joint_velocity_degrees_s(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float motor_velocity_rad_s,
    float *joint_velocity_degrees_s);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */
