/**
 * @file robot_config.c
 * @brief Owns the immutable seven-axis model and allocation-free drivetrain conversions.
 */

#include "robot_config.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>

#define ROBOT_CONFIG_PI 3.14159265358979323846F
#define ROBOT_CONFIG_DEG_TO_RAD (ROBOT_CONFIG_PI / 180.0F)
#define ROBOT_CONFIG_RAD_TO_DEG (180.0F / ROBOT_CONFIG_PI)

/*
 * Physical DH dimensions, joint zeros, directions, limits, and external reduction
 * ratios have not yet been measured. J1/J2 are the only connected unloaded motors;
 * their Master IDs follow the verified two-motor bench setup. All joint profiles
 * remain uncommissioned until direction, zero, limits, and external reduction are
 * measured; this keeps full multi-axis enable locked while allowing selected-axis
 * commissioning moves.
 */
static const RobotConfiguration robot_production_configuration = {
    {
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.0F, 0.0F}
    },
    {
        {1U, 0x11U, 1, 0.0F, 1.0F, -180.0F, 180.0F, 28.64789F, 60.0F, 0U},
        {2U, 0x12U, 1, 0.0F, 1.0F, -180.0F, 180.0F, 28.64789F, 60.0F, 0U},
        {3U, 0x13U, 1, 0.0F, 1.0F, 0.0F, 0.0F, 3.0F, 6.0F, 0U},
        {4U, 0x14U, 1, 0.0F, 1.0F, 0.0F, 0.0F, 3.0F, 6.0F, 0U},
        {5U, 0x15U, 1, 0.0F, 1.0F, 0.0F, 0.0F, 3.0F, 6.0F, 0U},
        {6U, 0x16U, 1, 0.0F, 1.0F, 0.0F, 0.0F, 3.0F, 6.0F, 0U},
        {7U, 0x17U, 1, 0.0F, 1.0F, 0.0F, 0.0F, 3.0F, 6.0F, 0U}
    },
    0x03U,
    0U
};

/**
 * @brief Returns the production seven-axis configuration.
 * @return Address of immutable static configuration storage.
 */
const RobotConfiguration *robot_config_get(void)
{
    return &robot_production_configuration;
}

/**
 * @brief Validates identifiers and all finite conversion parameters.
 * @param configuration Configuration to validate.
 * @return Explicit validation result.
 */
RobotConfigStatus robot_config_validate(const RobotConfiguration *configuration)
{
    uint8_t first_index;

    if (configuration == NULL)
    {
        return ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    if ((configuration->active_joint_mask == 0U) ||
        ((configuration->active_joint_mask & (uint8_t)~ROBOT_ALL_JOINTS_MASK) != 0U))
    {
        return ROBOT_CONFIG_STATUS_INVALID_MASK;
    }

    for (first_index = 0U; first_index < ROBOT_JOINT_COUNT; ++first_index)
    {
        const RobotDhParameter *dh_parameter = &configuration->dh[first_index];
        const RobotJointParameter *joint_parameter = &configuration->joint[first_index];
        uint8_t second_index;

        if (!isfinite(dh_parameter->theta_offset_degrees) ||
            !isfinite(dh_parameter->d_millimeters) ||
            !isfinite(dh_parameter->a_millimeters) ||
            !isfinite(dh_parameter->alpha_degrees))
        {
            return ROBOT_CONFIG_STATUS_INVALID_DH;
        }
        if ((joint_parameter->motor_id == 0U) || (joint_parameter->motor_id > 0x0FU) ||
            (joint_parameter->master_id > 0x7FFU))
        {
            return ROBOT_CONFIG_STATUS_INVALID_IDENTIFIER;
        }
        if ((joint_parameter->direction != 1) && (joint_parameter->direction != -1))
        {
            return ROBOT_CONFIG_STATUS_INVALID_DIRECTION;
        }
        if (!isfinite(joint_parameter->joint_zero_degrees) ||
            !isfinite(joint_parameter->external_reduction_ratio) ||
            (joint_parameter->external_reduction_ratio <= 0.0F))
        {
            return ROBOT_CONFIG_STATUS_INVALID_REDUCTION;
        }
        if ((joint_parameter->commissioned != 0U) &&
            (!isfinite(joint_parameter->minimum_degrees) ||
             !isfinite(joint_parameter->maximum_degrees) ||
             !isfinite(joint_parameter->maximum_velocity_degrees_s) ||
             !isfinite(joint_parameter->maximum_acceleration_degrees_s2) ||
             (joint_parameter->minimum_degrees >= joint_parameter->maximum_degrees) ||
             (joint_parameter->maximum_velocity_degrees_s <= 0.0F) ||
             (joint_parameter->maximum_acceleration_degrees_s2 <= 0.0F)))
        {
            return ROBOT_CONFIG_STATUS_INVALID_LIMIT;
        }

        if ((configuration->active_joint_mask & (uint8_t)(1U << first_index)) == 0U)
        {
            continue;
        }
        for (second_index = (uint8_t)(first_index + 1U);
             second_index < ROBOT_JOINT_COUNT;
             ++second_index)
        {
            if ((configuration->active_joint_mask & (uint8_t)(1U << second_index)) == 0U)
            {
                continue;
            }
            if ((joint_parameter->motor_id == configuration->joint[second_index].motor_id) ||
                (joint_parameter->master_id == configuration->joint[second_index].master_id))
            {
                return ROBOT_CONFIG_STATUS_DUPLICATE_IDENTIFIER;
            }
        }
    }
    return ROBOT_CONFIG_STATUS_OK;
}

/**
 * @brief Checks whether every active joint has a commissioned physical profile.
 * @param configuration Configuration to inspect.
 * @return One when all active profiles and DH data are commissioned; otherwise zero.
 */
uint8_t robot_config_active_profiles_are_commissioned(
    const RobotConfiguration *configuration)
{
    uint8_t joint_index;

    if (robot_config_validate(configuration) != ROBOT_CONFIG_STATUS_OK)
    {
        return 0U;
    }
    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; ++joint_index)
    {
        if (((configuration->active_joint_mask & (uint8_t)(1U << joint_index)) != 0U) &&
            (configuration->joint[joint_index].commissioned == 0U))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Converts a joint angle into the motor-output position command.
 * @param configuration Valid configuration.
 * @param joint_index Zero-based joint index.
 * @param joint_degrees Joint-space angle in degrees.
 * @param motor_position_rad Receives motor-output radians.
 * @return Conversion status.
 */
RobotConfigStatus robot_config_joint_to_motor_position_rad(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float joint_degrees,
    float *motor_position_rad)
{
    const RobotJointParameter *joint_parameter;

    if ((configuration == NULL) || (motor_position_rad == NULL) ||
        (joint_index >= ROBOT_JOINT_COUNT) || !isfinite(joint_degrees))
    {
        return ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    assert(configuration->joint[joint_index].external_reduction_ratio > 0.0F);
    joint_parameter = &configuration->joint[joint_index];
    *motor_position_rad = (float)joint_parameter->direction *
                          (joint_degrees - joint_parameter->joint_zero_degrees) *
                          joint_parameter->external_reduction_ratio *
                          ROBOT_CONFIG_DEG_TO_RAD;
    return isfinite(*motor_position_rad) ? ROBOT_CONFIG_STATUS_OK
                                         : ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief Converts motor-output radians back into a joint-space angle.
 * @param configuration Valid configuration.
 * @param joint_index Zero-based joint index.
 * @param motor_position_rad Motor-output position in radians.
 * @param joint_degrees Receives joint-space degrees.
 * @return Conversion status.
 */
RobotConfigStatus robot_config_motor_to_joint_position_degrees(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float motor_position_rad,
    float *joint_degrees)
{
    const RobotJointParameter *joint_parameter;

    if ((configuration == NULL) || (joint_degrees == NULL) ||
        (joint_index >= ROBOT_JOINT_COUNT) || !isfinite(motor_position_rad))
    {
        return ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    joint_parameter = &configuration->joint[joint_index];
    if (!isfinite(joint_parameter->external_reduction_ratio) ||
        (joint_parameter->external_reduction_ratio <= 0.0F))
    {
        return ROBOT_CONFIG_STATUS_INVALID_REDUCTION;
    }
    *joint_degrees = ((float)joint_parameter->direction * motor_position_rad *
                      ROBOT_CONFIG_RAD_TO_DEG /
                      joint_parameter->external_reduction_ratio) +
                     joint_parameter->joint_zero_degrees;
    return isfinite(*joint_degrees) ? ROBOT_CONFIG_STATUS_OK
                                    : ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief Converts an absolute joint velocity limit to a motor-output velocity limit.
 * @param configuration Valid configuration.
 * @param joint_index Zero-based joint index.
 * @param joint_velocity_degrees_s Joint velocity limit in degrees per second.
 * @param motor_velocity_rad_s Receives a nonnegative motor-output limit.
 * @return Conversion status.
 */
RobotConfigStatus robot_config_joint_to_motor_velocity_rad_s(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float joint_velocity_degrees_s,
    float *motor_velocity_rad_s)
{
    const RobotJointParameter *joint_parameter;

    if ((configuration == NULL) || (motor_velocity_rad_s == NULL) ||
        (joint_index >= ROBOT_JOINT_COUNT) || !isfinite(joint_velocity_degrees_s))
    {
        return ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    joint_parameter = &configuration->joint[joint_index];
    if (!isfinite(joint_parameter->external_reduction_ratio) ||
        (joint_parameter->external_reduction_ratio <= 0.0F))
    {
        return ROBOT_CONFIG_STATUS_INVALID_REDUCTION;
    }
    *motor_velocity_rad_s = fabsf(joint_velocity_degrees_s) *
                            joint_parameter->external_reduction_ratio *
                            ROBOT_CONFIG_DEG_TO_RAD;
    return isfinite(*motor_velocity_rad_s) ? ROBOT_CONFIG_STATUS_OK
                                           : ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief Converts signed motor-output velocity into signed joint velocity.
 * @param configuration Valid configuration.
 * @param joint_index Zero-based joint index.
 * @param motor_velocity_rad_s Signed motor-output radians per second.
 * @param joint_velocity_degrees_s Receives signed joint degrees per second.
 * @return Conversion status.
 */
RobotConfigStatus robot_config_motor_to_joint_velocity_degrees_s(
    const RobotConfiguration *configuration,
    uint8_t joint_index,
    float motor_velocity_rad_s,
    float *joint_velocity_degrees_s)
{
    const RobotJointParameter *joint_parameter;

    if ((configuration == NULL) || (joint_velocity_degrees_s == NULL) ||
        (joint_index >= ROBOT_JOINT_COUNT) || !isfinite(motor_velocity_rad_s))
    {
        return ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
    }
    joint_parameter = &configuration->joint[joint_index];
    if (!isfinite(joint_parameter->external_reduction_ratio) ||
        (joint_parameter->external_reduction_ratio <= 0.0F))
    {
        return ROBOT_CONFIG_STATUS_INVALID_REDUCTION;
    }
    *joint_velocity_degrees_s = (float)joint_parameter->direction *
                                motor_velocity_rad_s * ROBOT_CONFIG_RAD_TO_DEG /
                                joint_parameter->external_reduction_ratio;
    return isfinite(*joint_velocity_degrees_s) ? ROBOT_CONFIG_STATUS_OK
                                               : ROBOT_CONFIG_STATUS_INVALID_ARGUMENT;
}
