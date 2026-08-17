/**
 * @file arm_config.c
 * @brief Implements the immutable seven-axis configuration and safety checks.
 */

#include "arm_config.h"

#include <float.h>
#include <stddef.h>

#define ARM_STANDARD_CAN_ID_MAX (0x7FFU)

#define ARM_UNVERIFIED_JOINT(joint_number, esc_identifier, master_identifier) \
    {                                                                         \
        (joint_number), (esc_identifier), (master_identifier), 0,             \
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,         \
        0.0F, 0.0F, 0U                                                       \
    }

static const ArmConfig production_configuration = {
    ARM_JOINT_COUNT,
    {
        ARM_UNVERIFIED_JOINT(0U, 1U, 0x11U),
        ARM_UNVERIFIED_JOINT(1U, 2U, 0x12U),
        ARM_UNVERIFIED_JOINT(2U, 3U, 0x13U),
        ARM_UNVERIFIED_JOINT(3U, 4U, 0x14U),
        ARM_UNVERIFIED_JOINT(4U, 5U, 0x15U),
        ARM_UNVERIFIED_JOINT(5U, 6U, 0x16U),
        ARM_UNVERIFIED_JOINT(6U, 7U, 0x17U)
    }
};

/**
 * @brief Initializes a validation result before inspecting input data.
 * @param validation Result object to initialize.
 */
static void arm_config_initialize_validation(ArmConfigValidation *validation)
{
    validation->schema_errors = ARM_CONFIG_ERROR_NONE;
    validation->missing_verified_fields = 0U;
    validation->first_error_joint = ARM_CONFIG_FIRST_ERROR_NONE;
}

/**
 * @brief Records one validation error while preserving the first bad joint.
 * @param validation Result object to update.
 * @param error Error bit to set.
 * @param joint_index Joint associated with the error.
 */
static void arm_config_record_error(ArmConfigValidation *validation,
                                    uint32_t error,
                                    uint8_t joint_index)
{
    validation->schema_errors |= error;
    if (validation->first_error_joint == ARM_CONFIG_FIRST_ERROR_NONE)
    {
        validation->first_error_joint = joint_index;
    }
}

/**
 * @brief Checks whether a float is finite without depending on C99 isfinite.
 * @param value Value to inspect.
 * @return true when value is neither NaN nor infinity.
 */
static bool arm_config_float_is_finite(float value)
{
    return (value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX);
}

/**
 * @brief Validates confirmed physical values for one joint.
 * @param joint Joint configuration to inspect.
 * @param validation Result object receiving error bits.
 */
static void arm_config_validate_verified_values(const JointConfig *joint,
                                                ArmConfigValidation *validation)
{
    const uint32_t verified_fields = joint->verified_fields;
    const uint8_t joint_index = joint->joint_index;

    if ((verified_fields & ~ARM_JOINT_REQUIRED_ENABLE_FIELDS) != 0U)
    {
        arm_config_record_error(validation,
                                ARM_CONFIG_ERROR_VERIFIED_FIELDS,
                                joint_index);
    }

    if (((verified_fields & ARM_JOINT_VERIFIED_DIRECTION) != 0U) &&
        (joint->direction != -1) && (joint->direction != 1))
    {
        arm_config_record_error(validation, ARM_CONFIG_ERROR_DIRECTION, joint_index);
    }

    if ((verified_fields & ARM_JOINT_VERIFIED_LIMITS) != 0U)
    {
        if (!arm_config_float_is_finite(joint->soft_limit_min_rad) ||
            !arm_config_float_is_finite(joint->soft_limit_max_rad) ||
            (joint->soft_limit_min_rad >= joint->soft_limit_max_rad))
        {
            arm_config_record_error(validation, ARM_CONFIG_ERROR_LIMITS, joint_index);
        }
    }

    if (((verified_fields & ARM_JOINT_VERIFIED_MAX_VELOCITY) != 0U) &&
        (!arm_config_float_is_finite(joint->max_velocity_rad_s) ||
         (joint->max_velocity_rad_s <= 0.0F)))
    {
        arm_config_record_error(validation, ARM_CONFIG_ERROR_VELOCITY, joint_index);
    }

    if (((verified_fields & ARM_JOINT_VERIFIED_MAX_ACCELERATION) != 0U) &&
        (!arm_config_float_is_finite(joint->max_acceleration_rad_s2) ||
         (joint->max_acceleration_rad_s2 <= 0.0F)))
    {
        arm_config_record_error(validation, ARM_CONFIG_ERROR_ACCELERATION, joint_index);
    }

    if ((verified_fields & ARM_JOINT_VERIFIED_MIT_GAINS) != 0U)
    {
        if (!arm_config_float_is_finite(joint->mit_kp) ||
            !arm_config_float_is_finite(joint->mit_kd) ||
            (joint->mit_kp < 0.0F) || (joint->mit_kd < 0.0F))
        {
            arm_config_record_error(validation, ARM_CONFIG_ERROR_MIT_GAINS, joint_index);
        }
    }

    if ((verified_fields & ARM_JOINT_VERIFIED_MOTOR_RANGES) != 0U)
    {
        if (!arm_config_float_is_finite(joint->motor_pmax_rad) ||
            !arm_config_float_is_finite(joint->motor_vmax_rad_s) ||
            !arm_config_float_is_finite(joint->motor_tmax_nm) ||
            (joint->motor_pmax_rad <= 0.0F) ||
            (joint->motor_vmax_rad_s <= 0.0F) ||
            (joint->motor_tmax_nm <= 0.0F))
        {
            arm_config_record_error(validation, ARM_CONFIG_ERROR_MOTOR_RANGES, joint_index);
        }
    }

    if (((verified_fields & ARM_JOINT_VERIFIED_GEAR_RATIO) != 0U) &&
        (!arm_config_float_is_finite(joint->gear_ratio) ||
         (joint->gear_ratio <= 0.0F)))
    {
        arm_config_record_error(validation, ARM_CONFIG_ERROR_GEAR_RATIO, joint_index);
    }

    if ((verified_fields & ARM_JOINT_VERIFIED_COMPLETION_TOLERANCE) != 0U)
    {
        if (!arm_config_float_is_finite(joint->position_tolerance_rad) ||
            !arm_config_float_is_finite(joint->velocity_tolerance_rad_s) ||
            (joint->position_tolerance_rad <= 0.0F) ||
            (joint->velocity_tolerance_rad_s <= 0.0F))
        {
            arm_config_record_error(validation,
                                    ARM_CONFIG_ERROR_COMPLETION_TOLERANCE,
                                    joint_index);
        }
    }
}

/**
 * @brief Returns the immutable production configuration.
 * @return Pointer to process-lifetime static configuration storage.
 */
const ArmConfig *arm_config_get_production(void)
{
    return &production_configuration;
}

/**
 * @brief Validates structure and every physical field marked as verified.
 * @param configuration Configuration to inspect.
 * @param validation Output initialized on every call when non-null.
 * @return true when the schema and all confirmed values are valid.
 */
bool arm_config_validate_schema(const ArmConfig *configuration,
                                ArmConfigValidation *validation)
{
    uint8_t joint_index;
    uint8_t comparison_index;

    if (validation == NULL)
    {
        return false;
    }

    arm_config_initialize_validation(validation);
    if (configuration == NULL)
    {
        arm_config_record_error(validation,
                                ARM_CONFIG_ERROR_NULL,
                                ARM_CONFIG_FIRST_ERROR_NONE);
        return false;
    }

    if (configuration->joint_count != ARM_JOINT_COUNT)
    {
        arm_config_record_error(validation,
                                ARM_CONFIG_ERROR_JOINT_COUNT,
                                ARM_CONFIG_FIRST_ERROR_NONE);
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        const JointConfig *joint = &configuration->joints[joint_index];

        if (joint->joint_index != joint_index)
        {
            arm_config_record_error(validation,
                                    ARM_CONFIG_ERROR_JOINT_INDEX,
                                    joint_index);
        }

        if ((joint->esc_id == 0U) || (joint->esc_id > ARM_STANDARD_CAN_ID_MAX) ||
            (joint->master_id == 0U) || (joint->master_id > ARM_STANDARD_CAN_ID_MAX))
        {
            arm_config_record_error(validation, ARM_CONFIG_ERROR_CAN_ID, joint_index);
        }

        for (comparison_index = 0U; comparison_index < joint_index; ++comparison_index)
        {
            if (joint->esc_id == configuration->joints[comparison_index].esc_id)
            {
                arm_config_record_error(validation,
                                        ARM_CONFIG_ERROR_DUPLICATE_ESC_ID,
                                        joint_index);
            }
            if (joint->master_id == configuration->joints[comparison_index].master_id)
            {
                arm_config_record_error(validation,
                                        ARM_CONFIG_ERROR_DUPLICATE_MASTER_ID,
                                        joint_index);
            }
        }

        arm_config_validate_verified_values(joint, validation);
    }

    return validation->schema_errors == ARM_CONFIG_ERROR_NONE;
}

/**
 * @brief Checks whether every joint has all fields required for motor enable.
 * @param configuration Configuration to inspect.
 * @param validation Output containing schema errors and missing verified bits.
 * @return true only when schema validation passes and no required bit is absent.
 */
bool arm_config_is_enable_ready(const ArmConfig *configuration,
                                ArmConfigValidation *validation)
{
    uint8_t joint_index;

    if (!arm_config_validate_schema(configuration, validation))
    {
        return false;
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        validation->missing_verified_fields |=
            ARM_JOINT_REQUIRED_ENABLE_FIELDS &
            ~configuration->joints[joint_index].verified_fields;
    }

    return validation->missing_verified_fields == 0U;
}
