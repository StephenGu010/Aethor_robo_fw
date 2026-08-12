/**
 * @file arm_config.h
 * @brief Defines the static seven-axis configuration and validation contract.
 */

#ifndef APP_CONFIG_ARM_CONFIG_H
#define APP_CONFIG_ARM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_JOINT_COUNT (7U)
#define ARM_CONFIG_FIRST_ERROR_NONE (0xFFU)

/**
 * @brief Identifies physical joint parameters confirmed by measurement.
 */
typedef enum
{
    ARM_JOINT_VERIFIED_DIRECTION = (1UL << 0),
    ARM_JOINT_VERIFIED_LIMITS = (1UL << 1),
    ARM_JOINT_VERIFIED_MAX_VELOCITY = (1UL << 2),
    ARM_JOINT_VERIFIED_MAX_ACCELERATION = (1UL << 3),
    ARM_JOINT_VERIFIED_MIT_GAINS = (1UL << 4),
    ARM_JOINT_VERIFIED_MOTOR_RANGES = (1UL << 5),
    ARM_JOINT_VERIFIED_GEAR_RATIO = (1UL << 6)
} ArmJointVerifiedField;

#define ARM_JOINT_REQUIRED_ENABLE_FIELDS                                     \
    ((uint32_t)(ARM_JOINT_VERIFIED_DIRECTION | ARM_JOINT_VERIFIED_LIMITS |   \
                ARM_JOINT_VERIFIED_MAX_VELOCITY |                            \
                ARM_JOINT_VERIFIED_MAX_ACCELERATION |                        \
                ARM_JOINT_VERIFIED_MIT_GAINS |                               \
                ARM_JOINT_VERIFIED_MOTOR_RANGES |                            \
                ARM_JOINT_VERIFIED_GEAR_RATIO))

/**
 * @brief Reports structural and confirmed-value configuration failures.
 */
typedef enum
{
    ARM_CONFIG_ERROR_NONE = 0U,
    ARM_CONFIG_ERROR_NULL = (1UL << 0),
    ARM_CONFIG_ERROR_JOINT_COUNT = (1UL << 1),
    ARM_CONFIG_ERROR_JOINT_INDEX = (1UL << 2),
    ARM_CONFIG_ERROR_CAN_ID = (1UL << 3),
    ARM_CONFIG_ERROR_DUPLICATE_ESC_ID = (1UL << 4),
    ARM_CONFIG_ERROR_DUPLICATE_MASTER_ID = (1UL << 5),
    ARM_CONFIG_ERROR_DIRECTION = (1UL << 6),
    ARM_CONFIG_ERROR_LIMITS = (1UL << 7),
    ARM_CONFIG_ERROR_VELOCITY = (1UL << 8),
    ARM_CONFIG_ERROR_ACCELERATION = (1UL << 9),
    ARM_CONFIG_ERROR_MIT_GAINS = (1UL << 10),
    ARM_CONFIG_ERROR_MOTOR_RANGES = (1UL << 11),
    ARM_CONFIG_ERROR_GEAR_RATIO = (1UL << 12),
    ARM_CONFIG_ERROR_VERIFIED_FIELDS = (1UL << 13)
} ArmConfigError;

/**
 * @brief Stores static parameters for one robot joint.
 *
 * Unverified physical fields remain zero and their corresponding bit in
 * verified_fields remains clear. Zero is not interpreted as measured data.
 */
typedef struct
{
    uint8_t joint_index;
    uint16_t esc_id;
    uint16_t master_id;
    int8_t direction;
    float soft_limit_min_rad;
    float soft_limit_max_rad;
    float max_velocity_rad_s;
    float max_acceleration_rad_s2;
    float mit_kp;
    float mit_kd;
    float motor_pmax_rad;
    float motor_vmax_rad_s;
    float motor_tmax_nm;
    float gear_ratio;
    uint32_t verified_fields;
} JointConfig;

/**
 * @brief Stores the complete fixed-size seven-axis configuration.
 */
typedef struct
{
    uint8_t joint_count;
    JointConfig joints[ARM_JOINT_COUNT];
} ArmConfig;

/**
 * @brief Separates invalid configuration data from unverified enable fields.
 */
typedef struct
{
    uint32_t schema_errors;
    uint32_t missing_verified_fields;
    uint8_t first_error_joint;
} ArmConfigValidation;

/**
 * @brief Returns the immutable production configuration.
 * @return Pointer to process-lifetime static configuration storage.
 */
const ArmConfig *arm_config_get_production(void);

/**
 * @brief Validates structure and every physical field marked as verified.
 * @param configuration Configuration to inspect.
 * @param validation Output initialized on every call when non-null.
 * @return true when the schema and all confirmed values are valid.
 */
bool arm_config_validate_schema(const ArmConfig *configuration,
                                ArmConfigValidation *validation);

/**
 * @brief Checks whether every joint has all fields required for motor enable.
 * @param configuration Configuration to inspect.
 * @param validation Output containing schema errors and missing verified bits.
 * @return true only when schema validation passes and no required bit is absent.
 */
bool arm_config_is_enable_ready(const ArmConfig *configuration,
                                ArmConfigValidation *validation);

#endif
