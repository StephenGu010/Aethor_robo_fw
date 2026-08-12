/**
 * @file phase0_test_main.c
 * @brief Host-side tests for the PRD Phase 0 layered firmware baseline.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "arm_config.h"
#include "board_config.h"
#include "build_info.h"

/**
 * @brief Verifies the production configuration contains seven ordered joints.
 */
static void test_production_config_has_seven_ordered_joints(void)
{
    const ArmConfig *configuration = arm_config_get_production();
    uint8_t joint_index;

    assert(configuration != NULL);
    assert(configuration->joint_count == ARM_JOINT_COUNT);

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        assert(configuration->joints[joint_index].joint_index == joint_index);
        assert(configuration->joints[joint_index].esc_id == (uint16_t)(joint_index + 1U));
        assert(configuration->joints[joint_index].master_id == (uint16_t)(joint_index + 11U));
    }
}

/**
 * @brief Verifies unconfirmed physical parameters keep motor enable locked.
 */
static void test_production_config_is_not_enable_ready(void)
{
    ArmConfigValidation validation;
    const ArmConfig *configuration = arm_config_get_production();

    assert(arm_config_validate_schema(configuration, &validation));
    assert(validation.schema_errors == ARM_CONFIG_ERROR_NONE);
    assert(!arm_config_is_enable_ready(configuration, &validation));
    assert(validation.missing_verified_fields == ARM_JOINT_REQUIRED_ENABLE_FIELDS);
}

/**
 * @brief Verifies duplicate ESC identifiers are rejected.
 */
static void test_duplicate_can_id_is_rejected(void)
{
    ArmConfig configuration = *arm_config_get_production();
    ArmConfigValidation validation;

    configuration.joints[1].esc_id = configuration.joints[0].esc_id;

    assert(!arm_config_validate_schema(&configuration, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_DUPLICATE_ESC_ID) != 0U);
    assert(validation.first_error_joint == 1U);
}

/**
 * @brief Verifies a configuration with fewer than seven joints is rejected.
 */
static void test_missing_joint_is_rejected(void)
{
    ArmConfig configuration = *arm_config_get_production();
    ArmConfigValidation validation;

    configuration.joint_count = ARM_JOINT_COUNT - 1U;

    assert(!arm_config_validate_schema(&configuration, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_JOINT_COUNT) != 0U);
}

/**
 * @brief Verifies invalid direction is checked once marked as confirmed.
 */
static void test_verified_invalid_direction_is_rejected(void)
{
    ArmConfig configuration = *arm_config_get_production();
    ArmConfigValidation validation;

    configuration.joints[0].direction = 0;
    configuration.joints[0].verified_fields |= ARM_JOINT_VERIFIED_DIRECTION;

    assert(!arm_config_validate_schema(&configuration, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_DIRECTION) != 0U);
}

/**
 * @brief Verifies invalid confirmed software limits are rejected.
 */
static void test_verified_invalid_limits_are_rejected(void)
{
    ArmConfig configuration = *arm_config_get_production();
    ArmConfigValidation validation;

    configuration.joints[0].soft_limit_min_rad = 1.0F;
    configuration.joints[0].soft_limit_max_rad = -1.0F;
    configuration.joints[0].verified_fields |= ARM_JOINT_VERIFIED_LIMITS;

    assert(!arm_config_validate_schema(&configuration, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_LIMITS) != 0U);
}

/**
 * @brief Verifies null inputs fail deterministically without dereferencing them.
 */
static void test_null_configuration_is_rejected(void)
{
    ArmConfigValidation validation;

    assert(!arm_config_validate_schema(NULL, &validation));
    assert((validation.schema_errors & ARM_CONFIG_ERROR_NULL) != 0U);
    assert(!arm_config_validate_schema(arm_config_get_production(), NULL));
}

/**
 * @brief Verifies build identity and board contract values required by the PRD.
 */
static void test_build_and_board_identity_are_frozen(void)
{
    const BuildInfo *build_information = build_info_get();

    assert(build_information != NULL);
    assert(build_information->controller_id[0] != '\0');
    assert(build_information->arm_id[0] != '\0');
    assert(build_information->protocol_version[0] != '\0');
    assert(BOARD_FDCAN_NOMINAL_BITRATE == 1000000UL);
    assert(BOARD_FORMAL_UART_BAUDRATE == 921600UL);
    assert(BOARD_FORMAL_UART_VALIDATED == 0U);
}

/**
 * @brief Runs the Phase 0 configuration and identity test suite.
 * @return Zero when every assertion passes.
 */
int main(void)
{
    test_production_config_has_seven_ordered_joints();
    test_production_config_is_not_enable_ready();
    test_duplicate_can_id_is_rejected();
    test_missing_joint_is_rejected();
    test_verified_invalid_direction_is_rejected();
    test_verified_invalid_limits_are_rejected();
    test_null_configuration_is_rejected();
    test_build_and_board_identity_are_frozen();

    printf("PHASE0_TESTS_PASSED\n");
    return 0;
}
