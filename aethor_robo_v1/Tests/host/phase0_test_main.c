/**
 * @file phase0_test_main.c
 * @brief Host-side tests for the PRD Phase 0 layered firmware baseline.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "aethor_app.h"
#include "app_profile.h"
#include "arm_config.h"
#include "arm_controller.h"
#include "board_config.h"
#include "build_info.h"
#include "diagnostics.h"
#include "motion_types.h"
#include "motor_types.h"
#include "platform_contract.h"
#include "protocol_contract.h"

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
        assert(configuration->joints[joint_index].master_id == (uint16_t)(joint_index + 0x11U));
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
    assert(BOARD_FORMAL_SERIAL_TRANSPORT == BOARD_SERIAL_TRANSPORT_USB_CDC);
    assert(BOARD_USB_CDC_VALIDATED == 0U);
}

/**
 * @brief Verifies the default image is the bounded USB bench profile.
 */
static void test_default_application_profile_is_safe_bench_control(void)
{
    assert(AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE);
    assert(AETHOR_BENCH_MAX_RELATIVE_DEGREES == 3.0F);
    assert(AETHOR_BENCH_MAX_SPEED_DEGREES_S == 3.0F);
    assert(AETHOR_PRODUCTION_REQUIRES_COMPLETE_CONFIGURATION == 1U);
}

/**
 * @brief Verifies diagnostics initialization uses explicit unsampled watermarks.
 */
static void test_diagnostics_initialize_deterministically(void)
{
    Diagnostics diagnostics;

    assert(DIAGNOSTICS_CAPACITY == 256U);
    DiagnosticCounters counters;

    diagnostics_init(&diagnostics);

    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.service_cycles == 0U);
    assert(counters.config_validation_failures == 0U);
    assert(counters.can_rx_frames == 0U);
    assert(counters.can_tx_frames == 0U);
    assert(counters.uart_rx_bytes == 0U);
    assert(counters.uart_tx_bytes == 0U);
    assert(counters.minimum_stack_words == DIAGNOSTIC_WATERMARK_NOT_SAMPLED);
    assert(counters.minimum_heap_bytes == DIAGNOSTIC_WATERMARK_NOT_SAMPLED);
}

/**
 * @brief Verifies a full diagnostic ring retains newest events without allocation.
 */
static void test_diagnostics_ring_overwrites_oldest_event(void)
{
    Diagnostics diagnostics;
    DiagnosticEvent event;
    uint32_t event_index;

    diagnostics_init(&diagnostics);
    for (event_index = 0U; event_index < (DIAGNOSTICS_CAPACITY + 2U); ++event_index)
    {
        assert(diagnostics_push(&diagnostics,
                                (uint64_t)event_index,
                                DIAGNOSTIC_CODE_BOOT,
                                DIAGNOSTIC_SEVERITY_INFO,
                                event_index));
    }

    assert(diagnostics.count == DIAGNOSTICS_CAPACITY);
    assert(diagnostics.dropped_count == 2U);
    assert(diagnostics_get(&diagnostics, 0U, &event));
    assert(event.sequence == 3U);
    assert(event.detail == 2U);
    assert(diagnostics_get(&diagnostics, DIAGNOSTICS_CAPACITY - 1U, &event));
    assert(event.sequence == DIAGNOSTICS_CAPACITY + 2U);
    assert(!diagnostics_get(&diagnostics, DIAGNOSTICS_CAPACITY, &event));
}

/**
 * @brief Verifies the production controller safely latches incomplete config.
 */
static void test_arm_controller_latches_incomplete_config_fault(void)
{
    Diagnostics diagnostics;
    DiagnosticCounters counters;
    ArmController controller;
    ArmSnapshot snapshot;

    diagnostics_init(&diagnostics);
    arm_controller_init(&controller,
                        arm_config_get_production(),
                        &diagnostics,
                        1000ULL);

    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_BOOT);
    assert(snapshot.fault == ARM_FAULT_NONE);

    arm_controller_step(&controller, 2000ULL);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_SELF_TEST);

    arm_controller_step(&controller, 3000ULL);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INCOMPLETE);
    assert(snapshot.joint_count == ARM_JOINT_COUNT);
    assert(snapshot.fault_detail == ARM_JOINT_REQUIRED_ENABLE_FIELDS);

    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.service_cycles == 2U);
    assert(counters.config_validation_failures == 1U);

    arm_controller_step(&controller, 4000ULL);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INCOMPLETE);
    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.service_cycles == 3U);
    assert(counters.config_validation_failures == 1U);
}

/**
 * @brief Verifies malformed configuration produces the distinct invalid fault.
 */
static void test_arm_controller_latches_invalid_config_fault(void)
{
    ArmConfig invalid_configuration = *arm_config_get_production();
    Diagnostics diagnostics;
    ArmController controller;
    ArmSnapshot snapshot;

    invalid_configuration.joints[1].esc_id = invalid_configuration.joints[0].esc_id;
    diagnostics_init(&diagnostics);
    arm_controller_init(&controller, &invalid_configuration, &diagnostics, 1000ULL);
    arm_controller_step(&controller, 2000ULL);
    arm_controller_step(&controller, 3000ULL);

    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INVALID);
    assert((snapshot.fault_detail & ARM_CONFIG_ERROR_DUPLICATE_ESC_ID) != 0U);
}

/**
 * @brief Verifies public diagnostics and snapshot queries reject null outputs.
 */
static void test_phase0_state_queries_reject_null_outputs(void)
{
    Diagnostics diagnostics;
    ArmController controller;

    diagnostics_init(&diagnostics);
    arm_controller_init(&controller,
                        arm_config_get_production(),
                        &diagnostics,
                        0ULL);

    assert(!diagnostics_get(NULL, 0U, NULL));
    assert(!diagnostics_get_counters(&diagnostics, NULL));
    assert(!arm_controller_get_snapshot(&controller, NULL));
    assert(!arm_controller_get_snapshot(NULL, NULL));
}

/**
 * @brief Verifies fixed capacities and disabled Phase 0 platform capabilities.
 */
static void test_layer_contracts_are_frozen(void)
{
    MotionSnapshot motion_snapshot = {0};
    MotorFeedbackSnapshot motor_snapshot = {0};

    assert(PROTOCOL_MAX_LINE_LENGTH == 512U);
    assert(PROTOCOL_RECENT_RESULT_CAPACITY == 32U);
    assert(PROTOCOL_BUSINESS_COMMAND_CAPACITY == 8U);
    assert(PROTOCOL_PHASE0_STATUS == PROTOCOL_STATUS_NOT_AVAILABLE);
    assert(PLATFORM_CONTROL_PERIOD_US == 4000UL);
    assert(PLATFORM_FORMAL_LINK_VALIDATED == 0U);
    assert(motion_snapshot.valid_joint_mask == 0U);
    assert(motor_snapshot.valid_joint_mask == 0U);
}

/**
 * @brief Verifies the application facade reaches only the safe Phase 0 fault.
 */
static void test_aethor_app_latches_safe_phase0_fault(void)
{
    ArmSnapshot snapshot;
    CanFrame discovery_frame;
    CanTxPriority discovery_priority;
    DiagnosticEvent event;
    DiagnosticCounters counters;

    assert(!aethor_app_get_snapshot(&snapshot));
    assert(!aethor_app_get_snapshot(NULL));
    assert(!aethor_app_get_diagnostic(0U, &event));
    assert(!aethor_app_get_diagnostic_counters(&counters));
    assert(aethor_app_next_can_frame(500ULL,
                                     &discovery_frame,
                                     &discovery_priority) ==
           MOTOR_RUNTIME_STATUS_NOT_INITIALIZED);

    aethor_app_init(1000ULL);
    assert(aethor_app_get_snapshot(&snapshot));
    assert(snapshot.state == ARM_STATE_BOOT);
    assert(aethor_app_next_can_frame(1000ULL,
                                     &discovery_frame,
                                     &discovery_priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(discovery_priority == CAN_TX_PRIORITY_PARAMETER);
    assert(discovery_frame.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
    assert(discovery_frame.data[0] == 0x01U);
    assert(discovery_frame.data[3] == S3519_REGISTER_MASTER_ID);

    aethor_app_service(2000ULL);
    aethor_app_service(3000ULL);
    assert(aethor_app_get_snapshot(&snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INCOMPLETE);

    assert(aethor_app_get_diagnostic(0U, &event));
    assert(event.code == DIAGNOSTIC_CODE_BOOT);
    assert(aethor_app_get_diagnostic_counters(&counters));
    assert(counters.service_cycles == 2U);
    assert(counters.can_tx_frames == 0U);
    assert(counters.uart_tx_bytes == 0U);
}

/**
 * @brief Verifies reinitialization resets all static application state.
 */
static void test_aethor_app_reinitializes_deterministically(void)
{
    ArmSnapshot snapshot;
    DiagnosticCounters counters;

    aethor_app_service(4000ULL);
    aethor_app_init(5000ULL);

    assert(aethor_app_get_snapshot(&snapshot));
    assert(snapshot.state == ARM_STATE_BOOT);
    assert(snapshot.fault == ARM_FAULT_NONE);
    assert(aethor_app_get_diagnostic_counters(&counters));
    assert(counters.service_cycles == 0U);
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
    test_default_application_profile_is_safe_bench_control();
    test_diagnostics_initialize_deterministically();
    test_diagnostics_ring_overwrites_oldest_event();
    test_arm_controller_latches_incomplete_config_fault();
    test_arm_controller_latches_invalid_config_fault();
    test_phase0_state_queries_reject_null_outputs();
    test_layer_contracts_are_frozen();
    test_aethor_app_latches_safe_phase0_fault();
    test_aethor_app_reinitializes_deterministically();

    printf("PHASE0_TESTS_PASSED\n");
    return 0;
}
