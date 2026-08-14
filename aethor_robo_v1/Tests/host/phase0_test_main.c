/**
 * @file phase0_test_main.c
 * @brief Host-side tests for the PRD Phase 0 layered firmware baseline.
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "aethor_app.h"
#include "ascii_protocol.h"
#include "app_profile.h"
#include "arm_config.h"
#include "arm_controller.h"
#include "board_config.h"
#include "build_info.h"
#include "can_frame.h"
#include "diagnostics.h"
#include "joint_reference.h"
#include "motion_types.h"
#include "motor_types.h"
#include "platform_contract.h"
#include "protocol_contract.h"
#include "s3519_codec.h"

#define PHASE0_TEST_ACTION_TIMEOUT_US (500000ULL)

/**
 * @brief Builds deterministic commissioned joint parameters for domain tests.
 * @return Valid seven-axis configuration independent of real hardware values.
 */
static ArmConfig make_test_commissioned_configuration(void)
{
    ArmConfig configuration = *arm_config_get_production();
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        JointConfig *joint = &configuration.joints[joint_index];

        joint->direction = (joint_index == 1U) ? -1 : 1;
        joint->soft_limit_min_rad = -3.0F;
        joint->soft_limit_max_rad = 3.0F;
        joint->max_velocity_rad_s = 1.0F;
        joint->max_acceleration_rad_s2 = 2.0F;
        joint->mit_kp = 10.0F;
        joint->mit_kd = 1.0F;
        joint->motor_pmax_rad = 12.5F;
        joint->motor_vmax_rad_s = 45.0F;
        joint->motor_tmax_nm = 18.0F;
        joint->gear_ratio = 2.0F;
        joint->position_tolerance_rad = 0.01F;
        joint->velocity_tolerance_rad_s = 0.02F;
        joint->verified_fields = ARM_JOINT_REQUIRED_ENABLE_FIELDS;
    }
    return configuration;
}

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

/** @brief Verifies the default image keeps bench parsing separate from production. */
static void test_default_application_profile_is_safe_bench_control(void)
{
    assert(AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE);
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
    assert(counters.control_period_min_us == DIAGNOSTIC_WATERMARK_NOT_SAMPLED);
    assert(counters.control_group_skew_max_us ==
           DIAGNOSTIC_WATERMARK_NOT_SAMPLED);
}

/**
 * @brief Verifies runtime transport, timing, and resource samples remain monotonic.
 */
static void test_diagnostics_runtime_aggregation(void)
{
    Diagnostics diagnostics;
    RuntimeDiagnosticSample sample = {0};
    DiagnosticCounters counters;

    diagnostics_init(&diagnostics);
    diagnostics_record_control_period(&diagnostics, 4000U);
    diagnostics_record_control_period(&diagnostics, 4300U);
    sample.can_rx_frames = 10U;
    sample.can_tx_frames = 20U;
    sample.can_rx_overflow_count = 2U;
    sample.can_tx_error_count = 3U;
    sample.can_bus_off_count = 1U;
    sample.can_tx_queue_high_watermark = 7U;
    sample.control_group_reject_count = 4U;
    sample.usb_rx_bytes = 100U;
    sample.usb_rx_overflow_count = 5U;
    sample.usb_overlong_line_count = 6U;
    sample.usb_high_queue_high_watermark = 8U;
    sample.usb_query_queue_high_watermark = 9U;
    sample.usb_telemetry_queue_high_watermark = 3U;
    sample.usb_telemetry_drop_count = 11U;
    sample.usb_transmit_busy_count = 12U;
    sample.usb_transmit_error_count = 13U;
    sample.minimum_stack_words = 256U;
    sample.minimum_heap_bytes = 4096U;
    diagnostics_update_runtime_sample(&diagnostics, &sample);
    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.control_period_last_us == 4300U);
    assert(counters.control_period_min_us == 4000U);
    assert(counters.control_period_max_us == 4300U);
    assert(counters.control_deadline_miss_count == 1U);
    assert(counters.control_consecutive_miss_count == 1U);
    assert(counters.can_rx_frames == 10U);
    assert(counters.can_tx_frames == 20U);
    assert(counters.can_bus_off_count == 1U);
    assert(counters.usb_rx_bytes == 100U);
    assert(counters.usb_telemetry_drop_count == 11U);
    assert(counters.minimum_stack_words == 256U);
    assert(counters.minimum_heap_bytes == 4096U);

    diagnostics_record_control_period(&diagnostics, 3999U);
    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.control_consecutive_miss_count == 0U);
    assert(counters.control_period_min_us == 3999U);
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
static void test_arm_controller_bench_profile_keeps_enable_locked_without_fault(void)
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
    assert(snapshot.state == ARM_STATE_UNALIGNED);
    assert(snapshot.fault == ARM_FAULT_NONE);
    assert(snapshot.joint_count == ARM_JOINT_COUNT);
    assert(snapshot.fault_detail == ARM_JOINT_REQUIRED_ENABLE_FIELDS);

    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.service_cycles == 2U);
    assert(counters.config_validation_failures == 0U);

    arm_controller_step(&controller, 4000ULL);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_UNALIGNED);
    assert(snapshot.fault == ARM_FAULT_NONE);
    assert(diagnostics_get_counters(&diagnostics, &counters));
    assert(counters.service_cycles == 3U);
    assert(counters.config_validation_failures == 0U);
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
 * @brief Verifies commissioned software boots into the mandatory unaligned gate.
 */
static void test_arm_controller_enters_unaligned_after_self_test(void)
{
    ArmConfig configuration = make_test_commissioned_configuration();
    Diagnostics diagnostics;
    ArmController controller;
    ArmSnapshot snapshot;

    diagnostics_init(&diagnostics);
    arm_controller_init(&controller, &configuration, &diagnostics, 1000U);
    arm_controller_step(&controller, 2000U);
    arm_controller_step(&controller, 3000U);

    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_UNALIGNED);
    assert(snapshot.fault == ARM_FAULT_NONE);
    assert(snapshot.aligned == 0U);

    assert(arm_controller_mark_reference_aligned(&controller, 4000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_DISABLED);
    assert(snapshot.aligned == 1U);
    assert(snapshot.enabled == 0U);
    assert(snapshot.moving == 0U);
    assert(arm_controller_mark_reference_aligned(&controller, 5000U) ==
           ARM_TRANSITION_STATUS_OK);
}

/**
 * @brief Verifies READY, MOVING, STOPPING, and READY motion transitions.
 */
static void test_arm_controller_motion_transitions(void)
{
    ArmConfig configuration = make_test_commissioned_configuration();
    Diagnostics diagnostics;
    ArmController controller;
    ArmSnapshot snapshot;

    diagnostics_init(&diagnostics);
    arm_controller_init(&controller, &configuration, &diagnostics, 1000U);
    arm_controller_step(&controller, 2000U);
    arm_controller_step(&controller, 3000U);
    assert(arm_controller_mark_reference_aligned(&controller, 4000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_confirm_control_mode(
               &controller,
               ARM_CONTROL_MODE_POSITION_VELOCITY,
               5000U) == ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_begin_enable(&controller, 6000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_confirm_enabled(&controller, 7000U) ==
           ARM_TRANSITION_STATUS_OK);

    assert(arm_controller_begin_motion(&controller, 8000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_MOVING);
    assert(snapshot.enabled == 1U);
    assert(snapshot.moving == 1U);
    assert(arm_controller_begin_motion(&controller, 9000U) ==
           ARM_TRANSITION_STATUS_INVALID_STATE);

    assert(arm_controller_begin_controlled_stop(&controller, 10000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_STOPPING);
    assert(snapshot.enabled == 1U);
    assert(snapshot.moving == 1U);

    assert(arm_controller_complete_motion(&controller, 11000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_READY);
    assert(snapshot.enabled == 1U);
    assert(snapshot.moving == 0U);

    assert(arm_controller_latch_runtime_fault(&controller,
                                              ARM_FAULT_MOTION_CONTROL,
                                              77U,
                                              12000U) ==
           ARM_TRANSITION_STATUS_OK);
    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_MOTION_CONTROL);
    assert(snapshot.fault_detail == 77U);
    assert(snapshot.enabled == 0U);
    assert(snapshot.moving == 0U);
}

/**
 * @brief Verifies alignment converts motor radians into coherent joint degrees.
 */
static void test_joint_reference_alignment_and_reboot_invalidation(void)
{
    ArmConfig configuration = make_test_commissioned_configuration();
    JointReference reference;
    JointStateSnapshot joint_snapshot;
    MotorFeedbackSnapshot motor_snapshot = {0};
    float reference_degrees[ARM_JOINT_COUNT] = {10.0F, -20.0F, 0.0F, 0.0F,
                                                0.0F, 0.0F, 0.0F};
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        motor_snapshot.joints[joint_index].position_rad = 1.0F;
        motor_snapshot.joints[joint_index].velocity_rad_s = 0.2F;
        motor_snapshot.joints[joint_index].torque_nm = 0.3F;
    }
    motor_snapshot.valid_joint_mask = 0x7FU;
    motor_snapshot.generation = 5U;

    assert(joint_reference_init(&reference, &configuration) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_publish(&reference, &motor_snapshot, 2000U) ==
           JOINT_REFERENCE_STATUS_NOT_ALIGNED);
    assert(joint_reference_align(&reference,
                                 &motor_snapshot,
                                 reference_degrees,
                                 3000U) == JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_get_snapshot(&reference, &joint_snapshot) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_snapshot.aligned == 1U);
    assert(joint_snapshot.valid_joint_mask == 0x7FU);
    assert(joint_snapshot.position_deg[0] > 9.999F);
    assert(joint_snapshot.position_deg[0] < 10.001F);
    assert(joint_snapshot.position_deg[1] > -20.001F);
    assert(joint_snapshot.position_deg[1] < -19.999F);
    assert(joint_snapshot.velocity_deg_s[0] > 5.729F);
    assert(joint_snapshot.velocity_deg_s[0] < 5.731F);
    assert(joint_snapshot.velocity_deg_s[1] < -5.729F);

    assert(joint_reference_init(&reference, &configuration) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_get_snapshot(&reference, &joint_snapshot) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_snapshot.aligned == 0U);
    assert(joint_snapshot.valid_joint_mask == 0U);
}

/**
 * @brief Verifies aligned joint commands round-trip through motor coordinates.
 */
static void test_joint_reference_inverse_command_mapping(void)
{
    ArmConfig configuration = make_test_commissioned_configuration();
    JointReference reference;
    JointStateSnapshot joint_snapshot;
    MotorFeedbackSnapshot motor_snapshot = {0};
    float alignment_degrees[ARM_JOINT_COUNT] = {10.0F, -20.0F, 0.0F, 0.0F,
                                                0.0F, 0.0F, 0.0F};
    float target_position_rad[ARM_JOINT_COUNT] = {0.5F, -0.4F, 0.1F, 0.2F,
                                                  0.3F, -0.2F, -0.1F};
    float target_velocity_rad_s[ARM_JOINT_COUNT] = {0.2F, -0.1F, 0.0F, 0.1F,
                                                    -0.1F, 0.2F, -0.2F};
    float motor_position_rad[ARM_JOINT_COUNT];
    float motor_velocity_rad_s[ARM_JOINT_COUNT];
    uint8_t joint_index;

    motor_snapshot.valid_joint_mask = 0x7FU;
    motor_snapshot.generation = 1U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        motor_snapshot.joints[joint_index].position_rad = 1.0F;
    }
    assert(joint_reference_init(&reference, &configuration) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_joint_to_motor(&reference,
                                          target_position_rad,
                                          target_velocity_rad_s,
                                          motor_position_rad,
                                          motor_velocity_rad_s) ==
           JOINT_REFERENCE_STATUS_NOT_ALIGNED);
    assert(joint_reference_align(&reference,
                                 &motor_snapshot,
                                 alignment_degrees,
                                 1000U) == JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_joint_to_motor(&reference,
                                          target_position_rad,
                                          target_velocity_rad_s,
                                          motor_position_rad,
                                          motor_velocity_rad_s) ==
           JOINT_REFERENCE_STATUS_OK);

    motor_snapshot.generation = 2U;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        motor_snapshot.joints[joint_index].position_rad =
            motor_position_rad[joint_index];
        motor_snapshot.joints[joint_index].velocity_rad_s =
            motor_velocity_rad_s[joint_index];
    }
    assert(joint_reference_publish(&reference, &motor_snapshot, 2000U) ==
           JOINT_REFERENCE_STATUS_OK);
    assert(joint_reference_get_snapshot(&reference, &joint_snapshot) ==
           JOINT_REFERENCE_STATUS_OK);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        float expected_position_deg = target_position_rad[joint_index] *
                                      57.29577951308232F;
        float expected_velocity_deg_s = target_velocity_rad_s[joint_index] *
                                        57.29577951308232F;

        assert(joint_snapshot.position_deg[joint_index] >
               expected_position_deg - 0.001F);
        assert(joint_snapshot.position_deg[joint_index] <
               expected_position_deg + 0.001F);
        assert(joint_snapshot.velocity_deg_s[joint_index] >
               expected_velocity_deg_s - 0.001F);
        assert(joint_snapshot.velocity_deg_s[joint_index] <
               expected_velocity_deg_s + 0.001F);
    }
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
 * @brief Reinterprets one float as a raw little-endian register value.
 * @param value Floating-point register value.
 * @return Bit-identical unsigned register payload.
 */
static uint32_t phase0_float_to_raw_register(float value)
{
    uint32_t raw_value;

    memcpy(&raw_value, &value, sizeof(raw_value));
    return raw_value;
}

/**
 * @brief Encodes one position into the unsigned 16-bit S3519 feedback field.
 * @param position_rad Position within the discovered symmetric range.
 * @param position_max_rad Positive discovered PMAX value.
 * @return Quantized unsigned position payload.
 */
static uint16_t phase0_encode_feedback_position(float position_rad,
                                                float position_max_rad)
{
    double normalized_position;
    double encoded_position;

    assert(position_max_rad > 0.0F);
    assert(position_rad >= -position_max_rad);
    assert(position_rad <= position_max_rad);
    normalized_position =
        ((double)position_rad + (double)position_max_rad) /
        (2.0 * (double)position_max_rad);
    encoded_position = normalized_position * 65535.0;
    return (uint16_t)(encoded_position + 0.5);
}

/**
 * @brief Supplies a valid discovery value for the requested selected motor.
 * @param register_address Vendor register requested by the application.
 * @param esc_id One-based selected motor identifier.
 * @return Raw response value matching the production mapping.
 */
static uint32_t phase0_discovery_raw_value(uint8_t register_address,
                                           uint8_t esc_id)
{
    switch ((S3519Register)register_address)
    {
        case S3519_REGISTER_ACCELERATION:
            return phase0_float_to_raw_register(30.0F);
        case S3519_REGISTER_DECELERATION:
            return phase0_float_to_raw_register(-25.0F);
        case S3519_REGISTER_MAXIMUM_SPEED:
            return phase0_float_to_raw_register(20.0F);
        case S3519_REGISTER_MASTER_ID:
            return (uint32_t)(esc_id + 0x10U);
        case S3519_REGISTER_ESC_ID:
            return esc_id;
        case S3519_REGISTER_CONTROL_MODE:
            return 2U;
        case S3519_REGISTER_HARDWARE_VERSION:
            return 0x00010002U;
        case S3519_REGISTER_SOFTWARE_VERSION:
            return 0x00030004U;
        case S3519_REGISTER_SUB_VERSION:
            return 0x00000005U;
        case S3519_REGISTER_POSITION_RANGE:
            return phase0_float_to_raw_register(12.5F);
        case S3519_REGISTER_VELOCITY_RANGE:
            return phase0_float_to_raw_register(45.0F);
        case S3519_REGISTER_TORQUE_RANGE:
            return phase0_float_to_raw_register(18.0F);
        default:
            assert(0);
            return 0U;
    }
}

/**
 * @brief Submits one plain aethor-text-v1 application request.
 * @param line Request line without its optional transport terminator.
 * @param timestamp_us Monotonic request time.
 */
static void phase0_submit_request(const char *line, uint64_t timestamp_us)
{
    ProtocolOutputBatch output_batch;

    assert(line != NULL);
    assert(aethor_app_process_protocol_line(line,
                                            strlen(line),
                                            timestamp_us,
                                            &output_batch) ==
           PROTOCOL_ENGINE_STATUS_OK);
}

/**
 * @brief Feeds one centered S3519 feedback frame with an explicit driver state.
 * @param esc_id One-based motor identifier.
 * @param driver_state S3519 enabled or disabled state nibble.
 * @param timestamp_us Monotonic receive time.
 */
static void phase0_feed_feedback_position(uint8_t esc_id,
                                          uint8_t driver_state,
                                          float position_rad,
                                          uint64_t timestamp_us)
{
    uint16_t encoded_position =
        phase0_encode_feedback_position(position_rad, 12.5F);
    uint8_t payload[8] = {
        (uint8_t)((driver_state << 4U) | esc_id),
        (uint8_t)(encoded_position >> 8U),
        (uint8_t)(encoded_position & 0xFFU),
        0x80U, 0x08U, 0x00U, 35U, 27U
    };
    CanFrame frame;

    assert(can_frame_init(&frame,
                          (uint16_t)(esc_id + 0x10U),
                          payload,
                          sizeof(payload)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&frame, timestamp_us) ==
           MOTOR_RUNTIME_STATUS_OK);
}

/**
 * @brief Feeds one centered S3519 feedback frame with an explicit driver state.
 * @param esc_id One-based motor identifier.
 * @param driver_state S3519 enabled or disabled state nibble.
 * @param timestamp_us Monotonic receive time.
 */
static void phase0_feed_feedback(uint8_t esc_id,
                                 uint8_t driver_state,
                                 uint64_t timestamp_us)
{
    phase0_feed_feedback_position(esc_id,
                                  driver_state,
                                  0.0F,
                                  timestamp_us);
}

/**
 * @brief Reads one little-endian float from a POS_VEL command payload.
 * @param payload Four-byte little-endian payload field.
 * @return Bit-identical floating-point value.
 */
static float phase0_read_command_float(const uint8_t payload[4])
{
    uint32_t raw_value;
    float value;

    assert(payload != NULL);
    raw_value = (uint32_t)payload[0] |
                ((uint32_t)payload[1] << 8U) |
                ((uint32_t)payload[2] << 16U) |
                ((uint32_t)payload[3] << 24U);
    memcpy(&value, &raw_value, sizeof(value));
    return value;
}

/**
 * @brief Counts selected joints in one valid J1-J7 motor mask.
 * @param motor_mask Selected motor bit mask.
 * @return Number of selected motors.
 */
static uint8_t phase0_count_selected_motors(uint8_t motor_mask)
{
    uint8_t joint_index;
    uint8_t selected_count = 0U;

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((motor_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            ++selected_count;
        }
    }
    return selected_count;
}

/**
 * @brief Checks whether one frame is an exact selected vendor mode command.
 * @param frame Candidate CAN frame.
 * @param command Expected vendor command byte.
 * @return One for the expected special command, otherwise zero.
 */
static uint8_t phase0_is_mode_command(const CanFrame *frame,
                                      S3519ModeCommand command)
{
    uint8_t byte_index;

    if ((frame == NULL) || (frame->length != 8U) ||
        (frame->data[7] != (uint8_t)command))
    {
        return 0U;
    }
    for (byte_index = 0U; byte_index < 7U; ++byte_index)
    {
        if (frame->data[byte_index] != 0xFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief Starts a fresh aethor-text-v1 application session.
 * @param timestamp_us Mutable monotonic timestamp.
 * @param boot_id Nonzero deterministic boot identity.
 */
static void phase0_start_text_session(uint64_t *timestamp_us, uint32_t boot_id)
{
    assert(timestamp_us != NULL);
    aethor_app_init(*timestamp_us, boot_id);
    (void)aethor_app_service(++(*timestamp_us));
    (void)aethor_app_service(++(*timestamp_us));
    phase0_submit_request("1 hello", ++(*timestamp_us));
}

/**
 * @brief Responds to one discovery register read with a valid motor value.
 * @param request Read-only discovery request produced by the application.
 * @param timestamp_us Receive timestamp.
 */
static void phase0_respond_to_discovery_request(const CanFrame *request,
                                                uint64_t timestamp_us)
{
    CanFrame response;
    uint8_t payload[8] = {0U};
    uint8_t esc_id;
    uint32_t raw_value;

    assert(request != NULL);
    assert(request->identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
    assert(request->data[2] == 0x33U);
    esc_id = request->data[0];
    raw_value = phase0_discovery_raw_value(request->data[3], esc_id);
    payload[0] = esc_id;
    payload[2] = 0x33U;
    payload[3] = request->data[3];
    payload[4] = (uint8_t)(raw_value & 0xFFU);
    payload[5] = (uint8_t)((raw_value >> 8U) & 0xFFU);
    payload[6] = (uint8_t)((raw_value >> 16U) & 0xFFU);
    payload[7] = (uint8_t)((raw_value >> 24U) & 0xFFU);
    assert(can_frame_init(&response,
                          (uint16_t)(esc_id + 0x10U),
                          payload,
                          sizeof(payload)) == CAN_FRAME_STATUS_OK);
    assert(aethor_app_receive_can_frame(&response, timestamp_us) ==
           MOTOR_RUNTIME_STATUS_OK);
}

/**
 * @brief Responds to one POS_VEL control-mode readback request.
 * @param request Parameter read produced after the volatile mode write.
 * @param timestamp_us Receive timestamp.
 */
static void phase0_respond_to_mode_readback(const CanFrame *request,
                                            uint64_t timestamp_us)
{
    uint8_t payload[8];
    CanFrame response;
    MotorRuntimeStatus response_status;

    assert(request != NULL);
    assert(request->identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
    assert(request->data[2] == 0x33U);
    assert(request->data[3] == S3519_REGISTER_CONTROL_MODE);
    memset(payload, 0, sizeof(payload));
    payload[0] = request->data[0];
    payload[2] = 0x33U;
    payload[3] = S3519_REGISTER_CONTROL_MODE;
    payload[4] = 2U;
    assert(can_frame_init(&response,
                          (uint16_t)(request->data[0] + 0x10U),
                          payload,
                          sizeof(payload)) == CAN_FRAME_STATUS_OK);
    response_status = aethor_app_receive_can_frame(&response, timestamp_us);
    assert((response_status == MOTOR_RUNTIME_STATUS_OK) ||
           (response_status == MOTOR_RUNTIME_STATUS_ACTION_COMPLETE));
}

/**
 * @brief Completes a discovery pass while asserting every request stays selected.
 * @param motor_mask Expected discovery subset.
 * @param service_after_last_response One to service the action after the final reply.
 * @param timestamp_us Mutable monotonic timestamp.
 */
static void phase0_complete_discovery_subset(uint8_t motor_mask,
                                             uint8_t service_after_last_response,
                                             uint64_t *timestamp_us)
{
    uint16_t request_index;
    uint16_t request_count;

    assert(timestamp_us != NULL);
    request_count = (uint16_t)(phase0_count_selected_motors(motor_mask) *
                               MOTOR_DISCOVERY_REGISTER_COUNT);
    for (request_index = 0U; request_index < request_count; ++request_index)
    {
        CanFrame request;
        CanTxPriority priority;
        uint8_t joint_bit;

        assert(aethor_app_next_can_frame(++(*timestamp_us),
                                         &request,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER);
        assert(request.data[0] >= 1U);
        assert(request.data[0] <= ARM_JOINT_COUNT);
        joint_bit = (uint8_t)(1U << (request.data[0] - 1U));
        assert((motor_mask & joint_bit) != 0U);
        assert(!phase0_is_mode_command(&request, S3519_MODE_COMMAND_ENABLE));
        phase0_respond_to_discovery_request(&request, *timestamp_us);
        if ((request_index + 1U < request_count) ||
            (service_after_last_response != 0U))
        {
            (void)aethor_app_service(*timestamp_us);
        }
    }
}

/**
 * @brief Completes the selected volatile POS_VEL write/readback sequence.
 * @param motor_mask Expected mode-switch subset.
 * @param timestamp_us Mutable monotonic timestamp.
 */
static void phase0_complete_mode_switch_subset(uint8_t motor_mask,
                                               uint64_t *timestamp_us)
{
    uint8_t frame_index;
    uint8_t frame_count = (uint8_t)(2U *
                                    phase0_count_selected_motors(motor_mask));

    assert(timestamp_us != NULL);
    for (frame_index = 0U; frame_index < frame_count; ++frame_index)
    {
        CanFrame request;
        CanTxPriority priority;
        uint8_t joint_bit;

        assert(aethor_app_next_can_frame(++(*timestamp_us),
                                         &request,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER);
        assert(request.identifier == S3519_PARAMETER_COMMAND_IDENTIFIER);
        joint_bit = (uint8_t)(1U << (request.data[0] - 1U));
        assert((motor_mask & joint_bit) != 0U);
        if (request.data[2] == 0x33U)
        {
            phase0_respond_to_mode_readback(&request, *timestamp_us);
        }
        (void)aethor_app_service(*timestamp_us);
    }
}

/**
 * @brief Completes explicit bench initialization for one selected motor list.
 * @param request_line Complete bench init request with request id 2.
 * @param motor_mask Expected selected subset.
 * @param timestamp_us Mutable monotonic timestamp.
 */
static void phase0_initialize_motor_subset(const char *request_line,
                                           uint8_t motor_mask,
                                           uint64_t *timestamp_us)
{
    ProtocolOutputBatch output_batch;

    assert(request_line != NULL);
    assert(timestamp_us != NULL);
    phase0_submit_request(request_line, ++(*timestamp_us));
    (void)aethor_app_service(++(*timestamp_us));
    phase0_complete_discovery_subset(motor_mask, 1U, timestamp_us);
    phase0_complete_mode_switch_subset(motor_mask, timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 2 bench init result=completed") != NULL);
}

/**
 * @brief Enables one already initialized bench motor and confirms fresh feedback.
 * @param timestamp_us Mutable monotonic timestamp.
 */
static void phase0_enable_motor_one(uint64_t *timestamp_us)
{
    ProtocolOutputBatch output_batch;
    CanFrame enable_frame;
    CanTxPriority priority;

    assert(timestamp_us != NULL);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++(*timestamp_us));
    phase0_submit_request("3 bench enable 1", ++(*timestamp_us));
    (void)aethor_app_service(++(*timestamp_us));
    assert(aethor_app_next_can_frame(++(*timestamp_us),
                                     &enable_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(phase0_is_mode_command(&enable_frame, S3519_MODE_COMMAND_ENABLE));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++(*timestamp_us));
    assert(aethor_app_service(++(*timestamp_us)) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 3 bench enable result=completed") != NULL);
}

/**
 * @brief Asserts the application has no pending motor command frame.
 * @param timestamp_us Monotonic service time.
 */
static void phase0_assert_no_pending_can_frame(uint64_t timestamp_us)
{
    CanFrame frame;
    CanTxPriority priority;

    assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) !=
           MOTOR_RUNTIME_STATUS_FRAME_READY);
}

/**
 * @brief Advances one initialized selected subset to pending one-shot target frames.
 * @param initialization_request Bench init request with request id 2.
 * @param initialized_mask Motors made discoverable for selected/unselected tests.
 * @param move_request One-shot move request with request id 50.
 * @param move_mask Motors controlled by the one-shot action.
 * @param boot_id Deterministic nonzero boot identity.
 * @param timestamp_us Mutable monotonic timestamp.
 */
static void phase0_enter_one_shot_move_wait(
    const char *initialization_request,
    uint8_t initialized_mask,
    const char *move_request,
    uint8_t move_mask,
    uint32_t boot_id,
    uint64_t *timestamp_us)
{
    CanFrame frame;
    CanTxPriority priority;
    uint8_t joint_index;

    assert(initialization_request != NULL);
    assert(move_request != NULL);
    assert(timestamp_us != NULL);
    phase0_start_text_session(timestamp_us, boot_id);
    phase0_initialize_motor_subset(initialization_request,
                                   initialized_mask,
                                   timestamp_us);
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((move_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            phase0_feed_feedback((uint8_t)(joint_index + 1U),
                                 S3519_DRIVER_STATE_DISABLED,
                                 ++(*timestamp_us));
        }
    }
    phase0_submit_request(move_request, ++(*timestamp_us));
    (void)aethor_app_service(++(*timestamp_us));
    phase0_complete_mode_switch_subset(move_mask, timestamp_us);

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((move_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        assert(aethor_app_next_can_frame(++(*timestamp_us),
                                         &frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY);
        assert(frame.identifier == (uint16_t)(0x101U + joint_index));
        assert(phase0_is_mode_command(&frame,
                                      S3519_MODE_COMMAND_CLEAR_ERROR));
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((move_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            phase0_feed_feedback((uint8_t)(joint_index + 1U),
                                 S3519_DRIVER_STATE_DISABLED,
                                 ++(*timestamp_us));
            (void)aethor_app_service(++(*timestamp_us));
        }
    }

    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((move_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        assert(aethor_app_next_can_frame(++(*timestamp_us),
                                         &frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(frame.identifier == (uint16_t)(0x101U + joint_index));
        assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_ENABLE));
    }
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        if ((move_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            phase0_feed_feedback((uint8_t)(joint_index + 1U),
                                 S3519_DRIVER_STATE_ENABLED,
                                 ++(*timestamp_us));
            (void)aethor_app_service(++(*timestamp_us));
        }
    }
}

/**
 * @brief Asserts the complete standardized global control-deadline failure line.
 * @param output_batch Single formatted terminal result.
 * @param expected_stage Stable stage token for the interrupted one-shot phase.
 */
static void phase0_assert_control_deadline_failure(
    const ProtocolOutputBatch *output_batch,
    const char *expected_stage)
{
    char expected_output[PROTOCOL_ENGINE_MESSAGE_CAPACITY];
    int formatted_length;

    assert(output_batch != NULL);
    assert(expected_stage != NULL);
    assert(output_batch->count == 1U);
    formatted_length = snprintf(
        expected_output,
        sizeof(expected_output),
        "done 50 bench move result=failed stage=%s code=action_failed motor=?\n",
        expected_stage);
    assert(formatted_length > 0);
    assert((size_t)formatted_length < sizeof(expected_output));
    assert(strcmp(output_batch->messages[0].data, expected_output) == 0);
}

/**
 * @brief Completes selected motor discovery and POS_VEL mode readback.
 * @param timestamp_us Mutable monotonic timestamp used by the setup.
 */
static void phase0_initialize_selected_motors(uint64_t *timestamp_us)
{
    ProtocolOutputBatch output_batch;
    uint16_t response_index;

    assert(timestamp_us != NULL);
    phase0_submit_request("2 bench init 1,3", *timestamp_us);
    ++(*timestamp_us);
    (void)aethor_app_service(*timestamp_us);
    for (response_index = 0U;
         response_index < (uint16_t)(2U * MOTOR_DISCOVERY_REGISTER_COUNT);
         ++response_index)
    {
        CanFrame request;
        CanFrame response;
        CanTxPriority priority;
        uint8_t payload[8] = {0U};
        uint8_t esc_id;
        uint32_t raw_value;

        ++(*timestamp_us);
        assert(aethor_app_next_can_frame(*timestamp_us, &request, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER);
        esc_id = request.data[0];
        raw_value = phase0_discovery_raw_value(request.data[3], esc_id);
        payload[0] = esc_id;
        payload[2] = 0x33U;
        payload[3] = request.data[3];
        payload[4] = (uint8_t)(raw_value & 0xFFU);
        payload[5] = (uint8_t)((raw_value >> 8U) & 0xFFU);
        payload[6] = (uint8_t)((raw_value >> 16U) & 0xFFU);
        payload[7] = (uint8_t)((raw_value >> 24U) & 0xFFU);
        assert(can_frame_init(&response,
                              (uint16_t)(esc_id + 0x10U),
                              payload,
                              sizeof(payload)) == CAN_FRAME_STATUS_OK);
        assert(aethor_app_receive_can_frame(&response, *timestamp_us) ==
               MOTOR_RUNTIME_STATUS_OK);
        (void)aethor_app_service(*timestamp_us);
    }
    for (response_index = 0U; response_index < 4U; ++response_index)
    {
        CanFrame request;
        CanTxPriority priority;

        ++(*timestamp_us);
        assert(aethor_app_next_can_frame(*timestamp_us, &request, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_PARAMETER);
        if (request.data[2] == 0x33U)
        {
            uint8_t payload[8] = {
                request.data[0], 0U, 0x33U, S3519_REGISTER_CONTROL_MODE,
                2U, 0U, 0U, 0U
            };
            CanFrame response;
            MotorRuntimeStatus response_status;

            assert(can_frame_init(&response,
                                  (uint16_t)(request.data[0] + 0x10U),
                                  payload,
                                  sizeof(payload)) == CAN_FRAME_STATUS_OK);
            response_status =
                aethor_app_receive_can_frame(&response, *timestamp_us);
            assert((response_status == MOTOR_RUNTIME_STATUS_OK) ||
                   (response_status == MOTOR_RUNTIME_STATUS_ACTION_COMPLETE));
        }
        (void)aethor_app_service(*timestamp_us);
    }
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 2 bench init result=completed") != NULL);
}

/**
 * @brief Verifies an unfinished bench move repeats its exact selected target batch.
 */
static void test_aethor_app_repeats_unfinished_bench_target_batch(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame first_target;
    CanFrame second_target;
    CanFrame repeated_target;
    CanTxPriority priority;
    uint64_t timestamp_us = 1000U;

    aethor_app_init(timestamp_us, 9999U);
    (void)aethor_app_service(++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_submit_request("1 hello", ++timestamp_us);
    phase0_initialize_selected_motors(&timestamp_us);

    phase0_submit_request("3 bench enable 1,3", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &first_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &second_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 3 bench enable result=completed") != NULL);

    phase0_submit_request(
        "4 bench jog 1,3 delta=3.0 speed=1.0",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &first_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(first_target.identifier == 0x101U);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &second_target, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(second_target.identifier == 0x103U);
    (void)aethor_app_service(++timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &repeated_target,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(repeated_target.identifier == first_target.identifier);
    assert(repeated_target.length == first_target.length);
    assert(memcmp(repeated_target.data,
                  first_target.data,
                  first_target.length) == 0);
}

/**
 * @brief Verifies a cold one-shot move performs setup in the required safe order.
 */
static void test_aethor_app_one_shot_move_cold_setup_is_strictly_ordered(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame clear_frame;
    CanFrame enable_frame;
    CanFrame target_frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 1000U;

    phase0_start_text_session(&timestamp_us, 11001U);
    phase0_submit_request("50 bench move 1 position=90 speed=30",
                          ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    phase0_complete_discovery_subset(0x01U, 0U, &timestamp_us);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);

    phase0_complete_mode_switch_subset(0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &clear_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(clear_frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&clear_frame,
                                  S3519_MODE_COMMAND_CLEAR_ERROR));
    (void)aethor_app_service(++timestamp_us);
    phase0_assert_no_pending_can_frame(++timestamp_us);

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &enable_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(enable_frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&enable_frame, S3519_MODE_COMMAND_ENABLE));
    (void)aethor_app_service(++timestamp_us);
    phase0_assert_no_pending_can_frame(++timestamp_us);

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &target_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(target_frame.identifier == 0x101U);
    assert(!phase0_is_mode_command(&target_frame,
                                   S3519_MODE_COMMAND_ENABLE));
    assert(!phase0_is_mode_command(&target_frame,
                                   S3519_MODE_COMMAND_CLEAR_ERROR));
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Verifies mixed one-shot setup discovers only missing motors and scopes all setup.
 */
static void test_aethor_app_one_shot_move_discovers_only_missing_subset(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame setup_frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 2000U;
    uint8_t frame_index;

    phase0_start_text_session(&timestamp_us, 11002U);
    phase0_initialize_motor_subset("2 bench init 2", 0x02U, &timestamp_us);
    phase0_submit_request("50 bench move 1,2 position=90,-45 speed=30,20",
                          ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    phase0_complete_discovery_subset(0x01U, 0U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x03U, &timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &setup_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY);
        assert(setup_frame.identifier == (uint16_t)(0x101U + frame_index));
        assert(phase0_is_mode_command(&setup_frame,
                                      S3519_MODE_COMMAND_CLEAR_ERROR));
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &setup_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(setup_frame.identifier == (uint16_t)(0x101U + frame_index));
        assert(phase0_is_mode_command(&setup_frame,
                                      S3519_MODE_COMMAND_ENABLE));
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &setup_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(setup_frame.identifier == (uint16_t)(0x101U + frame_index));
        assert(!phase0_is_mode_command(&setup_frame,
                                       S3519_MODE_COMMAND_ENABLE));
    }
    phase0_assert_no_pending_can_frame(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Checks one one-shot validation failure completes before any motor frame.
 * @param move_request One-shot move request with request id 50.
 * @param feedback_state Driver state to publish when feedback is requested.
 * @param publish_feedback One to publish selected feedback before the move.
 * @param expected_result_fragment Stable terminal result fragment.
 * @param boot_id Unique deterministic boot identity.
 */
static void phase0_assert_one_shot_validation_failure(
    const char *move_request,
    uint8_t feedback_state,
    uint8_t publish_feedback,
    const char *expected_result_fragment,
    uint32_t boot_id)
{
    ProtocolOutputBatch output_batch;
    uint64_t timestamp_us = 3000U;

    phase0_start_text_session(&timestamp_us, boot_id);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    if (publish_feedback != 0U)
    {
        phase0_feed_feedback(1U, feedback_state, ++timestamp_us);
    }
    phase0_submit_request(move_request, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  expected_result_fragment) != NULL);
}

/**
 * @brief Verifies one-shot dynamic validation rejects range, speed, fault, and feedback errors.
 */
static void test_aethor_app_one_shot_move_fails_before_enable_on_invalid_target(void)
{
    phase0_assert_one_shot_validation_failure(
        "50 bench move 1 position=800 speed=30",
        S3519_DRIVER_STATE_DISABLED,
        1U,
        "stage=validate code=position_out_of_range motor=1",
        11003U);
    phase0_assert_one_shot_validation_failure(
        "50 bench move 1 position=90 speed=1200",
        S3519_DRIVER_STATE_DISABLED,
        1U,
        "stage=validate code=speed_out_of_range motor=1",
        11004U);
    phase0_assert_one_shot_validation_failure(
        "50 bench move 1 position=90 speed=30",
        S3519_DRIVER_STATE_DISABLED,
        0U,
        "stage=validate code=stale_feedback motor=1",
        11005U);
    phase0_assert_one_shot_validation_failure(
        "50 bench move 1 position=90 speed=30",
        S3519_DRIVER_STATE_FAULT_MINIMUM,
        1U,
        "stage=validate code=fault_present motor=1",
        11006U);
}

/**
 * @brief Verifies legacy jog accepts a dynamically safe move larger than three degrees.
 */
static void test_aethor_app_legacy_jog_accepts_dynamic_range_above_three_degrees(void)
{
    CanFrame target_frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 4000U;

    phase0_start_text_session(&timestamp_us, 11007U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_enable_motor_one(&timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request("4 bench jog 1 delta=4 speed=1", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &target_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(target_frame.identifier == 0x101U);
}

/**
 * @brief Checks one legacy jog dynamic validation failure emits no target frame.
 * @param jog_request Bench jog request with request id 4.
 * @param feedback_state Optional final feedback state.
 * @param publish_final_feedback One to refresh feedback before the request.
 * @param stale_offset_us Additional time before submitting the request.
 * @param boot_id Unique deterministic boot identity.
 */
static void phase0_assert_legacy_jog_validation_failure(
    const char *jog_request,
    uint8_t feedback_state,
    uint8_t publish_final_feedback,
    uint64_t stale_offset_us,
    uint32_t boot_id)
{
    ProtocolOutputBatch output_batch;
    uint64_t timestamp_us = 5000U;

    phase0_start_text_session(&timestamp_us, boot_id);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_enable_motor_one(&timestamp_us);
    if (publish_final_feedback != 0U)
    {
        phase0_feed_feedback(1U, feedback_state, ++timestamp_us);
    }
    timestamp_us += stale_offset_us;
    phase0_submit_request(jog_request, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strcmp(output_batch.messages[0].data,
                  "done 4 bench jog result=failed elapsed_ms=0 arrived=00\n") ==
           0);
}

/**
 * @brief Verifies legacy jog validates discovered ranges and fresh fault-free feedback.
 */
static void test_aethor_app_legacy_jog_rejects_dynamic_validation_failures(void)
{
    phase0_assert_legacy_jog_validation_failure(
        "4 bench jog 1 delta=800 speed=1",
        S3519_DRIVER_STATE_ENABLED,
        1U,
        0U,
        11008U);
    phase0_assert_legacy_jog_validation_failure(
        "4 bench jog 1 delta=1 speed=1200",
        S3519_DRIVER_STATE_ENABLED,
        1U,
        0U,
        11009U);
    phase0_assert_legacy_jog_validation_failure(
        "4 bench jog 1 delta=1 speed=1",
        S3519_DRIVER_STATE_ENABLED,
        0U,
        MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1U,
        11010U);
    phase0_assert_legacy_jog_validation_failure(
        "4 bench jog 1 delta=1 speed=1",
        S3519_DRIVER_STATE_FAULT_MINIMUM,
        1U,
        0U,
        11011U);
}

/**
 * @brief Verifies discovery failure disables only selected motors in both modes.
 */
static void test_aethor_app_one_shot_discovery_failure_disables_selected_unknown_modes(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    MotorRuntimeStatus runtime_status;
    uint64_t timestamp_us = 5500U;
    uint8_t retry_index;

    phase0_start_text_session(&timestamp_us, 11016U);
    phase0_submit_request(
        "50 bench move 1,3 position=90,-45 speed=30,20",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[0] == 1U);
    for (retry_index = 1U;
         retry_index < MOTOR_DISCOVERY_MAX_ATTEMPTS;
         ++retry_index)
    {
        timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.data[0] == 1U);
    }
    timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
    runtime_status = aethor_app_next_can_frame(timestamp_us, &frame, &priority);
    assert(runtime_status == MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR);
    assert(aethor_app_service(++timestamp_us) == 0U);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(frame.identifier == 0x001U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x101U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x003U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x103U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) !=
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Verifies a pre-enabled motor receives selected disable after mode failure.
 */
static void test_aethor_app_one_shot_mode_failure_disables_pre_enabled_motor(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    MotorRuntimeStatus runtime_status;
    uint64_t timestamp_us = 6000U;
    uint8_t retry_index;

    phase0_start_text_session(&timestamp_us, 11012U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request("50 bench move 1 position=90 speed=30",
                          ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[0] == 1U);
    assert(frame.data[2] == 0x55U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[0] == 1U);
    assert(frame.data[2] == 0x33U);
    for (retry_index = 1U;
         retry_index < MOTOR_DISCOVERY_MAX_ATTEMPTS;
         ++retry_index)
    {
        timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.data[0] == 1U);
        assert(frame.data[2] == 0x33U);
    }
    timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
    runtime_status = aethor_app_next_can_frame(timestamp_us, &frame, &priority);
    assert(runtime_status == MOTOR_RUNTIME_STATUS_ACTION_FAILED);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(frame.identifier == 0x001U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) !=
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Verifies clear feedback timeout starts selected best-effort disable.
 */
static void test_aethor_app_one_shot_clear_timeout_disables_selected_motor(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 7000U;

    phase0_start_text_session(&timestamp_us, 11013U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request("50 bench move 1 position=90 speed=30",
                          ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x01U, &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_CLEAR_ERROR));

    timestamp_us += PHASE0_TEST_ACTION_TIMEOUT_US + 1U;
    assert(aethor_app_service(timestamp_us) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Verifies the final clear frame starts a complete new feedback deadline window.
 */
static void test_aethor_app_one_shot_clear_deadline_starts_after_final_frame(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 8000U;
    uint64_t clear_batch_started_us;

    phase0_start_text_session(&timestamp_us, 11014U);
    phase0_initialize_motor_subset("2 bench init 1,2", 0x03U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_submit_request(
        "50 bench move 1,2 position=90,-45 speed=30,20",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x03U, &timestamp_us);
    clear_batch_started_us = timestamp_us;

    timestamp_us = clear_batch_started_us + 400000U;
    assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x101U);
    timestamp_us = clear_batch_started_us + 499000U;
    assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x102U);

    timestamp_us = clear_batch_started_us +
                   PHASE0_TEST_ACTION_TIMEOUT_US + 1U;
    assert(aethor_app_service(timestamp_us) == 0U);
    phase0_assert_no_pending_can_frame(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_ENABLE));
}

/**
 * @brief Verifies retained selected discovery survives an unselected discovery failure.
 */
static void test_aethor_app_one_shot_uses_selected_verified_after_unselected_failure(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    MotorRuntimeStatus runtime_status;
    uint64_t timestamp_us = 9000U;
    uint8_t retry_index;

    phase0_start_text_session(&timestamp_us, 11015U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_submit_request("3 bench init 2", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.data[0] == 2U);
    for (retry_index = 1U;
         retry_index < MOTOR_DISCOVERY_MAX_ATTEMPTS;
         ++retry_index)
    {
        timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
        assert(aethor_app_next_can_frame(timestamp_us, &frame, &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(frame.data[0] == 2U);
    }
    timestamp_us += MOTOR_DISCOVERY_REQUEST_TIMEOUT_US;
    runtime_status = aethor_app_next_can_frame(timestamp_us, &frame, &priority);
    assert(runtime_status != MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_submit_request("50 bench move 1 position=90 speed=30",
                          ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x01U, &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_CLEAR_ERROR));
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

    aethor_app_init(1000ULL, 1234U);
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
    assert(snapshot.state == ARM_STATE_UNALIGNED);
    assert(snapshot.fault == ARM_FAULT_NONE);

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
    aethor_app_init(5000ULL, 5678U);

    assert(aethor_app_get_snapshot(&snapshot));
    assert(snapshot.state == ARM_STATE_BOOT);
    assert(snapshot.fault == ARM_FAULT_NONE);
    assert(aethor_app_get_diagnostic_counters(&counters));
    assert(counters.service_cycles == 0U);
}

/**
 * @brief Verifies an idle disabled controller does not arm the link watchdog.
 */
static void test_aethor_app_idle_link_does_not_timeout(void)
{
    static const char hello_line[] = "1 hello\n";
    ProtocolOutputBatch output_batch;
    ArmSnapshot arm_snapshot;
    CanFrame emergency_frame;

    aethor_app_init(1000U, 7777U);
    assert(aethor_app_process_protocol_line(hello_line,
                                            sizeof(hello_line) - 1U,
                                            2000U,
                                            &output_batch) ==
           PROTOCOL_ENGINE_STATUS_OK);
    assert(aethor_app_service(1002000U) == 0U);
    assert(aethor_app_get_snapshot(&arm_snapshot));
    assert(arm_snapshot.state != ARM_STATE_FAULT);
    assert(arm_snapshot.fault == ARM_FAULT_NONE);
    assert(arm_snapshot.enabled == 0U);
    assert(arm_snapshot.moving == 0U);
    assert(aethor_app_pop_emergency_can_frame(&emergency_frame) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
}

/**
 * @brief Verifies one submitted move resends, holds, disables, and completes without ping.
 */
static void test_aethor_app_one_shot_move_completes_without_serial_keepalive(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame first_targets[2];
    CanFrame repeated_targets[2];
    CanFrame hold_frames[2];
    CanFrame disable_frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 12000U;
    uint8_t frame_index;
    static const float target_positions_rad[2] = {
        0.0F, -0.78539816339F
    };
    static const float target_speeds_rad_s[2] = {
        0.52359877559F, 0.34906585040F
    };

    phase0_start_text_session(&timestamp_us, 11017U);
    phase0_initialize_motor_subset("2 bench init 1,3", 0x05U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request(
        "50 bench move 1,3 position=0,-45 speed=30,20",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x05U, &timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        CanFrame clear_frame;

        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &clear_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY);
        assert(clear_frame.identifier ==
               ((frame_index == 0U) ? 0x101U : 0x103U));
        assert(phase0_is_mode_command(&clear_frame,
                                      S3519_MODE_COMMAND_CLEAR_ERROR));
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        CanFrame enable_frame;

        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &enable_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(enable_frame.identifier ==
               ((frame_index == 0U) ? 0x101U : 0x103U));
        assert(phase0_is_mode_command(&enable_frame,
                                      S3519_MODE_COMMAND_ENABLE));
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &first_targets[frame_index],
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
        assert(first_targets[frame_index].identifier ==
               ((frame_index == 0U) ? 0x101U : 0x103U));
        assert(fabsf(phase0_read_command_float(
                         &first_targets[frame_index].data[0]) -
                     target_positions_rad[frame_index]) < 0.000001F);
        assert(fabsf(phase0_read_command_float(
                         &first_targets[frame_index].data[4]) -
                     target_speeds_rad_s[frame_index]) < 0.000001F);
    }
    phase0_assert_no_pending_can_frame(++timestamp_us);

    timestamp_us += PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US + 10000U;
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &repeated_targets[frame_index],
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(repeated_targets[frame_index].identifier ==
               first_targets[frame_index].identifier);
        assert(repeated_targets[frame_index].length ==
               first_targets[frame_index].length);
        assert(memcmp(repeated_targets[frame_index].data,
                      first_targets[frame_index].data,
                      first_targets[frame_index].length) == 0);
    }

    phase0_feed_feedback_position(1U,
                                  S3519_DRIVER_STATE_ENABLED,
                                  target_positions_rad[0],
                                  ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    phase0_feed_feedback_position(3U,
                                  S3519_DRIVER_STATE_ENABLED,
                                  target_positions_rad[1],
                                  ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &hold_frames[frame_index],
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(hold_frames[frame_index].identifier ==
               first_targets[frame_index].identifier);
        assert(memcmp(hold_frames[frame_index].data,
                      first_targets[frame_index].data,
                      sizeof(float)) == 0);
        assert(phase0_read_command_float(
                   &hold_frames[frame_index].data[4]) == 0.0F);
    }
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);

    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &disable_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY);
        assert(disable_frame.identifier ==
               ((frame_index == 0U) ? 0x101U : 0x103U));
        assert(phase0_is_mode_command(&disable_frame,
                                      S3519_MODE_COMMAND_DISABLE));
    }
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);

    phase0_feed_feedback_position(1U,
                                  S3519_DRIVER_STATE_DISABLED,
                                  target_positions_rad[0],
                                  ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    phase0_feed_feedback_position(3U,
                                  S3519_DRIVER_STATE_DISABLED,
                                  target_positions_rad[1],
                                  ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 50 bench move result=completed") != NULL);
    phase0_assert_no_pending_can_frame(++timestamp_us);
}

/**
 * @brief Verifies an unrepresentable finite motion deadline fails into selected cleanup.
 */
static void test_aethor_app_one_shot_unbounded_timeout_fails_closed(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 13000U;

    phase0_start_text_session(&timestamp_us, 11020U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_submit_request(
        "50 bench move 1 position=90 speed=0.000000000000000001",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x01U, &timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(phase0_is_mode_command(&frame,
                                  S3519_MODE_COMMAND_CLEAR_ERROR));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_ENABLE));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(frame.identifier == 0x101U);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "stage=motion code=action_failed motor=1") != NULL);
    phase0_assert_no_pending_can_frame(++timestamp_us);
}

/**
 * @brief Verifies a large but representable finite deadline still starts motion.
 */
static void test_aethor_app_one_shot_large_finite_timeout_starts_motion(void)
{
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 13500U;

    phase0_start_text_session(&timestamp_us, 11021U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    phase0_submit_request(
        "50 bench move 1 position=1 speed=0.000001",
        ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_complete_mode_switch_subset(0x01U, &timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(phase0_is_mode_command(&frame,
                                  S3519_MODE_COMMAND_CLEAR_ERROR));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_ENABLE));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);

    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(frame.identifier == 0x101U);
    assert(!phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
}

/**
 * @brief Verifies selected stale feedback immediately stops target retransmission.
 */
static void test_aethor_app_one_shot_move_selected_stale_fails_immediately(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 17000U;

    phase0_enter_one_shot_move_wait(
        "2 bench init 1",
        0x01U,
        "50 bench move 1 position=1 speed=0.000001",
        0x01U,
        11022U,
        &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);

    timestamp_us += MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1U;
    assert(aethor_app_service(timestamp_us) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "stage=motion code=stale_feedback motor=1") != NULL);
    phase0_assert_no_pending_can_frame(++timestamp_us);
}

/**
 * @brief Verifies only selected driver faults cancel one-shot motion.
 */
static void test_aethor_app_one_shot_move_ignores_unselected_fault(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame first_targets[2];
    CanFrame repeated_frame;
    CanFrame disable_frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 18000U;
    uint8_t frame_index;

    phase0_enter_one_shot_move_wait(
        "2 bench init 1,2,3",
        0x07U,
        "50 bench move 1,3 position=1,-1 speed=1,1",
        0x05U,
        11023U,
        &timestamp_us);
    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &first_targets[frame_index],
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_feed_feedback(2U, S3519_DRIVER_STATE_FAULT_MINIMUM, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &repeated_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(memcmp(repeated_frame.data,
                  first_targets[0].data,
                  first_targets[0].length) == 0);
    assert(aethor_app_next_can_frame(++timestamp_us,
                                     &repeated_frame,
                                     &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);

    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_FAULT_MINIMUM, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);
    for (frame_index = 0U; frame_index < 2U; ++frame_index)
    {
        assert(aethor_app_next_can_frame(++timestamp_us,
                                         &disable_frame,
                                         &priority) ==
               MOTOR_RUNTIME_STATUS_FRAME_READY);
        assert(priority == CAN_TX_PRIORITY_EMERGENCY);
        assert(disable_frame.identifier ==
               ((frame_index == 0U) ? 0x101U : 0x103U));
        assert(phase0_is_mode_command(&disable_frame,
                                      S3519_MODE_COMMAND_DISABLE));
    }
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    phase0_feed_feedback(3U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "stage=motion code=fault_present motor=3") != NULL);
}

/**
 * @brief Verifies control deadline failure keeps the existing all-axis emergency path.
 */
static void test_aethor_app_one_shot_control_deadline_is_global(void)
{
    ProtocolOutputBatch output_batch;
    ArmSnapshot arm_snapshot;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 19000U;
    uint8_t disabled_motor_mask = 0U;
    uint8_t service_index;

    phase0_enter_one_shot_move_wait(
        "2 bench init 1",
        0x01U,
        "50 bench move 1 position=1 speed=0.000001",
        0x01U,
        11024U,
        &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    for (service_index = 0U; service_index < 3U; ++service_index)
    {
        timestamp_us += 5001U;
        phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, timestamp_us);
        (void)aethor_app_service(timestamp_us);
    }

    while (aethor_app_pop_emergency_can_frame(&frame) != 0U)
    {
        uint8_t motor_number = (frame.identifier >= 0x101U)
                                   ? (uint8_t)(frame.identifier - 0x100U)
                                   : (uint8_t)frame.identifier;

        assert(motor_number >= 1U);
        assert(motor_number <= ARM_JOINT_COUNT);
        assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
        disabled_motor_mask |= (uint8_t)(1U << (motor_number - 1U));
    }
    assert(disabled_motor_mask == 0x7FU);
    assert(aethor_app_get_snapshot(&arm_snapshot));
    assert(arm_snapshot.fault == ARM_FAULT_CONTROL_DEADLINE);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    phase0_assert_control_deadline_failure(&output_batch, "motion");
}

/**
 * @brief Verifies a third control miss outranks simultaneous selected stale feedback.
 */
static void test_aethor_app_one_shot_control_deadline_outranks_selected_stale(void)
{
    ProtocolOutputBatch output_batch;
    ArmSnapshot arm_snapshot;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 19500U;
    uint8_t disabled_motor_mask = 0U;
    uint8_t service_index;

    phase0_enter_one_shot_move_wait(
        "2 bench init 1",
        0x01U,
        "50 bench move 1 position=1 speed=0.000001",
        0x01U,
        11027U,
        &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    for (service_index = 0U; service_index < 2U; ++service_index)
    {
        timestamp_us += 5001U;
        phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, timestamp_us);
        assert(aethor_app_service(timestamp_us) == 0U);
    }
    timestamp_us += MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1U;
    (void)aethor_app_service(timestamp_us);

    while (aethor_app_pop_emergency_can_frame(&frame) != 0U)
    {
        uint8_t motor_number = (frame.identifier >= 0x101U)
                                   ? (uint8_t)(frame.identifier - 0x100U)
                                   : (uint8_t)frame.identifier;

        assert(motor_number >= 1U);
        assert(motor_number <= ARM_JOINT_COUNT);
        assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
        disabled_motor_mask |= (uint8_t)(1U << (motor_number - 1U));
    }
    assert(disabled_motor_mask == 0x7FU);
    assert(aethor_app_get_snapshot(&arm_snapshot));
    assert(arm_snapshot.fault == ARM_FAULT_CONTROL_DEADLINE);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    phase0_assert_control_deadline_failure(&output_batch, "motion");
    phase0_assert_no_pending_can_frame(++timestamp_us);
}

/**
 * @brief Verifies HOLD stale feedback and DISABLE fault both fail without new targets.
 */
static void test_aethor_app_one_shot_shutdown_safety_fails_closed(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 20000U;

    phase0_enter_one_shot_move_wait(
        "2 bench init 1",
        0x01U,
        "50 bench move 1 position=0 speed=1",
        0x01U,
        11025U,
        &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    timestamp_us += MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US + 1U;
    assert(aethor_app_service(timestamp_us) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "stage=hold code=stale_feedback motor=1") != NULL);

    timestamp_us = 21000U;
    phase0_enter_one_shot_move_wait(
        "2 bench init 1",
        0x01U,
        "50 bench move 1 position=0 speed=1",
        0x01U,
        11026U,
        &timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_JOINT_CONTROL);
    assert(phase0_read_command_float(&frame.data[4]) == 0.0F);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    phase0_feed_feedback(1U,
                         S3519_DRIVER_STATE_FAULT_MINIMUM,
                         ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 0U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 0U);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(priority == CAN_TX_PRIORITY_EMERGENCY);
    assert(phase0_is_mode_command(&frame, S3519_MODE_COMMAND_DISABLE));
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_DISABLED, ++timestamp_us);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "stage=disable code=fault_present motor=1") != NULL);
}

/**
 * @brief Verifies legacy enable and jog still arm the link watchdog.
 */
static void test_aethor_app_legacy_bench_actions_still_require_keepalive(void)
{
    ProtocolOutputBatch output_batch;
    CanFrame frame;
    CanTxPriority priority;
    uint64_t timestamp_us = 14000U;

    phase0_start_text_session(&timestamp_us, 11018U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request("3 bench enable 1", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    assert(aethor_app_service(++timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    timestamp_us += PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US + 1U;
    assert(aethor_app_service(timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "link_timeout") != NULL);
    assert(aethor_app_pop_emergency_can_frame(&frame) == 1U);

    timestamp_us = 16000U;
    phase0_start_text_session(&timestamp_us, 11019U);
    phase0_initialize_motor_subset("2 bench init 1", 0x01U, &timestamp_us);
    phase0_enable_motor_one(&timestamp_us);
    phase0_feed_feedback(1U, S3519_DRIVER_STATE_ENABLED, ++timestamp_us);
    phase0_submit_request("4 bench jog 1 delta=4 speed=1", ++timestamp_us);
    (void)aethor_app_service(++timestamp_us);
    assert(aethor_app_next_can_frame(++timestamp_us, &frame, &priority) ==
           MOTOR_RUNTIME_STATUS_FRAME_READY);
    timestamp_us += PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US + 1U;
    assert(aethor_app_service(timestamp_us) == 1U);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data,
                  "done 4 bench jog result=cancelled") != NULL);
    assert(aethor_app_pop_protocol_result_output(&output_batch) == 1U);
    assert(strstr(output_batch.messages[0].data, "link_timeout") != NULL);
    assert(aethor_app_pop_emergency_can_frame(&frame) == 1U);
}

/**
 * @brief Verifies severe CAN/USB transport faults latch and schedule all-axis disable.
 */
static void test_aethor_app_transport_fault_stops_and_disables(void)
{
    ArmSnapshot arm_snapshot;
    CanFrame emergency_frame;
    uint8_t emergency_count = 0U;

    aethor_app_init(1000U, 8888U);
    assert(aethor_app_report_transport_fault(0x00000007U, 2000U) == 0U);
    assert(aethor_app_get_snapshot(&arm_snapshot));
    assert(arm_snapshot.state == ARM_STATE_FAULT);
    assert(arm_snapshot.fault == ARM_FAULT_TRANSPORT);
    assert(arm_snapshot.fault_detail == 0x00000007U);
    while (aethor_app_pop_emergency_can_frame(&emergency_frame) != 0U)
    {
        assert(emergency_frame.data[7] == S3519_MODE_COMMAND_DISABLE);
        ++emergency_count;
    }
    assert(emergency_count == MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES);
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
    test_diagnostics_runtime_aggregation();
    test_diagnostics_ring_overwrites_oldest_event();
    test_arm_controller_bench_profile_keeps_enable_locked_without_fault();
    test_arm_controller_latches_invalid_config_fault();
    test_arm_controller_enters_unaligned_after_self_test();
    test_arm_controller_motion_transitions();
    test_joint_reference_alignment_and_reboot_invalidation();
    test_joint_reference_inverse_command_mapping();
    test_phase0_state_queries_reject_null_outputs();
    test_layer_contracts_are_frozen();
    test_aethor_app_latches_safe_phase0_fault();
    test_aethor_app_reinitializes_deterministically();
    test_aethor_app_idle_link_does_not_timeout();
    test_aethor_app_one_shot_move_completes_without_serial_keepalive();
    test_aethor_app_one_shot_unbounded_timeout_fails_closed();
    test_aethor_app_one_shot_large_finite_timeout_starts_motion();
    test_aethor_app_one_shot_move_selected_stale_fails_immediately();
    test_aethor_app_one_shot_move_ignores_unselected_fault();
    test_aethor_app_one_shot_control_deadline_is_global();
    test_aethor_app_one_shot_control_deadline_outranks_selected_stale();
    test_aethor_app_one_shot_shutdown_safety_fails_closed();
    test_aethor_app_legacy_bench_actions_still_require_keepalive();
    test_aethor_app_repeats_unfinished_bench_target_batch();
    test_aethor_app_one_shot_move_cold_setup_is_strictly_ordered();
    test_aethor_app_one_shot_move_discovers_only_missing_subset();
    test_aethor_app_one_shot_move_fails_before_enable_on_invalid_target();
    test_aethor_app_legacy_jog_accepts_dynamic_range_above_three_degrees();
    test_aethor_app_legacy_jog_rejects_dynamic_validation_failures();
    test_aethor_app_one_shot_discovery_failure_disables_selected_unknown_modes();
    test_aethor_app_one_shot_mode_failure_disables_pre_enabled_motor();
    test_aethor_app_one_shot_clear_timeout_disables_selected_motor();
    test_aethor_app_one_shot_clear_deadline_starts_after_final_frame();
    test_aethor_app_one_shot_uses_selected_verified_after_unselected_failure();
    test_aethor_app_transport_fault_stops_and_disables();

    printf("PHASE0_TESTS_PASSED\n");
    return 0;
}
