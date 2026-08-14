/**
 * @file phase0_test_main.c
 * @brief Host-side tests for the PRD Phase 0 layered firmware baseline.
 */

#include <assert.h>
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
static void phase0_feed_feedback(uint8_t esc_id,
                                 uint8_t driver_state,
                                 uint64_t timestamp_us)
{
    uint8_t payload[8] = {
        (uint8_t)((driver_state << 4U) | esc_id),
        0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 35U, 27U
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
    test_aethor_app_repeats_unfinished_bench_target_batch();
    test_aethor_app_transport_fault_stops_and_disables();

    printf("PHASE0_TESTS_PASSED\n");
    return 0;
}
