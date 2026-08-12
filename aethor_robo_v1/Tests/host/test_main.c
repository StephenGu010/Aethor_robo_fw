/**
 * @file test_main.c
 * @brief Windows host tests for the classic CAN, S3519, USB command, and synchronized trajectory modules.
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "dm_motor_protocol.h"
#include "dual_motor_controller.h"
#include "fdcan_classic_codec.h"
#include "firmware_probe.h"
#include "joint_controller.h"
#include "robot_config.h"
#include "sync_trajectory.h"
#include "usb_cdc_transport.h"
#include "usb_command.h"

static int test_failure_count = 0;
static char usb_transmit_capture[512];
static size_t usb_transmit_capture_length = 0U;
static uint32_t usb_transmit_busy_cycles = 0U;
static FdcanClassicFrame joint_controller_last_frame;
static FdcanClassicFrame joint_controller_sent_frames[256];
static uint32_t joint_controller_send_count = 0U;
static RobotConfiguration joint_controller_test_configuration;
static FdcanClassicFrame dual_motor_sent_frames[512];
static uint32_t dual_motor_sent_times_ms[512];
static uint32_t dual_motor_send_count = 0U;
static uint32_t dual_motor_current_time_ms = 0U;
static int32_t dual_motor_fail_at_send_index = -1;
static char firmware_probe_capture[512];
static size_t firmware_probe_capture_length = 0U;

/**
 * @brief Records a failed integer comparison without stopping the remaining tests.
 * @param actual Actual integer value.
 * @param expected Expected integer value.
 * @param expression Description of the checked expression.
 */
static void expect_integer(int actual, int expected, const char *expression)
{
    if (actual != expected)
    {
        printf("FAIL: %s: actual=%d expected=%d\n", expression, actual, expected);
        test_failure_count++;
    }
}

/**
 * @brief Records a failed floating-point comparison.
 * @param actual Actual floating-point value.
 * @param expected Expected floating-point value.
 * @param tolerance Maximum accepted absolute error.
 * @param expression Description of the checked expression.
 */
static void expect_float(float actual, float expected, float tolerance, const char *expression)
{
    if (!isfinite(actual) || fabsf(actual - expected) > tolerance)
    {
        printf("FAIL: %s: actual=%.7f expected=%.7f tolerance=%.7f\n",
               expression, actual, expected, tolerance);
        test_failure_count++;
    }
}

/**
 * @brief Verifies the static seven-axis model, active mask, validation, and drivetrain conversions.
 */
static void test_robot_configuration(void)
{
    RobotConfiguration test_configuration = *robot_config_get();
    const RobotConfiguration *production_configuration = robot_config_get();
    float motor_position_rad = 0.0f;
    float joint_position_degrees = 0.0f;
    float motor_velocity_rad_s = 0.0f;
    float joint_velocity_degrees_s = 0.0f;
    uint8_t joint_index;

    expect_integer(production_configuration != NULL, 1, "production robot configuration exists");
    expect_integer(production_configuration->active_joint_mask,
                   0x03,
                   "only the two connected unloaded motors are active by default");
    expect_integer(production_configuration->dh_parameters_valid,
                   0,
                   "unmeasured production DH values remain explicitly invalid");
    expect_integer(robot_config_validate(production_configuration),
                   ROBOT_CONFIG_STATUS_OK,
                   "production configuration is structurally valid");
    expect_integer(robot_config_active_profiles_are_commissioned(production_configuration),
                   0,
                   "unmeasured bench profiles keep full multi-axis enable locked");

    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; ++joint_index)
    {
        expect_integer(isfinite(production_configuration->dh[joint_index].theta_offset_degrees),
                       1,
                       "DH theta offset is finite");
        expect_integer(isfinite(production_configuration->dh[joint_index].d_millimeters),
                       1,
                       "DH d value is finite");
        expect_integer(isfinite(production_configuration->dh[joint_index].a_millimeters),
                       1,
                       "DH a value is finite");
        expect_integer(isfinite(production_configuration->dh[joint_index].alpha_degrees),
                       1,
                       "DH alpha value is finite");
    }

    test_configuration.active_joint_mask = 0x7FU;
    test_configuration.dh_parameters_valid = 1U;
    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; ++joint_index)
    {
        test_configuration.joint[joint_index].commissioned = 1U;
        test_configuration.joint[joint_index].minimum_degrees = -180.0f;
        test_configuration.joint[joint_index].maximum_degrees = 180.0f;
        test_configuration.joint[joint_index].maximum_velocity_degrees_s = 30.0f;
        test_configuration.joint[joint_index].maximum_acceleration_degrees_s2 = 60.0f;
    }
    expect_integer(robot_config_validate(&test_configuration),
                   ROBOT_CONFIG_STATUS_OK,
                   "seven active joints have unique valid bus identifiers");
    expect_integer(robot_config_active_profiles_are_commissioned(&test_configuration),
                   1,
                   "a complete seven-axis test profile unlocks full control");

    test_configuration.joint[0].direction = -1;
    test_configuration.joint[0].joint_zero_degrees = 10.0f;
    test_configuration.joint[0].external_reduction_ratio = 2.5f;
    expect_integer(robot_config_joint_to_motor_position_rad(&test_configuration,
                                                            0U,
                                                            30.0f,
                                                            &motor_position_rad),
                   ROBOT_CONFIG_STATUS_OK,
                   "joint position converts through direction zero and reduction");
    expect_float(motor_position_rad,
                 -0.87266463f,
                 0.00001f,
                 "joint position conversion produces motor-output radians");
    expect_integer(robot_config_motor_to_joint_position_degrees(&test_configuration,
                                                                0U,
                                                                motor_position_rad,
                                                                &joint_position_degrees),
                   ROBOT_CONFIG_STATUS_OK,
                   "motor position converts back to joint degrees");
    expect_float(joint_position_degrees,
                 30.0f,
                 0.0001f,
                 "position conversion round trip preserves joint angle");

    expect_integer(robot_config_joint_to_motor_velocity_rad_s(&test_configuration,
                                                              0U,
                                                              20.0f,
                                                              &motor_velocity_rad_s),
                   ROBOT_CONFIG_STATUS_OK,
                   "joint velocity limit converts to motor-output radians per second");
    expect_float(motor_velocity_rad_s,
                 0.87266463f,
                 0.00001f,
                 "velocity limit conversion includes external reduction");
    expect_integer(robot_config_motor_to_joint_velocity_degrees_s(&test_configuration,
                                                                  0U,
                                                                  -motor_velocity_rad_s,
                                                                  &joint_velocity_degrees_s),
                   ROBOT_CONFIG_STATUS_OK,
                   "signed motor velocity converts back to joint velocity");
    expect_float(joint_velocity_degrees_s,
                 20.0f,
                 0.0001f,
                 "signed velocity conversion preserves direction");

    test_configuration.joint[1].master_id = test_configuration.joint[0].master_id;
    expect_integer(robot_config_validate(&test_configuration),
                   ROBOT_CONFIG_STATUS_DUPLICATE_IDENTIFIER,
                   "active joints reject duplicate Master IDs");
    test_configuration.joint[1].master_id = 0x12U;
    test_configuration.joint[0].external_reduction_ratio = 0.0f;
    expect_integer(robot_config_validate(&test_configuration),
                   ROBOT_CONFIG_STATUS_INVALID_REDUCTION,
                   "zero external reduction is rejected");
}

/**
 * @brief Captures one transport transmission and optionally simulates USBD_BUSY.
 * @param data Bytes selected for transmission.
 * @param length Number of bytes selected for transmission.
 * @return Transport callback result.
 */
static UsbCdcTransmitResult test_usb_transmit(const uint8_t *data, uint16_t length)
{
    if (usb_transmit_busy_cycles > 0U)
    {
        usb_transmit_busy_cycles--;
        return USB_CDC_TRANSMIT_BUSY;
    }
    if ((usb_transmit_capture_length + length) > sizeof(usb_transmit_capture))
    {
        return USB_CDC_TRANSMIT_ERROR;
    }

    memcpy(&usb_transmit_capture[usb_transmit_capture_length], data, length);
    usb_transmit_capture_length += length;
    return USB_CDC_TRANSMIT_OK;
}

/**
 * @brief Captures one structured firmware probe line for host assertions.
 * @param line Null-terminated probe line without CR/LF.
 * @return Zero when the bounded capture accepted the line.
 */
static int test_firmware_probe_emit(const char *line)
{
    size_t line_length = strlen(line);

    if ((firmware_probe_capture_length + line_length + 1U) >=
        sizeof(firmware_probe_capture))
    {
        return -1;
    }
    memcpy(&firmware_probe_capture[firmware_probe_capture_length], line, line_length);
    firmware_probe_capture_length += line_length;
    firmware_probe_capture[firmware_probe_capture_length++] = '\n';
    firmware_probe_capture[firmware_probe_capture_length] = '\0';
    return 0;
}

/**
 * @brief Verifies transition-only structured probe output and command correlation.
 */
static void test_firmware_probe(void)
{
    FirmwareProbeSnapshot snapshot = {0};
    uint32_t output_count;

    memset(firmware_probe_capture, 0, sizeof(firmware_probe_capture));
    firmware_probe_capture_length = 0U;
    snapshot.stage = FIRMWARE_PROBE_STAGE_BOOT;
    snapshot.active_mask = 0x03U;
    firmware_probe_init(test_firmware_probe_emit, &snapshot, 10U);
    expect_integer(strstr(firmware_probe_capture, "event=boot") != NULL,
                   1,
                   "probe announces boot and active motor mask");

    output_count = firmware_probe_get_statistics()->emitted_line_count;
    firmware_probe_observe(&snapshot, 15U);
    expect_integer((int)firmware_probe_get_statistics()->emitted_line_count,
                   (int)output_count,
                   "unchanged probe snapshots do not flood COM7");

    snapshot.stage = FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY;
    snapshot.ready_mask = 0x01U;
    firmware_probe_observe(&snapshot, 20U);
    expect_integer(strstr(firmware_probe_capture,
                          "event=stage old=BOOT new=RANGE_DISCOVERY") != NULL,
                   1,
                   "stage transition is emitted with stable names");
    expect_integer(strstr(firmware_probe_capture,
                          "event=masks active=0x03 ready=0x01 enabled=0x00") != NULL,
                   1,
                   "mask transition identifies the affected stage");

    firmware_probe_record_command("MOVE_JOINTS", 27U, "accepted", 25U);
    expect_integer(strstr(firmware_probe_capture,
                          "event=command cmd_seq=27 name=MOVE_JOINTS result=accepted") != NULL,
                   1,
                   "command probes include a correlation sequence");

    snapshot.stage = FIRMWARE_PROBE_STAGE_FAULT;
    snapshot.fault_code = 4U;
    snapshot.fault_joint = 2U;
    snapshot.bus_off = 1U;
    firmware_probe_observe(&snapshot, 30U);
    expect_integer(strstr(firmware_probe_capture,
                          "event=fault code=4 joint=2 busoff=1") != NULL,
                   1,
                   "fault probe identifies code joint and Bus-Off state");
}

/**
 * @brief Handles transport test lines using the production USB command parser.
 * @param line Complete input line.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Zero when parsing succeeds, otherwise non-zero.
 */
static int test_usb_line_handler(const char *line, char *response, size_t response_capacity)
{
    UsbCommand command;
    UsbCommandStatus status =
        usb_command_process_line(line, &command, response, response_capacity);

    return (status == USB_COMMAND_STATUS_OK) ? 0 : -1;
}

/**
 * @brief Captures a joint-controller CAN transmission.
 * @param frame CAN frame emitted by the controller.
 * @return Zero to model a successful transport submission.
 */
static int test_joint_controller_send(const FdcanClassicFrame *frame)
{
    if (joint_controller_send_count >=
        (sizeof(joint_controller_sent_frames) /
         sizeof(joint_controller_sent_frames[0])))
    {
        return -1;
    }
    joint_controller_last_frame = *frame;
    joint_controller_sent_frames[joint_controller_send_count] = *frame;
    joint_controller_send_count++;
    return 0;
}

/**
 * @brief Captures one dual-motor controller transmission with its simulated time.
 * @param frame CAN frame emitted by the controller.
 * @return Zero on success, or non-zero at the configured failure index.
 */
static int test_dual_motor_send(const FdcanClassicFrame *frame)
{
    if ((dual_motor_fail_at_send_index >= 0) &&
        (dual_motor_send_count == (uint32_t)dual_motor_fail_at_send_index))
    {
        return -1;
    }
    if (dual_motor_send_count >=
        (sizeof(dual_motor_sent_frames) / sizeof(dual_motor_sent_frames[0])))
    {
        return -1;
    }

    dual_motor_sent_frames[dual_motor_send_count] = *frame;
    dual_motor_sent_times_ms[dual_motor_send_count] = dual_motor_current_time_ms;
    dual_motor_send_count++;
    return 0;
}

/**
 * @brief Resets the captured dual-motor transport state.
 */
static void test_dual_motor_reset_transport(void)
{
    memset(dual_motor_sent_frames, 0, sizeof(dual_motor_sent_frames));
    memset(dual_motor_sent_times_ms, 0, sizeof(dual_motor_sent_times_ms));
    dual_motor_send_count = 0U;
    dual_motor_current_time_ms = 0U;
    dual_motor_fail_at_send_index = -1;
}

/**
 * @brief Encodes a symmetric feedback value using the documented S3519 mapping.
 * @param value Floating-point value to encode.
 * @param maximum Positive mapping limit.
 * @param bits Encoded bit width.
 * @return Encoded unsigned value.
 */
static uint32_t test_encode_symmetric(float value, float maximum, uint8_t bits)
{
    float normalized = (value + maximum) / (2.0f * maximum);
    uint32_t encoded_maximum = (1UL << bits) - 1UL;

    if (normalized < 0.0f)
    {
        normalized = 0.0f;
    }
    if (normalized > 1.0f)
    {
        normalized = 1.0f;
    }
    return (uint32_t)(normalized * (float)encoded_maximum + 0.5f);
}

/**
 * @brief Injects one parameter response on the motor's configured Master ID.
 * @param motor_id Motor CAN identifier.
 * @param command_echo Parameter response command marker.
 * @param register_address Register address in the response.
 * @param raw_value Raw four-byte register value.
 * @param time_ms Simulated receive time.
 */
static void test_dual_motor_inject_parameter(uint8_t motor_id,
                                             uint8_t command_echo,
                                             uint8_t register_address,
                                             uint32_t raw_value,
                                             uint32_t time_ms)
{
    FdcanClassicFrame frame = {
        (uint16_t)(0x10U + motor_id),
        8U,
        {0U}
    };

    frame.data[0] = motor_id;
    frame.data[2] = command_echo;
    frame.data[3] = register_address;
    frame.data[4] = (uint8_t)(raw_value & 0xFFU);
    frame.data[5] = (uint8_t)((raw_value >> 8U) & 0xFFU);
    frame.data[6] = (uint8_t)((raw_value >> 16U) & 0xFFU);
    frame.data[7] = (uint8_t)((raw_value >> 24U) & 0xFFU);
    dual_motor_controller_on_can_frame(&frame, time_ms);
}

/**
 * @brief Injects one status feedback frame on an explicitly selected Master ID.
 * @param master_identifier CAN identifier used for the feedback frame.
 * @param motor_id Motor CAN identifier.
 * @param motor_state S3519 state nibble.
 * @param position_rad Output-shaft position in radians.
 * @param velocity_rad_s Output-shaft velocity in radians per second.
 * @param time_ms Simulated receive time.
 */
static void test_dual_motor_inject_feedback_on_master(uint16_t master_identifier,
                                                      uint8_t motor_id,
                                                      uint8_t motor_state,
                                                      float position_rad,
                                                      float velocity_rad_s,
                                                      uint32_t time_ms)
{
    uint32_t encoded_position = test_encode_symmetric(position_rad, 12.5f, 16U);
    uint32_t encoded_velocity = test_encode_symmetric(velocity_rad_s, 30.0f, 12U);
    uint32_t encoded_torque = test_encode_symmetric(0.0f, 10.0f, 12U);
    FdcanClassicFrame frame = {
        master_identifier,
        8U,
        {0U}
    };

    frame.data[0] = (uint8_t)((motor_state << 4U) | motor_id);
    frame.data[1] = (uint8_t)(encoded_position >> 8U);
    frame.data[2] = (uint8_t)encoded_position;
    frame.data[3] = (uint8_t)(encoded_velocity >> 4U);
    frame.data[4] = (uint8_t)((encoded_velocity << 4U) | (encoded_torque >> 8U));
    frame.data[5] = (uint8_t)encoded_torque;
    frame.data[6] = 35U;
    frame.data[7] = 30U;
    dual_motor_controller_on_can_frame(&frame, time_ms);
}

/**
 * @brief Injects one status feedback frame on the motor's matching Master ID.
 * @param motor_id Motor CAN identifier.
 * @param motor_state S3519 state nibble.
 * @param position_rad Output-shaft position in radians.
 * @param velocity_rad_s Output-shaft velocity in radians per second.
 * @param time_ms Simulated receive time.
 */
static void test_dual_motor_inject_feedback(uint8_t motor_id,
                                            uint8_t motor_state,
                                            float position_rad,
                                            float velocity_rad_s,
                                            uint32_t time_ms)
{
    test_dual_motor_inject_feedback_on_master((uint16_t)(0x10U + motor_id),
                                              motor_id,
                                              motor_state,
                                              position_rad,
                                              velocity_rad_s,
                                              time_ms);
}

/**
 * @brief Responds to newly captured startup and mode-command frames.
 * @param first_frame_index First unprocessed captured frame.
 * @param first_position_rad Simulated motor-one position.
 * @param second_position_rad Simulated motor-two position.
 */
static void test_dual_motor_respond_to_frames(uint32_t first_frame_index,
                                              float first_position_rad,
                                              float second_position_rad)
{
    uint32_t frame_index;

    for (frame_index = first_frame_index;
         frame_index < dual_motor_send_count;
         ++frame_index)
    {
        const FdcanClassicFrame *frame = &dual_motor_sent_frames[frame_index];
        uint32_t response_time_ms = dual_motor_sent_times_ms[frame_index];

        if ((frame->identifier == DM_MOTOR_PARAMETER_COMMAND_ID) && (frame->length == 8U))
        {
            uint8_t motor_id = frame->data[0];
            uint8_t command_marker = frame->data[2];
            uint8_t register_address = frame->data[3];
            uint32_t raw_value = 0U;

            if (register_address == DM_MOTOR_REGISTER_CONTROL_MODE)
            {
                raw_value = 2U;
            }
            else
            {
                float float_value = 10.0f;

                if (register_address == DM_MOTOR_REGISTER_POSITION_RANGE)
                {
                    float_value = 12.5f;
                }
                else if (register_address == DM_MOTOR_REGISTER_VELOCITY_RANGE)
                {
                    float_value = 30.0f;
                }
                memcpy(&raw_value, &float_value, sizeof(raw_value));
            }
            test_dual_motor_inject_parameter(motor_id,
                                             command_marker,
                                             register_address,
                                             raw_value,
                                             response_time_ms);
        }
        else if ((frame->identifier == 0x101U) || (frame->identifier == 0x102U))
        {
            uint8_t motor_id = (uint8_t)(frame->identifier - 0x100U);
            float position_rad = (motor_id == 1U) ? first_position_rad : second_position_rad;

            if (frame->data[7] == DM_MOTOR_MODE_COMMAND_DISABLE)
            {
                test_dual_motor_inject_feedback(motor_id,
                                                0U,
                                                position_rad,
                                                0.0f,
                                                response_time_ms);
            }
            else if (frame->data[7] == DM_MOTOR_MODE_COMMAND_ENABLE)
            {
                test_dual_motor_inject_feedback(motor_id,
                                                1U,
                                                position_rad,
                                                0.0f,
                                                response_time_ms);
            }
        }
    }
}

/**
 * @brief Advances the controller until startup reaches READY.
 * @param first_position_rad Simulated motor-one position.
 * @param second_position_rad Simulated motor-two position.
 * @return Simulated time at which READY was observed.
 */
static uint32_t test_dual_motor_complete_startup(float first_position_rad,
                                                 float second_position_rad)
{
    uint32_t time_ms = dual_motor_current_time_ms;
    uint32_t deadline_ms = time_ms + 5000U;

    for (; time_ms <= deadline_ms; time_ms += 5U)
    {
        uint32_t first_frame_index = dual_motor_send_count;

        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 0U);
        test_dual_motor_respond_to_frames(first_frame_index,
                                          first_position_rad,
                                          second_position_rad);
        if (dual_motor_controller_get_state()->stage == DUAL_MOTOR_STAGE_READY)
        {
            return time_ms;
        }
    }
    return UINT32_MAX;
}

/**
 * @brief Advances startup through parameter confirmation but withholds status feedback.
 * @return Simulated time at which REQUESTING_FEEDBACK was entered.
 */
static uint32_t test_dual_motor_complete_configuration_without_feedback(void)
{
    uint32_t time_ms;

    for (time_ms = 0U; time_ms <= 5000U; time_ms += 5U)
    {
        uint32_t first_frame_index = dual_motor_send_count;

        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 0U);
        if (dual_motor_controller_get_state()->stage ==
            DUAL_MOTOR_STAGE_REQUESTING_FEEDBACK)
        {
            return time_ms;
        }
        test_dual_motor_respond_to_frames(first_frame_index, 0.0f, 0.0f);
    }
    return UINT32_MAX;
}

/**
 * @brief Initializes the controller, accepts one debounced press, and reaches MOVING.
 * @param first_position_rad Simulated motor-one start position.
 * @param second_position_rad Simulated motor-two start position.
 * @return Simulated MOVING time, or UINT32_MAX on failure.
 */
static uint32_t test_dual_motor_start_motion(float first_position_rad,
                                             float second_position_rad)
{
    uint32_t time_ms;
    uint32_t press_cycle;

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_startup(first_position_rad,
                                               second_position_rad);
    if (time_ms == UINT32_MAX)
    {
        return UINT32_MAX;
    }

    for (press_cycle = 0U; press_cycle < 5U; ++press_cycle)
    {
        uint32_t first_frame_index = dual_motor_send_count;

        time_ms += 5U;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
        test_dual_motor_respond_to_frames(first_frame_index,
                                          first_position_rad,
                                          second_position_rad);
    }

    time_ms += 5U;
    dual_motor_current_time_ms = time_ms;
    dual_motor_controller_step(time_ms, 1U);
    return (dual_motor_controller_get_state()->stage == DUAL_MOTOR_STAGE_MOVING)
               ? time_ms
               : UINT32_MAX;
}

/**
 * @brief Advances one-motor startup through mode, ranges, and disabled feedback.
 * @param start_time_ms Simulated initialization time.
 * @param position_rad Simulated output-shaft position.
 * @return Simulated time when READY is reached, or UINT32_MAX on timeout.
 */
static uint32_t test_joint_controller_complete_startup(uint32_t start_time_ms,
                                                       float position_rad)
{
    uint32_t time_ms;
    uint32_t deadline_ms = start_time_ms + 5000U;
    uint32_t handled_send_count = joint_controller_send_count;

    for (time_ms = start_time_ms; time_ms <= deadline_ms; time_ms += 5U)
    {
        joint_controller_step(time_ms);
        while (handled_send_count < joint_controller_send_count)
        {
            FdcanClassicFrame frame = joint_controller_last_frame;

            handled_send_count++;
            if (frame.identifier == DM_MOTOR_PARAMETER_COMMAND_ID)
            {
                uint32_t raw_value = 2U;

                if (frame.data[3] != DM_MOTOR_REGISTER_CONTROL_MODE)
                {
                    float parameter_value = 10.0f;

                    if (frame.data[3] == DM_MOTOR_REGISTER_POSITION_RANGE)
                    {
                        parameter_value = 12.5f;
                    }
                    else if (frame.data[3] == DM_MOTOR_REGISTER_VELOCITY_RANGE)
                    {
                        parameter_value = 30.0f;
                    }
                    memcpy(&raw_value, &parameter_value, sizeof(raw_value));
                }
                {
                    FdcanClassicFrame response = {0x11U, 8U, {0U}};

                    response.data[0] = 1U;
                    response.data[2] = frame.data[2];
                    response.data[3] = frame.data[3];
                    response.data[4] = (uint8_t)raw_value;
                    response.data[5] = (uint8_t)(raw_value >> 8U);
                    response.data[6] = (uint8_t)(raw_value >> 16U);
                    response.data[7] = (uint8_t)(raw_value >> 24U);
                    joint_controller_on_can_frame(&response, time_ms);
                }
            }
            else if ((frame.identifier == 0x101U) &&
                     (frame.data[7] == DM_MOTOR_MODE_COMMAND_DISABLE))
            {
                uint32_t encoded_position = test_encode_symmetric(position_rad, 12.5f, 16U);
                uint32_t encoded_velocity = test_encode_symmetric(0.0f, 30.0f, 12U);
                uint32_t encoded_torque = test_encode_symmetric(0.0f, 10.0f, 12U);
                FdcanClassicFrame response = {0x11U, 8U, {0U}};

                response.data[0] = 1U;
                response.data[1] = (uint8_t)(encoded_position >> 8U);
                response.data[2] = (uint8_t)encoded_position;
                response.data[3] = (uint8_t)(encoded_velocity >> 4U);
                response.data[4] = (uint8_t)((encoded_velocity << 4U) |
                                             (encoded_torque >> 8U));
                response.data[5] = (uint8_t)encoded_torque;
                response.data[6] = 35U;
                response.data[7] = 30U;
                joint_controller_on_can_frame(&response, time_ms);
            }
        }
        if (joint_controller_get_state()->stage == JOINT_CONTROLLER_STAGE_READY)
        {
            return time_ms;
        }
    }
    return UINT32_MAX;
}

/**
 * @brief Injects one one-motor joint-controller status frame.
 * @param motor_state S3519 state nibble.
 * @param position_rad Output-shaft position in radians.
 * @param velocity_rad_s Output-shaft velocity in radians per second.
 * @param time_ms Simulated receive time.
 */
static void test_joint_controller_inject_feedback_on_master(
    uint16_t master_identifier,
    uint8_t motor_id,
    uint8_t motor_state,
    float position_rad,
    float velocity_rad_s,
    uint32_t time_ms)
{
    uint32_t encoded_position = test_encode_symmetric(position_rad, 12.5f, 16U);
    uint32_t encoded_velocity = test_encode_symmetric(velocity_rad_s, 30.0f, 12U);
    uint32_t encoded_torque = test_encode_symmetric(0.0f, 10.0f, 12U);
    FdcanClassicFrame response = {master_identifier, 8U, {0U}};

    response.data[0] = (uint8_t)((motor_state << 4U) | motor_id);
    response.data[1] = (uint8_t)(encoded_position >> 8U);
    response.data[2] = (uint8_t)encoded_position;
    response.data[3] = (uint8_t)(encoded_velocity >> 4U);
    response.data[4] = (uint8_t)((encoded_velocity << 4U) |
                                 (encoded_torque >> 8U));
    response.data[5] = (uint8_t)encoded_torque;
    response.data[6] = 35U;
    response.data[7] = 30U;
    joint_controller_on_can_frame(&response, time_ms);
}

/**
 * @brief Injects one status frame for motor one on Master ID 0x11.
 * @param motor_state S3519 state nibble.
 * @param position_rad Output-shaft position in radians.
 * @param velocity_rad_s Output-shaft velocity in radians per second.
 * @param time_ms Simulated receive time.
 */
static void test_joint_controller_inject_feedback(uint8_t motor_state,
                                                  float position_rad,
                                                  float velocity_rad_s,
                                                  uint32_t time_ms)
{
    test_joint_controller_inject_feedback_on_master(0x11U,
                                                     1U,
                                                     motor_state,
                                                     position_rad,
                                                     velocity_rad_s,
                                                     time_ms);
}

/**
 * @brief Injects one raw motor-parameter response on an explicit Master ID.
 * @param master_identifier Standard CAN feedback identifier.
 * @param motor_id Motor identifier encoded in D0/D1.
 * @param command_marker Parameter response marker, 0x33 or 0x55.
 * @param register_address Register echoed by the drive.
 * @param raw_value Raw little-endian register value.
 * @param time_ms Simulated receive time.
 */
static void test_joint_controller_inject_parameter_raw(
    uint16_t master_identifier,
    uint8_t motor_id,
    uint8_t command_marker,
    uint8_t register_address,
    uint32_t raw_value,
    uint32_t time_ms)
{
    FdcanClassicFrame response = {master_identifier, 8U, {0U}};

    response.data[0] = motor_id;
    response.data[2] = command_marker;
    response.data[3] = register_address;
    response.data[4] = (uint8_t)raw_value;
    response.data[5] = (uint8_t)(raw_value >> 8U);
    response.data[6] = (uint8_t)(raw_value >> 16U);
    response.data[7] = (uint8_t)(raw_value >> 24U);
    joint_controller_on_can_frame(&response, time_ms);
}

/**
 * @brief Verifies strict classic CAN length and identifier validation.
 */
static void test_classic_can_codec(void)
{
    uint8_t dlc = 0U;
    uint8_t length = 0U;
    FdcanClassicFrame frame;
    uint8_t payload[8] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U};
    uint8_t current_length;

    for (current_length = 0U; current_length <= 8U; current_length++)
    {
        expect_integer(fdcan_classic_length_to_dlc(current_length, &dlc), FDCAN_CLASSIC_STATUS_OK,
                       "classic length converts to DLC");
        expect_integer(dlc, current_length, "classic DLC equals byte length");
        expect_integer(fdcan_classic_dlc_to_length(dlc, &length), FDCAN_CLASSIC_STATUS_OK,
                       "classic DLC converts to length");
        expect_integer(length, current_length, "classic length round trip");
    }

    expect_integer(fdcan_classic_length_to_dlc(9U, &dlc), FDCAN_CLASSIC_STATUS_INVALID_LENGTH,
                   "classic CAN rejects more than eight bytes");
    expect_integer(fdcan_classic_dlc_to_length(9U, &length), FDCAN_CLASSIC_STATUS_INVALID_DLC,
                   "classic CAN rejects FD DLC");
    expect_integer(fdcan_classic_frame_init(&frame, 0x800U, payload, 8U),
                   FDCAN_CLASSIC_STATUS_INVALID_ID, "classic CAN rejects identifiers above 0x7FF");
    expect_integer(fdcan_classic_frame_init(&frame, 0x105U, payload, 8U),
                   FDCAN_CLASSIC_STATUS_OK, "classic CAN accepts a valid standard frame");
}

/**
 * @brief Verifies the documented S3519 position-speed and mode command frames.
 */
static void test_dm_motor_frame_encoding(void)
{
    static const uint8_t expected_position_speed[8] = {
        0xCFU, 0x0FU, 0x49U, 0x40U, 0x00U, 0x00U, 0x20U, 0x42U
    };
    static const uint8_t expected_enable[8] = {
        0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU
    };
    FdcanClassicFrame frame;

    expect_integer(dm_motor_pack_position_velocity(5U, 3.1415899f, 40.0f, &frame),
                   DM_MOTOR_STATUS_OK, "S3519 position-speed command encodes");
    expect_integer(frame.identifier, 0x105, "S3519 position-speed identifier");
    expect_integer(frame.length, 8, "S3519 position-speed length");
    if (memcmp(frame.data, expected_position_speed, sizeof(expected_position_speed)) != 0)
    {
        uint8_t byte_index;

        printf("FAIL: S3519 documented position-speed byte order: actual=");
        for (byte_index = 0U; byte_index < sizeof(expected_position_speed); ++byte_index)
        {
            printf("%02X%s", frame.data[byte_index],
                   (byte_index + 1U == sizeof(expected_position_speed)) ? "\n" : " ");
        }
        test_failure_count++;
    }

    expect_integer(dm_motor_pack_position_velocity(1U, NAN, 0.5f, &frame),
                   DM_MOTOR_STATUS_INVALID_RANGE,
                   "S3519 position-speed command rejects NaN position");
    expect_integer(dm_motor_pack_position_velocity(1U, 0.0f, INFINITY, &frame),
                   DM_MOTOR_STATUS_INVALID_RANGE,
                   "S3519 position-speed command rejects infinite velocity");

    expect_integer(dm_motor_pack_mode_command(5U, DM_MOTOR_MODE_COMMAND_ENABLE, &frame),
                   DM_MOTOR_STATUS_OK, "S3519 enable command encodes");
    expect_integer(frame.identifier, 0x105, "S3519 enable identifier uses position-mode offset");
    expect_integer(memcmp(frame.data, expected_enable, sizeof(expected_enable)), 0,
                   "S3519 enable payload");
    expect_float(dm_motor_output_to_rotor_angle(2.0f), 38.4f, 0.0001f,
                 "19.2 ratio is telemetry-only multiplication");
}

/**
 * @brief Verifies the non-persistent CTRL_MODE write frame and response decoding.
 */
static void test_dm_motor_parameter_write(void)
{
    static const uint8_t expected_mode_write[8] = {
        0x01U, 0x00U, 0x55U, 0x0AU, 0x02U, 0x00U, 0x00U, 0x00U
    };
    FdcanClassicFrame frame;
    FdcanClassicFrame response = {
        0x11U,
        8U,
        {0x01U, 0x00U, 0x55U, 0x0AU, 0x02U, 0x00U, 0x00U, 0x00U}
    };
    DmMotorParameterResponse decoded_response;

    expect_integer(dm_motor_pack_parameter_write_u32(1U,
                                                     DM_MOTOR_REGISTER_CONTROL_MODE,
                                                     2U,
                                                     &frame),
                   DM_MOTOR_STATUS_OK,
                   "S3519 CTRL_MODE write encodes");
    expect_integer(frame.identifier, 0x7FF, "S3519 parameter write identifier");
    expect_integer(frame.length, 8, "S3519 parameter write length");
    expect_integer(memcmp(frame.data, expected_mode_write, sizeof(expected_mode_write)),
                   0,
                   "S3519 CTRL_MODE write payload");

    expect_integer(dm_motor_decode_parameter_response(&response, &decoded_response),
                   DM_MOTOR_STATUS_OK,
                   "S3519 accepts 0x55 parameter response");
    expect_integer(decoded_response.motor_id, 1, "S3519 response motor identifier");
    expect_integer(decoded_response.register_address,
                   DM_MOTOR_REGISTER_CONTROL_MODE,
                   "S3519 response register");
    expect_integer((int)decoded_response.raw_value, 2, "S3519 response uint32 value");
}

/**
 * @brief Verifies feedback bit-field decoding and strict eight-byte validation.
 */
static void test_dm_motor_feedback_decoding(void)
{
    FdcanClassicFrame frame = {
        0x000U,
        8U,
        {0x21U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 40U, 35U}
    };
    DmMotorRanges ranges = {12.5f, 30.0f, 10.0f};
    DmMotorFeedback feedback;

    expect_integer(dm_motor_decode_feedback(&frame, &ranges, &feedback),
                   DM_MOTOR_STATUS_OK, "S3519 feedback decodes");
    expect_integer(feedback.motor_id, 1, "S3519 motor ID uses low nibble");
    expect_integer(feedback.state, 2, "S3519 state uses high nibble");
    expect_float(feedback.position_rad, 0.00019f, 0.001f, "S3519 midpoint position");
    expect_float(feedback.velocity_rad_s, 0.00733f, 0.02f, "S3519 midpoint velocity");
    expect_float(feedback.torque_nm, 0.00244f, 0.01f, "S3519 midpoint torque");

    frame.length = 7U;
    expect_integer(dm_motor_decode_feedback(&frame, &ranges, &feedback),
                   DM_MOTOR_STATUS_INVALID_LENGTH, "S3519 feedback requires eight bytes");
}

/**
 * @brief Verifies strict parsing of the selected USB CDC command surface.
 */
static void test_usb_command_parser(void)
{
    UsbCommand command;
    char response[128];

    expect_integer(usb_command_process_line("#PING", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "PING parses");
    expect_integer(command.type, USB_COMMAND_TYPE_PING, "PING command type");
    expect_integer(usb_command_is_allowed_in_key_control(command.type),
                   1,
                   "PING is allowed in key-only mode");
    expect_integer(strcmp(response, "ok PONG"), 0, "PING response");

    expect_integer(usb_command_process_line("#ECHO hello motor", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "ECHO parses");
    expect_integer(strcmp(response, "ok hello motor"), 0, "ECHO response");

    expect_integer(usb_command_process_line("#GETSTATE", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "GET_STATE parses");
    expect_integer(usb_command_is_allowed_in_key_control(command.type),
                   1,
                   "GETSTATE is allowed in key-only mode");

    expect_integer(usb_command_process_line("#SELECT 7", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "SELECT parses");
    expect_integer(command.selected_joint, 7, "SELECT joint value");

    expect_integer(usb_command_process_line(">1,2,3,4,5,6,7", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "seven-axis MOVE accepts default speed");
    expect_float(command.speed_percent, 100.0f, 0.0001f, "MOVE default speed is one hundred percent");
    expect_integer(command.move_behavior,
                   USB_MOVE_BEHAVIOR_SEQUENTIAL,
                   "greater-than MOVE selects sequential compatibility behavior");

    expect_integer(usb_command_process_line("&1,2,3,4,5,6,7,25", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "seven-axis MOVE parses");
    expect_integer(command.type, USB_COMMAND_TYPE_MOVE_JOINTS, "MOVE command type");
    expect_integer(command.move_behavior,
                   USB_MOVE_BEHAVIOR_INTERRUPTABLE,
                   "ampersand MOVE selects interruptible compatibility behavior");
    expect_float(command.joint_degrees[6], 7.0f, 0.0001f, "MOVE seventh joint");
    expect_float(command.speed_percent, 25.0f, 0.0001f, "MOVE speed percentage");

    expect_integer(usb_command_process_line(">1,2,3,4,5,6,nan,25", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_INVALID_NUMBER, "MOVE rejects NaN");
    expect_integer(usb_command_process_line(">1,2,3,4,5,6", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_INVALID_ARGUMENT_COUNT, "MOVE requires seven joint values");
    expect_integer(usb_command_process_line("#SELECT 8", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OUT_OF_RANGE, "SELECT rejects joint eight");

    expect_integer(usb_command_process_line("!START", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "START still parses for an explicit rejection");
    expect_integer(usb_command_is_allowed_in_key_control(command.type),
                   0,
                   "START is rejected in key-only mode");
    expect_integer(usb_command_process_line("!HOME", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "HOME parses");
    expect_integer(command.type, USB_COMMAND_TYPE_HOME, "HOME command type");
    expect_integer(usb_command_process_line("#GETENABLE", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "GETENABLE parses");
    expect_integer(command.type, USB_COMMAND_TYPE_GET_ENABLE, "GETENABLE command type");
    expect_integer(usb_command_process_line("#GETCAPS", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "GETCAPS parses");
    expect_integer(command.type, USB_COMMAND_TYPE_GET_CAPABILITIES, "GETCAPS command type");
    expect_integer(usb_command_is_allowed_in_key_control(command.type),
                   0,
                   "GETCAPS is outside the strict key-only query surface");
    expect_integer(usb_command_process_line("#GETDH 7", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "GETDH joint query parses");
    expect_integer(command.type, USB_COMMAND_TYPE_GET_DH, "GETDH command type");
    expect_integer(command.query_joint, 7, "GETDH retains the requested joint");
    expect_integer(usb_command_process_line("#GETCONFIG 1", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_OK, "GETCONFIG joint query parses");
    expect_integer(command.type, USB_COMMAND_TYPE_GET_CONFIG, "GETCONFIG command type");
    expect_integer(usb_command_process_line("@1,2,3,4,5,6", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_UNSUPPORTED_COMMAND,
                   "inverse-kinematics command is explicitly unsupported");
    expect_integer(usb_command_process_line("!RGBON", &command, response, sizeof(response)),
                   USB_COMMAND_STATUS_UNSUPPORTED_COMMAND,
                   "RGB command is explicitly unsupported");
}

/**
 * @brief Verifies fragmented, coalesced, overlong, and busy USB CDC transport behavior.
 */
static void test_usb_cdc_transport(void)
{
    static const uint8_t first_fragment[] = "#PI";
    static const uint8_t second_fragment[] = "NG\r\n#ECHO joined\n";
    uint8_t long_line[USB_CDC_LINE_CAPACITY + 8U];
    uint32_t service_index;

    memset(usb_transmit_capture, 0, sizeof(usb_transmit_capture));
    usb_transmit_capture_length = 0U;
    usb_transmit_busy_cycles = 1U;
    usb_cdc_transport_init(test_usb_transmit, test_usb_line_handler);

    expect_integer(usb_cdc_transport_receive(first_fragment, sizeof(first_fragment) - 1U),
                   USB_CDC_TRANSPORT_STATUS_OK, "USB accepts first command fragment");
    expect_integer(usb_cdc_transport_receive(second_fragment, sizeof(second_fragment) - 1U),
                   USB_CDC_TRANSPORT_STATUS_OK, "USB accepts sticky command fragment");
    for (service_index = 0U; service_index < 8U; ++service_index)
    {
        usb_cdc_transport_service();
    }
    usb_transmit_capture[usb_transmit_capture_length] = '\0';
    expect_integer(strcmp(usb_transmit_capture, "ok PONG\r\nok joined\r\n"), 0,
                   "USB reconstructs fragmented and coalesced lines while retrying busy TX");

    usb_transmit_capture_length = 0U;
    memset(usb_transmit_capture, 0, sizeof(usb_transmit_capture));
    expect_integer(usb_cdc_transport_queue_line("probe seq=1 event=test"),
                   USB_CDC_TRANSPORT_STATUS_OK,
                   "task context can enqueue an unsolicited probe line");
    usb_cdc_transport_service();
    usb_transmit_capture[usb_transmit_capture_length] = '\0';
    expect_integer(strcmp(usb_transmit_capture, "probe seq=1 event=test\r\n"),
                   0,
                   "queued probe line uses the CDC response framing");

    memset(long_line, 'A', sizeof(long_line));
    long_line[sizeof(long_line) - 1U] = '\n';
    expect_integer(usb_cdc_transport_receive(long_line, sizeof(long_line)),
                   USB_CDC_TRANSPORT_STATUS_OK, "USB buffers overlong input for rejection");
    usb_cdc_transport_service();
    usb_cdc_transport_service();
    expect_integer(usb_cdc_transport_get_statistics()->overlong_line_count, 1,
                   "USB counts and rejects overlong lines");
}

/**
 * @brief Verifies a common-progress trajectory reaches all moving joints together without exceeding limits.
 */
static void test_synchronized_trajectory(void)
{
    const float start_degrees[SYNC_TRAJECTORY_JOINT_COUNT] = {0.0f, 5.0f, -2.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    const float target_degrees[SYNC_TRAJECTORY_JOINT_COUNT] = {30.0f, -5.0f, 8.0f, 0.0f, 6.0f, -15.0f, 3.0f};
    const float maximum_velocity[SYNC_TRAJECTORY_JOINT_COUNT] = {20.0f, 15.0f, 10.0f, 8.0f, 12.0f, 18.0f, 5.0f};
    const float maximum_acceleration[SYNC_TRAJECTORY_JOINT_COUNT] = {40.0f, 30.0f, 20.0f, 16.0f, 24.0f, 36.0f, 10.0f};
    SyncTrajectory trajectory;
    float position[SYNC_TRAJECTORY_JOINT_COUNT];
    float velocity[SYNC_TRAJECTORY_JOINT_COUNT];
    float previous_position[SYNC_TRAJECTORY_JOINT_COUNT];
    float previous_measured_velocity[SYNC_TRAJECTORY_JOINT_COUNT] = {0.0f};
    uint32_t step_count = 0U;
    uint8_t joint_index;
    uint8_t complete = 0U;

    expect_integer(sync_trajectory_start(&trajectory, start_degrees, target_degrees,
                                         maximum_velocity, maximum_acceleration, 50.0f),
                   SYNC_TRAJECTORY_STATUS_OK, "synchronized trajectory starts");
    memcpy(previous_position, start_degrees, sizeof(previous_position));

    while (complete == 0U && step_count < 10000U)
    {
        expect_integer(sync_trajectory_step(&trajectory, 0.005f, position, velocity, &complete),
                       SYNC_TRAJECTORY_STATUS_OK, "synchronized trajectory steps");
        for (joint_index = 0U; joint_index < SYNC_TRAJECTORY_JOINT_COUNT; joint_index++)
        {
            float measured_velocity = fabsf((position[joint_index] - previous_position[joint_index]) / 0.005f);
            if (measured_velocity > maximum_velocity[joint_index] * 0.5f + 0.05f)
            {
                printf("FAIL: joint %u exceeded scaled velocity: %.4f\n", joint_index + 1U, measured_velocity);
                test_failure_count++;
            }
            if (fabsf(measured_velocity - previous_measured_velocity[joint_index]) / 0.005f >
                maximum_acceleration[joint_index] * 0.5f + 0.2f)
            {
                printf("FAIL: joint %u exceeded scaled acceleration\n", joint_index + 1U);
                test_failure_count++;
            }
            previous_measured_velocity[joint_index] = measured_velocity;
        }
        memcpy(previous_position, position, sizeof(previous_position));
        step_count++;
    }

    expect_integer(complete, 1, "synchronized trajectory completes");
    for (joint_index = 0U; joint_index < SYNC_TRAJECTORY_JOINT_COUNT; joint_index++)
    {
        expect_float(position[joint_index], target_degrees[joint_index], 0.001f,
                     "all joints finish at their target on the same step");
    }
    expect_float(position[3], 0.0f, 0.0001f, "zero-displacement joint remains fixed");
}

/**
 * @brief Verifies one-percent motion is slower than one-hundred-percent motion.
 */
static void test_trajectory_speed_scaling(void)
{
    const float start_degrees[SYNC_TRAJECTORY_JOINT_COUNT] = {0.0f};
    const float target_degrees[SYNC_TRAJECTORY_JOINT_COUNT] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    const float maximum_velocity[SYNC_TRAJECTORY_JOINT_COUNT] = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};
    const float maximum_acceleration[SYNC_TRAJECTORY_JOINT_COUNT] = {20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f, 20.0f};
    SyncTrajectory trajectory;
    float position[SYNC_TRAJECTORY_JOINT_COUNT];
    float velocity[SYNC_TRAJECTORY_JOINT_COUNT];
    uint8_t complete = 0U;
    uint32_t full_speed_steps = 0U;
    uint32_t one_percent_steps = 0U;

    expect_integer(sync_trajectory_start(&trajectory, start_degrees, target_degrees,
                                         maximum_velocity, maximum_acceleration, 100.0f),
                   SYNC_TRAJECTORY_STATUS_OK, "one-hundred-percent trajectory starts");
    while ((complete == 0U) && (full_speed_steps < 10000U))
    {
        (void)sync_trajectory_step(&trajectory, 0.005f, position, velocity, &complete);
        full_speed_steps++;
    }

    complete = 0U;
    expect_integer(sync_trajectory_start(&trajectory, start_degrees, target_degrees,
                                         maximum_velocity, maximum_acceleration, 1.0f),
                   SYNC_TRAJECTORY_STATUS_OK, "one-percent trajectory starts");
    while ((complete == 0U) && (one_percent_steps < 100000U))
    {
        (void)sync_trajectory_step(&trajectory, 0.005f, position, velocity, &complete);
        one_percent_steps++;
    }
    if (one_percent_steps <= full_speed_steps)
    {
        printf("FAIL: one-percent speed must take longer: slow=%lu fast=%lu\n",
               (unsigned long)one_percent_steps, (unsigned long)full_speed_steps);
        test_failure_count++;
    }
}

/**
 * @brief Verifies range discovery, commissioning lock, output units, and safety shutdowns.
 */
static void test_joint_controller_safety(void)
{
    UsbCommand command;
    char response[192];
    float encoded_position;

    joint_controller_test_configuration = *robot_config_get();
    joint_controller_test_configuration.active_joint_mask = 0x01U;
    joint_controller_test_configuration.joint[0].commissioned = 0U;
    joint_controller_send_count = 0U;
    joint_controller_init(test_joint_controller_send,
                          &joint_controller_test_configuration,
                          100U);

    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_START;
    expect_integer(joint_controller_submit(&command, 100U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_INVALID_SELECTION,
                   "commissioning requires an explicit selected joint");

    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_SELECT;
    command.selected_joint = 1U;
    expect_integer(joint_controller_submit(&command, 100U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "commissioning selects one joint");

    command.type = USB_COMMAND_TYPE_START;
    expect_integer(joint_controller_submit(&command, 100U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_RANGES_NOT_READY,
                   "motor cannot enable before runtime ranges are read");

    expect_integer(test_joint_controller_complete_startup(100U, 0.0f) != UINT32_MAX,
                   1,
                   "one-motor startup reaches READY");
    expect_integer(joint_controller_submit(&command, 2300U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "selected motor enables after range discovery");
    expect_integer(joint_controller_last_frame.data[7], DM_MOTOR_MODE_COMMAND_ENABLE,
                   "enable frame reaches selected motor");
    test_joint_controller_inject_feedback(1U, 0.0f, 0.0f, 2301U);

    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_MOVE_JOINTS;
    command.joint_degrees[0] = 2.0f;
    command.speed_percent = 100.0f;
    expect_integer(joint_controller_submit(&command, 2302U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "commissioning accepts a safe single-axis move");
    joint_controller_step(2307U);
    memcpy(&encoded_position, joint_controller_last_frame.data, sizeof(encoded_position));
    if (!(encoded_position >= 0.0f && encoded_position <= (2.0f * 3.1415927f / 180.0f)))
    {
        printf("FAIL: CAN position must remain an output-shaft radian command: %.7f\n",
               encoded_position);
        test_failure_count++;
    }

    joint_controller_step(2352U);
    expect_integer(joint_controller_get_state()->enabled_mask, 0,
                   "missing feedback disables the motor after fifty milliseconds");
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_FEEDBACK_TIMEOUT,
                   "feedback timeout reason is latched");

    joint_controller_init(test_joint_controller_send,
                          &joint_controller_test_configuration,
                          200U);
    expect_integer(test_joint_controller_complete_startup(200U, 0.0f) != UINT32_MAX,
                   1,
                   "invalid-command setup reaches READY");
    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_SELECT;
    command.selected_joint = 1U;
    expect_integer(joint_controller_submit(&command, 2399U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "invalid-command fixture selects motor one");
    command.type = USB_COMMAND_TYPE_START;
    expect_integer(joint_controller_submit(&command, 2400U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "selected motor re-enables for invalid-command test");
    test_joint_controller_inject_feedback(1U, 0.0f, 0.0f, 2401U);
    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_MOVE_JOINTS;
    command.joint_degrees[0] = 4.0f;
    command.speed_percent = 100.0f;
    expect_integer(joint_controller_submit(&command, 2402U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_COMMISSIONING_LIMIT,
                   "commissioning rejects more than three degrees from enable position");
    expect_integer(joint_controller_get_state()->enabled_mask, 0,
                   "invalid commissioning command immediately disables the motor");
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_INVALID_COMMAND,
                   "invalid command reason is latched");
    command.type = USB_COMMAND_TYPE_DISABLE;
    expect_integer(joint_controller_submit(&command, 2403U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "DISABLE remains available after a fault");
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_INVALID_COMMAND,
                   "DISABLE does not clear a latched fault without reset");

    joint_controller_init(test_joint_controller_send,
                          &joint_controller_test_configuration,
                          300U);
    joint_controller_on_bus_off();
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_BUS_OFF, "Bus-Off latches the safety state");

    joint_controller_init(test_joint_controller_send,
                          &joint_controller_test_configuration,
                          400U);
    expect_integer(test_joint_controller_complete_startup(400U, 0.0f) != UINT32_MAX,
                   1,
                   "command-timeout setup reaches READY");
    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_SELECT;
    command.selected_joint = 1U;
    expect_integer(joint_controller_submit(&command, 2599U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "command-timeout fixture selects motor one");
    command.type = USB_COMMAND_TYPE_START;
    expect_integer(joint_controller_submit(&command, 2600U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "selected motor enables for command-timeout test");
    test_joint_controller_inject_feedback(1U, 0.0f, 0.0f, 2601U);
    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_MOVE_JOINTS;
    command.joint_degrees[0] = 2.0f;
    command.speed_percent = 100.0f;
    expect_integer(joint_controller_submit(&command, 2602U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "command-timeout setup starts one trajectory");
    joint_controller_step(32603U);
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_COMMAND_TIMEOUT,
                   "thirty-second movement timeout is enforced before feedback timeout");

    joint_controller_init(test_joint_controller_send,
                          &joint_controller_test_configuration,
                          20000U);
    expect_integer(test_joint_controller_complete_startup(20000U, 0.0f) != UINT32_MAX,
                   1,
                   "motor-fault setup reaches READY");
    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_SELECT;
    command.selected_joint = 1U;
    expect_integer(joint_controller_submit(&command, 22199U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "motor-fault fixture selects motor one");
    command.type = USB_COMMAND_TYPE_START;
    expect_integer(joint_controller_submit(&command, 22200U, response, sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK, "selected motor enables for fault test");
    {
        FdcanClassicFrame fault_frame = {
            0x11U, 8U, {0x81U, 0x80U, 0x00U, 0x80U, 0x08U, 0x00U, 40U, 35U}
        };

        joint_controller_on_can_frame(&fault_frame, 22201U);
    }
    expect_integer(joint_controller_get_state()->enabled_mask, 0,
                   "motor fault immediately stops software control");
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_MOTOR_FAULT,
                   "motor fault reason is latched");
}

/**
 * @brief Verifies the two connected motors complete startup and move only after enable feedback.
 */
static void test_joint_controller_two_motor_bench_profile(void)
{
    RobotConfiguration test_configuration = *robot_config_get();
    const RobotConfiguration *configuration = &test_configuration;
    UsbCommand command;
    char response[320];
    uint32_t processed_frame_count = 0U;
    uint32_t time_ms;
    uint32_t ready_time_ms = UINT32_MAX;
    uint8_t wrong_master_checked = 0U;
    float first_position_command_rad;
    float second_position_command_rad;
    float first_velocity_limit_rad_s;
    float second_velocity_limit_rad_s;

    test_configuration.joint[0].commissioned = 1U;
    test_configuration.joint[1].commissioned = 1U;

    memset(joint_controller_sent_frames, 0, sizeof(joint_controller_sent_frames));
    joint_controller_send_count = 0U;
    joint_controller_init(test_joint_controller_send, configuration, 0U);

    for (time_ms = 0U; time_ms <= 5000U; time_ms += 5U)
    {
        joint_controller_step(time_ms);
        while (processed_frame_count < joint_controller_send_count)
        {
            const FdcanClassicFrame *request =
                &joint_controller_sent_frames[processed_frame_count++];

            if (request->identifier == DM_MOTOR_PARAMETER_COMMAND_ID)
            {
                uint8_t motor_id = request->data[0];
                uint8_t register_address = request->data[3];
                uint32_t raw_value = 2U;

                if (register_address != DM_MOTOR_REGISTER_CONTROL_MODE)
                {
                    float parameter_value = 10.0f;

                    if (register_address == DM_MOTOR_REGISTER_POSITION_RANGE)
                    {
                        parameter_value = 12.5f;
                    }
                    else if (register_address == DM_MOTOR_REGISTER_VELOCITY_RANGE)
                    {
                        parameter_value = 30.0f;
                    }
                    memcpy(&raw_value, &parameter_value, sizeof(raw_value));
                }
                if (wrong_master_checked == 0U)
                {
                    test_joint_controller_inject_parameter_raw(0x12U,
                                                               motor_id,
                                                               request->data[2],
                                                               register_address,
                                                               raw_value,
                                                               time_ms);
                    expect_integer(joint_controller_get_state()->mode_ready_mask,
                                   0,
                                   "wrong Master ID cannot satisfy motor-one startup");
                    wrong_master_checked = 1U;
                }
                test_joint_controller_inject_parameter_raw(
                    configuration->joint[motor_id - 1U].master_id,
                    motor_id,
                    request->data[2],
                    register_address,
                    raw_value,
                    time_ms);
            }
            else if ((request->identifier == 0x101U) ||
                     (request->identifier == 0x102U))
            {
                uint8_t motor_id = (uint8_t)(request->identifier - 0x100U);

                if (request->data[7] == DM_MOTOR_MODE_COMMAND_DISABLE)
                {
                    test_joint_controller_inject_feedback_on_master(
                        configuration->joint[motor_id - 1U].master_id,
                        motor_id,
                        0U,
                        0.0f,
                        0.0f,
                        time_ms);
                }
            }
        }
        if (joint_controller_get_state()->stage == JOINT_CONTROLLER_STAGE_READY)
        {
            ready_time_ms = time_ms;
            break;
        }
    }

    expect_integer(ready_time_ms != UINT32_MAX, 1, "two-motor startup reaches READY");
    expect_integer(joint_controller_get_state()->mode_ready_mask,
                   0x03,
                   "both motors confirm position-velocity mode");
    expect_integer(joint_controller_get_state()->ranges_ready_mask,
                   0x03,
                   "both motors provide PMAX VMAX and TMAX");
    expect_integer(joint_controller_get_state()->feedback_ready_mask,
                   0x03,
                   "both motors provide disabled status feedback");

    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_START;
    processed_frame_count = joint_controller_send_count;
    expect_integer(joint_controller_submit(&command,
                                           ready_time_ms + 1U,
                                           response,
                                           sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "START submits both enable frames");
    expect_integer(joint_controller_send_count - processed_frame_count,
                   2,
                   "both enable frames share one service boundary");
    expect_integer(joint_controller_sent_frames[processed_frame_count].identifier,
                   0x101,
                   "first enable targets motor one");
    expect_integer(joint_controller_sent_frames[processed_frame_count + 1U].identifier,
                   0x102,
                   "second enable targets motor two");
    expect_integer(joint_controller_get_state()->enabled_mask,
                   0,
                   "enable requests do not count as confirmed feedback");
    expect_integer(joint_controller_get_state()->enable_pending_mask,
                   0x03,
                   "both enable acknowledgements are pending");

    memset(&command, 0, sizeof(command));
    command.type = USB_COMMAND_TYPE_MOVE_JOINTS;
    command.joint_degrees[0] = 10.0f;
    command.joint_degrees[1] = -10.0f;
    command.speed_percent = 100.0f;
    expect_integer(joint_controller_submit(&command,
                                           ready_time_ms + 2U,
                                           response,
                                           sizeof(response)),
                   JOINT_CONTROLLER_STATUS_NOT_ENABLED,
                   "move is blocked until both enable states are confirmed");

    test_joint_controller_inject_feedback_on_master(0x11U,
                                                     1U,
                                                     1U,
                                                     0.0f,
                                                     0.0f,
                                                     ready_time_ms + 3U);
    test_joint_controller_inject_feedback_on_master(0x12U,
                                                     2U,
                                                     1U,
                                                     0.0f,
                                                     0.0f,
                                                     ready_time_ms + 3U);
    expect_integer(joint_controller_get_state()->enabled_mask,
                   0x03,
                   "both state-one feedback frames confirm enable");

    expect_integer(joint_controller_submit(&command,
                                           ready_time_ms + 4U,
                                           response,
                                           sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "two-motor joint move starts after enable confirmation");
    processed_frame_count = joint_controller_send_count;
    joint_controller_step(ready_time_ms + 9U);
    expect_integer(joint_controller_send_count - processed_frame_count,
                   2,
                   "one 5 ms control cycle sends both position-speed frames");
    expect_integer(joint_controller_sent_frames[processed_frame_count].identifier,
                   0x101,
                   "first control frame targets motor one");
    expect_integer(joint_controller_sent_frames[processed_frame_count + 1U].identifier,
                   0x102,
                   "second control frame targets motor two");
    memcpy(&first_position_command_rad,
           joint_controller_sent_frames[processed_frame_count].data,
           sizeof(first_position_command_rad));
    memcpy(&first_velocity_limit_rad_s,
           &joint_controller_sent_frames[processed_frame_count].data[4],
           sizeof(first_velocity_limit_rad_s));
    memcpy(&second_position_command_rad,
           joint_controller_sent_frames[processed_frame_count + 1U].data,
           sizeof(second_position_command_rad));
    memcpy(&second_velocity_limit_rad_s,
           &joint_controller_sent_frames[processed_frame_count + 1U].data[4],
           sizeof(second_velocity_limit_rad_s));
    expect_integer(first_position_command_rad > second_position_command_rad,
                   1,
                   "opposite joint targets produce ordered signed positions");
    expect_integer(first_velocity_limit_rad_s >= 0.0f,
                   1,
                   "motor-one velocity limit is non-negative");
    expect_integer(second_velocity_limit_rad_s >= 0.0f,
                   1,
                   "motor-two velocity limit is non-negative");
    expect_integer(first_velocity_limit_rad_s <= 0.5001f,
                   1,
                   "motor-one command respects the 0.5 rad/s bench limit");
    expect_integer(second_velocity_limit_rad_s <= 0.5001f,
                   1,
                   "motor-two command respects the 0.5 rad/s bench limit");
    expect_integer(joint_controller_submit(&command,
                                           ready_time_ms + 10U,
                                           response,
                                           sizeof(response)),
                   JOINT_CONTROLLER_STATUS_COMMAND_BUSY,
                   "sequential prefix cannot replace an active trajectory");
    command.move_behavior = USB_MOVE_BEHAVIOR_INTERRUPTABLE;
    expect_integer(joint_controller_submit(&command,
                                           ready_time_ms + 10U,
                                           response,
                                           sizeof(response)),
                   JOINT_CONTROLLER_STATUS_OK,
                   "interruptible prefix replaces an active trajectory");
}

/**
 * @brief Verifies startup retries and an explicit CTRL_MODE mismatch fault.
 */
static void test_joint_controller_startup_faults(void)
{
    RobotConfiguration configuration = *robot_config_get();

    configuration.active_joint_mask = 0x01U;
    joint_controller_send_count = 0U;
    joint_controller_init(test_joint_controller_send, &configuration, 0U);
    joint_controller_step(2000U);
    joint_controller_step(2100U);
    joint_controller_step(2200U);
    joint_controller_step(2300U);
    joint_controller_step(2400U);
    expect_integer(joint_controller_send_count,
                   4,
                   "startup performs one request and three retries");
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_STARTUP_TIMEOUT,
                   "missing startup response latches a timeout");
    expect_integer(joint_controller_get_state()->stage,
                   JOINT_CONTROLLER_STAGE_FAULT,
                   "startup timeout enters FAULT stage");

    joint_controller_send_count = 0U;
    joint_controller_init(test_joint_controller_send, &configuration, 0U);
    joint_controller_step(2000U);
    test_joint_controller_inject_parameter_raw(0x11U,
                                               1U,
                                               0x55U,
                                               DM_MOTOR_REGISTER_CONTROL_MODE,
                                               3U,
                                               2001U);
    expect_integer(joint_controller_get_state()->safety_reason,
                   JOINT_CONTROLLER_SAFETY_MODE_MISMATCH,
                   "CTRL_MODE other than two latches a mismatch");
}

/**
 * @brief Verifies startup configuration, key debounce, one-turn motion, and hold behavior.
 */
static void test_dual_motor_key_motion(void)
{
    const float first_start_position_rad = 1.0f;
    const float second_start_position_rad = -0.5f;
    const float two_pi = 6.2831853071795864769f;
    uint32_t time_ms;
    uint32_t first_frame_index;
    uint32_t scan_index;
    uint32_t position_frame_count = 0U;
    uint8_t mode_write_mask = 0U;
    uint8_t mode_read_mask = 0U;
    uint32_t press_deadline_ms;
    uint32_t enable_deadline_ms;
    float first_target_rad;
    float second_target_rad;

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_startup(first_start_position_rad,
                                               second_start_position_rad);
    expect_integer(time_ms != UINT32_MAX, 1, "dual-motor startup reaches READY");
    expect_integer(dual_motor_controller_get_state()->mode_ready_mask,
                   0x03,
                   "both motors confirm position-speed mode");
    expect_integer(dual_motor_controller_get_state()->ranges_ready_mask,
                   0x03,
                   "both motors report PMAX VMAX and TMAX");
    expect_integer(dual_motor_controller_get_state()->fresh_feedback_mask,
                   0x03,
                   "both motors provide disabled startup feedback");
    for (scan_index = 0U; scan_index < dual_motor_send_count; ++scan_index)
    {
        const FdcanClassicFrame *startup_frame = &dual_motor_sent_frames[scan_index];

        if ((startup_frame->identifier == DM_MOTOR_PARAMETER_COMMAND_ID) &&
            (startup_frame->data[3] == DM_MOTOR_REGISTER_CONTROL_MODE) &&
            (startup_frame->data[0] >= 1U) && (startup_frame->data[0] <= 2U))
        {
            uint8_t motor_bit = (uint8_t)(1U << (startup_frame->data[0] - 1U));

            if (startup_frame->data[2] == 0x55U)
            {
                mode_write_mask |= motor_bit;
            }
            if (startup_frame->data[2] == 0x33U)
            {
                mode_read_mask |= motor_bit;
            }
        }
    }
    expect_integer(mode_write_mask, 0x03, "startup writes CTRL_MODE for both motors");
    expect_integer(mode_read_mask, 0x03, "startup reads CTRL_MODE back for both motors");

    first_frame_index = dual_motor_send_count;
    dual_motor_current_time_ms = time_ms + 5U;
    dual_motor_controller_step(time_ms + 5U, 1U);
    test_dual_motor_respond_to_frames(first_frame_index,
                                      first_start_position_rad,
                                      second_start_position_rad);
    first_frame_index = dual_motor_send_count;
    dual_motor_current_time_ms = time_ms + 10U;
    dual_motor_controller_step(time_ms + 10U, 0U);
    test_dual_motor_respond_to_frames(first_frame_index,
                                      first_start_position_rad,
                                      second_start_position_rad);
    expect_integer(dual_motor_controller_get_state()->move_accepted,
                   0,
                   "five-millisecond key bounce is ignored");

    time_ms += 15U;
    press_deadline_ms = time_ms + 20U;
    for (; time_ms <= press_deadline_ms; time_ms += 5U)
    {
        first_frame_index = dual_motor_send_count;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
        test_dual_motor_respond_to_frames(first_frame_index,
                                          first_start_position_rad,
                                          second_start_position_rad);
    }
    expect_integer(dual_motor_controller_get_state()->move_accepted,
                   1,
                   "twenty-millisecond press edge accepts exactly one move");
    first_target_rad = dual_motor_controller_get_state()->target_position_rad[0];
    second_target_rad = dual_motor_controller_get_state()->target_position_rad[1];
    expect_float(first_target_rad -
                     dual_motor_controller_get_state()->initial_position_rad[0],
                 two_pi,
                 0.001f,
                 "motor one target adds one output-shaft revolution");
    expect_float(second_target_rad -
                     dual_motor_controller_get_state()->initial_position_rad[1],
                 two_pi,
                 0.001f,
                 "motor two target adds one output-shaft revolution");

    enable_deadline_ms = time_ms + 50U;
    for (; time_ms < enable_deadline_ms; time_ms += 5U)
    {
        first_frame_index = dual_motor_send_count;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
        test_dual_motor_respond_to_frames(first_frame_index,
                                          first_start_position_rad,
                                          second_start_position_rad);
        if (dual_motor_controller_get_state()->stage == DUAL_MOTOR_STAGE_MOVING)
        {
            break;
        }
    }
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_MOVING,
                   "both enable feedbacks start motion");

    first_frame_index = dual_motor_send_count;
    dual_motor_current_time_ms = time_ms + 5U;
    dual_motor_controller_step(time_ms + 5U, 1U);
    for (scan_index = first_frame_index; scan_index < dual_motor_send_count; ++scan_index)
    {
        const FdcanClassicFrame *frame = &dual_motor_sent_frames[scan_index];

        if ((frame->identifier == 0x101U) || (frame->identifier == 0x102U))
        {
            float encoded_velocity_rad_s;

            memcpy(&encoded_velocity_rad_s, &frame->data[4], sizeof(encoded_velocity_rad_s));
            expect_float(encoded_velocity_rad_s,
                         0.5f,
                         0.0001f,
                         "position-speed frame limits speed to 0.5 rad/s");
            position_frame_count++;
        }
    }
    expect_integer((int)position_frame_count,
                   2,
                   "both position-speed frames are emitted in one service cycle");

    for (scan_index = 0U; scan_index < 3U; ++scan_index)
    {
        time_ms += 5U;
        test_dual_motor_inject_feedback(1U, 1U, first_target_rad, 0.0f, time_ms);
        test_dual_motor_inject_feedback(2U, 1U, second_target_rad, 0.0f, time_ms);
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
    }
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_HOLDING,
                   "three settled feedback cycles enter holding");
    expect_integer(dual_motor_controller_get_state()->arrived_mask,
                   0x03,
                   "both motors satisfy the arrival qualification");

    first_frame_index = dual_motor_send_count;
    time_ms += 5U;
    test_dual_motor_inject_feedback(1U, 1U, first_target_rad, 0.0f, time_ms);
    test_dual_motor_inject_feedback(2U, 1U, second_target_rad, 0.0f, time_ms);
    dual_motor_current_time_ms = time_ms;
    dual_motor_controller_step(time_ms, 1U);
    expect_integer((int)(dual_motor_send_count - first_frame_index),
                   2,
                   "HOLDING continues to submit both final targets");
    for (scan_index = first_frame_index; scan_index < dual_motor_send_count; ++scan_index)
    {
        float held_position_rad;
        float expected_position_rad =
            (dual_motor_sent_frames[scan_index].identifier == 0x101U)
                ? first_target_rad
                : second_target_rad;

        memcpy(&held_position_rad,
               &dual_motor_sent_frames[scan_index].data[0],
               sizeof(held_position_rad));
        expect_float(held_position_rad,
                     expected_position_rad,
                     0.0001f,
                     "HOLDING preserves the final absolute target");
    }

    for (scan_index = 0U; scan_index < 5U; ++scan_index)
    {
        time_ms += 5U;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 0U);
    }
    for (scan_index = 0U; scan_index < 5U; ++scan_index)
    {
        time_ms += 5U;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
    }
    expect_float(dual_motor_controller_get_state()->target_position_rad[0],
                 first_target_rad,
                 0.0001f,
                 "later key presses cannot add another revolution");
    expect_integer(dual_motor_controller_get_state()->move_accepted,
                   1,
                   "one-move-per-boot latch remains set");
}

/**
 * @brief Verifies ignored early input, configuration retry failure, and Bus-Off latching.
 */
static void test_dual_motor_startup_and_faults(void)
{
    uint32_t time_ms;
    uint32_t first_frame_index;

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    for (time_ms = 5U; time_ms <= 30U; time_ms += 5U)
    {
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
    }
    for (; time_ms <= 55U; time_ms += 5U)
    {
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 0U);
    }
    time_ms = test_dual_motor_complete_startup(0.0f, 0.0f);
    expect_integer(dual_motor_controller_get_state()->move_accepted,
                   0,
                   "press completed before READY is never replayed");
    first_frame_index = dual_motor_send_count;
    dual_motor_current_time_ms = time_ms + 5U;
    dual_motor_controller_step(time_ms + 5U, 0U);
    expect_integer((int)(dual_motor_send_count - first_frame_index),
                   2,
                   "READY refreshes both disabled feedback channels every service cycle");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    dual_motor_current_time_ms = 2000U;
    dual_motor_controller_step(2000U, 0U);
    dual_motor_current_time_ms = 2100U;
    dual_motor_controller_step(2100U, 0U);
    dual_motor_current_time_ms = 2200U;
    dual_motor_controller_step(2200U, 0U);
    dual_motor_current_time_ms = 2300U;
    dual_motor_controller_step(2300U, 0U);
    expect_integer((int)dual_motor_send_count,
                   4,
                   "one initial mode request permits three retries");
    dual_motor_current_time_ms = 2400U;
    dual_motor_controller_step(2400U, 0U);
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_FAULT,
                   "three unanswered retries latch a startup fault");
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_CONFIGURATION_TIMEOUT,
                   "startup retry exhaustion records configuration timeout");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_configuration_without_feedback();
    expect_integer(time_ms != UINT32_MAX,
                   1,
                   "configuration reaches disabled-feedback acquisition");
    dual_motor_current_time_ms = time_ms + 5U;
    dual_motor_controller_step(time_ms + 5U, 0U);
    dual_motor_current_time_ms = time_ms + 60U;
    dual_motor_controller_step(time_ms + 60U, 0U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT,
                   "missing startup feedback faults after fifty milliseconds");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    dual_motor_controller_on_bus_off();
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_FAULT,
                   "Bus-Off enters the latched fault stage");
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_BUS_OFF,
                   "Bus-Off records the explicit fault reason");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    dual_motor_controller_on_can_start_failure();
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_FAULT,
                   "CAN startup failure enters the diagnostic fault stage");
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_CAN_TRANSMIT,
                   "CAN startup failure reports a transmit fault");
}

/**
 * @brief Verifies invalid values, target bounds, runtime timeouts, motor faults, and TX failure.
 */
static void test_dual_motor_runtime_faults(void)
{
    uint32_t time_ms;
    uint32_t press_cycle;

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    dual_motor_current_time_ms = 2000U;
    dual_motor_controller_step(2000U, 0U);
    test_dual_motor_inject_parameter(1U,
                                     0x55U,
                                     DM_MOTOR_REGISTER_CONTROL_MODE,
                                     3U,
                                     2000U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_INVALID_PARAMETER,
                   "incorrect CTRL_MODE readback latches an invalid-parameter fault");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    dual_motor_current_time_ms = 2000U;
    dual_motor_controller_step(2000U, 0U);
    test_dual_motor_inject_parameter(1U,
                                     0x55U,
                                     DM_MOTOR_REGISTER_CONTROL_MODE,
                                     2U,
                                     2000U);
    dual_motor_current_time_ms = 2005U;
    dual_motor_controller_step(2005U, 0U);
    test_dual_motor_inject_parameter(1U,
                                     0x33U,
                                     DM_MOTOR_REGISTER_CONTROL_MODE,
                                     2U,
                                     2005U);
    dual_motor_current_time_ms = 2010U;
    dual_motor_controller_step(2010U, 0U);
    test_dual_motor_inject_parameter(1U,
                                     0x33U,
                                     DM_MOTOR_REGISTER_POSITION_RANGE,
                                     0x7FC00000UL,
                                     2010U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_INVALID_PARAMETER,
                   "NaN parameter feedback is rejected");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_startup(7.0f, 0.0f);
    for (press_cycle = 0U; press_cycle < 5U; ++press_cycle)
    {
        uint32_t first_frame_index = dual_motor_send_count;

        time_ms += 5U;
        dual_motor_current_time_ms = time_ms;
        dual_motor_controller_step(time_ms, 1U);
        test_dual_motor_respond_to_frames(first_frame_index, 7.0f, 0.0f);
    }
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_TARGET_RANGE,
                   "a one-turn target beyond PMAX is rejected before enable");

    time_ms = test_dual_motor_start_motion(0.0f, 0.0f);
    expect_integer(time_ms != UINT32_MAX, 1, "runtime timeout fixture reaches MOVING");
    dual_motor_current_time_ms = time_ms + 55U;
    dual_motor_controller_step(time_ms + 55U, 1U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_FEEDBACK_TIMEOUT,
                   "moving feedback older than fifty milliseconds faults");

    time_ms = test_dual_motor_start_motion(0.0f, 0.0f);
    expect_integer(time_ms != UINT32_MAX, 1, "movement-timeout fixture reaches MOVING");
    test_dual_motor_inject_feedback(1U, 1U, 0.0f, 0.2f, time_ms + 30001U);
    test_dual_motor_inject_feedback(2U, 1U, 0.0f, 0.2f, time_ms + 30001U);
    dual_motor_current_time_ms = time_ms + 30001U;
    dual_motor_controller_step(time_ms + 30001U, 1U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_MOVEMENT_TIMEOUT,
                   "motion not settled within thirty seconds faults");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_startup(0.0f, 0.0f);
    test_dual_motor_inject_feedback_on_master(0x11U,
                                              2U,
                                              8U,
                                              0.0f,
                                              0.0f,
                                              time_ms + 1U);
    expect_integer(dual_motor_controller_get_state()->stage,
                   DUAL_MOTOR_STAGE_READY,
                   "feedback with a mismatched Master ID and D0 motor ID is ignored");
    test_dual_motor_inject_feedback(2U, 8U, 0.0f, 0.0f, time_ms + 1U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_MOTOR,
                   "S3519 fault-state feedback latches a motor fault");

    test_dual_motor_reset_transport();
    dual_motor_controller_init(test_dual_motor_send, 0U);
    time_ms = test_dual_motor_complete_startup(0.0f, 0.0f);
    dual_motor_fail_at_send_index = (int32_t)dual_motor_send_count;
    dual_motor_current_time_ms = time_ms + 5U;
    dual_motor_controller_step(time_ms + 5U, 0U);
    expect_integer(dual_motor_controller_get_state()->fault_reason,
                   DUAL_MOTOR_FAULT_CAN_TRANSMIT,
                   "CAN submission failure latches a transmit fault");
}

/**
 * @brief Runs all host tests and returns a non-zero exit code on failure.
 * @return Zero when every test passes; otherwise one.
 */
int main(void)
{
    test_robot_configuration();
    test_firmware_probe();
    test_classic_can_codec();
    test_dm_motor_frame_encoding();
    test_dm_motor_parameter_write();
    test_dm_motor_feedback_decoding();
    test_usb_command_parser();
    test_usb_cdc_transport();
    test_synchronized_trajectory();
    test_trajectory_speed_scaling();
    test_joint_controller_safety();
    test_joint_controller_two_motor_bench_profile();
    test_joint_controller_startup_faults();
    test_dual_motor_key_motion();
    test_dual_motor_startup_and_faults();
    test_dual_motor_runtime_faults();

    if (test_failure_count != 0)
    {
        printf("HOST_TESTS_FAILED=%d\n", test_failure_count);
        return 1;
    }

    printf("HOST_TESTS_PASSED\n");
    return 0;
}
