/**
 * @file joint_controller.c
 * @brief Static S3519 seven-joint control, commissioning limits, and safety handling.
 */

#include "joint_controller.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define JOINT_CONTROLLER_PI 3.14159265358979323846F
#define JOINT_CONTROLLER_RAD_TO_DEG (180.0F / JOINT_CONTROLLER_PI)
#define JOINT_CONTROLLER_RANGE_POSITION_BIT 0x01U
#define JOINT_CONTROLLER_RANGE_VELOCITY_BIT 0x02U
#define JOINT_CONTROLLER_RANGE_TORQUE_BIT 0x04U
#define JOINT_CONTROLLER_ALL_RANGES_MASK 0x07U
#define JOINT_CONTROLLER_CONTROL_MODE_VALUE 2U

static JointControllerState joint_state;
static const RobotConfiguration *joint_robot_configuration;
static DmMotorRanges joint_motor_ranges[JOINT_COUNT];
static uint8_t joint_range_component_mask[JOINT_COUNT];
static float joint_enable_reference_degrees[JOINT_COUNT];
static float joint_target_degrees[JOINT_COUNT];
static uint8_t joint_arrival_cycle_count[JOINT_COUNT];
static uint8_t joint_disable_pending_mask;
static uint8_t joint_mode_request_index;
static uint8_t joint_discovery_request_index;
static uint8_t joint_startup_request_waiting;
static uint8_t joint_startup_retry_count;
static uint8_t joint_feedback_request_count;
static uint32_t joint_initialization_time_ms;
static uint32_t joint_startup_request_time_ms;
static uint32_t joint_last_ready_poll_time_ms;
static SyncTrajectory joint_trajectory;
static JointControllerSendFunction joint_send_function;

/**
 * @brief Convert one configured Master ID into a zero-based active joint index.
 * @param master_id Standard CAN feedback identifier.
 * @param joint_index Receives the zero-based joint index.
 * @return One when the Master ID belongs to an active joint, otherwise zero.
 */
static uint8_t joint_controller_master_to_index(uint16_t master_id,
                                                uint8_t *joint_index)
{
    uint8_t index;

    if ((joint_index == NULL) || (joint_robot_configuration == NULL))
    {
        return 0U;
    }
    for (index = 0U; index < JOINT_COUNT; ++index)
    {
        if (((joint_state.active_mask & (uint8_t)(1U << index)) != 0U) &&
            (joint_robot_configuration->joint[index].master_id == master_id))
        {
            *joint_index = index;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief Submit one already-packed frame through the injected CAN transport.
 * @param frame Frame to submit.
 * @return Controller status describing transport success.
 */
static JointControllerStatus joint_controller_send_frame(const FdcanClassicFrame *frame)
{
    if ((joint_send_function == NULL) || (frame == NULL))
    {
        return JOINT_CONTROLLER_STATUS_CAN_ERROR;
    }
    return (joint_send_function(frame) == 0) ?
        JOINT_CONTROLLER_STATUS_OK : JOINT_CONTROLLER_STATUS_CAN_ERROR;
}

/**
 * @brief Send a special mode command to one joint.
 * @param joint_index Zero-based joint index.
 * @param mode_command S3519 special mode command.
 * @return Controller status describing packing and transport success.
 */
static JointControllerStatus joint_controller_send_mode(uint8_t joint_index,
                                                        DmMotorModeCommand mode_command)
{
    FdcanClassicFrame frame;

    if ((joint_index >= JOINT_COUNT) ||
        (dm_motor_pack_mode_command(joint_robot_configuration->joint[joint_index].motor_id,
                                    mode_command,
                                    &frame) != DM_MOTOR_STATUS_OK))
    {
        return JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    return joint_controller_send_frame(&frame);
}

/**
 * @brief Stop active motion and disable every currently enabled motor.
 * @param reason Safety reason to latch.
 * @param can_transmit Non-zero when disable frames may still be submitted.
 */
static void joint_controller_safety_shutdown(JointControllerSafetyReason reason,
                                             uint8_t can_transmit)
{
    uint8_t enabled_mask = (uint8_t)(joint_state.enabled_mask |
                                     joint_state.enable_pending_mask);
    uint8_t joint_index;

    joint_trajectory.active = 0U;
    joint_state.trajectory_active = 0U;
    joint_state.safety_reason = reason;
    joint_state.enabled_mask = 0U;
    joint_state.enable_pending_mask = 0U;
    joint_state.stage = JOINT_CONTROLLER_STAGE_FAULT;

    if (can_transmit == 0U)
    {
        joint_disable_pending_mask = 0U;
        return;
    }

    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        if ((enabled_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            (void)joint_controller_send_mode(joint_index, DM_MOTOR_MODE_COMMAND_DISABLE);
        }
    }
}

/**
 * @brief Format one response while detecting truncation.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @param format printf-compatible format string.
 * @return Controller status describing whether the response fitted.
 */
static JointControllerStatus joint_controller_format_response(char *response,
                                                              size_t response_capacity,
                                                              const char *format,
                                                              ...)
{
    va_list arguments;
    int written_length;

    if ((response == NULL) || (response_capacity == 0U) || (format == NULL))
    {
        return JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    va_start(arguments, format);
    written_length = vsnprintf(response, response_capacity, format, arguments);
    va_end(arguments);
    if ((written_length < 0) || ((size_t)written_length >= response_capacity))
    {
        response[0] = '\0';
        return JOINT_CONTROLLER_STATUS_RESPONSE_TOO_SMALL;
    }
    return JOINT_CONTROLLER_STATUS_OK;
}

/**
 * @brief Determine whether every joint has a complete real commissioning profile.
 * @return One only after every compile-time joint profile is marked commissioned.
 */
static uint8_t joint_controller_all_joints_are_commissioned(void)
{
    return robot_config_active_profiles_are_commissioned(joint_robot_configuration);
}

/**
 * @brief Start the selected motor while the commissioning lock is active.
 * @param time_ms Current monotonic time in milliseconds.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @return Controller status.
 */
static JointControllerStatus joint_controller_start_selected(uint32_t time_ms,
                                                             char *response,
                                                             size_t response_capacity)
{
    uint8_t joint_index;
    uint8_t joint_bit;
    JointControllerStatus send_status;

    if (joint_state.commissioning_lock_active == 0U)
    {
        const uint8_t all_joints_mask = joint_state.active_mask;

        if (joint_state.motor_fault_mask != 0U)
        {
            return JOINT_CONTROLLER_STATUS_SAFETY_LATCHED;
        }
        if ((joint_state.mode_ready_mask & all_joints_mask) != all_joints_mask ||
            (joint_state.ranges_ready_mask & all_joints_mask) != all_joints_mask ||
            (joint_state.feedback_ready_mask & all_joints_mask) != all_joints_mask)
        {
            return JOINT_CONTROLLER_STATUS_RANGES_NOT_READY;
        }
        joint_state.enabled_mask = 0U;
        joint_state.enable_pending_mask = 0U;
        for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
        {
            if ((all_joints_mask & (uint8_t)(1U << joint_index)) == 0U)
            {
                continue;
            }
            send_status = joint_controller_send_mode(joint_index,
                                                     DM_MOTOR_MODE_COMMAND_ENABLE);
            if (send_status != JOINT_CONTROLLER_STATUS_OK)
            {
                joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT,
                                                 1U);
                return send_status;
            }
            joint_state.enable_pending_mask |= (uint8_t)(1U << joint_index);
            joint_enable_reference_degrees[joint_index] =
                joint_state.joint_position_degrees[joint_index];
            joint_state.last_feedback_time_ms[joint_index] = time_ms;
        }
        joint_state.last_command_time_ms = time_ms;
        joint_state.stage = JOINT_CONTROLLER_STAGE_ENABLING;
        return joint_controller_format_response(response,
                                                response_capacity,
                                                "ok enabled=0x%02X",
                                                (unsigned int)all_joints_mask);
    }

    if ((joint_state.selected_joint < 1U) || (joint_state.selected_joint > JOINT_COUNT))
    {
        return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
    }
    joint_index = (uint8_t)(joint_state.selected_joint - 1U);
    joint_bit = (uint8_t)(1U << joint_index);
    if ((joint_state.active_mask & joint_bit) == 0U)
    {
        return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
    }
    if ((joint_state.motor_fault_mask & joint_bit) != 0U)
    {
        return JOINT_CONTROLLER_STATUS_SAFETY_LATCHED;
    }
    if (((joint_state.mode_ready_mask & joint_bit) == 0U) ||
        ((joint_state.ranges_ready_mask & joint_bit) == 0U) ||
        ((joint_state.feedback_ready_mask & joint_bit) == 0U))
    {
        return JOINT_CONTROLLER_STATUS_RANGES_NOT_READY;
    }

    send_status = joint_controller_send_mode(joint_index, DM_MOTOR_MODE_COMMAND_ENABLE);
    if (send_status != JOINT_CONTROLLER_STATUS_OK)
    {
        joint_state.safety_reason = JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT;
        return send_status;
    }

    joint_state.enabled_mask = 0U;
    joint_state.enable_pending_mask = joint_bit;
    joint_enable_reference_degrees[joint_index] =
        joint_state.joint_position_degrees[joint_index];
    joint_state.last_feedback_time_ms[joint_index] = time_ms;
    joint_state.last_command_time_ms = time_ms;
    joint_state.stage = JOINT_CONTROLLER_STAGE_ENABLING;
    return joint_controller_format_response(response,
                                            response_capacity,
                                            "ok enabled=%u",
                                            (unsigned int)joint_state.selected_joint);
}

/**
 * @brief Validate and start a synchronized target command.
 * @param command Parsed seven-joint move command.
 * @param time_ms Current monotonic time in milliseconds.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @return Controller status.
 */
static JointControllerStatus joint_controller_start_move(const UsbCommand *command,
                                                         uint32_t time_ms,
                                                         char *response,
                                                         size_t response_capacity)
{
    float target_degrees[JOINT_COUNT];
    float maximum_velocity[JOINT_COUNT];
    float maximum_acceleration[JOINT_COUNT];
    uint8_t joint_index;

    if (joint_state.enabled_mask == 0U)
    {
        return JOINT_CONTROLLER_STATUS_NOT_ENABLED;
    }
    if ((joint_state.trajectory_active != 0U) &&
        (command->move_behavior == USB_MOVE_BEHAVIOR_SEQUENTIAL))
    {
        return JOINT_CONTROLLER_STATUS_COMMAND_BUSY;
    }

    memcpy(target_degrees,
           joint_target_degrees,
           sizeof(target_degrees));
    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        maximum_velocity[joint_index] =
            joint_robot_configuration->joint[joint_index].maximum_velocity_degrees_s;
        maximum_acceleration[joint_index] =
            joint_robot_configuration->joint[joint_index].maximum_acceleration_degrees_s2;
    }

    if (joint_state.commissioning_lock_active != 0U)
    {
        uint8_t selected_index = (uint8_t)(joint_state.selected_joint - 1U);
        float selected_target = command->joint_degrees[selected_index];

        if (fabsf(selected_target - joint_enable_reference_degrees[selected_index]) >
            JOINT_CONTROLLER_COMMISSIONING_POSITION_LIMIT_DEG)
        {
            joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
            return JOINT_CONTROLLER_STATUS_COMMISSIONING_LIMIT;
        }
        target_degrees[selected_index] = selected_target;
        maximum_velocity[selected_index] = JOINT_CONTROLLER_COMMISSIONING_VELOCITY_DEG_S;
        maximum_acceleration[selected_index] = JOINT_CONTROLLER_COMMISSIONING_ACCELERATION_DEG_S2;
    }
    else
    {
        memcpy(target_degrees, command->joint_degrees, sizeof(target_degrees));
        for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
        {
            if ((target_degrees[joint_index] <
                 joint_robot_configuration->joint[joint_index].minimum_degrees) ||
                (target_degrees[joint_index] >
                 joint_robot_configuration->joint[joint_index].maximum_degrees))
            {
                joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
                return JOINT_CONTROLLER_STATUS_COMMISSIONING_LIMIT;
            }
        }
    }

    if (sync_trajectory_start(&joint_trajectory,
                              joint_state.commanded_position_degrees,
                              target_degrees,
                              maximum_velocity,
                              maximum_acceleration,
                              command->speed_percent) != SYNC_TRAJECTORY_STATUS_OK)
    {
        return JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }

    memset(joint_arrival_cycle_count, 0, sizeof(joint_arrival_cycle_count));
    memset(joint_state.first_arrival_time_ms, 0, sizeof(joint_state.first_arrival_time_ms));
    memcpy(joint_target_degrees, target_degrees, sizeof(joint_target_degrees));
    joint_state.trajectory_active = joint_trajectory.active;
    joint_state.last_command_time_ms = time_ms;
    joint_state.stage = (joint_trajectory.active != 0U)
                            ? JOINT_CONTROLLER_STAGE_MOVING
                            : JOINT_CONTROLLER_STAGE_HOLDING;
    return joint_controller_format_response(response,
                                            response_capacity,
                                            "ok move speed=%.1f",
                                            (double)command->speed_percent);
}

/**
 * @brief Starts a synchronized move to the calibrated zero-angle home pose.
 * @param time_ms Current monotonic time in milliseconds.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @return Controller status from the regular joint-move path.
 */
static JointControllerStatus joint_controller_start_home(uint32_t time_ms,
                                                         char *response,
                                                         size_t response_capacity)
{
    UsbCommand home_command;
    uint8_t joint_index;

    memset(&home_command, 0, sizeof(home_command));
    home_command.type = USB_COMMAND_TYPE_MOVE_JOINTS;
    home_command.speed_percent = 20.0F;
    home_command.move_behavior = USB_MOVE_BEHAVIOR_SEQUENTIAL;
    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        home_command.joint_degrees[joint_index] =
            joint_robot_configuration->dh[joint_index].theta_offset_degrees;
    }
    return joint_controller_start_move(&home_command,
                                       time_ms,
                                       response,
                                       response_capacity);
}

/**
 * @brief Formats one requested DH row or a compact DH capability summary.
 * @param query_joint One-based joint number, or zero for the summary.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @return Controller response status.
 */
static JointControllerStatus joint_controller_format_dh(uint8_t query_joint,
                                                        char *response,
                                                        size_t response_capacity)
{
    if (query_joint == 0U)
    {
        return joint_controller_format_response(response,
                                                response_capacity,
                                                "ok dh joints=7 valid=%u query=#GETDH_<1-7>",
                                                (unsigned int)joint_robot_configuration->dh_parameters_valid);
    }
    if (query_joint > JOINT_COUNT)
    {
        return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
    }
    {
        const RobotDhParameter *dh_parameter =
            &joint_robot_configuration->dh[query_joint - 1U];

        return joint_controller_format_response(response,
                                                response_capacity,
                                                "ok dh joint=%u theta=%.4f d_mm=%.4f a_mm=%.4f alpha=%.4f valid=%u",
                                                (unsigned int)query_joint,
                                                (double)dh_parameter->theta_offset_degrees,
                                                (double)dh_parameter->d_millimeters,
                                                (double)dh_parameter->a_millimeters,
                                                (double)dh_parameter->alpha_degrees,
                                                (unsigned int)joint_robot_configuration->dh_parameters_valid);
    }
}

/**
 * @brief Formats one joint drivetrain profile or a compact profile summary.
 * @param query_joint One-based joint number, or zero for the summary.
 * @param response Destination response buffer.
 * @param response_capacity Destination capacity in bytes.
 * @return Controller response status.
 */
static JointControllerStatus joint_controller_format_configuration(
    uint8_t query_joint,
    char *response,
    size_t response_capacity)
{
    if (query_joint == 0U)
    {
        return joint_controller_format_response(response,
                                                response_capacity,
                                                "ok config joints=7 active=0x%02X commissioned=%u",
                                                (unsigned int)joint_state.active_mask,
                                                (unsigned int)(joint_state.commissioning_lock_active == 0U));
    }
    if (query_joint > JOINT_COUNT)
    {
        return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
    }
    {
        const RobotJointParameter *joint_parameter =
            &joint_robot_configuration->joint[query_joint - 1U];

        return joint_controller_format_response(response,
                                                response_capacity,
                                                "ok config joint=%u active=%u motor=%u master=0x%03X dir=%d zero=%.4f reduction=%.5f min=%.3f max=%.3f vmax=%.3f amax=%.3f commissioned=%u",
                                                (unsigned int)query_joint,
                                                (unsigned int)((joint_state.active_mask &
                                                    (uint8_t)(1U << (query_joint - 1U))) != 0U),
                                                (unsigned int)joint_parameter->motor_id,
                                                (unsigned int)joint_parameter->master_id,
                                                (int)joint_parameter->direction,
                                                (double)joint_parameter->joint_zero_degrees,
                                                (double)joint_parameter->external_reduction_ratio,
                                                (double)joint_parameter->minimum_degrees,
                                                (double)joint_parameter->maximum_degrees,
                                                (double)joint_parameter->maximum_velocity_degrees_s,
                                                (double)joint_parameter->maximum_acceleration_degrees_s2,
                                                (unsigned int)joint_parameter->commissioned);
    }
}

/**
 * @brief Reset all static controller state without enabling any motor.
 * @param send_function Non-blocking CAN frame submission callback.
 * @param configuration Immutable robot and drivetrain configuration.
 * @param time_ms Current monotonic time in milliseconds.
 */
void joint_controller_init(JointControllerSendFunction send_function,
                           const RobotConfiguration *configuration,
                           uint32_t time_ms)
{
    memset(&joint_state, 0, sizeof(joint_state));
    memset(joint_motor_ranges, 0, sizeof(joint_motor_ranges));
    memset(joint_range_component_mask, 0, sizeof(joint_range_component_mask));
    memset(joint_enable_reference_degrees, 0, sizeof(joint_enable_reference_degrees));
    memset(joint_target_degrees, 0, sizeof(joint_target_degrees));
    memset(joint_arrival_cycle_count, 0, sizeof(joint_arrival_cycle_count));
    memset(&joint_trajectory, 0, sizeof(joint_trajectory));
    joint_send_function = send_function;
    joint_robot_configuration = configuration;
    joint_disable_pending_mask = 0U;
    joint_mode_request_index = 0U;
    joint_discovery_request_index = 0U;
    joint_startup_request_waiting = 0U;
    joint_startup_retry_count = 0U;
    joint_feedback_request_count = 0U;
    joint_initialization_time_ms = time_ms;
    joint_startup_request_time_ms = time_ms;
    joint_last_ready_poll_time_ms = time_ms;
    joint_state.selected_joint = 0U;
    joint_state.last_command_time_ms = time_ms;
    joint_state.active_mask = (configuration != NULL) ? configuration->active_joint_mask : 0U;
    joint_state.stage = JOINT_CONTROLLER_STAGE_CONFIG_VALIDATE;
    if ((send_function == NULL) ||
        (robot_config_validate(configuration) != ROBOT_CONFIG_STATUS_OK))
    {
        joint_state.safety_reason = JOINT_CONTROLLER_SAFETY_INVALID_COMMAND;
        joint_state.stage = JOINT_CONTROLLER_STAGE_FAULT;
        return;
    }
    joint_state.commissioning_lock_active =
        (joint_controller_all_joints_are_commissioned() == 0U) ? 1U : 0U;
    joint_state.stage = JOINT_CONTROLLER_STAGE_BOOT_DELAY;
}

/**
 * @brief Execute one parsed USB command against the controller state machine.
 * @param command Validated USB command.
 * @param time_ms Current monotonic time in milliseconds.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Explicit controller status.
 */
JointControllerStatus joint_controller_submit(const UsbCommand *command,
                                              uint32_t time_ms,
                                              char *response,
                                              size_t response_capacity)
{
    if ((command == NULL) || (response == NULL) || (response_capacity == 0U))
    {
        return JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT;
    }
    response[0] = '\0';

    if ((joint_state.safety_reason != JOINT_CONTROLLER_SAFETY_NONE) &&
        (command->type != USB_COMMAND_TYPE_GET_STATE) &&
        (command->type != USB_COMMAND_TYPE_GET_JOINT_POSITIONS) &&
        (command->type != USB_COMMAND_TYPE_GET_MOTOR_POSITIONS) &&
        (command->type != USB_COMMAND_TYPE_GET_ENABLE) &&
        (command->type != USB_COMMAND_TYPE_GET_CAPABILITIES) &&
        (command->type != USB_COMMAND_TYPE_GET_DH) &&
        (command->type != USB_COMMAND_TYPE_GET_CONFIG) &&
        (command->type != USB_COMMAND_TYPE_DISABLE))
    {
        return JOINT_CONTROLLER_STATUS_SAFETY_LATCHED;
    }

    switch (command->type)
    {
        case USB_COMMAND_TYPE_SELECT:
            if ((command->selected_joint < 1U) || (command->selected_joint > JOINT_COUNT))
            {
                return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
            }
            if ((joint_state.active_mask &
                 (uint8_t)(1U << (command->selected_joint - 1U))) == 0U)
            {
                return JOINT_CONTROLLER_STATUS_INVALID_SELECTION;
            }
            if (joint_state.enabled_mask != 0U)
            {
                return JOINT_CONTROLLER_STATUS_NOT_ENABLED;
            }
            joint_state.selected_joint = command->selected_joint;
            joint_state.last_command_time_ms = time_ms;
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok selected=%u",
                                                    (unsigned int)command->selected_joint);

        case USB_COMMAND_TYPE_START:
            return joint_controller_start_selected(time_ms, response, response_capacity);

        case USB_COMMAND_TYPE_STOP:
            joint_trajectory.active = 0U;
            joint_state.trajectory_active = 0U;
            memcpy(joint_target_degrees,
                   joint_state.commanded_position_degrees,
                   sizeof(joint_target_degrees));
            joint_state.last_command_time_ms = time_ms;
            joint_state.stage = (joint_state.enabled_mask != 0U)
                                    ? JOINT_CONTROLLER_STAGE_HOLDING
                                    : JOINT_CONTROLLER_STAGE_READY;
            return joint_controller_format_response(response, response_capacity, "ok stopped");

        case USB_COMMAND_TYPE_DISABLE:
        {
            JointControllerSafetyReason preserved_reason = joint_state.safety_reason;
            uint8_t can_transmit =
                ((preserved_reason != JOINT_CONTROLLER_SAFETY_BUS_OFF) &&
                 (preserved_reason != JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT))
                    ? 1U
                    : 0U;

            joint_controller_safety_shutdown(preserved_reason, can_transmit);
            joint_state.last_command_time_ms = time_ms;
            joint_state.stage = (preserved_reason == JOINT_CONTROLLER_SAFETY_NONE)
                                    ? JOINT_CONTROLLER_STAGE_READY
                                    : JOINT_CONTROLLER_STAGE_FAULT;
            return joint_controller_format_response(response, response_capacity, "ok disabled");
        }

        case USB_COMMAND_TYPE_HOME:
            return joint_controller_start_home(time_ms, response, response_capacity);

        case USB_COMMAND_TYPE_MOVE_JOINTS:
            return joint_controller_start_move(command, time_ms, response, response_capacity);

        case USB_COMMAND_TYPE_GET_JOINT_POSITIONS:
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                                                    (double)joint_state.joint_position_degrees[0],
                                                    (double)joint_state.joint_position_degrees[1],
                                                    (double)joint_state.joint_position_degrees[2],
                                                    (double)joint_state.joint_position_degrees[3],
                                                    (double)joint_state.joint_position_degrees[4],
                                                    (double)joint_state.joint_position_degrees[5],
                                                    (double)joint_state.joint_position_degrees[6]);

        case USB_COMMAND_TYPE_GET_MOTOR_POSITIONS:
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok %.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                                                    (double)joint_state.motor_position_degrees[0],
                                                    (double)joint_state.motor_position_degrees[1],
                                                    (double)joint_state.motor_position_degrees[2],
                                                    (double)joint_state.motor_position_degrees[3],
                                                    (double)joint_state.motor_position_degrees[4],
                                                    (double)joint_state.motor_position_degrees[5],
                                                    (double)joint_state.motor_position_degrees[6]);

        case USB_COMMAND_TYPE_GET_STATE:
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok stage=%u active=0x%02X selected=%u enabled=0x%02X ranges=0x%02X feedback=0x%02X arrived=0x%02X faults=0x%02X lock=%u safety=%u",
                                                    (unsigned int)joint_state.stage,
                                                    (unsigned int)joint_state.active_mask,
                                                    (unsigned int)joint_state.selected_joint,
                                                    (unsigned int)joint_state.enabled_mask,
                                                    (unsigned int)joint_state.ranges_ready_mask,
                                                    (unsigned int)joint_state.feedback_ready_mask,
                                                    (unsigned int)joint_state.arrived_mask,
                                                    (unsigned int)joint_state.motor_fault_mask,
                                                    (unsigned int)joint_state.commissioning_lock_active,
                                                    (unsigned int)joint_state.safety_reason);

        case USB_COMMAND_TYPE_GET_ENABLE:
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok enabled=0x%02X",
                                                    (unsigned int)joint_state.enabled_mask);

        case USB_COMMAND_TYPE_GET_CAPABILITIES:
            return joint_controller_format_response(response,
                                                    response_capacity,
                                                    "ok caps joints=7 active=0x%02X control=joint-angle dh=%u ik=0 rgb=0 heap=0 probe=1",
                                                    (unsigned int)joint_state.active_mask,
                                                    (unsigned int)joint_robot_configuration->dh_parameters_valid);

        case USB_COMMAND_TYPE_GET_DH:
            return joint_controller_format_dh(command->query_joint,
                                              response,
                                              response_capacity);

        case USB_COMMAND_TYPE_GET_CONFIG:
            return joint_controller_format_configuration(command->query_joint,
                                                         response,
                                                         response_capacity);

        case USB_COMMAND_TYPE_PING:
        case USB_COMMAND_TYPE_ECHO:
            joint_state.last_command_time_ms = time_ms;
            return JOINT_CONTROLLER_STATUS_OK;

        default:
            break;
    }

    return JOINT_CONTROLLER_STATUS_INVALID_ARGUMENT;
}

/**
 * @brief Latch a startup timeout after the configured number of retries.
 */
static void joint_controller_latch_startup_timeout(void)
{
    joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_STARTUP_TIMEOUT, 1U);
}

/**
 * @brief Update retry state for one outstanding startup request.
 * @param time_ms Current monotonic time in milliseconds.
 * @return One when the caller should retransmit the current request.
 */
static uint8_t joint_controller_startup_request_is_due(uint32_t time_ms)
{
    if (joint_startup_request_waiting == 0U)
    {
        return 1U;
    }
    if ((time_ms - joint_startup_request_time_ms) < JOINT_CONTROLLER_STARTUP_TIMEOUT_MS)
    {
        return 0U;
    }

    joint_startup_request_waiting = 0U;
    joint_startup_retry_count++;
    if (joint_startup_retry_count > JOINT_CONTROLLER_STARTUP_RETRY_LIMIT)
    {
        joint_controller_latch_startup_timeout();
        return 0U;
    }
    return 1U;
}

/**
 * @brief Submit one startup frame and arm its response timeout.
 * @param frame Validated protocol frame.
 * @param time_ms Current monotonic time in milliseconds.
 */
static void joint_controller_submit_startup_frame(const FdcanClassicFrame *frame,
                                                  uint32_t time_ms)
{
    if (joint_controller_send_frame(frame) != JOINT_CONTROLLER_STATUS_OK)
    {
        joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT, 1U);
        return;
    }
    joint_startup_request_waiting = 1U;
    joint_startup_request_time_ms = time_ms;
}

/**
 * @brief Configure and confirm position-velocity mode on every active motor.
 * @param time_ms Current monotonic time in milliseconds.
 */
static void joint_controller_service_mode_setup(uint32_t time_ms)
{
    uint8_t joint_index;
    uint8_t request_is_read;
    FdcanClassicFrame frame;
    DmMotorStatus protocol_status;

    while (joint_mode_request_index < (JOINT_COUNT * 2U))
    {
        joint_index = (uint8_t)(joint_mode_request_index / 2U);
        if ((joint_state.active_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            break;
        }
        joint_mode_request_index = (uint8_t)((joint_index + 1U) * 2U);
    }
    if (joint_mode_request_index >= (JOINT_COUNT * 2U))
    {
        if ((joint_state.mode_ready_mask & joint_state.active_mask) !=
            joint_state.active_mask)
        {
            joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_MODE_MISMATCH, 1U);
            return;
        }
        joint_startup_request_waiting = 0U;
        joint_startup_retry_count = 0U;
        joint_state.stage = JOINT_CONTROLLER_STAGE_RANGE_DISCOVERY;
        return;
    }
    if (joint_controller_startup_request_is_due(time_ms) == 0U)
    {
        return;
    }

    joint_index = (uint8_t)(joint_mode_request_index / 2U);
    request_is_read = (uint8_t)(joint_mode_request_index & 0x01U);
    if (request_is_read != 0U)
    {
        protocol_status = dm_motor_pack_parameter_read(
            joint_robot_configuration->joint[joint_index].motor_id,
            DM_MOTOR_REGISTER_CONTROL_MODE,
            &frame);
    }
    else
    {
        protocol_status = dm_motor_pack_parameter_write_u32(
            joint_robot_configuration->joint[joint_index].motor_id,
            DM_MOTOR_REGISTER_CONTROL_MODE,
            JOINT_CONTROLLER_CONTROL_MODE_VALUE,
            &frame);
    }
    if (protocol_status != DM_MOTOR_STATUS_OK)
    {
        joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
        return;
    }
    joint_controller_submit_startup_frame(&frame, time_ms);
}

/**
 * @brief Read PMAX, VMAX, and TMAX from every active motor.
 * @param time_ms Current monotonic time in milliseconds.
 */
static void joint_controller_service_range_discovery(uint32_t time_ms)
{
    static const DmMotorRegister discovery_registers[3] = {
        DM_MOTOR_REGISTER_POSITION_RANGE,
        DM_MOTOR_REGISTER_VELOCITY_RANGE,
        DM_MOTOR_REGISTER_TORQUE_RANGE
    };
    uint8_t joint_index;
    uint8_t register_index;
    FdcanClassicFrame frame;

    while (joint_discovery_request_index < (JOINT_COUNT * 3U))
    {
        joint_index = (uint8_t)(joint_discovery_request_index / 3U);
        if ((joint_state.active_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            break;
        }
        joint_discovery_request_index = (uint8_t)((joint_index + 1U) * 3U);
    }
    if (joint_discovery_request_index >= (JOINT_COUNT * 3U))
    {
        uint8_t missing_feedback_mask =
            (uint8_t)(joint_state.active_mask & (uint8_t)~joint_state.feedback_ready_mask);

        if ((joint_state.ranges_ready_mask & joint_state.active_mask) !=
            joint_state.active_mask)
        {
            joint_controller_latch_startup_timeout();
            return;
        }
        if (missing_feedback_mask == 0U)
        {
            joint_state.stage = JOINT_CONTROLLER_STAGE_READY;
            return;
        }
        if ((joint_feedback_request_count != 0U) &&
            ((time_ms - joint_startup_request_time_ms) <
             JOINT_CONTROLLER_STARTUP_TIMEOUT_MS))
        {
            return;
        }
        if (joint_feedback_request_count > JOINT_CONTROLLER_STARTUP_RETRY_LIMIT)
        {
            joint_controller_latch_startup_timeout();
            return;
        }
        for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
        {
            if ((missing_feedback_mask & (uint8_t)(1U << joint_index)) != 0U)
            {
                if (joint_controller_send_mode(joint_index,
                                               DM_MOTOR_MODE_COMMAND_DISABLE) !=
                    JOINT_CONTROLLER_STATUS_OK)
                {
                    joint_controller_safety_shutdown(
                        JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT,
                        1U);
                    return;
                }
            }
        }
        joint_feedback_request_count++;
        joint_startup_request_time_ms = time_ms;
        return;
    }
    if (joint_controller_startup_request_is_due(time_ms) == 0U)
    {
        return;
    }

    joint_index = (uint8_t)(joint_discovery_request_index / 3U);
    register_index = (uint8_t)(joint_discovery_request_index % 3U);
    if (dm_motor_pack_parameter_read(joint_robot_configuration->joint[joint_index].motor_id,
                                     discovery_registers[register_index],
                                     &frame) != DM_MOTOR_STATUS_OK)
    {
        joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
        return;
    }
    joint_controller_submit_startup_frame(&frame, time_ms);
}

/**
 * @brief Advance the non-blocking boot, mode setup, and discovery sequence.
 * @param time_ms Current monotonic time in milliseconds.
 */
static void joint_controller_service_startup(uint32_t time_ms)
{
    if (joint_state.safety_reason != JOINT_CONTROLLER_SAFETY_NONE)
    {
        return;
    }
    if (joint_state.stage == JOINT_CONTROLLER_STAGE_BOOT_DELAY)
    {
        if ((time_ms - joint_initialization_time_ms) < JOINT_CONTROLLER_BOOT_DELAY_MS)
        {
            return;
        }
        joint_state.stage = JOINT_CONTROLLER_STAGE_MODE_SETUP;
    }
    if (joint_state.stage == JOINT_CONTROLLER_STAGE_MODE_SETUP)
    {
        joint_controller_service_mode_setup(time_ms);
    }
    else if (joint_state.stage == JOINT_CONTROLLER_STAGE_RANGE_DISCOVERY)
    {
        joint_controller_service_range_discovery(time_ms);
    }
}

/**
 * @brief Request fresh disabled feedback while the controller remains ready.
 * @param time_ms Current monotonic time in milliseconds.
 */
static void joint_controller_service_ready_feedback(uint32_t time_ms)
{
    uint8_t joint_index;

    if ((joint_state.stage != JOINT_CONTROLLER_STAGE_READY) ||
        (joint_state.enabled_mask != 0U) ||
        ((time_ms - joint_last_ready_poll_time_ms) <
         JOINT_CONTROLLER_READY_POLL_INTERVAL_MS))
    {
        return;
    }
    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        if ((joint_state.active_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            if (joint_controller_send_mode(joint_index,
                                           DM_MOTOR_MODE_COMMAND_DISABLE) !=
                JOINT_CONTROLLER_STATUS_OK)
            {
                joint_controller_safety_shutdown(
                    JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT,
                    1U);
                return;
            }
        }
    }
    joint_last_ready_poll_time_ms = time_ms;
}

/**
 * @brief Advance the 200 Hz control loop, enforce timeouts, and send setpoints.
 * @param time_ms Current monotonic time in milliseconds.
 */
void joint_controller_step(uint32_t time_ms)
{
    float setpoint_degrees[JOINT_COUNT];
    float setpoint_velocity_degrees_s[JOINT_COUNT];
    uint8_t trajectory_complete = 1U;
    uint8_t joint_index;

    if (joint_disable_pending_mask != 0U)
    {
        uint8_t pending_mask = joint_disable_pending_mask;

        joint_disable_pending_mask = 0U;
        for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
        {
            if ((pending_mask & (uint8_t)(1U << joint_index)) != 0U)
            {
                (void)joint_controller_send_mode(joint_index, DM_MOTOR_MODE_COMMAND_DISABLE);
            }
        }
    }

    joint_controller_service_startup(time_ms);
    joint_controller_service_ready_feedback(time_ms);
    if (joint_state.stage == JOINT_CONTROLLER_STAGE_ENABLING)
    {
        for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
        {
            if (((joint_state.enable_pending_mask & (uint8_t)(1U << joint_index)) != 0U) &&
                ((time_ms - joint_state.last_feedback_time_ms[joint_index]) >
                 JOINT_CONTROLLER_FEEDBACK_TIMEOUT_MS))
            {
                joint_controller_safety_shutdown(
                    JOINT_CONTROLLER_SAFETY_FEEDBACK_TIMEOUT,
                    1U);
                return;
            }
        }
        if ((joint_state.enabled_mask & joint_state.enable_pending_mask) !=
            joint_state.enable_pending_mask)
        {
            return;
        }
        joint_state.enable_pending_mask = 0U;
        joint_state.stage = JOINT_CONTROLLER_STAGE_HOLDING;
    }
    if (joint_state.enabled_mask == 0U)
    {
        return;
    }
    if ((joint_state.trajectory_active != 0U) &&
        ((time_ms - joint_state.last_command_time_ms) >
         JOINT_CONTROLLER_COMMAND_TIMEOUT_MS))
    {
        joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_COMMAND_TIMEOUT, 1U);
        return;
    }
    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        if (((joint_state.enabled_mask & (uint8_t)(1U << joint_index)) != 0U) &&
            ((time_ms - joint_state.last_feedback_time_ms[joint_index]) >
             JOINT_CONTROLLER_FEEDBACK_TIMEOUT_MS))
        {
            joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_FEEDBACK_TIMEOUT, 1U);
            return;
        }
    }

    memcpy(setpoint_degrees,
           joint_state.commanded_position_degrees,
           sizeof(setpoint_degrees));
    memset(setpoint_velocity_degrees_s, 0, sizeof(setpoint_velocity_degrees_s));
    if (joint_trajectory.active != 0U)
    {
        if (sync_trajectory_step(&joint_trajectory,
                                 (float)JOINT_CONTROLLER_PERIOD_MS / 1000.0F,
                                 setpoint_degrees,
                                 setpoint_velocity_degrees_s,
                                 &trajectory_complete) != SYNC_TRAJECTORY_STATUS_OK)
        {
            joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT, 1U);
            return;
        }
        joint_state.trajectory_active = (trajectory_complete == 0U) ? 1U : 0U;
        memcpy(joint_state.commanded_position_degrees,
               setpoint_degrees,
               sizeof(joint_state.commanded_position_degrees));
        if (trajectory_complete != 0U)
        {
            joint_state.stage = JOINT_CONTROLLER_STAGE_HOLDING;
        }
    }

    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        if ((joint_state.enabled_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            FdcanClassicFrame frame;
            float position_rad;
            float velocity_limit_rad_s;

            if ((robot_config_joint_to_motor_position_rad(joint_robot_configuration,
                                                          joint_index,
                                                          setpoint_degrees[joint_index],
                                                          &position_rad) !=
                 ROBOT_CONFIG_STATUS_OK) ||
                (robot_config_joint_to_motor_velocity_rad_s(
                     joint_robot_configuration,
                     joint_index,
                     setpoint_velocity_degrees_s[joint_index],
                     &velocity_limit_rad_s) != ROBOT_CONFIG_STATUS_OK) ||
                (dm_motor_pack_position_velocity(
                     joint_robot_configuration->joint[joint_index].motor_id,
                     position_rad,
                     fabsf(velocity_limit_rad_s),
                     &frame) != DM_MOTOR_STATUS_OK) ||
                (joint_controller_send_frame(&frame) != JOINT_CONTROLLER_STATUS_OK))
            {
                joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT, 1U);
                return;
            }
        }
    }
}

/**
 * @brief Consume one accepted Master-ID response from the FDCAN receive callback.
 * @param frame Accepted eight-byte classic CAN frame.
 * @param time_ms Current monotonic receive time in milliseconds.
 */
void joint_controller_on_can_frame(const FdcanClassicFrame *frame, uint32_t time_ms)
{
    uint8_t joint_index;

    if ((frame == NULL) || (frame->length != 8U) ||
        (joint_controller_master_to_index(frame->identifier, &joint_index) == 0U))
    {
        return;
    }

    if ((frame->data[1] == 0U) &&
        ((frame->data[2] == 0x33U) || (frame->data[2] == 0x55U)) &&
        ((frame->data[3] == DM_MOTOR_REGISTER_CONTROL_MODE) ||
         (frame->data[3] == DM_MOTOR_REGISTER_POSITION_RANGE) ||
         (frame->data[3] == DM_MOTOR_REGISTER_VELOCITY_RANGE) ||
         (frame->data[3] == DM_MOTOR_REGISTER_TORQUE_RANGE)))
    {
        DmMotorParameterResponse parameter_response;

        if ((dm_motor_decode_parameter_response(frame, &parameter_response) != DM_MOTOR_STATUS_OK) ||
            (parameter_response.motor_id !=
             joint_robot_configuration->joint[joint_index].motor_id))
        {
            return;
        }

        if (parameter_response.register_address == DM_MOTOR_REGISTER_CONTROL_MODE)
        {
            uint8_t expected_joint_index =
                (uint8_t)(joint_mode_request_index / 2U);

            if ((joint_state.stage != JOINT_CONTROLLER_STAGE_MODE_SETUP) ||
                (joint_startup_request_waiting == 0U) ||
                (expected_joint_index != joint_index))
            {
                return;
            }
            if (parameter_response.raw_value != JOINT_CONTROLLER_CONTROL_MODE_VALUE)
            {
                joint_controller_safety_shutdown(
                    JOINT_CONTROLLER_SAFETY_MODE_MISMATCH,
                    1U);
                return;
            }
            if ((joint_mode_request_index & 0x01U) != 0U)
            {
                joint_state.mode_ready_mask |= (uint8_t)(1U << joint_index);
            }
            joint_mode_request_index++;
            joint_startup_request_waiting = 0U;
            joint_startup_retry_count = 0U;
            return;
        }

        if ((joint_state.stage != JOINT_CONTROLLER_STAGE_RANGE_DISCOVERY) ||
            (joint_startup_request_waiting == 0U) ||
            ((uint8_t)(joint_discovery_request_index / 3U) != joint_index) ||
            (parameter_response.register_address != (uint8_t)(
                DM_MOTOR_REGISTER_POSITION_RANGE +
                (joint_discovery_request_index % 3U))) ||
            (!isfinite(parameter_response.float_value)) ||
            (parameter_response.float_value <= 0.0F))
        {
            return;
        }

        if (parameter_response.register_address == DM_MOTOR_REGISTER_POSITION_RANGE)
        {
            joint_motor_ranges[joint_index].position_max_rad = parameter_response.float_value;
            joint_range_component_mask[joint_index] |= JOINT_CONTROLLER_RANGE_POSITION_BIT;
        }
        else if (parameter_response.register_address == DM_MOTOR_REGISTER_VELOCITY_RANGE)
        {
            joint_motor_ranges[joint_index].velocity_max_rad_s = parameter_response.float_value;
            joint_range_component_mask[joint_index] |= JOINT_CONTROLLER_RANGE_VELOCITY_BIT;
        }
        else if (parameter_response.register_address == DM_MOTOR_REGISTER_TORQUE_RANGE)
        {
            joint_motor_ranges[joint_index].torque_max_nm = parameter_response.float_value;
            joint_range_component_mask[joint_index] |= JOINT_CONTROLLER_RANGE_TORQUE_BIT;
        }

        if (joint_range_component_mask[joint_index] == JOINT_CONTROLLER_ALL_RANGES_MASK)
        {
            joint_state.ranges_ready_mask |= (uint8_t)(1U << joint_index);
        }
        joint_discovery_request_index++;
        joint_startup_request_waiting = 0U;
        joint_startup_retry_count = 0U;
        return;
    }

    if (((frame->data[0] & 0x0FU) ==
         joint_robot_configuration->joint[joint_index].motor_id) &&
        ((joint_state.ranges_ready_mask & (uint8_t)(1U << joint_index)) != 0U))
    {
        DmMotorFeedback feedback;

        if (dm_motor_decode_feedback(frame,
                                     &joint_motor_ranges[joint_index],
                                     &feedback) != DM_MOTOR_STATUS_OK)
        {
            return;
        }
        joint_state.motor_position_degrees[joint_index] =
            feedback.position_rad * JOINT_CONTROLLER_RAD_TO_DEG;
        joint_state.motor_velocity_degrees_s[joint_index] =
            feedback.velocity_rad_s * JOINT_CONTROLLER_RAD_TO_DEG;
        joint_state.motor_state[joint_index] = feedback.state;
        if ((robot_config_motor_to_joint_position_degrees(
                 joint_robot_configuration,
                 joint_index,
                 feedback.position_rad,
                 &joint_state.joint_position_degrees[joint_index]) !=
             ROBOT_CONFIG_STATUS_OK) ||
            (robot_config_motor_to_joint_velocity_degrees_s(
                 joint_robot_configuration,
                 joint_index,
                 feedback.velocity_rad_s,
                 &joint_state.joint_velocity_degrees_s[joint_index]) !=
             ROBOT_CONFIG_STATUS_OK))
        {
            joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
            return;
        }
        joint_state.last_feedback_time_ms[joint_index] = time_ms;
        joint_state.feedback_ready_mask |= (uint8_t)(1U << joint_index);

        if (feedback.state >= 8U)
        {
            joint_state.motor_fault_mask |= (uint8_t)(1U << joint_index);
            joint_disable_pending_mask |= joint_state.enabled_mask;
            joint_state.enabled_mask = 0U;
            joint_state.trajectory_active = 0U;
            joint_trajectory.active = 0U;
            joint_state.safety_reason = JOINT_CONTROLLER_SAFETY_MOTOR_FAULT;
            joint_state.stage = JOINT_CONTROLLER_STAGE_FAULT;
            return;
        }

        if (feedback.state == 1U)
        {
            joint_state.enabled_mask |= (uint8_t)(1U << joint_index);
        }
        else
        {
            joint_state.enabled_mask &= (uint8_t)~(uint8_t)(1U << joint_index);
            if ((joint_state.enable_pending_mask & (uint8_t)(1U << joint_index)) == 0U)
            {
                joint_state.commanded_position_degrees[joint_index] =
                    joint_state.joint_position_degrees[joint_index];
                joint_target_degrees[joint_index] =
                    joint_state.joint_position_degrees[joint_index];
            }
        }

        if ((joint_state.stage == JOINT_CONTROLLER_STAGE_ENABLING) &&
            (joint_state.enable_pending_mask != 0U) &&
            ((joint_state.enabled_mask & joint_state.enable_pending_mask) ==
             joint_state.enable_pending_mask))
        {
            joint_state.enable_pending_mask = 0U;
            joint_state.stage = JOINT_CONTROLLER_STAGE_HOLDING;
        }

        if ((fabsf(joint_state.joint_position_degrees[joint_index] -
                   joint_target_degrees[joint_index]) <=
             JOINT_CONTROLLER_ARRIVAL_POSITION_DEG) &&
            (fabsf(joint_state.joint_velocity_degrees_s[joint_index]) <=
             JOINT_CONTROLLER_ARRIVAL_VELOCITY_DEG_S))
        {
            if (joint_arrival_cycle_count[joint_index] < JOINT_CONTROLLER_ARRIVAL_CYCLES)
            {
                joint_arrival_cycle_count[joint_index]++;
            }
            if ((joint_arrival_cycle_count[joint_index] == JOINT_CONTROLLER_ARRIVAL_CYCLES) &&
                (joint_state.first_arrival_time_ms[joint_index] == 0U))
            {
                joint_state.first_arrival_time_ms[joint_index] = time_ms;
                joint_state.arrived_mask |= (uint8_t)(1U << joint_index);
            }
        }
        else
        {
            joint_arrival_cycle_count[joint_index] = 0U;
            joint_state.arrived_mask &= (uint8_t)~(uint8_t)(1U << joint_index);
        }
    }
}

/**
 * @brief Latch Bus-Off and stop all software control without restarting the bus.
 */
void joint_controller_on_bus_off(void)
{
    joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_BUS_OFF, 0U);
}

/**
 * @brief Latch an FDCAN initialization failure while preserving USB diagnostics.
 */
void joint_controller_on_can_start_failure(void)
{
    joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_CAN_TRANSMIT, 0U);
}

/**
 * @brief Stop and disable enabled motors after an invalid USB command or parameter.
 */
void joint_controller_on_invalid_command(void)
{
    joint_controller_safety_shutdown(JOINT_CONTROLLER_SAFETY_INVALID_COMMAND, 1U);
}

/**
 * @brief Get the read-only public controller state.
 * @return Address of the static controller state.
 */
const JointControllerState *joint_controller_get_state(void)
{
    return &joint_state;
}

/**
 * @brief Get the read-only seven-joint compile-time configuration.
 * @return Address of the first configuration element.
 */
const RobotConfiguration *joint_controller_get_configuration(void)
{
    return joint_robot_configuration;
}
