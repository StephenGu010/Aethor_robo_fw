/**
 * @file aethor_application.c
 * @brief Connects PA15, FDCAN1, USB CDC diagnostics, and the two-motor key controller.
 */

#include "aethor_application.h"

#include <stdio.h>
#include <string.h>

#include "bsp_fdcan.h"
#include "dual_motor_controller.h"
#include "firmware_probe.h"
#include "main.h"
#include "robot_config.h"
#include "usb_cdc_transport.h"
#include "usb_command.h"
#include "usbd_cdc_if.h"

static uint32_t aethor_application_command_sequence;

/**
 * @brief Queue one task-context structured probe line on USB CDC.
 * @param line Bounded diagnostic line without CR/LF.
 * @return Zero when queued, otherwise non-zero.
 */
static int aethor_application_probe_emit(const char *line)
{
    return (usb_cdc_transport_queue_line(line) == USB_CDC_TRANSPORT_STATUS_OK) ? 0 : -1;
}

/**
 * @brief Maps the two-motor controller state into a stable serial probe stage.
 * @param controller_state Current two-motor controller state.
 * @return Stable firmware probe stage.
 */
static FirmwareProbeStage aethor_application_probe_stage(
    const DualMotorControllerState *controller_state)
{
    switch (controller_state->stage)
    {
        case DUAL_MOTOR_STAGE_BOOT_DELAY:
            return FIRMWARE_PROBE_STAGE_BOOT;
        case DUAL_MOTOR_STAGE_CONFIGURING:
            return (controller_state->mode_ready_mask == DUAL_MOTOR_ALL_MASK)
                       ? FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY
                       : FIRMWARE_PROBE_STAGE_MODE_SETUP;
        case DUAL_MOTOR_STAGE_REQUESTING_FEEDBACK:
            return FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY;
        case DUAL_MOTOR_STAGE_READY:
            return FIRMWARE_PROBE_STAGE_READY;
        case DUAL_MOTOR_STAGE_ENABLING:
            return FIRMWARE_PROBE_STAGE_ENABLING;
        case DUAL_MOTOR_STAGE_MOVING:
            return FIRMWARE_PROBE_STAGE_MOVING;
        case DUAL_MOTOR_STAGE_HOLDING:
            return FIRMWARE_PROBE_STAGE_HOLDING;
        case DUAL_MOTOR_STAGE_FAULT:
        default:
            return FIRMWARE_PROBE_STAGE_FAULT;
    }
}

/**
 * @brief Capture the two-motor controller and CAN state for transition probes.
 * @return Bounded probe snapshot.
 */
static FirmwareProbeSnapshot aethor_application_probe_snapshot(void)
{
    const DualMotorControllerState *controller_state =
        dual_motor_controller_get_state();
    const FdcanDriverState *can_state = fdcan_classic_get_state();
    FirmwareProbeSnapshot snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.stage = aethor_application_probe_stage(controller_state);
    snapshot.active_mask = DUAL_MOTOR_ALL_MASK;
    snapshot.ready_mask = (uint8_t)(controller_state->mode_ready_mask &
                                    controller_state->ranges_ready_mask &
                                    controller_state->fresh_feedback_mask);
    snapshot.enabled_mask = controller_state->enabled_mask;
    snapshot.arrived_mask = controller_state->arrived_mask;
    snapshot.fault_code = (uint8_t)controller_state->fault_reason;
    snapshot.fault_joint = 0U;
    snapshot.bus_off = can_state->bus_off_latched;
    return snapshot;
}

/**
 * @brief Returns a stable probe name for a parsed USB command.
 * @param command_type Parsed command type.
 * @return Static command name.
 */
static const char *aethor_application_command_name(UsbCommandType command_type)
{
    switch (command_type)
    {
        case USB_COMMAND_TYPE_PING:
            return "PING";
        case USB_COMMAND_TYPE_ECHO:
            return "ECHO";
        case USB_COMMAND_TYPE_GET_STATE:
            return "GET_STATE";
        case USB_COMMAND_TYPE_GET_ENABLE:
            return "GET_ENABLE";
        case USB_COMMAND_TYPE_GET_CAPABILITIES:
            return "GET_CAPABILITIES";
        case USB_COMMAND_TYPE_GET_JOINT_POSITIONS:
            return "GET_JOINT_POSITIONS";
        case USB_COMMAND_TYPE_GET_MOTOR_POSITIONS:
            return "GET_MOTOR_POSITIONS";
        case USB_COMMAND_TYPE_GET_CONFIG:
            return "GET_CONFIG";
        default:
            return "MOTION_COMMAND";
    }
}

/**
 * @brief Submit one controller frame to FDCAN1.
 * @param frame Frame produced by the motor protocol layer.
 * @return Zero on success, otherwise non-zero.
 */
static int aethor_application_can_send(const FdcanClassicFrame *frame)
{
    return (fdcan_classic_send(&hfdcan1, frame) == FDCAN_DRIVER_STATUS_OK) ? 0 : -1;
}

/**
 * @brief Adapt Cube USB CDC transmit results to the transport callback contract.
 * @param data Response bytes.
 * @param length Response length.
 * @return Transport transmit status.
 */
static UsbCdcTransmitResult aethor_application_usb_transmit(const uint8_t *data,
                                                           uint16_t length)
{
    uint8_t usb_status = CDC_Transmit_HS((uint8_t *)data, length);

    if (usb_status == USBD_OK)
    {
        return USB_CDC_TRANSMIT_OK;
    }
    if (usb_status == USBD_BUSY)
    {
        return USB_CDC_TRANSMIT_BUSY;
    }
    return USB_CDC_TRANSMIT_ERROR;
}

/**
 * @brief Format two-motor, key, feedback-age, CAN, and probe diagnostics.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Zero when the response fitted, otherwise non-zero.
 */
static int aethor_application_format_state(char *response,
                                           size_t response_capacity)
{
    const DualMotorControllerState *controller_state =
        dual_motor_controller_get_state();
    const FdcanDriverState *can_state = fdcan_classic_get_state();
    const UsbCdcTransportStatistics *usb_statistics =
        usb_cdc_transport_get_statistics();
    const FirmwareProbeStatistics *probe_statistics =
        firmware_probe_get_statistics();
    uint32_t current_time_ms = HAL_GetTick();
    uint32_t first_feedback_age_ms =
        current_time_ms - controller_state->last_feedback_time_ms[0];
    uint32_t second_feedback_age_ms =
        current_time_ms - controller_state->last_feedback_time_ms[1];
    int written_length;

    written_length = snprintf(
        response,
        response_capacity,
        "ok stage=%s key=%u accepted=%u moves=%lu next_dir=%d mode=0x%02X ranges=0x%02X feedback=0x%02X enabled=0x%02X arrived=0x%02X motor_state=%u,%u start=%.4f,%.4f target=%.4f,%.4f pos=%.4f,%.4f vel=%.4f,%.4f fb_age=%lu,%lu fault=%u can_started=%u warning=%u passive=%u busoff=%u usb_drop=%lu probe_drop=%lu",
        firmware_probe_stage_name(aethor_application_probe_stage(controller_state)),
        (unsigned int)controller_state->stable_key_pressed,
        (unsigned int)controller_state->move_accepted,
        (unsigned long)controller_state->accepted_move_count,
        (int)controller_state->next_direction,
        (unsigned int)controller_state->mode_ready_mask,
        (unsigned int)controller_state->ranges_ready_mask,
        (unsigned int)controller_state->fresh_feedback_mask,
        (unsigned int)controller_state->enabled_mask,
        (unsigned int)controller_state->arrived_mask,
        (unsigned int)controller_state->motor_state[0],
        (unsigned int)controller_state->motor_state[1],
        (double)controller_state->initial_position_rad[0],
        (double)controller_state->initial_position_rad[1],
        (double)controller_state->target_position_rad[0],
        (double)controller_state->target_position_rad[1],
        (double)controller_state->measured_position_rad[0],
        (double)controller_state->measured_position_rad[1],
        (double)controller_state->measured_velocity_rad_s[0],
        (double)controller_state->measured_velocity_rad_s[1],
        (unsigned long)first_feedback_age_ms,
        (unsigned long)second_feedback_age_ms,
        (unsigned int)controller_state->fault_reason,
        (unsigned int)can_state->started,
        (unsigned int)can_state->warning,
        (unsigned int)can_state->error_passive,
        (unsigned int)can_state->bus_off_latched,
        (unsigned long)usb_statistics->dropped_response_count,
        (unsigned long)probe_statistics->dropped_line_count);

    return ((written_length >= 0) && ((size_t)written_length < response_capacity)) ? 0 : -1;
}

/**
 * @brief Format one accepted read-only USB query.
 * @param command Parsed read-only command.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Zero after producing a response.
 */
static int aethor_application_process_read_only_command(
    const UsbCommand *command,
    char *response,
    size_t response_capacity)
{
    if (command->type == USB_COMMAND_TYPE_GET_STATE)
    {
        if (aethor_application_format_state(response, response_capacity) != 0)
        {
            (void)snprintf(response, response_capacity, "err response-too-small");
        }
    }
    return 0;
}

/**
 * @brief Parse one complete USB line while enforcing key-only motion authority.
 * @param line Complete command line.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Zero after producing either a success or explicit error response.
 */
static int aethor_application_process_line(const char *line,
                                           char *response,
                                           size_t response_capacity)
{
    UsbCommand command;
    UsbCommandStatus parse_status =
        usb_command_process_line(line, &command, response, response_capacity);
    uint8_t command_is_read_only;

    aethor_application_command_sequence++;
    if (parse_status != USB_COMMAND_STATUS_OK)
    {
        (void)snprintf(response,
                       response_capacity,
                       "err parse=%u",
                       (unsigned int)parse_status);
        firmware_probe_record_command("PARSE",
                                      aethor_application_command_sequence,
                                      "rejected",
                                      HAL_GetTick());
        return 0;
    }

    command_is_read_only = usb_command_is_allowed_in_key_control(command.type);
    firmware_probe_record_command(aethor_application_command_name(command.type),
                                  aethor_application_command_sequence,
                                  (command_is_read_only != 0U)
                                      ? "accepted"
                                      : "key-only-rejected",
                                  HAL_GetTick());
    if (command_is_read_only == 0U)
    {
        (void)snprintf(response, response_capacity, "err key-only-control");
        return 0;
    }
    return aethor_application_process_read_only_command(&command,
                                                        response,
                                                        response_capacity);
}

/**
 * @brief Initialize USB diagnostics, two-motor key control, and FDCAN1.
 * @return Zero after diagnostics are initialized, including CAN-failure reporting.
 */
int aethor_application_init(void)
{
    const RobotConfiguration *configuration = robot_config_get();
    FirmwareProbeSnapshot initial_probe_snapshot;

    usb_cdc_transport_init(aethor_application_usb_transmit,
                           aethor_application_process_line);
    aethor_application_command_sequence = 0U;
    dual_motor_controller_init(aethor_application_can_send, HAL_GetTick());
    memset(&initial_probe_snapshot, 0, sizeof(initial_probe_snapshot));
    initial_probe_snapshot.stage = FIRMWARE_PROBE_STAGE_BOOT;
    initial_probe_snapshot.active_mask = DUAL_MOTOR_ALL_MASK;
    firmware_probe_init(aethor_application_probe_emit,
                        &initial_probe_snapshot,
                        HAL_GetTick());
    if (bsp_can_init(configuration) != FDCAN_DRIVER_STATUS_OK)
    {
        dual_motor_controller_on_can_start_failure();
    }
    return 0;
}

/**
 * @brief Run one 5 ms USB, PA15 key, controller, and probe service cycle.
 */
void aethor_application_service(void)
{
    uint32_t current_time_ms = HAL_GetTick();
    uint8_t user_key_is_pressed =
        (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET)
            ? 1U
            : 0U;
    FirmwareProbeSnapshot probe_snapshot;

    usb_cdc_transport_service();
    dual_motor_controller_step(current_time_ms, user_key_is_pressed);
    probe_snapshot = aethor_application_probe_snapshot();
    firmware_probe_observe(&probe_snapshot, current_time_ms);
}

/**
 * @brief Forward one accepted FDCAN1 frame from ISR context to the key controller.
 * @param frame Accepted classic CAN frame.
 */
void fdcan_classic_frame_received(const FdcanClassicFrame *frame)
{
    dual_motor_controller_on_can_frame(frame, HAL_GetTick());
}

/**
 * @brief Forward a latched Bus-Off event without restarting FDCAN in the ISR.
 */
void fdcan_classic_bus_off_received(void)
{
    dual_motor_controller_on_bus_off();
}
