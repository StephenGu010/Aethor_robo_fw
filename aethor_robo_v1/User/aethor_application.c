/**
 * @file aethor_application.c
 * @brief Bridges STM32 HAL FDCAN/USB CDC with the hardware-independent controller.
 */

#include "aethor_application.h"

#include <stdio.h>
#include <string.h>

#include "bsp_fdcan.h"
#include "firmware_probe.h"
#include "joint_controller.h"
#include "robot_config.h"
#include "usb_cdc_transport.h"
#include "usb_command.h"
#include "usbd_cdc_if.h"

static uint32_t aethor_application_command_sequence;

/**
 * @brief Queue one structured probe line on the task-context USB CDC transmitter.
 * @param line Bounded diagnostic line without CR/LF.
 * @return Zero when queued, otherwise non-zero.
 */
static int aethor_application_probe_emit(const char *line)
{
    return (usb_cdc_transport_queue_line(line) == USB_CDC_TRANSPORT_STATUS_OK) ? 0 : -1;
}

/**
 * @brief Maps the joint controller state machine to stable probe stages.
 * @param controller_state Current controller state.
 * @return Stable firmware probe stage.
 */
static FirmwareProbeStage aethor_application_probe_stage(
    const JointControllerState *controller_state)
{
    switch (controller_state->stage)
    {
        case JOINT_CONTROLLER_STAGE_CONFIG_VALIDATE:
            return FIRMWARE_PROBE_STAGE_CONFIG_VALIDATE;
        case JOINT_CONTROLLER_STAGE_BOOT_DELAY:
            return FIRMWARE_PROBE_STAGE_BOOT;
        case JOINT_CONTROLLER_STAGE_MODE_SETUP:
            return FIRMWARE_PROBE_STAGE_MODE_SETUP;
        case JOINT_CONTROLLER_STAGE_RANGE_DISCOVERY:
            return FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY;
        case JOINT_CONTROLLER_STAGE_READY:
            return FIRMWARE_PROBE_STAGE_READY;
        case JOINT_CONTROLLER_STAGE_ENABLING:
            return FIRMWARE_PROBE_STAGE_ENABLING;
        case JOINT_CONTROLLER_STAGE_MOVING:
            return FIRMWARE_PROBE_STAGE_MOVING;
        case JOINT_CONTROLLER_STAGE_HOLDING:
            return FIRMWARE_PROBE_STAGE_HOLDING;
        case JOINT_CONTROLLER_STAGE_FAULT:
        default:
            return FIRMWARE_PROBE_STAGE_FAULT;
    }
}

/**
 * @brief Finds the first one-based joint represented by a bit mask.
 * @param joint_mask Seven-bit joint mask.
 * @return One-based joint number, or zero when the mask is empty.
 */
static uint8_t aethor_application_first_joint(uint8_t joint_mask)
{
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        if ((joint_mask & (uint8_t)(1U << joint_index)) != 0U)
        {
            return (uint8_t)(joint_index + 1U);
        }
    }
    return 0U;
}

/**
 * @brief Calculates the spread between first-arrival timestamps of arrived active joints.
 * @param controller_state Current controller state.
 * @return Arrival spread in milliseconds, or zero before two joints arrive.
 */
static uint32_t aethor_application_arrival_spread_ms(
    const JointControllerState *controller_state)
{
    uint32_t earliest_time_ms = UINT32_MAX;
    uint32_t latest_time_ms = 0U;
    uint8_t arrived_count = 0U;
    uint8_t joint_index;

    for (joint_index = 0U; joint_index < JOINT_COUNT; ++joint_index)
    {
        uint8_t joint_bit = (uint8_t)(1U << joint_index);
        uint32_t arrival_time_ms = controller_state->first_arrival_time_ms[joint_index];

        if (((controller_state->active_mask & joint_bit) == 0U) ||
            ((controller_state->arrived_mask & joint_bit) == 0U) ||
            (arrival_time_ms == 0U))
        {
            continue;
        }
        if (arrival_time_ms < earliest_time_ms)
        {
            earliest_time_ms = arrival_time_ms;
        }
        if (arrival_time_ms > latest_time_ms)
        {
            latest_time_ms = arrival_time_ms;
        }
        arrived_count++;
    }
    return (arrived_count >= 2U) ? (latest_time_ms - earliest_time_ms) : 0U;
}

/**
 * @brief Captures the current controller and FDCAN health for transition probes.
 * @return Bounded snapshot suitable for task-context comparison.
 */
static FirmwareProbeSnapshot aethor_application_probe_snapshot(void)
{
    const JointControllerState *controller_state = joint_controller_get_state();
    const FdcanDriverState *can_state = fdcan_classic_get_state();
    FirmwareProbeSnapshot snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.stage = aethor_application_probe_stage(controller_state);
    snapshot.active_mask = controller_state->active_mask;
    snapshot.ready_mask = (uint8_t)(controller_state->mode_ready_mask &
                                    controller_state->ranges_ready_mask &
                                    controller_state->feedback_ready_mask &
                                    controller_state->active_mask);
    snapshot.enabled_mask = controller_state->enabled_mask;
    snapshot.arrived_mask = controller_state->arrived_mask;
    snapshot.fault_code = (uint8_t)controller_state->safety_reason;
    snapshot.fault_joint = aethor_application_first_joint(controller_state->motor_fault_mask);
    snapshot.bus_off = can_state->bus_off_latched;
    return snapshot;
}

/**
 * @brief Returns a stable command name without copying the caller's payload into logs.
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
        case USB_COMMAND_TYPE_SELECT:
            return "SELECT";
        case USB_COMMAND_TYPE_START:
            return "START";
        case USB_COMMAND_TYPE_STOP:
            return "STOP";
        case USB_COMMAND_TYPE_DISABLE:
            return "DISABLE";
        case USB_COMMAND_TYPE_HOME:
            return "HOME";
        case USB_COMMAND_TYPE_MOVE_JOINTS:
            return "MOVE_JOINTS";
        case USB_COMMAND_TYPE_GET_JOINT_POSITIONS:
            return "GET_JOINT_POSITIONS";
        case USB_COMMAND_TYPE_GET_MOTOR_POSITIONS:
            return "GET_MOTOR_POSITIONS";
        case USB_COMMAND_TYPE_GET_STATE:
            return "GET_STATE";
        case USB_COMMAND_TYPE_GET_ENABLE:
            return "GET_ENABLE";
        case USB_COMMAND_TYPE_GET_CAPABILITIES:
            return "GET_CAPABILITIES";
        case USB_COMMAND_TYPE_GET_DH:
            return "GET_DH";
        case USB_COMMAND_TYPE_GET_CONFIG:
            return "GET_CONFIG";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Submit one controller frame to the active FDCAN1 classic bus.
 * @param frame Frame generated by the motor protocol layer.
 * @return Zero on success, otherwise non-zero.
 */
static int aethor_application_can_send(const FdcanClassicFrame *frame)
{
    return (fdcan_classic_send(&hfdcan1, frame) == FDCAN_DRIVER_STATUS_OK) ? 0 : -1;
}

/**
 * @brief Adapt the Cube USB CDC transmit result to the transport callback contract.
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
 * @brief Format the combined controller, feedback-age, and CAN health state.
 * @param response Destination response text.
 * @param response_capacity Destination capacity in bytes.
 * @return Zero when the response fitted, otherwise non-zero.
 */
static int aethor_application_format_state(char *response, size_t response_capacity)
{
    const JointControllerState *controller_state = joint_controller_get_state();
    const FdcanDriverState *can_state = fdcan_classic_get_state();
    const UsbCdcTransportStatistics *usb_statistics =
        usb_cdc_transport_get_statistics();
    const FirmwareProbeStatistics *probe_statistics =
        firmware_probe_get_statistics();
    uint32_t now_ms = HAL_GetTick();
    uint32_t first_feedback_age_ms =
        now_ms - controller_state->last_feedback_time_ms[0];
    uint32_t second_feedback_age_ms =
        now_ms - controller_state->last_feedback_time_ms[1];
    uint8_t selected_index = (controller_state->selected_joint > 0U)
                                 ? (uint8_t)(controller_state->selected_joint - 1U)
                                 : 0U;

    int written_length = snprintf(response,
                                  response_capacity,
                                  "ok stage=%s active=0x%02X mode=0x%02X ranges=0x%02X feedback=0x%02X pending=0x%02X enabled=0x%02X arrived=0x%02X motor_fault=0x%02X lock=%u safety=%u motor_state=%u,%u joint=%.3f,%.3f motor=%.3f,%.3f fb_age=%lu,%lu arrival=%lu arrivals=%lu,%lu,%lu,%lu,%lu,%lu,%lu sync=%lu can_started=%u warning=%u passive=%u busoff=%u usb_drop=%lu probe_drop=%lu",
                                  firmware_probe_stage_name(
                                      aethor_application_probe_stage(controller_state)),
                                  (unsigned int)controller_state->active_mask,
                                  (unsigned int)controller_state->mode_ready_mask,
                                  (unsigned int)controller_state->ranges_ready_mask,
                                  (unsigned int)controller_state->feedback_ready_mask,
                                  (unsigned int)controller_state->enable_pending_mask,
                                  (unsigned int)controller_state->enabled_mask,
                                  (unsigned int)controller_state->arrived_mask,
                                  (unsigned int)controller_state->motor_fault_mask,
                                  (unsigned int)controller_state->commissioning_lock_active,
                                  (unsigned int)controller_state->safety_reason,
                                  (unsigned int)controller_state->motor_state[0],
                                  (unsigned int)controller_state->motor_state[1],
                                  (double)controller_state->joint_position_degrees[0],
                                  (double)controller_state->joint_position_degrees[1],
                                  (double)controller_state->motor_position_degrees[0],
                                  (double)controller_state->motor_position_degrees[1],
                                  (unsigned long)first_feedback_age_ms,
                                  (unsigned long)second_feedback_age_ms,
                                  (unsigned long)controller_state->first_arrival_time_ms[
                                      selected_index],
                                  (unsigned long)controller_state->first_arrival_time_ms[0],
                                  (unsigned long)controller_state->first_arrival_time_ms[1],
                                  (unsigned long)controller_state->first_arrival_time_ms[2],
                                  (unsigned long)controller_state->first_arrival_time_ms[3],
                                  (unsigned long)controller_state->first_arrival_time_ms[4],
                                  (unsigned long)controller_state->first_arrival_time_ms[5],
                                  (unsigned long)controller_state->first_arrival_time_ms[6],
                                  (unsigned long)aethor_application_arrival_spread_ms(
                                      controller_state),
                                  (unsigned int)can_state->started,
                                  (unsigned int)can_state->warning,
                                  (unsigned int)can_state->error_passive,
                                  (unsigned int)can_state->bus_off_latched,
                                  (unsigned long)usb_statistics->dropped_response_count,
                                  (unsigned long)probe_statistics->dropped_line_count);

    return ((written_length >= 0) && ((size_t)written_length < response_capacity)) ? 0 : -1;
}

/**
 * @brief Parse and execute one complete USB CDC protocol line.
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
    JointControllerStatus controller_status;
    UsbCommandStatus parse_status =
        usb_command_process_line(line, &command, response, response_capacity);

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

    if (command.type == USB_COMMAND_TYPE_GET_STATE)
    {
        if (aethor_application_format_state(response, response_capacity) != 0)
        {
            (void)snprintf(response, response_capacity, "err response-too-small");
        }
        controller_status = JOINT_CONTROLLER_STATUS_OK;
    }
    else if ((command.type == USB_COMMAND_TYPE_PING) ||
             (command.type == USB_COMMAND_TYPE_ECHO))
    {
        controller_status = JOINT_CONTROLLER_STATUS_OK;
    }
    else
    {
        controller_status = joint_controller_submit(&command,
                                                    HAL_GetTick(),
                                                    response,
                                                    response_capacity);
        if (controller_status != JOINT_CONTROLLER_STATUS_OK)
        {
            (void)snprintf(response,
                           response_capacity,
                           "err controller=%u",
                           (unsigned int)controller_status);
        }
    }

    firmware_probe_record_command(aethor_application_command_name(command.type),
                                  aethor_application_command_sequence,
                                  (controller_status == JOINT_CONTROLLER_STATUS_OK)
                                      ? "accepted"
                                      : "rejected",
                                  HAL_GetTick());
    return 0;
}

/**
 * @brief Initialize static controller and USB transport state.
 * @return Zero when the FDCAN bus initialized, otherwise non-zero.
 */
int aethor_application_init(void)
{
    FirmwareProbeSnapshot initial_probe_snapshot;
    const RobotConfiguration *configuration = robot_config_get();

    usb_cdc_transport_init(aethor_application_usb_transmit,
                           aethor_application_process_line);
    aethor_application_command_sequence = 0U;
    memset(&initial_probe_snapshot, 0, sizeof(initial_probe_snapshot));
    initial_probe_snapshot.stage = FIRMWARE_PROBE_STAGE_CONFIG_VALIDATE;
    initial_probe_snapshot.active_mask = configuration->active_joint_mask;
    firmware_probe_init(aethor_application_probe_emit,
                        &initial_probe_snapshot,
                        HAL_GetTick());
    joint_controller_init(aethor_application_can_send,
                          configuration,
                          HAL_GetTick());
    if (bsp_can_init(configuration) != FDCAN_DRIVER_STATUS_OK)
    {
        joint_controller_on_can_start_failure();
    }
    return 0;
}

/**
 * @brief Run one 5 ms application service cycle from the default RTOS task.
 */
void aethor_application_service(void)
{
    usb_cdc_transport_service();
    joint_controller_step(HAL_GetTick());
    {
        FirmwareProbeSnapshot probe_snapshot = aethor_application_probe_snapshot();

        firmware_probe_observe(&probe_snapshot, HAL_GetTick());
    }
}

/**
 * @brief Forward one accepted FDCAN frame from the interrupt callback.
 * @param frame Accepted classic CAN frame.
 */
void fdcan_classic_frame_received(const FdcanClassicFrame *frame)
{
    joint_controller_on_can_frame(frame, HAL_GetTick());
}

/**
 * @brief Forward a latched Bus-Off event without restarting FDCAN in the ISR.
 */
void fdcan_classic_bus_off_received(void)
{
    joint_controller_on_bus_off();
}
