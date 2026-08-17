/**
 * @file bsp_fdcan.c
 * @brief Strict STM32H7 FDCAN1 classic-CAN transport and error supervision.
 */

#include "bsp_fdcan.h"

#include <stddef.h>
#include <string.h>

static FdcanDriverState fdcan_driver_state;

/**
 * @brief Configure one exact FIFO0 filter for each active motor Master ID.
 * @param configuration Validated robot configuration.
 * @return Explicit driver status.
 */
FdcanDriverStatus can_filter_init(const RobotConfiguration *configuration)
{
    FDCAN_FilterTypeDef standard_filter = {0};
    uint8_t joint_index;
    uint8_t filter_index = 0U;

    if ((robot_config_validate(configuration) != ROBOT_CONFIG_STATUS_OK) ||
        (hfdcan1.Init.StdFiltersNbr < ROBOT_JOINT_COUNT))
    {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    standard_filter.IdType = FDCAN_STANDARD_ID;
    standard_filter.FilterType = FDCAN_FILTER_MASK;
    standard_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    standard_filter.FilterID2 = 0x7FFU;

    for (joint_index = 0U; joint_index < ROBOT_JOINT_COUNT; ++joint_index)
    {
        if ((configuration->active_joint_mask & (uint8_t)(1U << joint_index)) == 0U)
        {
            continue;
        }
        standard_filter.FilterIndex = filter_index;
        standard_filter.FilterID1 = configuration->joint[joint_index].master_id;
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &standard_filter) != HAL_OK)
        {
            return FDCAN_DRIVER_STATUS_HAL_ERROR;
        }
        filter_index++;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }
    if (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1U) != HAL_OK)
    {
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    return FDCAN_DRIVER_STATUS_OK;
}

/**
 * @brief Start FDCAN1 as the only active motor bus and enable monitored interrupts.
 * @param configuration Validated robot configuration used to build exact filters.
 * @return Explicit driver status.
 */
FdcanDriverStatus bsp_can_init(const RobotConfiguration *configuration)
{
    uint32_t notification_mask = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_ERROR_WARNING |
                                 FDCAN_IT_ERROR_PASSIVE |
                                 FDCAN_IT_BUS_OFF;

    memset(&fdcan_driver_state, 0, sizeof(fdcan_driver_state));
    if (can_filter_init(configuration) != FDCAN_DRIVER_STATUS_OK)
    {
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, notification_mask, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    fdcan_driver_state.started = 1U;
    return FDCAN_DRIVER_STATUS_OK;
}

/**
 * @brief Send one validated 11-bit classic CAN data frame without BRS.
 * @param fdcan_handle HAL FDCAN handle; only FDCAN1 is accepted for motor traffic.
 * @param frame Validated classic CAN frame.
 * @return Explicit driver status.
 */
FdcanDriverStatus fdcan_classic_send(FDCAN_HandleTypeDef *fdcan_handle,
                                     const FdcanClassicFrame *frame)
{
    FDCAN_TxHeaderTypeDef transmit_header = {0};
    uint8_t dlc;

    if ((fdcan_handle == NULL) || (frame == NULL))
    {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if ((fdcan_handle != &hfdcan1) || (fdcan_driver_state.started == 0U))
    {
        return FDCAN_DRIVER_STATUS_NOT_READY;
    }
    if (fdcan_driver_state.bus_off_latched != 0U)
    {
        return FDCAN_DRIVER_STATUS_BUS_OFF;
    }
    if ((fdcan_classic_length_to_dlc(frame->length, &dlc) != FDCAN_CLASSIC_STATUS_OK) ||
        (frame->identifier > FDCAN_CLASSIC_MAX_IDENTIFIER))
    {
        return FDCAN_DRIVER_STATUS_INVALID_FRAME;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan_handle) == 0U)
    {
        return FDCAN_DRIVER_STATUS_TX_QUEUE_FULL;
    }

    transmit_header.Identifier = frame->identifier;
    transmit_header.IdType = FDCAN_STANDARD_ID;
    transmit_header.TxFrameType = FDCAN_DATA_FRAME;
    transmit_header.DataLength = dlc;
    transmit_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    transmit_header.BitRateSwitch = FDCAN_BRS_OFF;
    transmit_header.FDFormat = FDCAN_CLASSIC_CAN;
    transmit_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    transmit_header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan_handle,
                                      &transmit_header,
                                      (uint8_t *)frame->data) != HAL_OK)
    {
        fdcan_driver_state.transmit_error_count++;
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }

    return FDCAN_DRIVER_STATUS_OK;
}

/**
 * @brief Receive and strictly validate one 11-bit classic CAN data frame.
 * @param fdcan_handle HAL FDCAN handle.
 * @param frame Destination classic CAN frame.
 * @return Explicit driver status.
 */
FdcanDriverStatus fdcan_classic_receive(FDCAN_HandleTypeDef *fdcan_handle,
                                        FdcanClassicFrame *frame)
{
    FDCAN_RxHeaderTypeDef receive_header = {0};
    uint8_t payload[FDCAN_CLASSIC_MAX_DATA_LENGTH] = {0U};
    uint8_t length;

    if ((fdcan_handle == NULL) || (frame == NULL))
    {
        return FDCAN_DRIVER_STATUS_INVALID_ARGUMENT;
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(fdcan_handle, FDCAN_RX_FIFO0) == 0U)
    {
        return FDCAN_DRIVER_STATUS_NO_MESSAGE;
    }
    if (HAL_FDCAN_GetRxMessage(fdcan_handle,
                               FDCAN_RX_FIFO0,
                               &receive_header,
                               payload) != HAL_OK)
    {
        fdcan_driver_state.receive_error_count++;
        return FDCAN_DRIVER_STATUS_HAL_ERROR;
    }
    if ((receive_header.IdType != FDCAN_STANDARD_ID) ||
        (receive_header.RxFrameType != FDCAN_DATA_FRAME) ||
        (receive_header.FDFormat != FDCAN_CLASSIC_CAN) ||
        (receive_header.BitRateSwitch != FDCAN_BRS_OFF) ||
        (fdcan_classic_dlc_to_length((uint8_t)receive_header.DataLength, &length) !=
         FDCAN_CLASSIC_STATUS_OK))
    {
        fdcan_driver_state.receive_error_count++;
        return FDCAN_DRIVER_STATUS_INVALID_FRAME;
    }
    if (fdcan_classic_frame_init(frame,
                                 (uint16_t)receive_header.Identifier,
                                 payload,
                                 length) != FDCAN_CLASSIC_STATUS_OK)
    {
        fdcan_driver_state.receive_error_count++;
        return FDCAN_DRIVER_STATUS_INVALID_FRAME;
    }

    return FDCAN_DRIVER_STATUS_OK;
}

/**
 * @brief Get the read-only FDCAN1 health snapshot.
 * @return Address of the static driver state.
 */
const FdcanDriverState *fdcan_classic_get_state(void)
{
    return &fdcan_driver_state;
}

/**
 * @brief Weak application hook invoked for each accepted FDCAN1 frame.
 * @param frame Accepted classic CAN frame.
 */
__weak void fdcan_classic_frame_received(const FdcanClassicFrame *frame)
{
    (void)frame;
}

/**
 * @brief Weak safety hook invoked once FDCAN1 enters Bus-Off.
 */
__weak void fdcan_classic_bus_off_received(void)
{
}

/**
 * @brief Drain all available FDCAN1 FIFO0 frames during one receive callback.
 */
void fdcan1_rx_callback(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U)
    {
        FdcanClassicFrame received_frame;

        if (fdcan_classic_receive(&hfdcan1, &received_frame) == FDCAN_DRIVER_STATUS_OK)
        {
            fdcan_classic_frame_received(&received_frame);
        }
    }
}

/**
 * @brief Compatibility placeholder because FDCAN2 motor traffic is intentionally disabled.
 */
void fdcan2_rx_callback(void)
{
}

/**
 * @brief Compatibility placeholder because FDCAN3 motor traffic is intentionally disabled.
 */
void fdcan3_rx_callback(void)
{
}

/**
 * @brief HAL callback for FIFO0 notifications; only FDCAN1 is serviced.
 * @param fdcan_handle HAL FDCAN handle that raised the callback.
 * @param receive_interrupts Active FIFO0 interrupt flags.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan_handle,
                               uint32_t receive_interrupts)
{
    if ((fdcan_handle == &hfdcan1) &&
        ((receive_interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U))
    {
        fdcan1_rx_callback();
    }
}

/**
 * @brief HAL callback that records Error Warning, Error Passive, and Bus-Off.
 * @param fdcan_handle HAL FDCAN handle that raised the callback.
 * @param error_status_interrupts Active error-status interrupt flags.
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan_handle,
                                   uint32_t error_status_interrupts)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};

    if (fdcan_handle != &hfdcan1)
    {
        return;
    }

    if (HAL_FDCAN_GetProtocolStatus(fdcan_handle, &protocol_status) == HAL_OK)
    {
        fdcan_driver_state.warning = (uint8_t)protocol_status.Warning;
        fdcan_driver_state.error_passive = (uint8_t)protocol_status.ErrorPassive;
    }
    if ((error_status_interrupts & FDCAN_IT_ERROR_WARNING) != 0U)
    {
        fdcan_driver_state.warning_event_count++;
    }
    if ((error_status_interrupts & FDCAN_IT_ERROR_PASSIVE) != 0U)
    {
        fdcan_driver_state.error_passive_event_count++;
    }
    if ((error_status_interrupts & FDCAN_IT_BUS_OFF) != 0U)
    {
        fdcan_driver_state.bus_off_latched = 1U;
        fdcan_driver_state.started = 0U;
        fdcan_driver_state.bus_off_event_count++;
        fdcan_classic_bus_off_received();
    }
}

/**
 * @brief Compatibility wrapper around the strict classic CAN send API.
 * @param fdcan_handle HAL FDCAN handle.
 * @param identifier Standard CAN identifier.
 * @param data Payload bytes.
 * @param length Payload length.
 * @return Zero on success, otherwise one.
 */
uint8_t fdcanx_send_data(hcan_t *fdcan_handle, uint16_t identifier,
                         uint8_t *data, uint32_t length)
{
    FdcanClassicFrame frame;

    if ((length > UINT8_MAX) ||
        (fdcan_classic_frame_init(&frame, identifier, data, (uint8_t)length) !=
         FDCAN_CLASSIC_STATUS_OK))
    {
        return 1U;
    }
    return (fdcan_classic_send(fdcan_handle, &frame) == FDCAN_DRIVER_STATUS_OK) ? 0U : 1U;
}

/**
 * @brief Compatibility wrapper around the strict classic CAN receive API.
 * @param fdcan_handle HAL FDCAN handle.
 * @param received_identifier Receives the standard identifier.
 * @param buffer Receives up to eight payload bytes.
 * @return Received length, or zero when no valid frame was available.
 */
uint8_t fdcanx_receive(hcan_t *fdcan_handle, uint16_t *received_identifier,
                       uint8_t *buffer)
{
    FdcanClassicFrame frame;

    if ((received_identifier == NULL) || (buffer == NULL) ||
        (fdcan_classic_receive(fdcan_handle, &frame) != FDCAN_DRIVER_STATUS_OK))
    {
        return 0U;
    }

    *received_identifier = frame.identifier;
    memcpy(buffer, frame.data, frame.length);
    return frame.length;
}
