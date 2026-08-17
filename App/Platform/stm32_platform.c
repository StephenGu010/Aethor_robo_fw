/**
 * @file stm32_platform.c
 * @brief Adapts static Aethor queues to STM32H723 FDCAN1 and USB CDC callbacks.
 */

#include "stm32_platform.h"

#include <stddef.h>
#include <string.h>

#include "fdcan.h"
#include "usbd_cdc_if.h"

#define STM32_PLATFORM_CAN_RX_ISR_BUDGET (32U)

static CanRxInbox platform_can_rx_inbox;
static CanTxScheduler platform_can_tx_scheduler;
static UsbCdcStream platform_usb_stream;
static Stm32PlatformDiagnostics platform_diagnostics;
static Stm32PlatformIsrNotifier platform_can_rx_notifier;
static Stm32PlatformIsrNotifier platform_usb_rx_notifier;
static Stm32PlatformIsrNotifier platform_usb_tx_notifier;

/**
 * @brief Maps a Classic CAN byte length to the HAL DLC constant.
 */
static uint8_t stm32_platform_length_to_hal_dlc(uint8_t length, uint32_t *hal_dlc)
{
    static const uint32_t dlc_by_length[CAN_CLASSIC_MAX_DATA_LENGTH + 1U] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    if ((hal_dlc == NULL) || (length > CAN_CLASSIC_MAX_DATA_LENGTH))
    {
        return 0U;
    }
    *hal_dlc = dlc_by_length[length];
    return 1U;
}

/**
 * @brief Maps a HAL Classic CAN DLC constant to its byte length.
 */
static uint8_t stm32_platform_hal_dlc_to_length(uint32_t hal_dlc, uint8_t *length)
{
    uint8_t candidate_length;

    if (length == NULL)
    {
        return 0U;
    }
    for (candidate_length = 0U;
         candidate_length <= CAN_CLASSIC_MAX_DATA_LENGTH;
         ++candidate_length)
    {
        uint32_t candidate_dlc;

        if ((stm32_platform_length_to_hal_dlc(candidate_length, &candidate_dlc) != 0U) &&
            (candidate_dlc == hal_dlc))
        {
            *length = candidate_length;
            return 1U;
        }
    }
    return 0U;
}

/**
 * @brief Adapts the generated CDC transmitter to the pure stream result enum.
 */
static UsbCdcStreamTransmitResult stm32_platform_usb_transmit(const uint8_t *data,
                                                              uint16_t length)
{
    uint8_t usb_status;

    if ((data == NULL) || (length == 0U))
    {
        return USB_CDC_STREAM_TRANSMIT_ERROR;
    }

    usb_status = CDC_Transmit_HS((uint8_t *)data, length);
    if (usb_status == USBD_OK)
    {
        return USB_CDC_STREAM_TRANSMIT_OK;
    }
    if (usb_status == USBD_BUSY)
    {
        return USB_CDC_STREAM_TRANSMIT_BUSY;
    }
    return USB_CDC_STREAM_TRANSMIT_ERROR;
}

/**
 * @brief Configures seven exact standard-ID filters and rejects all other traffic.
 */
static Stm32PlatformStatus stm32_platform_configure_can_filters(
    const ArmConfig *configuration)
{
    FDCAN_FilterTypeDef filter = {0};
    uint8_t joint_index;

    if ((configuration == NULL) ||
        (hfdcan1.Init.StdFiltersNbr < ARM_JOINT_COUNT))
    {
        return STM32_PLATFORM_STATUS_INVALID_ARGUMENT;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID2 = CAN_STANDARD_MAX_IDENTIFIER;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    {
        filter.FilterIndex = joint_index;
        filter.FilterID1 = configuration->joints[joint_index].master_id;
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
        {
            return STM32_PLATFORM_STATUS_CAN_CONFIG_ERROR;
        }
    }

    if ((HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT,
                                      FDCAN_REJECT_REMOTE,
                                      FDCAN_REJECT_REMOTE) != HAL_OK) ||
        (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1U) != HAL_OK))
    {
        return STM32_PLATFORM_STATUS_CAN_CONFIG_ERROR;
    }
    return STM32_PLATFORM_STATUS_OK;
}

/**
 * @brief Initializes static queues, exact filters, FDCAN1, and USB transport state.
 */
Stm32PlatformStatus stm32_platform_init(const ArmConfig *configuration)
{
    ArmConfigValidation validation;
    Stm32PlatformStatus filter_status;
    uint32_t notification_mask = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_ERROR_WARNING |
                                 FDCAN_IT_ERROR_PASSIVE |
                                 FDCAN_IT_BUS_OFF;

    memset(&platform_diagnostics, 0, sizeof(platform_diagnostics));
    can_rx_inbox_init(&platform_can_rx_inbox);
    can_tx_scheduler_init(&platform_can_tx_scheduler);
    usb_cdc_stream_init(&platform_usb_stream, stm32_platform_usb_transmit);
    platform_diagnostics.initialized = 1U;

    if ((configuration == NULL) ||
        !arm_config_validate_schema(configuration, &validation))
    {
        platform_diagnostics.initialization_status =
            STM32_PLATFORM_STATUS_INVALID_ARGUMENT;
        return platform_diagnostics.initialization_status;
    }

    filter_status = stm32_platform_configure_can_filters(configuration);
    if (filter_status != STM32_PLATFORM_STATUS_OK)
    {
        platform_diagnostics.initialization_status = filter_status;
        return filter_status;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        platform_diagnostics.initialization_status = STM32_PLATFORM_STATUS_CAN_START_ERROR;
        return platform_diagnostics.initialization_status;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, notification_mask, 0U) != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(&hfdcan1);
        platform_diagnostics.initialization_status =
            STM32_PLATFORM_STATUS_CAN_NOTIFICATION_ERROR;
        return platform_diagnostics.initialization_status;
    }

    platform_diagnostics.can_started = 1U;
    platform_diagnostics.initialization_status = STM32_PLATFORM_STATUS_OK;
    return STM32_PLATFORM_STATUS_OK;
}

/**
 * @brief Registers task notification hooks invoked by STM32 peripheral callbacks.
 */
void stm32_platform_set_isr_notifiers(
    Stm32PlatformIsrNotifier can_rx_notifier,
    Stm32PlatformIsrNotifier usb_rx_notifier,
    Stm32PlatformIsrNotifier usb_tx_notifier)
{
    platform_can_rx_notifier = can_rx_notifier;
    platform_usb_rx_notifier = usb_rx_notifier;
    platform_usb_tx_notifier = usb_tx_notifier;
}

/** @brief Copies one USB receive packet into the static ISR ring. */
UsbCdcStreamStatus stm32_platform_usb_receive_isr(const uint8_t *data,
                                                  uint32_t length)
{
    UsbCdcStreamStatus receive_status =
        usb_cdc_stream_receive_isr(&platform_usb_stream, data, length);

    if ((receive_status == USB_CDC_STREAM_STATUS_OK) &&
        (platform_usb_rx_notifier != NULL))
    {
        platform_usb_rx_notifier();
    }
    return receive_status;
}

/** @brief Releases the USB in-flight buffer after endpoint completion. */
void stm32_platform_usb_tx_complete_isr(void)
{
    usb_cdc_stream_on_tx_complete_isr(&platform_usb_stream);
    if (platform_usb_tx_notifier != NULL)
    {
        platform_usb_tx_notifier();
    }
}

/** @brief Extracts one complete USB protocol line in task context. */
UsbCdcStreamStatus stm32_platform_usb_next_line(char *line,
                                                size_t line_capacity,
                                                uint16_t *line_length)
{
    return usb_cdc_stream_next_line(&platform_usb_stream,
                                    line,
                                    line_capacity,
                                    line_length);
}

/** @brief Queues one high-priority encoded USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_high_priority(const uint8_t *data,
                                                          uint16_t length)
{
    return usb_cdc_stream_queue_high_priority(&platform_usb_stream, data, length);
}

/** @brief Queues one query response encoded USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_query(const uint8_t *data,
                                                  uint16_t length)
{
    return usb_cdc_stream_queue_query(&platform_usb_stream, data, length);
}

/** @brief Queues one replaceable telemetry USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_telemetry(const uint8_t *data,
                                                      uint16_t length)
{
    return usb_cdc_stream_queue_telemetry(&platform_usb_stream, data, length);
}

/** @brief Services one USB transmit attempt in task context. */
void stm32_platform_usb_service_tx(void)
{
    usb_cdc_stream_service_tx(&platform_usb_stream);
}

/** @brief Pops one raw CAN frame copied by the FDCAN ISR. */
CanRxInboxStatus stm32_platform_can_pop_received(CanFrame *frame)
{
    return can_rx_inbox_pop(&platform_can_rx_inbox, frame);
}

/** @brief Queues one CAN frame with its scheduling priority. */
CanTxSchedulerStatus stm32_platform_can_submit(CanTxPriority priority,
                                               const CanFrame *frame)
{
    return can_tx_scheduler_submit(&platform_can_tx_scheduler, priority, frame);
}

/** @brief Atomically queues one ordered J1 through J7 control group. */
CanTxSchedulerStatus stm32_platform_can_submit_control_group(
    const CanFrame *frames,
    uint8_t frame_count)
{
    return can_tx_scheduler_submit_control_group(&platform_can_tx_scheduler,
                                                 frames,
                                                 frame_count);
}

/**
 * @brief Moves a bounded number of scheduled frames into the HAL TX FIFO.
 */
Stm32PlatformStatus stm32_platform_can_service_tx(uint8_t maximum_frame_count)
{
    uint8_t transmitted_count;

    if (platform_can_rx_inbox.bus_off_latched != 0U)
    {
        return STM32_PLATFORM_STATUS_CAN_BUS_OFF;
    }
    if (platform_diagnostics.can_started == 0U)
    {
        return platform_diagnostics.initialization_status;
    }

    for (transmitted_count = 0U;
         transmitted_count < maximum_frame_count;
         ++transmitted_count)
    {
        CanFrame frame;
        CanTxPriority priority;
        FDCAN_TxHeaderTypeDef transmit_header = {0};

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
        {
            break;
        }
        if (can_tx_scheduler_pop(&platform_can_tx_scheduler, &frame, &priority) !=
            CAN_TX_SCHEDULER_STATUS_OK)
        {
            break;
        }
        (void)priority;
        if (stm32_platform_length_to_hal_dlc(frame.length,
                                             &transmit_header.DataLength) == 0U)
        {
            ++platform_diagnostics.can_tx_error_count;
            return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
        }

        transmit_header.Identifier = frame.identifier;
        transmit_header.IdType = FDCAN_STANDARD_ID;
        transmit_header.TxFrameType = FDCAN_DATA_FRAME;
        transmit_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        transmit_header.BitRateSwitch = FDCAN_BRS_OFF;
        transmit_header.FDFormat = FDCAN_CLASSIC_CAN;
        transmit_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        transmit_header.MessageMarker = 0U;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                          &transmit_header,
                                          frame.data) != HAL_OK)
        {
            ++platform_diagnostics.can_tx_error_count;
            return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
        }
        ++platform_diagnostics.can_tx_success_count;
    }
    return STM32_PLATFORM_STATUS_OK;
}

/** @brief Returns read-only platform transport diagnostics. */
const Stm32PlatformDiagnostics *stm32_platform_get_diagnostics(void)
{
    platform_diagnostics.can_rx_frame_count =
        platform_can_rx_inbox.received_frame_count;
    platform_diagnostics.can_rx_overflow_count =
        platform_can_rx_inbox.dropped_frame_count;
    platform_diagnostics.can_tx_queue_high_watermark =
        platform_can_tx_scheduler.high_watermark;
    platform_diagnostics.control_group_reject_count =
        platform_can_tx_scheduler.atomic_group_reject_count;
    platform_diagnostics.usb_rx_byte_count =
        platform_usb_stream.received_byte_count;
    platform_diagnostics.usb_rx_overflow_count =
        platform_usb_stream.dropped_byte_count;
    platform_diagnostics.usb_overlong_line_count =
        platform_usb_stream.overlong_line_count;
    platform_diagnostics.usb_high_queue_high_watermark =
        platform_usb_stream.high_priority_high_watermark;
    platform_diagnostics.usb_query_queue_high_watermark =
        platform_usb_stream.query_high_watermark;
    platform_diagnostics.usb_telemetry_queue_high_watermark =
        platform_usb_stream.telemetry_high_watermark;
    platform_diagnostics.usb_telemetry_drop_count =
        platform_usb_stream.telemetry_replaced_count;
    platform_diagnostics.usb_high_queue_full_count =
        platform_usb_stream.high_priority_queue_full_count;
    platform_diagnostics.usb_transmit_busy_count =
        platform_usb_stream.transmit_busy_count;
    platform_diagnostics.usb_transmit_error_count =
        platform_usb_stream.transmit_error_count;
    return &platform_diagnostics;
}

/**
 * @brief Copies only validated raw FDCAN1 frames into the ISR inbox.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *fdcan_handle,
                               uint32_t receive_interrupts)
{
    uint8_t processed_count;
    uint8_t accepted_frame_count = 0U;

    if ((fdcan_handle != &hfdcan1) ||
        ((receive_interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    for (processed_count = 0U;
         (processed_count < STM32_PLATFORM_CAN_RX_ISR_BUDGET) &&
         (HAL_FDCAN_GetRxFifoFillLevel(fdcan_handle, FDCAN_RX_FIFO0) != 0U);
         ++processed_count)
    {
        FDCAN_RxHeaderTypeDef receive_header = {0};
        uint8_t payload[CAN_CLASSIC_MAX_DATA_LENGTH] = {0U};
        uint8_t length;
        CanFrame frame;

        if (HAL_FDCAN_GetRxMessage(fdcan_handle,
                                   FDCAN_RX_FIFO0,
                                   &receive_header,
                                   payload) != HAL_OK)
        {
            ++platform_diagnostics.can_rx_invalid_metadata_count;
            break;
        }
        if ((receive_header.IdType != FDCAN_STANDARD_ID) ||
            (receive_header.RxFrameType != FDCAN_DATA_FRAME) ||
            (receive_header.FDFormat != FDCAN_CLASSIC_CAN) ||
            (receive_header.BitRateSwitch != FDCAN_BRS_OFF) ||
            (receive_header.Identifier > CAN_STANDARD_MAX_IDENTIFIER) ||
            (stm32_platform_hal_dlc_to_length(receive_header.DataLength, &length) == 0U) ||
            (can_frame_init(&frame,
                            (uint16_t)receive_header.Identifier,
                            payload,
                            length) != CAN_FRAME_STATUS_OK))
        {
            ++platform_diagnostics.can_rx_invalid_metadata_count;
            continue;
        }
        if (can_rx_inbox_push_isr(&platform_can_rx_inbox, &frame) !=
            CAN_RX_INBOX_STATUS_OK)
        {
            ++platform_diagnostics.can_rx_overflow_count;
        }
        else
        {
            ++accepted_frame_count;
        }
    }
    if ((accepted_frame_count != 0U) && (platform_can_rx_notifier != NULL))
    {
        platform_can_rx_notifier();
    }
}

/**
 * @brief Latches warning, passive, and Bus-Off state without recovery in ISR context.
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *fdcan_handle,
                                   uint32_t error_status_interrupts)
{
    if (fdcan_handle != &hfdcan1)
    {
        return;
    }
    if ((error_status_interrupts & FDCAN_IT_BUS_OFF) != 0U)
    {
        can_rx_inbox_latch_bus_off_isr(&platform_can_rx_inbox);
        platform_diagnostics.can_started = 0U;
        ++platform_diagnostics.can_bus_off_count;
    }
}
