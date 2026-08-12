/**
 * @file stm32_platform.h
 * @brief Defines the STM32H723 FDCAN1 and USB CDC adapter boundary.
 */

#ifndef APP_PLATFORM_STM32_PLATFORM_H
#define APP_PLATFORM_STM32_PLATFORM_H

#include <stddef.h>
#include <stdint.h>

#include "arm_config.h"
#include "can_rx_inbox.h"
#include "can_tx_scheduler.h"
#include "usb_cdc_stream.h"

/**
 * @brief Reports STM32 platform initialization and non-blocking IO outcomes.
 */
typedef enum
{
    STM32_PLATFORM_STATUS_OK = 0,
    STM32_PLATFORM_STATUS_INVALID_ARGUMENT,
    STM32_PLATFORM_STATUS_CAN_CONFIG_ERROR,
    STM32_PLATFORM_STATUS_CAN_START_ERROR,
    STM32_PLATFORM_STATUS_CAN_NOTIFICATION_ERROR,
    STM32_PLATFORM_STATUS_CAN_TX_ERROR,
    STM32_PLATFORM_STATUS_CAN_BUS_OFF
} Stm32PlatformStatus;

/**
 * @brief Stores latched STM32 transport health and counters.
 */
typedef struct
{
    Stm32PlatformStatus initialization_status;
    uint32_t can_rx_invalid_metadata_count;
    uint32_t can_rx_overflow_count;
    uint32_t can_tx_success_count;
    uint32_t can_tx_error_count;
    uint32_t can_bus_off_count;
    uint8_t can_started;
    uint8_t initialized;
} Stm32PlatformDiagnostics;

/** @brief Signature of a zero-allocation ISR-to-task notification hook. */
typedef void (*Stm32PlatformIsrNotifier)(void);

/** @brief Initializes static queues, exact filters, FDCAN1, and USB transport state. */
Stm32PlatformStatus stm32_platform_init(const ArmConfig *configuration);

/**
 * @brief Registers task notification hooks invoked by STM32 peripheral callbacks.
 * @param can_rx_notifier Called after at least one CAN frame enters the RX inbox.
 * @param usb_rx_notifier Called after USB bytes enter the RX stream.
 * @param usb_tx_notifier Called after a USB transmit completes.
 */
void stm32_platform_set_isr_notifiers(
    Stm32PlatformIsrNotifier can_rx_notifier,
    Stm32PlatformIsrNotifier usb_rx_notifier,
    Stm32PlatformIsrNotifier usb_tx_notifier);

/** @brief Copies one USB receive packet into the static ISR ring. */
UsbCdcStreamStatus stm32_platform_usb_receive_isr(const uint8_t *data,
                                                  uint32_t length);

/** @brief Releases the USB in-flight buffer after endpoint completion. */
void stm32_platform_usb_tx_complete_isr(void);

/** @brief Extracts one complete USB protocol line in task context. */
UsbCdcStreamStatus stm32_platform_usb_next_line(char *line,
                                                size_t line_capacity,
                                                uint16_t *line_length);

/** @brief Queues one high-priority encoded USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_high_priority(const uint8_t *data,
                                                          uint16_t length);

/** @brief Queues one query response encoded USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_query(const uint8_t *data,
                                                  uint16_t length);

/** @brief Queues one replaceable telemetry USB frame. */
UsbCdcStreamStatus stm32_platform_usb_queue_telemetry(const uint8_t *data,
                                                      uint16_t length);

/** @brief Services one USB transmit attempt in task context. */
void stm32_platform_usb_service_tx(void);

/** @brief Pops one raw CAN frame copied by the FDCAN ISR. */
CanRxInboxStatus stm32_platform_can_pop_received(CanFrame *frame);

/** @brief Queues one CAN frame with its scheduling priority. */
CanTxSchedulerStatus stm32_platform_can_submit(CanTxPriority priority,
                                               const CanFrame *frame);

/** @brief Atomically queues one ordered J1 through J7 control group. */
CanTxSchedulerStatus stm32_platform_can_submit_control_group(
    const CanFrame *frames,
    uint8_t frame_count);

/** @brief Moves a bounded number of scheduled frames into the HAL TX FIFO. */
Stm32PlatformStatus stm32_platform_can_service_tx(uint8_t maximum_frame_count);

/** @brief Returns read-only platform transport diagnostics. */
const Stm32PlatformDiagnostics *stm32_platform_get_diagnostics(void);

#endif
