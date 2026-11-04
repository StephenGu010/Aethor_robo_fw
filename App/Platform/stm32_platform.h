/**
 * @file stm32_platform.h
 * @brief Defines the STM32H723 FDCAN1 and USB CDC adapter boundary.
 */

#ifndef APP_PLATFORM_STM32_PLATFORM_H
#define APP_PLATFORM_STM32_PLATFORM_H

#include <stddef.h>
#include <stdint.h>
#include "../Config/debug_ui_config.h"
#include "can_frame.h"
/* Independent offline-reviewed adapter gate. No current App/Core producer enables this path. */
#ifndef AETHOR_MOTOR_PERSISTENCE_TRANSPORT
#define AETHOR_MOTOR_PERSISTENCE_TRANSPORT (0U)
#endif
#if (AETHOR_MOTOR_PERSISTENCE_TRANSPORT != 0) && (AETHOR_MOTOR_PERSISTENCE_TRANSPORT != 1)
#error "Motor persistence transport flag must be zero or one."
#endif
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
#include "motor_parameter_persistence.h"
#define STM32_PLATFORM_PERSISTENCE_API_VERSION (1U)
/** @brief Final caller-owned health/owner/STOP permit; called with IRQ masked after genuine empty RX.
 * Must not reenter Platform APIs. The caller must provide its enclosing task ownership critical section. */
typedef uint8_t (*Stm32PersistenceSubmitted)(const MotorPersistenceIdentity *identity, uint64_t boundary_us);
/** @brief Reserve immutable fixed-ID7 bytes and identity, then invoke the final permit before HAL.
 * Return 1 means an owned receipt must be collected, including callback/HAL failure; never retry that pair.
 * now_us is sampled before the RX-empty observation; timeout is 1..100000 us without renewal.
 * Only 45 reads, exact PMAX512/mode2 and AA01 are admitted. The caller supplies full transaction authority.
 * This adapter does not establish App exclusivity, health, Save once-token or persistent-storage success. */
uint8_t stm32_platform_can_submit_persistence(const CanFrame *frame, const MotorPersistenceIdentity *identity,
    uint64_t now_us, uint64_t timeout_us, Stm32PersistenceSubmitted submitted);
/** @brief Poll an owned receipt; pending debt prevents release even after cancellation or timeout.
 * HAL-accepted requests remain owned before the first visible pending. Unknown accepted buffer
 * identity retains a fault lock until genuine reset; zero status or timeout never proves clearance.
 * A HAL refusal with no observed request debt may produce an immediate failed receipt.
 * now_us uses the same monotonic clock as submission. completed_observed_us is the software observation
 * time of this collection, never a physical SOF/EOF or motor sample time. Outputs change only on return 1. */
uint8_t stm32_platform_can_collect_persistence(MotorPersistenceIdentity *identity, uint64_t now_us,
    uint64_t *completed_observed_us, uint8_t *succeeded);
/** @brief Latch cancellation only for the exact identity; an abort request never proves TXBRP cleared. */
uint8_t stm32_platform_can_cancel_persistence(const MotorPersistenceIdentity *identity);
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
#include "../DebugUi/debug_ui_single_static_tx.h"
/** @brief Submission validates the full phase identity before the HAL call. */
typedef uint8_t (*Stm32StaticSubmitted)(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us);
/** @brief Reserve the sole strict TX slot for exact startup bytes or the original selected preflight. */
uint8_t stm32_platform_can_submit_static(const CanFrame *frame, const DebugUiStaticTxIdentity *identity,
    uint64_t timeout_us, Stm32StaticSubmitted submitted);
/** @brief Sole TX event consumer returns all identity components and the actual SOF timestamp. */
uint8_t stm32_platform_can_collect_static(DebugUiStaticTxIdentity *identity, uint8_t *succeeded, uint16_t *sof);
/** @brief Return immutable RX SOF and the full strict identity of the last popped frame. */
uint8_t stm32_platform_can_static_rx_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity, uint16_t *sof, uint8_t *sof_valid);
/** @brief Actual-submission hook freezes one identity's selected feedback sequence before HAL queueing. */
typedef uint8_t (*Stm32SingleStaticSubmitted)(uint32_t epoch, uint32_t request_id, uint64_t boundary_us);
/** @brief Reserve one exact selected DISABLE and its hardware receipt; ordinary CAN scheduling is unchanged. */
uint8_t stm32_platform_can_submit_single_static_preflight(const CanFrame *frame,
    uint32_t epoch, uint32_t request_id, uint64_t action_timeout_us, Stm32SingleStaticSubmitted submitted);
/** @brief Collect one exact pinned TX receipt without interpreting its polling time as physical TX time. */
uint8_t stm32_platform_can_collect_single_static_receipt(uint32_t *epoch, uint32_t *request_id, uint8_t *succeeded);
/** @brief Return the last popped frame's original hardware SOF order tag, never decode or polling time. */
uint8_t stm32_platform_can_single_static_rx_identity(const CanFrame *frame, uint32_t *epoch, uint32_t *request_id);
#endif

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
    uint32_t can_rx_frame_count;
    uint32_t can_tx_queue_high_watermark;
    uint32_t control_group_reject_count;
    uint32_t usb_rx_byte_count;
    uint32_t usb_rx_overflow_count;
    uint32_t usb_overlong_line_count;
    uint32_t usb_high_queue_high_watermark;
    uint32_t usb_query_queue_high_watermark;
    uint32_t usb_telemetry_queue_high_watermark;
    uint32_t usb_telemetry_drop_count;
    uint32_t usb_high_queue_full_count;
    uint32_t usb_transmit_busy_count;
    uint32_t usb_transmit_error_count;
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
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
/** @brief Collects only a pinned fixed-read FIFO completion; caller holds the task critical boundary. */
uint8_t stm32_platform_can_collect_capability_receipt(CanFrame *frame, uint8_t *succeeded);
/** @brief Copies the last genuinely published empty-RX-FIFO bound; caller holds task critical. */
uint64_t stm32_platform_can_capability_rx_boundary(void);
#endif
/** @brief Returns true only after the legacy scheduler and FDCAN1 FIFO are drained. */
uint8_t stm32_platform_can_is_idle(void);

/** @brief Strict read-only fault-release proof includes independent TX buffers, cancellation requests and capability work. */
uint8_t stm32_platform_remote_release_is_idle(void);

/** @brief Returns read-only platform transport diagnostics. */
const Stm32PlatformDiagnostics *stm32_platform_get_diagnostics(void);

/** @brief Caller holds task critical; timestamp must precede the FIFO-empty observation. */
void stm32_platform_can_observe_rx_boundary(uint64_t before_observation_us);
/** @brief Pops immutable arrival lower bound; zero denotes untrusted timing. */
CanRxInboxStatus stm32_platform_can_pop_received_at(CanFrame *frame, uint64_t *lower_bound_us);

#endif
