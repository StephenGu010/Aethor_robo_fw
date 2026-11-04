/**
 * @file stm32_platform.c
 * @brief Adapts static Aethor queues to STM32H723 FDCAN1 and USB CDC callbacks.
 */

#include "stm32_platform.h"

#include <stddef.h>
#include <string.h>

#include "fdcan.h"
#include "usbd_cdc_if.h"
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
#include "../DebugUi/debug_ui_single_static_tx.h"
/** @brief One phase/axis/sequence-pinned static slot; no ordinary frame or ADRC receipt can satisfy it. */
static struct {
    DebugUiSingleStaticTx proof;
    DebugUiSingleStaticWireOrder wire;
    CanFrame frame;
    Stm32SingleStaticSubmitted submitted;
    Stm32StaticSubmitted static_submitted;
    DebugUiStaticTxIdentity identity;
    uint32_t epoch;
    uint32_t request_id;
    uint8_t active;
    uint8_t terminal_failure;
    uint8_t terminal_ready;
    uint8_t terminal_succeeded;
    uint8_t cancellation_requested;
    uint8_t wire_invalid;
    uint8_t hardware_debt;
    uint8_t unknown_buffer;
    uint8_t failure_notified;
    uint8_t abort_requested;
    uint32_t debt_mask;
    uint32_t submitted_ms;
    uint32_t action_timeout_ms;
} platform_single_static;
/** @brief Immutable per-inbox-slot RX SOF tags share the original inbox publication index and lifetime. */
typedef struct { uint32_t epoch, request_id; uint16_t timestamp; DebugUiStaticTxIdentity identity; } Stm32SingleStaticRx;
static Stm32SingleStaticRx platform_single_static_rx[CAN_RX_INBOX_CAPACITY + 1U];
static Stm32SingleStaticRx platform_single_static_last_rx;
static CanFrame platform_single_static_last_frame;
static uint8_t platform_single_static_timestamp_ready;
static uint8_t platform_single_static_next_marker;

/** @brief Accept only this marked successful classic data-frame event; timestamp is TX SOF, not EOF. */
static void stm32_platform_single_static_poll_event(void)
{
    unsigned event_count = 0U;
    if ((hfdcan1.Instance->IR & FDCAN_IR_TEFL) || (hfdcan1.Instance->TXEFS & FDCAN_TXEFS_TEFL))
        platform_single_static.wire_invalid = 1U;
    while ((hfdcan1.Instance->TXEFS & FDCAN_TXEFS_EFFL) && event_count++ < hfdcan1.Init.TxEventsNbr) {
        FDCAN_TxEventFifoTypeDef event;
        uint16_t elapsed, transmitted;
        if (HAL_FDCAN_GetTxEvent(&hfdcan1, &event) != HAL_OK) { platform_single_static.wire_invalid = 1U; break; }
        elapsed = (uint16_t)(HAL_FDCAN_GetTimestampCounter(&hfdcan1) - platform_single_static.wire.anchor);
        transmitted = (uint16_t)((uint16_t)event.TxTimestamp - platform_single_static.wire.anchor);
        if (platform_single_static.wire.event_seen || event.MessageMarker != platform_single_static.wire.marker ||
            event.Identifier != platform_single_static.frame.identifier || event.IdType != FDCAN_STANDARD_ID ||
            event.TxFrameType != FDCAN_DATA_FRAME || event.FDFormat != FDCAN_CLASSIC_CAN ||
            event.BitRateSwitch != FDCAN_BRS_OFF || event.DataLength != 8U || event.EventType != FDCAN_TX_EVENT ||
            elapsed >= 32768U || transmitted > elapsed) { platform_single_static.wire_invalid = 1U; continue; }
        platform_single_static.wire.transmitted = (uint16_t)event.TxTimestamp;
        platform_single_static.wire.event_seen = 1U;
    }
}
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
#include "motor_capability_collector.h"
/** @brief Task-critical fixed-read FIFO slot, never shared with the dedicated motion channel. */
static MotorCapabilityFifoTracker platform_capability_tracker;
static uint32_t platform_capability_submitted_ms;
static uint8_t platform_capability_cancelled;
#endif

#define STM32_PLATFORM_CAN_RX_ISR_BUDGET (32U)

static CanRxInbox platform_can_rx_inbox;
/** @brief Written only with FDCAN IRQ masked; ISR reads an untorn 64-bit lower bound. */
static uint64_t platform_rx_lower_bound_us;
static CanTxScheduler platform_can_tx_scheduler;
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
/** @brief One immutable explicit maintenance reservation; independent of legacy bare-byte collectors. */
static struct {
    MotorPersistenceTracker tracker;
    MotorPersistenceIdentity identity;
    CanFrame frame;
    uint64_t submitted_us, deadline_us, last_observed_us;
    uint32_t debt_mask;
    uint8_t active, failed, abort_requested, hardware_debt, unknown_buffer;
} platform_persistence;
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
/* Shared MIT/POS control debt: DISABLE must outlive pending selected controls. */
static uint32_t platform_pos_control_buffers[ARM_JOINT_COUNT];
static uint32_t platform_pos_cancel_pending;
#endif
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief One independent terminal reservation survives cancellation and collection of the previous strict transaction. */
static struct {
    CanFrame frame;
    DebugUiStaticTxIdentity identity;
    Stm32StaticSubmitted submitted;
    uint64_t timeout_us;
    uint32_t reserved_ms;
    uint8_t active;
} platform_pos_terminal;
#endif
#endif
static UsbCdcStream platform_usb_stream;
static Stm32PlatformDiagnostics platform_diagnostics;
static Stm32PlatformIsrNotifier platform_can_rx_notifier;
static Stm32PlatformIsrNotifier platform_usb_rx_notifier;
static Stm32PlatformIsrNotifier platform_usb_tx_notifier;

#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
/** @brief Cancel once only after the accepted strict request becomes visible; never abort terminal POS DISABLE. */
static void stm32_platform_single_static_abort_pending(void)
{
    if (!platform_single_static.active || !platform_single_static.hardware_debt ||
        platform_single_static.abort_requested || !hfdcan1.Instance ||
        platform_single_static.identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE) return;
    if (platform_single_static.unknown_buffer) platform_single_static.debt_mask |= hfdcan1.Instance->TXBRP;
    if (!(hfdcan1.Instance->TXBRP & platform_single_static.debt_mask)) return;
    platform_single_static.abort_requested = 1U;
    if (HAL_FDCAN_AbortTxRequest(&hfdcan1, platform_single_static.debt_mask) != HAL_OK)
        ++platform_diagnostics.can_tx_error_count;
}

/** @brief Advance transport ownership independently of the single App-facing receipt; all-zero accepted status stays busy. */
static void stm32_platform_single_static_observe_debt(void)
{
    uint8_t succeeded;
    if (!platform_single_static.active || !hfdcan1.Instance) return;
    if (platform_single_static.hardware_debt || platform_single_static.proof.armed || platform_single_static.terminal_ready)
        stm32_platform_single_static_poll_event();
    if (platform_single_static.hardware_debt && (platform_can_rx_inbox.bus_off_latched ||
        platform_single_static.wire_invalid ||
        (uint32_t)(HAL_GetTick() - platform_single_static.submitted_ms) >= platform_single_static.action_timeout_ms))
        platform_single_static.terminal_failure = 1U;
    if (platform_single_static.cancellation_requested) platform_single_static.terminal_failure = 1U;
    if (platform_single_static.terminal_failure && platform_single_static.identity.phase != DEBUG_UI_STATIC_TX_POS_DISABLE)
        platform_single_static.cancellation_requested = 1U;
    if (platform_single_static.cancellation_requested) stm32_platform_single_static_abort_pending();
    if (debug_ui_single_static_tx_collect(&platform_single_static.proof, hfdcan1.Instance->TXBRP,
        hfdcan1.Instance->TXBTO, hfdcan1.Instance->TXBCF, &succeeded)) {
        platform_single_static.terminal_ready = 1U;
        platform_single_static.terminal_succeeded = (uint8_t)(succeeded && !platform_single_static.terminal_failure &&
            !platform_single_static.cancellation_requested);
        if (!platform_single_static.unknown_buffer) platform_single_static.hardware_debt = 0U;
    }
    /* The FIFO was drained before this nonzero marker/anchor was installed. Only the full fresh,
     * matching successful event can terminate unobserved/unknown pending, and it never grants success. */
    if (platform_single_static.hardware_debt && platform_single_static.wire.event_seen &&
        !platform_single_static.wire_invalid && !hfdcan1.Instance->TXBRP) {
        platform_single_static.hardware_debt = 0U;
        platform_single_static.proof.armed = 0U;
        platform_single_static.terminal_ready = platform_single_static.terminal_failure = 1U;
        platform_single_static.terminal_succeeded = 0U;
    }
    if (platform_single_static.failure_notified && !platform_single_static.hardware_debt) {
        platform_single_static.active = 0U;
        platform_single_static.terminal_succeeded = 0U;
    }
}
#endif

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
    platform_rx_lower_bound_us = 0ULL;
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    memset(&platform_persistence, 0, sizeof(platform_persistence));
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    memset(&platform_single_static, 0, sizeof(platform_single_static));
    memset(platform_single_static_rx, 0, sizeof(platform_single_static_rx));
    memset(&platform_single_static_last_rx, 0, sizeof(platform_single_static_last_rx));
    platform_single_static_next_marker = 0U;
    platform_single_static_timestamp_ready = (uint8_t)(hfdcan1.Init.TxEventsNbr != 0U &&
        HAL_FDCAN_ConfigTimestampCounter(&hfdcan1, FDCAN_TIMESTAMP_PRESC_16) == HAL_OK &&
        HAL_FDCAN_EnableTimestampCounter(&hfdcan1, FDCAN_TIMESTAMP_INTERNAL) == HAL_OK);
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
    memset(&platform_capability_tracker, 0, sizeof(platform_capability_tracker));
    platform_capability_cancelled = 0U;
#endif
    can_tx_scheduler_init(&platform_can_tx_scheduler);
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    memset(platform_pos_control_buffers, 0, sizeof(platform_pos_control_buffers));
    platform_pos_cancel_pending = 0U;
#endif
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    memset(&platform_pos_terminal, 0, sizeof(platform_pos_terminal));
#endif
#endif
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

/** @brief Establishes a conservative bound only after hardware RX backlog and TX drain.
 * Caller masks FDCAN IRQ and samples time BEFORE this check. TXBTO polling is not
 * a wire timestamp. A prompt reply already in FIFO retains the previous/unknown
 * bound and cannot authorize handoff; a subsequent reply is required.
 * All configured filters route to FIFO0; unexpected FIFO1 backlog also blocks.
 */
void stm32_platform_can_observe_rx_boundary(uint64_t before_observation_us)
{
    if (before_observation_us != 0ULL && platform_diagnostics.can_started != 0U &&
        platform_can_rx_inbox.bus_off_latched == 0U && hfdcan1.Instance != NULL)
    {
        platform_rx_lower_bound_us = can_rx_inbox_observe_lower_bound(
            platform_rx_lower_bound_us, before_observation_us,
            HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0),
            HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1),
            hfdcan1.Instance->TXBRP);
    }
    else { platform_rx_lower_bound_us = 0ULL; }
}

/** @brief Dequeue preserves ISR-published arrival evidence despite arbitrary task delay. */
CanRxInboxStatus stm32_platform_can_pop_received_at(CanFrame *frame, uint64_t *lower_bound_us)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    uint32_t interrupt_mask = __get_PRIMASK();
    uint8_t read_index;
    CanRxInboxStatus status;
    __disable_irq();
    read_index = platform_can_rx_inbox.read_index;
    status = can_rx_inbox_pop_at(&platform_can_rx_inbox, frame, lower_bound_us);
    if (status == CAN_RX_INBOX_STATUS_OK) {
        __DMB();
        platform_single_static_last_rx = platform_single_static_rx[read_index];
        platform_single_static_last_frame = *frame;
    }
    __set_PRIMASK(interrupt_mask);
    return status;
#else
    return can_rx_inbox_pop_at(&platform_can_rx_inbox, frame, lower_bound_us);
#endif
}

/** @brief Queues one CAN frame with its scheduling priority. */
CanTxSchedulerStatus stm32_platform_can_submit(CanTxPriority priority,
                                               const CanFrame *frame)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (platform_single_static.active && platform_single_static.failure_notified && priority != CAN_TX_PRIORITY_EMERGENCY)
        return CAN_TX_SCHEDULER_STATUS_FULL;
#endif
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (platform_persistence.active) {
        if (priority != CAN_TX_PRIORITY_EMERGENCY) return CAN_TX_SCHEDULER_STATUS_FULL;
        (void)stm32_platform_can_cancel_persistence(&platform_persistence.identity);
    }
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (priority == CAN_TX_PRIORITY_EMERGENCY && frame && frame->length == 8U &&
        ((frame->identifier >= 0x101U && frame->identifier <= 0x107U) ||
         (frame->identifier >= 1U && frame->identifier <= 7U)) && frame->data[7] == 0xFDU) {
        unsigned byte_index;
        uint8_t disable = 1U, entry_index = 0U;
        uint8_t joint_index = (uint8_t)((frame->identifier & 0xFFU) - 1U);
        for (byte_index = 0U; byte_index < 7U; ++byte_index) if (frame->data[byte_index] != 0xFFU) disable = 0U;
        if (disable) {
            /* Withdraw selected MIT and POS controls before this disable can be scheduled. */
            while (entry_index < platform_can_tx_scheduler.count) {
                CanTxSchedulerEntry *entry = &platform_can_tx_scheduler.entries[entry_index];
                if (entry->priority == CAN_TX_PRIORITY_JOINT_CONTROL &&
                    (entry->frame.identifier == 1U + joint_index || entry->frame.identifier == 0x101U + joint_index)) {
                    memmove(entry, entry + 1, (platform_can_tx_scheduler.count - entry_index - 1U) * sizeof(*entry));
                    --platform_can_tx_scheduler.count;
                } else ++entry_index;
            }
            platform_pos_cancel_pending |= platform_pos_control_buffers[joint_index] & hfdcan1.Instance->TXBRP;
            if (platform_pos_cancel_pending && HAL_FDCAN_AbortTxRequest(&hfdcan1, platform_pos_cancel_pending) != HAL_OK)
                ++platform_diagnostics.can_tx_error_count;
        }
    }
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (priority == CAN_TX_PRIORITY_EMERGENCY && platform_single_static.active &&
        !platform_single_static.cancellation_requested) {
        /* Explicit emergency traffic preempts the proof; retain the pinned buffer until it is actually no longer pending. */
        platform_single_static.cancellation_requested = 1U;
        if (platform_single_static.hardware_debt) {
            stm32_platform_single_static_abort_pending();
        } else if (!platform_single_static.terminal_ready) {
            /* The strict slot was admitted only with an empty scheduler. Remove its one exact entry
             * before accepting emergency traffic; never allow cancelled PRELOAD/ENABLE to reach HAL. */
            uint8_t entry_index;
            for (entry_index = 0U; entry_index < platform_can_tx_scheduler.count; ++entry_index) {
                CanTxSchedulerEntry *entry = &platform_can_tx_scheduler.entries[entry_index];
                if (entry->frame.identifier == platform_single_static.frame.identifier && entry->frame.length == 8U &&
                    !memcmp(entry->frame.data, platform_single_static.frame.data, 8U)) {
                    memmove(entry, entry + 1, (platform_can_tx_scheduler.count - entry_index - 1U) * sizeof(*entry));
                    --platform_can_tx_scheduler.count;
                    break;
                }
            }
            platform_single_static.terminal_failure = 1U;
        }
    }
#endif
    return can_tx_scheduler_submit(&platform_can_tx_scheduler, priority, frame);
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
/** @brief Queue one selected DISABLE only when the original action timeout fits strictly inside half a SOF counter wrap.
 * At nominal 80 MHz / (1 * (1+59+20)) = 1 Mbit/s and TCP=16, half-wrap is 524288 us.
 * The actual peripheral clock and nominal timing are checked; this path never changes the bus rate or timeout.
 */
uint8_t stm32_platform_can_submit_single_static_preflight(const CanFrame *frame,
    uint32_t epoch, uint32_t request_id, uint64_t action_timeout_us, Stm32SingleStaticSubmitted submitted)
{
    unsigned byte_index;
    uint32_t nominal_cycles;
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (platform_persistence.active) return 0U;
#endif
    /* HAL H723 nominal timing field bounds also prevent malformed multiplication overflow. */
    if (!hfdcan1.Init.NominalPrescaler || hfdcan1.Init.NominalPrescaler > 512U ||
        !hfdcan1.Init.NominalTimeSeg1 || hfdcan1.Init.NominalTimeSeg1 > 256U ||
        !hfdcan1.Init.NominalTimeSeg2 || hfdcan1.Init.NominalTimeSeg2 > 128U) return 0U;
    nominal_cycles = hfdcan1.Init.NominalPrescaler *
        (1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2);
    if (!frame || !submitted || !epoch || !request_id || platform_single_static.active || platform_can_tx_scheduler.count ||
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        platform_pos_terminal.active ||
#endif
        !platform_single_static_timestamp_ready || !debug_ui_single_static_wire_window(
            HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN), nominal_cycles, action_timeout_us) ||
        frame->identifier < 1U || frame->identifier > 7U || frame->length != 8U || frame->data[7] != 0xFDU) return 0U;
    for (byte_index = 0U; byte_index < 7U; ++byte_index) if (frame->data[byte_index] != 0xFFU) return 0U;
    if (can_tx_scheduler_submit(&platform_can_tx_scheduler, CAN_TX_PRIORITY_EMERGENCY, frame) != CAN_TX_SCHEDULER_STATUS_OK)
        return 0U;
    memset(&platform_single_static, 0, sizeof(platform_single_static));
    platform_single_static.frame = *frame;
    platform_single_static.epoch = epoch;
    platform_single_static.request_id = request_id;
    platform_single_static.submitted = submitted;
    platform_single_static.identity.epoch = epoch;
    platform_single_static.identity.request_id = request_id;
    platform_single_static.identity.sequence = 1U;
    platform_single_static.identity.motor_id = (uint8_t)frame->identifier;
    platform_single_static.identity.phase = DEBUG_UI_STATIC_TX_PREFLIGHT;
    platform_single_static.action_timeout_ms = (uint32_t)((action_timeout_us + 999ULL) / 1000ULL);
    ++platform_single_static_next_marker;
    if (!platform_single_static_next_marker) ++platform_single_static_next_marker;
    platform_single_static.wire.marker = platform_single_static_next_marker;
    platform_single_static.active = 1U;
    return 1U;
}

/** @brief Reserve the same one-slot proof for an explicitly scoped startup frame; no second FIFO consumer. */
uint8_t stm32_platform_can_submit_static(const CanFrame *frame, const DebugUiStaticTxIdentity *identity,
    uint64_t timeout_us, Stm32StaticSubmitted submitted)
{
    uint32_t cycles;
    unsigned index;
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (platform_persistence.active) return 0U;
#endif
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    if (frame && identity && identity->phase == DEBUG_UI_STATIC_TX_POS_DISABLE) {
        CanTxSchedulerStatus queued;
        if (!submitted || !identity->epoch || !identity->request_id || !identity->sequence ||
            !identity->motor_id || identity->motor_id > ARM_JOINT_COUNT ||
            (frame->identifier != identity->motor_id && frame->identifier != 0x100U + identity->motor_id) ||
            frame->length != 8U || frame->data[7] != 0xFDU ||
            platform_pos_terminal.active || (platform_single_static.active &&
                platform_single_static.identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE)) return 0U;
        for (index = 0U; index < 7U; ++index) if (frame->data[index] != 0xFFU) return 0U;
        /* Reuse emergency cancellation, then move only this new entry into independent bounded storage.
         * A full emergency queue cannot erase this final selected DISABLE opportunity. */
        queued = stm32_platform_can_submit(CAN_TX_PRIORITY_EMERGENCY, frame);
        if (queued == CAN_TX_SCHEDULER_STATUS_OK) --platform_can_tx_scheduler.count;
        platform_pos_terminal.frame = *frame;
        platform_pos_terminal.identity = *identity;
        platform_pos_terminal.submitted = submitted;
        platform_pos_terminal.timeout_us = timeout_us;
        platform_pos_terminal.reserved_ms = HAL_GetTick();
        platform_pos_terminal.active = 1U;
        return 1U;
    }
#endif
    if (!frame || !identity || !submitted || !identity->epoch || !identity->request_id || !identity->sequence ||
        !identity->motor_id || identity->motor_id > 7U || identity->phase < DEBUG_UI_STATIC_TX_PREFLIGHT ||
        (identity->phase > DEBUG_UI_STATIC_TX_FIRST_HOLD &&
         (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || identity->phase != DEBUG_UI_STATIC_TX_TAKEOVER)) || frame->length != 8U ||
        (frame->identifier != identity->motor_id &&
         (!AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST || frame->identifier != 0x100U + identity->motor_id)) ||
        platform_single_static.active || platform_can_tx_scheduler.count ||
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        platform_pos_terminal.active ||
#endif
        !platform_single_static_timestamp_ready || !hfdcan1.Init.NominalPrescaler || hfdcan1.Init.NominalPrescaler > 512U ||
        !hfdcan1.Init.NominalTimeSeg1 || hfdcan1.Init.NominalTimeSeg1 > 256U ||
        !hfdcan1.Init.NominalTimeSeg2 || hfdcan1.Init.NominalTimeSeg2 > 128U) return 0U;
    if (identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT || identity->phase == DEBUG_UI_STATIC_TX_ENABLE) {
        for (index = 0U; index < 7U; ++index) if (frame->data[index] != 0xFFU) return 0U;
        if (frame->data[7] != (identity->phase == DEBUG_UI_STATIC_TX_PREFLIGHT ? 0xFDU : 0xFCU)) return 0U;
    }
    cycles = hfdcan1.Init.NominalPrescaler * (1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2);
    if (!debug_ui_single_static_wire_window(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN), cycles, timeout_us)) return 0U;
    if (can_tx_scheduler_submit(&platform_can_tx_scheduler, CAN_TX_PRIORITY_JOINT_CONTROL, frame) != CAN_TX_SCHEDULER_STATUS_OK) return 0U;
    memset(&platform_single_static, 0, sizeof(platform_single_static));
    platform_single_static.frame = *frame;
    platform_single_static.identity = *identity;
    platform_single_static.epoch = identity->epoch;
    platform_single_static.request_id = identity->request_id;
    platform_single_static.static_submitted = submitted;
    platform_single_static.action_timeout_ms = (uint32_t)((timeout_us + 999ULL) / 1000ULL);
    ++platform_single_static_next_marker;
    if (!platform_single_static_next_marker) ++platform_single_static_next_marker;
    platform_single_static.wire.marker = platform_single_static_next_marker;
    platform_single_static.active = 1U;
    return 1U;
}

/** @brief Report one strict outcome; an early failed outcome is not physical slot clearance.
 * The caller must continue polling after that one failure while is_idle remains false. Original
 * identity/frame and accepted hardware debt survive App cleanup, until reliable terminal evidence;
 * no second receipt is emitted when that debt eventually clears. Success still requires pending and SOF. */
uint8_t stm32_platform_can_collect_single_static_receipt(uint32_t *epoch, uint32_t *request_id, uint8_t *succeeded)
{
    uint8_t ready;
    if (!epoch || !request_id || !succeeded || !platform_single_static.active) return 0U;
    stm32_platform_single_static_observe_debt();
    if (!platform_single_static.active) return 0U;
    /* A failed receipt is an App failure/disable intent, not transport clearance. Keep immutable
     * identity/frame and busy ownership, reject new traffic, and keep polling late pending after it. */
    if (platform_single_static.hardware_debt) {
        if (!platform_single_static.terminal_failure || platform_single_static.failure_notified ||
            (platform_single_static.proof.pending_seen && !platform_single_static.unknown_buffer) ||
            platform_single_static.identity.phase == DEBUG_UI_STATIC_TX_POS_DISABLE) return 0U;
        platform_single_static.failure_notified = 1U;
        *epoch = platform_single_static.epoch; *request_id = platform_single_static.request_id; *succeeded = 0U;
        platform_single_static.terminal_succeeded = 0U;
        return 1U;
    }
    ready = (uint8_t)(platform_single_static.terminal_failure || platform_single_static.terminal_ready);
    *succeeded = platform_single_static.terminal_succeeded;
    if (!ready) return 0U;
    if (!platform_single_static.terminal_failure && *succeeded && !platform_single_static.wire.event_seen &&
        !platform_single_static.wire_invalid && !platform_single_static.cancellation_requested &&
        (uint32_t)(HAL_GetTick() - platform_single_static.submitted_ms) < platform_single_static.action_timeout_ms) return 0U;
    if (platform_single_static.terminal_failure || platform_single_static.cancellation_requested ||
        platform_single_static.wire_invalid || !platform_single_static.wire.event_seen ||
        (uint32_t)(HAL_GetTick() - platform_single_static.submitted_ms) >= platform_single_static.action_timeout_ms) *succeeded = 0U;
    *epoch = platform_single_static.epoch;
    *request_id = platform_single_static.request_id;
    platform_single_static.terminal_succeeded = *succeeded;
    platform_single_static.active = 0U;
    return 1U;
}

/** @brief Preserve each raw RX hardware SOF through delayed decode; only a successful matching wire TX may qualify it. */
uint8_t stm32_platform_can_single_static_rx_identity(const CanFrame *frame, uint32_t *epoch, uint32_t *request_id)
{
    uint16_t elapsed, received;
    if (!frame || !epoch || !request_id || !platform_single_static.terminal_succeeded ||
        platform_single_static.wire_invalid || platform_single_static.cancellation_requested ||
        platform_single_static_last_rx.epoch != platform_single_static.epoch ||
        platform_single_static_last_rx.request_id != platform_single_static.request_id ||
        frame->identifier != platform_single_static_last_frame.identifier || frame->length != platform_single_static_last_frame.length ||
        memcmp(frame->data, platform_single_static_last_frame.data, frame->length)) return 0U;
    elapsed = (uint16_t)(HAL_FDCAN_GetTimestampCounter(&hfdcan1) - platform_single_static.wire.anchor);
    received = (uint16_t)(platform_single_static_last_rx.timestamp - platform_single_static.wire.anchor);
    if (elapsed >= 32768U || received > elapsed || !debug_ui_single_static_wire_after(
        &platform_single_static.wire, platform_single_static_last_rx.timestamp)) return 0U;
    *epoch = platform_single_static_last_rx.epoch;
    *request_id = platform_single_static_last_rx.request_id;
    return 1U;
}
/** @brief Return a complete receipt from the existing sole strict collector. */
uint8_t stm32_platform_can_collect_static(DebugUiStaticTxIdentity *identity, uint8_t *succeeded, uint16_t *sof)
{
    uint32_t epoch, request_id;
    if (!identity || !succeeded || !sof || !stm32_platform_can_collect_single_static_receipt(&epoch, &request_id, succeeded)) return 0U;
    *identity = platform_single_static.identity;
    if (!identity->phase) { identity->epoch = epoch; identity->request_id = request_id; identity->phase = DEBUG_UI_STATIC_TX_PREFLIGHT; }
    *sof = platform_single_static.wire.transmitted;
    return 1U;
}
/** @brief Match the immutable per-inbox identity, including phase, motor and sequence. */
uint8_t stm32_platform_can_static_rx_identity(const CanFrame *frame, DebugUiStaticTxIdentity *identity, uint16_t *sof, uint8_t *sof_valid)
{
    uint32_t epoch, request_id;
    if (!identity || !sof || !sof_valid) return 0U;
    *sof_valid = 0U;
    if (frame && frame->identifier == platform_single_static_last_frame.identifier &&
        frame->length == platform_single_static_last_frame.length && frame->length <= 8U &&
        !memcmp(frame->data, platform_single_static_last_frame.data, frame->length)) {
        *sof = platform_single_static_last_rx.timestamp;
        *sof_valid = 1U;
    }
    if (!stm32_platform_can_single_static_rx_identity(frame, &epoch, &request_id)) return 0U;
    if (!debug_ui_static_tx_identity_equal(&platform_single_static_last_rx.identity, &platform_single_static.identity)) return 0U;
    *identity = platform_single_static_last_rx.identity;
    *sof = platform_single_static_last_rx.timestamp;
    return 1U;
}

#endif

/** @brief Atomically queues one ordered J1 through J7 control group. */
CanTxSchedulerStatus stm32_platform_can_submit_control_group(
    const CanFrame *frames,
    uint8_t frame_count)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (platform_single_static.active && platform_single_static.failure_notified) return CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED;
#endif
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (platform_persistence.active) return CAN_TX_SCHEDULER_STATUS_GROUP_REJECTED;
#endif
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
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    stm32_platform_single_static_observe_debt();
#endif

    if (platform_can_rx_inbox.bus_off_latched != 0U)
    {
        return STM32_PLATFORM_STATUS_CAN_BUS_OFF;
    }
    if (platform_diagnostics.can_started == 0U)
    {
        return platform_diagnostics.initialization_status;
    }
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    /* Explicit maintenance ownership is resolved before any legacy RID byte classification.
     * Even terminal receipts retain this reservation until the sole consumer collects them. */
    if (platform_persistence.active) return STM32_PLATFORM_STATUS_OK;
#endif

    for (transmitted_count = 0U;
         transmitted_count < maximum_frame_count;
         ++transmitted_count)
    {
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        platform_pos_cancel_pending &= hfdcan1.Instance->TXBRP;
        if (platform_pos_cancel_pending) break;
#endif
        CanFrame frame;
        CanTxPriority priority;
        FDCAN_TxHeaderTypeDef transmit_header = {0};
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        uint8_t single_static_request = 0U;
        uint32_t single_static_interrupt_mask = 0U;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        uint8_t terminal_disable_request = 0U;
#endif
        stm32_platform_single_static_observe_debt();
        if (platform_single_static.active && platform_single_static.hardware_debt) break;
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
        uint32_t capability_interrupt_mask = 0U;
        uint8_t capability_request;
        /* A capability slot cannot be reused until its exact completion is collected.
         * This branch is inactive for every existing non-capability transaction. */
        if (platform_capability_tracker.active) { break; }
#endif

        if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
        {
            break;
        }
#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        if (platform_pos_terminal.active) {
            uint32_t nominal_cycles = 0U;
            /* Old completion remains visible to the sole collector before this slot is reused. */
            if (platform_single_static.active) break;
            memset(&platform_single_static, 0, sizeof(platform_single_static));
            platform_single_static.frame = platform_pos_terminal.frame;
            platform_single_static.identity = platform_pos_terminal.identity;
            platform_single_static.epoch = platform_pos_terminal.identity.epoch;
            platform_single_static.request_id = platform_pos_terminal.identity.request_id;
            platform_single_static.static_submitted = platform_pos_terminal.submitted;
            if (hfdcan1.Init.NominalPrescaler && hfdcan1.Init.NominalPrescaler <= 512U &&
                hfdcan1.Init.NominalTimeSeg1 && hfdcan1.Init.NominalTimeSeg1 <= 256U &&
                hfdcan1.Init.NominalTimeSeg2 && hfdcan1.Init.NominalTimeSeg2 <= 128U)
                nominal_cycles = hfdcan1.Init.NominalPrescaler * (1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2);
            platform_single_static.terminal_failure = (uint8_t)(!platform_single_static_timestamp_ready ||
                !debug_ui_single_static_wire_window(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN),
                    nominal_cycles, platform_pos_terminal.timeout_us));
            if (!platform_single_static.terminal_failure) {
                uint32_t timeout_ms = (uint32_t)((platform_pos_terminal.timeout_us + 999ULL) / 1000ULL);
                uint32_t elapsed_ms = (uint32_t)(HAL_GetTick() - platform_pos_terminal.reserved_ms);
                if (elapsed_ms >= timeout_ms) platform_single_static.terminal_failure = 1U;
                else platform_single_static.action_timeout_ms = timeout_ms - elapsed_ms;
            }
            ++platform_single_static_next_marker;
            if (!platform_single_static_next_marker) ++platform_single_static_next_marker;
            platform_single_static.wire.marker = platform_single_static_next_marker;
            platform_single_static.active = 1U;
            platform_pos_terminal.active = 0U;
            frame = platform_single_static.frame;
            priority = CAN_TX_PRIORITY_EMERGENCY;
            terminal_disable_request = 1U;
        } else
#endif
        if (can_tx_scheduler_pop(&platform_can_tx_scheduler, &frame, &priority) !=
            CAN_TX_SCHEDULER_STATUS_OK)
        {
            break;
        }
        (void)priority;
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        single_static_request = (uint8_t)(platform_single_static.active && !platform_single_static.terminal_ready &&
            !platform_single_static.proof.armed &&
            (!platform_single_static.terminal_failure
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
             || terminal_disable_request
#endif
            ) &&
            frame.identifier == platform_single_static.frame.identifier && frame.length == 8U &&
            memcmp(frame.data, platform_single_static.frame.data, 8U) == 0);
        if (single_static_request) {
            uint64_t submission_lower_us;
            single_static_interrupt_mask = __get_PRIMASK(); __disable_irq();
            submission_lower_us = (uint64_t)HAL_GetTick() * 1000ULL;
            if (!submission_lower_us || platform_can_rx_inbox.write_index != platform_can_rx_inbox.read_index ||
                HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) ||
                HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) || hfdcan1.Instance->TXBRP) {
                platform_single_static.terminal_failure = 1U;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                if (!terminal_disable_request) {
#endif
                __set_PRIMASK(single_static_interrupt_mask);
                return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                }
#endif
            }
            platform_rx_lower_bound_us = submission_lower_us;
            /* Remove every retained event before choosing this transaction's hardware anchor. */
            if (!platform_single_static.terminal_failure) {
                unsigned discarded = 0U;
                while ((hfdcan1.Instance->TXEFS & FDCAN_TXEFS_EFFL) && discarded++ < hfdcan1.Init.TxEventsNbr) {
                    FDCAN_TxEventFifoTypeDef previous;
                    if (HAL_FDCAN_GetTxEvent(&hfdcan1, &previous) != HAL_OK) break;
                }
                if ((hfdcan1.Instance->TXEFS & (FDCAN_TXEFS_EFFL | FDCAN_TXEFS_TEFL)) ||
                    (hfdcan1.Instance->IR & FDCAN_IR_TEFL)) {
                    platform_single_static.terminal_failure = 1U;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                    if (!terminal_disable_request) {
#endif
                    __set_PRIMASK(single_static_interrupt_mask); return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                    }
#endif
                }
            }
            platform_single_static.wire.anchor = HAL_FDCAN_GetTimestampCounter(&hfdcan1);
            platform_single_static.submitted_ms = HAL_GetTick();
            if (!platform_single_static.terminal_failure && !(platform_single_static.static_submitted ?
                platform_single_static.static_submitted(&platform_single_static.identity, submission_lower_us) :
                platform_single_static.submitted(platform_single_static.epoch, platform_single_static.request_id, submission_lower_us))) {
                platform_single_static.terminal_failure = 1U;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                if (!terminal_disable_request) {
#endif
                __set_PRIMASK(single_static_interrupt_mask);
                return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
                }
#endif
            }
        }
#endif
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
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        if (single_static_request) {
            transmit_header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
            transmit_header.MessageMarker = platform_single_static.wire.marker;
        }
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
        capability_request = motor_capability_is_request(&frame);
        if (capability_request)
        { capability_interrupt_mask = __get_PRIMASK(); __disable_irq(); }
#endif
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,
                                          &transmit_header,
                                          frame.data) != HAL_OK)
        {
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
            if (single_static_request) {
                platform_single_static.terminal_failure = 1U;
                __set_PRIMASK(single_static_interrupt_mask);
            }
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
            if (motor_capability_is_request(&frame))
            {
                (void)motor_capability_fifo_arm(&platform_capability_tracker, &frame, 1UL, 0UL, 0UL);
                platform_capability_submitted_ms = HAL_GetTick();
                platform_capability_cancelled = 1U;
            }
            if (capability_request) { __set_PRIMASK(capability_interrupt_mask); }
#endif
            ++platform_diagnostics.can_tx_error_count;
            return STM32_PLATFORM_STATUS_CAN_TX_ERROR;
        }
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        {
            uint8_t joint_index;
            uint32_t buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
                platform_pos_control_buffers[joint_index] &= ~buffer_mask;
            if (!single_static_request && priority == CAN_TX_PRIORITY_JOINT_CONTROL &&
                ((frame.identifier >= 1U && frame.identifier <= 7U) ||
                 (frame.identifier >= 0x101U && frame.identifier <= 0x107U)))
                platform_pos_control_buffers[(frame.identifier & 0xFFU) - 1U] |= buffer_mask;
        }
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        if (single_static_request) {
            uint32_t buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            platform_single_static.hardware_debt = 1U; /* HAL_OK wrote TXBAR; TXBRP may still be invisible. */
            platform_single_static.debt_mask = hfdcan1.Instance->TXBRP;
            if (buffer_mask && !(buffer_mask & (buffer_mask - 1U))) platform_single_static.debt_mask |= buffer_mask;
            if (!debug_ui_single_static_tx_arm(&platform_single_static.proof, buffer_mask,
                hfdcan1.Instance->TXBRP, hfdcan1.Instance->TXBTO) ||
                (hfdcan1.Instance->TXBRP & ~buffer_mask)) {
                platform_single_static.unknown_buffer = platform_single_static.terminal_failure = 1U;
            }
            __set_PRIMASK(single_static_interrupt_mask);
        }
#endif
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
        if (motor_capability_is_request(&frame))
        {
            uint32_t buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            (void)motor_capability_fifo_arm(&platform_capability_tracker, &frame, buffer_mask,
                hfdcan1.Instance->TXBRP, hfdcan1.Instance->TXBTO);
            platform_capability_submitted_ms = HAL_GetTick();
            platform_capability_cancelled = 0U;
        }
        if (capability_request) { __set_PRIMASK(capability_interrupt_mask); }
#endif
        ++platform_diagnostics.can_tx_success_count;
    }
    return STM32_PLATFORM_STATUS_OK;
}

#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
/** @brief Admit the exact fixed module frame set without granting arbitrary parameter writes. */
static uint8_t stm32_platform_persistence_frame_valid(const CanFrame *frame)
{
    unsigned index;
    static const uint8_t pmax[8] = {7U, 0U, 0x55U, 21U, 0U, 0U, 0U, 0x44U};
    static const uint8_t mode[8] = {7U, 0U, 0x55U, 10U, 2U, 0U, 0U, 0U};
    if (!frame || frame->identifier != 0x7FFU || frame->data[0] != 7U || frame->data[1] != 0U) return 0U;
    if (frame->length == 8U) return (uint8_t)(!memcmp(frame->data, pmax, 8U) || !memcmp(frame->data, mode, 8U));
    if (frame->length != 4U) return 0U;
    if (frame->data[2] == 0xAAU) return (uint8_t)(frame->data[3] == 1U);
    if (frame->data[2] != 0x33U) return 0U;
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index)
        if (frame->data[3] == motor_persistence_rid(index)) return 1U;
    return 0U;
}

/** @brief Latch failure before requesting cancellation, retaining all observed hardware debt. */
static void stm32_platform_persistence_cancel_locked(void)
{
    platform_persistence.failed = 1U;
    motor_persistence_tracker_cancel(&platform_persistence.tracker);
    /* HAL accepted an unknown slot: capture later visible debt without pretending it proves the complete request identity. */
    if (platform_persistence.unknown_buffer && hfdcan1.Instance)
        platform_persistence.debt_mask |= hfdcan1.Instance->TXBRP;
    if (!platform_persistence.abort_requested && hfdcan1.Instance &&
        (hfdcan1.Instance->TXBRP & platform_persistence.debt_mask)) {
        platform_persistence.abort_requested = 1U;
        if (HAL_FDCAN_AbortTxRequest(&hfdcan1, platform_persistence.debt_mask) != HAL_OK)
            ++platform_diagnostics.can_tx_error_count;
    }
}

/** @brief Submit explicit identity first, bypassing legacy collector recognition without changing its whitelist. */
uint8_t stm32_platform_can_submit_persistence(const CanFrame *frame, const MotorPersistenceIdentity *identity,
    uint64_t now_us, uint64_t timeout_us, Stm32PersistenceSubmitted submitted)
{
    uint32_t interrupt_mask, buffer_mask;
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef hal_status;
    if (!stm32_platform_persistence_frame_valid(frame) || !identity || !submitted || !identity->boot_epoch ||
        !identity->request_id || !identity->owner_generation || !identity->sequence || !identity->phase ||
        identity->phase > MOTOR_PERSISTENCE_READBACK || !now_us || !timeout_us ||
        timeout_us > MOTOR_PERSISTENCE_FRAME_BUDGET_US || now_us > UINT64_MAX - timeout_us) return 0U;
    interrupt_mask = __get_PRIMASK(); __disable_irq();
    if (platform_persistence.active || !hfdcan1.Instance || !platform_diagnostics.can_started ||
        platform_can_rx_inbox.bus_off_latched || platform_can_tx_scheduler.count ||
        !hfdcan1.Init.TxFifoQueueElmtsNbr || hfdcan1.Instance->TXBRP ||
        HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != hfdcan1.Init.TxFifoQueueElmtsNbr ||
        platform_can_rx_inbox.read_index != platform_can_rx_inbox.write_index ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1)
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
        || platform_capability_tracker.active
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        || platform_single_static.active || platform_pos_cancel_pending
#if AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
        || platform_pos_terminal.active
#endif
#endif
        ) { __set_PRIMASK(interrupt_mask); return 0U; }
    stm32_platform_can_observe_rx_boundary(now_us);
    if (platform_rx_lower_bound_us != now_us) { __set_PRIMASK(interrupt_mask); return 0U; }
    memset(&platform_persistence, 0, sizeof(platform_persistence));
    platform_persistence.identity = *identity; platform_persistence.frame = *frame;
    platform_persistence.submitted_us = platform_persistence.last_observed_us = now_us;
    platform_persistence.deadline_us = now_us + timeout_us; platform_persistence.active = 1U;
    /* The callback owns health/STOP/generation validation and latches Save before the HAL call.
     * Recheck raw backlog afterward; no refusal can fall through into ordinary scheduling. */
    if (!submitted(&platform_persistence.identity, now_us) || !platform_diagnostics.can_started ||
        platform_can_rx_inbox.bus_off_latched || platform_can_tx_scheduler.count ||
        platform_can_rx_inbox.read_index != platform_can_rx_inbox.write_index || hfdcan1.Instance->TXBRP ||
        HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != hfdcan1.Init.TxFifoQueueElmtsNbr ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1)) {
        platform_persistence.debt_mask = hfdcan1.Instance->TXBRP;
        platform_persistence.hardware_debt = platform_persistence.unknown_buffer =
            (uint8_t)(platform_persistence.debt_mask != 0U);
        stm32_platform_persistence_cancel_locked(); __set_PRIMASK(interrupt_mask); return 1U;
    }
    header.Identifier = platform_persistence.frame.identifier; header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME; header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF; header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    (void)stm32_platform_length_to_hal_dlc(platform_persistence.frame.length, &header.DataLength);
    hal_status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, platform_persistence.frame.data);
    buffer_mask = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
    platform_persistence.debt_mask = hfdcan1.Instance->TXBRP;
    /* HAL_OK means TXBAR was written, even while a running Tx scan leaves all status bits zero. */
    platform_persistence.hardware_debt = (uint8_t)(hal_status == HAL_OK || platform_persistence.debt_mask != 0U);
    if (buffer_mask && !(buffer_mask & (buffer_mask - 1U))) {
        platform_persistence.debt_mask |= buffer_mask;
        (void)motor_persistence_tracker_arm(&platform_persistence.tracker, &platform_persistence.identity,
            buffer_mask, hfdcan1.Instance->TXBRP, hfdcan1.Instance->TXBTO);
    }
    platform_persistence.unknown_buffer = (uint8_t)(platform_persistence.hardware_debt &&
        (!platform_persistence.tracker.active || (hfdcan1.Instance->TXBRP & ~buffer_mask)));
    if (hal_status != HAL_OK || platform_persistence.unknown_buffer) {
        ++platform_diagnostics.can_tx_error_count; stm32_platform_persistence_cancel_locked();
    } else ++platform_diagnostics.can_tx_success_count; /* Existing counter denotes HAL acceptance only. */
    __set_PRIMASK(interrupt_mask); return 1U;
}

/** @brief Exact ownership cancellation cannot be undone by any later successful TXBTO bit. */
uint8_t stm32_platform_can_cancel_persistence(const MotorPersistenceIdentity *identity)
{
    uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    if (!platform_persistence.active || !motor_persistence_identity_equal(identity, &platform_persistence.identity))
        { __set_PRIMASK(interrupt_mask); return 0U; }
    stm32_platform_persistence_cancel_locked(); __set_PRIMASK(interrupt_mask); return 1U;
}

/** @brief Observe pending then TXBTO, preserving failure and cancellation until every pinned pending bit clears. */
uint8_t stm32_platform_can_collect_persistence(MotorPersistenceIdentity *identity, uint64_t now_us,
    uint64_t *completed_observed_us, uint8_t *succeeded)
{
    uint32_t interrupt_mask;
    MotorPersistenceIdentity tracked_identity;
    uint8_t completed = 0U, available = 0U;
    if (!identity || !completed_observed_us || !succeeded) return 0U;
    interrupt_mask = __get_PRIMASK(); __disable_irq();
    if (!platform_persistence.active || !hfdcan1.Instance)
        { __set_PRIMASK(interrupt_mask); return 0U; }
    if (!now_us || now_us < platform_persistence.last_observed_us || now_us >= platform_persistence.deadline_us ||
        platform_can_rx_inbox.bus_off_latched) stm32_platform_persistence_cancel_locked();
    if (now_us >= platform_persistence.last_observed_us) platform_persistence.last_observed_us = now_us;
    /* A cancellation made before the first visible pending stays sticky and aborts once when that request appears. */
    if (platform_persistence.failed) stm32_platform_persistence_cancel_locked();
    if (platform_persistence.hardware_debt && platform_persistence.tracker.active)
        available = motor_persistence_tracker_collect(&platform_persistence.tracker, hfdcan1.Instance->TXBRP,
            hfdcan1.Instance->TXBTO, hfdcan1.Instance->TXBCF, &tracked_identity, &completed);
    /* Feed pending observations to the tracker before checking availability. HAL acceptance plus
     * all-zero status is unresolved, not a failure receipt. An unknown accepted buffer stays locked
     * until a genuine reset; this adapter has no API that fabricates such reset/clearance evidence. */
    if ((hfdcan1.Instance->TXBRP & platform_persistence.debt_mask) || platform_persistence.unknown_buffer ||
        (platform_persistence.hardware_debt && !available))
        { __set_PRIMASK(interrupt_mask); return 0U; }
    *identity = platform_persistence.identity; *completed_observed_us = now_us;
    *succeeded = (uint8_t)(completed && !platform_persistence.failed);
    platform_persistence.active = 0U; __set_PRIMASK(interrupt_mask); return 1U;
}
#endif

#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
/** @brief Returns the actual FIFO-empty lower bound rather than inventing a post-submit timestamp. */
uint64_t stm32_platform_can_capability_rx_boundary(void)
{ return platform_rx_lower_bound_us; }

/** @brief Polls only the pinned capability FIFO request; old TXBTO and cancelled requests fail closed. */
uint8_t stm32_platform_can_collect_capability_receipt(CanFrame *frame, uint8_t *succeeded)
{
    uint8_t available;
    if (frame == NULL || succeeded == NULL || !platform_capability_tracker.active ||
        hfdcan1.Instance == NULL) { return 0U; }
    if (platform_capability_cancelled && !platform_capability_tracker.pending_proven)
    { return motor_capability_fifo_collect(&platform_capability_tracker, 0UL, 0UL, 1UL, frame, succeeded); }
    if ((uint32_t)(HAL_GetTick() - platform_capability_submitted_ms) >= 20U &&
        (hfdcan1.Instance->TXBRP & platform_capability_tracker.buffer_mask) && !platform_capability_cancelled)
    {
        platform_capability_cancelled = 1U;
        (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, platform_capability_tracker.buffer_mask);
    }
    available = motor_capability_fifo_collect(&platform_capability_tracker,
        hfdcan1.Instance->TXBRP, hfdcan1.Instance->TXBTO, hfdcan1.Instance->TXBCF, frame, succeeded);
    if (available && platform_capability_cancelled) { *succeeded = 0U; }
    return available;
}
#endif

/** @brief Confirms that no legacy frame can emerge after an ADRC owner transfer. */
uint8_t stm32_platform_can_is_idle(void)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (platform_single_static.active) return 0U;
#endif
#if AETHOR_MOTOR_PERSISTENCE_TRANSPORT
    if (platform_persistence.active) return 0U;
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
    if (platform_pos_terminal.active) return 0U;
#endif
    if (platform_diagnostics.can_started == 0U ||
        platform_can_rx_inbox.bus_off_latched != 0U ||
        platform_can_tx_scheduler.count != 0U ||
        hfdcan1.Init.TxFifoQueueElmtsNbr == 0U)
    { return 0U; }
    return (uint8_t)(HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) ==
        hfdcan1.Init.TxFifoQueueElmtsNbr);
}

/** @brief Fault release must observe every physical transmit buffer and pending cancellation, beyond FIFO handoff. */
uint8_t stm32_platform_remote_release_is_idle(void)
{
    if (hfdcan1.Instance == NULL || !stm32_platform_can_is_idle() ||
        hfdcan1.Instance->TXBRP != 0U || hfdcan1.Instance->TXBCR != 0U ||
        platform_can_rx_inbox.read_index != platform_can_rx_inbox.write_index ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) != 0U ||
        HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1) != 0U) return 0U;
#if AETHOR_ADRC_BENCH || AETHOR_ADRC_LCD_INTEGRATED
    if (platform_capability_tracker.active) return 0U;
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
    if (platform_pos_cancel_pending) return 0U;
#endif
    return 1U;
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
#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
        {
            uint8_t write_index = platform_can_rx_inbox.write_index;
            platform_single_static_rx[write_index].timestamp = (uint16_t)receive_header.RxTimestamp;
            platform_single_static_rx[write_index].epoch = platform_single_static.epoch;
            platform_single_static_rx[write_index].request_id = platform_single_static.request_id;
            platform_single_static_rx[write_index].identity = platform_single_static.identity;
            __DMB();
        }
#endif
        if (can_rx_inbox_push_isr_at(&platform_can_rx_inbox, &frame,
            platform_rx_lower_bound_us) !=
            CAN_RX_INBOX_STATUS_OK)
        {
            platform_rx_lower_bound_us = 0ULL;
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
