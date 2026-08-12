/**
 * @file usb_cdc_stream.h
 * @brief Defines an allocation-free USB CDC ISR-to-task byte and transmit stream.
 */

#ifndef APP_PLATFORM_USB_CDC_STREAM_H
#define APP_PLATFORM_USB_CDC_STREAM_H

#include <stddef.h>
#include <stdint.h>

#define USB_CDC_STREAM_RX_CAPACITY (1024U)
#define USB_CDC_STREAM_LINE_CAPACITY (513U)
#define USB_CDC_STREAM_RESPONSE_CAPACITY (8U)
#define USB_CDC_STREAM_MESSAGE_CAPACITY (520U)

/**
 * @brief Reports results from the non-blocking physical CDC transmitter.
 */
typedef enum
{
    USB_CDC_STREAM_TRANSMIT_OK = 0,
    USB_CDC_STREAM_TRANSMIT_BUSY,
    USB_CDC_STREAM_TRANSMIT_ERROR
} UsbCdcStreamTransmitResult;

/**
 * @brief Reports receive, line, and transmit queue outcomes.
 */
typedef enum
{
    USB_CDC_STREAM_STATUS_OK = 0,
    USB_CDC_STREAM_STATUS_REPLACED,
    USB_CDC_STREAM_STATUS_EMPTY,
    USB_CDC_STREAM_STATUS_INVALID_ARGUMENT,
    USB_CDC_STREAM_STATUS_RX_OVERFLOW,
    USB_CDC_STREAM_STATUS_LINE_TOO_LONG,
    USB_CDC_STREAM_STATUS_OUTPUT_TOO_SMALL,
    USB_CDC_STREAM_STATUS_RESPONSE_QUEUE_FULL,
    USB_CDC_STREAM_STATUS_MESSAGE_TOO_LONG,
    USB_CDC_STREAM_STATUS_NOT_INITIALIZED
} UsbCdcStreamStatus;

/**
 * @brief Owns one immutable queued USB message.
 */
typedef struct
{
    uint16_t length;
    uint8_t data[USB_CDC_STREAM_MESSAGE_CAPACITY];
} UsbCdcStreamMessage;

/** @brief Signature of the STM32-specific non-blocking CDC transmit adapter. */
typedef UsbCdcStreamTransmitResult (*UsbCdcStreamTransmitFunction)(
    const uint8_t *data,
    uint16_t length);

/**
 * @brief Owns all static USB CDC receive and transmit state.
 */
typedef struct
{
    uint8_t rx_bytes[USB_CDC_STREAM_RX_CAPACITY + 1U];
    char line_buffer[USB_CDC_STREAM_LINE_CAPACITY];
    UsbCdcStreamMessage response_messages[USB_CDC_STREAM_RESPONSE_CAPACITY];
    UsbCdcStreamMessage telemetry_message;
    UsbCdcStreamMessage active_message;
    UsbCdcStreamTransmitFunction transmit_function;
    volatile uint16_t rx_write_index;
    volatile uint16_t rx_read_index;
    uint16_t line_length;
    uint8_t response_write_index;
    uint8_t response_read_index;
    uint8_t response_count;
    uint8_t response_high_watermark;
    uint8_t telemetry_pending;
    volatile uint8_t tx_in_flight;
    uint8_t discarding_overlong_line;
    uint8_t initialized;
    volatile uint32_t received_byte_count;
    volatile uint32_t dropped_byte_count;
    uint32_t overlong_line_count;
    uint32_t response_queue_full_count;
    uint32_t telemetry_replaced_count;
    uint32_t transmit_busy_count;
    uint32_t transmit_error_count;
} UsbCdcStream;

/** @brief Initializes an empty USB CDC stream around a physical transmitter. */
void usb_cdc_stream_init(UsbCdcStream *stream,
                         UsbCdcStreamTransmitFunction transmit_function);

/** @brief Copies one USB receive packet from callback/ISR context. */
UsbCdcStreamStatus usb_cdc_stream_receive_isr(UsbCdcStream *stream,
                                              const uint8_t *data,
                                              uint32_t length);

/** @brief Extracts one LF or CRLF terminated line in task context. */
UsbCdcStreamStatus usb_cdc_stream_next_line(UsbCdcStream *stream,
                                            char *line,
                                            size_t line_capacity,
                                            uint16_t *line_length);

/** @brief Queues a non-droppable protocol response in FIFO order. */
UsbCdcStreamStatus usb_cdc_stream_queue_response(UsbCdcStream *stream,
                                                 const uint8_t *data,
                                                 uint16_t length);

/** @brief Stores the latest telemetry frame, replacing an older pending frame. */
UsbCdcStreamStatus usb_cdc_stream_queue_telemetry(UsbCdcStream *stream,
                                                  const uint8_t *data,
                                                  uint16_t length);

/** @brief Attempts one priority-ordered non-blocking transmit in task context. */
void usb_cdc_stream_service_tx(UsbCdcStream *stream);

/** @brief Releases the immutable in-flight buffer from the USB completion callback. */
void usb_cdc_stream_on_tx_complete_isr(UsbCdcStream *stream);

#endif
