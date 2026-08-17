/**
 * @file usb_cdc_transport.c
 * @brief Allocation-free USB CDC transport for interrupt-to-task communication.
 */

#include "usb_cdc_transport.h"

#include <string.h>

/** @brief One bounded response queued for deferred CDC transmission. */
typedef struct
{
    uint16_t length;
    uint8_t data[USB_CDC_TX_MESSAGE_CAPACITY];
} UsbCdcTransmitMessage;

static volatile uint16_t usb_cdc_rx_write_index;
static volatile uint16_t usb_cdc_rx_read_index;
static uint8_t usb_cdc_rx_ring[USB_CDC_RX_RING_CAPACITY];
static char usb_cdc_line[USB_CDC_LINE_CAPACITY];
static uint16_t usb_cdc_line_length;
static uint8_t usb_cdc_discarding_overlong_line;
static UsbCdcTransmitMessage usb_cdc_tx_queue[USB_CDC_TX_QUEUE_DEPTH];
static UsbCdcTransmitMessage usb_cdc_tx_active_message;
static uint8_t usb_cdc_tx_write_index;
static uint8_t usb_cdc_tx_read_index;
static uint8_t usb_cdc_tx_count;
static UsbCdcTransmitFunction usb_cdc_transmit_function;
static UsbCdcLineHandler usb_cdc_line_handler;
static UsbCdcTransportStatistics usb_cdc_statistics;

/**
 * @brief Queue a task-context text line and append the protocol CR/LF terminator.
 * @param line Null-terminated response or diagnostic line.
 * @return Explicit queueing status.
 */
UsbCdcTransportStatus usb_cdc_transport_queue_line(const char *line)
{
    UsbCdcTransmitMessage *message;
    size_t line_length;

    if (line == NULL)
    {
        return USB_CDC_TRANSPORT_STATUS_INVALID_ARGUMENT;
    }
    if ((usb_cdc_transmit_function == NULL) || (usb_cdc_line_handler == NULL))
    {
        return USB_CDC_TRANSPORT_STATUS_NOT_INITIALIZED;
    }
    if (usb_cdc_tx_count >= USB_CDC_TX_QUEUE_DEPTH)
    {
        usb_cdc_statistics.dropped_response_count++;
        return USB_CDC_TRANSPORT_STATUS_TX_QUEUE_FULL;
    }

    line_length = strlen(line);
    if ((line_length + 2U) > USB_CDC_TX_MESSAGE_CAPACITY)
    {
        usb_cdc_statistics.dropped_response_count++;
        return USB_CDC_TRANSPORT_STATUS_MESSAGE_TOO_LONG;
    }

    message = &usb_cdc_tx_queue[usb_cdc_tx_write_index];
    memcpy(message->data, line, line_length);
    message->data[line_length] = '\r';
    message->data[line_length + 1U] = '\n';
    message->length = (uint16_t)(line_length + 2U);
    usb_cdc_tx_write_index =
        (uint8_t)((usb_cdc_tx_write_index + 1U) % USB_CDC_TX_QUEUE_DEPTH);
    usb_cdc_tx_count++;
    return USB_CDC_TRANSPORT_STATUS_OK;
}

/**
 * @brief Dispatch one complete input line to the configured command handler.
 */
static void usb_cdc_transport_dispatch_line(void)
{
    char response[USB_CDC_TX_MESSAGE_CAPACITY - 2U];
    int handler_result;

    usb_cdc_line[usb_cdc_line_length] = '\0';
    response[0] = '\0';
    handler_result = usb_cdc_line_handler(usb_cdc_line, response, sizeof(response));
    if (handler_result != 0)
    {
        (void)usb_cdc_transport_queue_line("err command");
    }
    else if (response[0] == '\0')
    {
        (void)usb_cdc_transport_queue_line("ok");
    }
    else
    {
        (void)usb_cdc_transport_queue_line(response);
    }
    usb_cdc_line_length = 0U;
}

/**
 * @brief Consume one received byte and update line assembly state.
 * @param received_byte Byte removed from the receive ring.
 */
static void usb_cdc_transport_consume_byte(uint8_t received_byte)
{
    if ((received_byte == '\r') || (received_byte == '\n'))
    {
        if (usb_cdc_discarding_overlong_line != 0U)
        {
            usb_cdc_discarding_overlong_line = 0U;
            usb_cdc_line_length = 0U;
            usb_cdc_statistics.overlong_line_count++;
            (void)usb_cdc_transport_queue_line("err line-too-long");
        }
        else if (usb_cdc_line_length > 0U)
        {
            usb_cdc_transport_dispatch_line();
        }
        return;
    }

    if (usb_cdc_discarding_overlong_line != 0U)
    {
        return;
    }
    if (usb_cdc_line_length >= (USB_CDC_LINE_CAPACITY - 1U))
    {
        usb_cdc_discarding_overlong_line = 1U;
        return;
    }

    usb_cdc_line[usb_cdc_line_length] = (char)received_byte;
    usb_cdc_line_length++;
}

/**
 * @brief Reset transport state and attach platform callbacks.
 * @param transmit_function Non-blocking CDC transmit callback.
 * @param line_handler Complete-line command callback.
 */
void usb_cdc_transport_init(UsbCdcTransmitFunction transmit_function,
                            UsbCdcLineHandler line_handler)
{
    usb_cdc_rx_write_index = 0U;
    usb_cdc_rx_read_index = 0U;
    usb_cdc_line_length = 0U;
    usb_cdc_discarding_overlong_line = 0U;
    usb_cdc_tx_write_index = 0U;
    usb_cdc_tx_read_index = 0U;
    usb_cdc_tx_count = 0U;
    usb_cdc_transmit_function = transmit_function;
    usb_cdc_line_handler = line_handler;
    memset(&usb_cdc_statistics, 0, sizeof(usb_cdc_statistics));
}

/**
 * @brief Copy one CDC receive packet into the interrupt-safe single-producer ring.
 * @param data Received packet bytes.
 * @param length Number of received bytes.
 * @return Explicit transport status; complete packets are rejected on insufficient space.
 */
UsbCdcTransportStatus usb_cdc_transport_receive(const uint8_t *data, uint32_t length)
{
    uint16_t write_index;
    uint16_t read_index;
    uint16_t free_space;
    uint32_t byte_index;

    if ((data == NULL) && (length != 0U))
    {
        return USB_CDC_TRANSPORT_STATUS_INVALID_ARGUMENT;
    }
    if ((usb_cdc_transmit_function == NULL) || (usb_cdc_line_handler == NULL))
    {
        return USB_CDC_TRANSPORT_STATUS_NOT_INITIALIZED;
    }

    write_index = usb_cdc_rx_write_index;
    read_index = usb_cdc_rx_read_index;
    free_space = (uint16_t)((read_index + USB_CDC_RX_RING_CAPACITY -
                            write_index - 1U) % USB_CDC_RX_RING_CAPACITY);
    if (length > free_space)
    {
        usb_cdc_statistics.dropped_byte_count += length;
        return USB_CDC_TRANSPORT_STATUS_RX_OVERFLOW;
    }

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        usb_cdc_rx_ring[write_index] = data[byte_index];
        write_index = (uint16_t)((write_index + 1U) % USB_CDC_RX_RING_CAPACITY);
    }
    usb_cdc_rx_write_index = write_index;
    usb_cdc_statistics.received_byte_count += length;
    return USB_CDC_TRANSPORT_STATUS_OK;
}

/**
 * @brief Process received lines and retry one queued non-blocking transmission.
 */
void usb_cdc_transport_service(void)
{
    while (usb_cdc_rx_read_index != usb_cdc_rx_write_index)
    {
        uint8_t received_byte = usb_cdc_rx_ring[usb_cdc_rx_read_index];

        usb_cdc_rx_read_index =
            (uint16_t)((usb_cdc_rx_read_index + 1U) % USB_CDC_RX_RING_CAPACITY);
        usb_cdc_transport_consume_byte(received_byte);
    }

    if ((usb_cdc_tx_count > 0U) && (usb_cdc_transmit_function != NULL))
    {
        UsbCdcTransmitMessage *message = &usb_cdc_tx_queue[usb_cdc_tx_read_index];
        UsbCdcTransmitResult transmit_result;

        usb_cdc_tx_active_message = *message;
        transmit_result = usb_cdc_transmit_function(usb_cdc_tx_active_message.data,
                                                    usb_cdc_tx_active_message.length);

        if (transmit_result == USB_CDC_TRANSMIT_OK)
        {
            usb_cdc_tx_read_index =
                (uint8_t)((usb_cdc_tx_read_index + 1U) % USB_CDC_TX_QUEUE_DEPTH);
            usb_cdc_tx_count--;
        }
        else if (transmit_result == USB_CDC_TRANSMIT_BUSY)
        {
            usb_cdc_statistics.transmit_busy_count++;
        }
        else
        {
            usb_cdc_statistics.transmit_error_count++;
            usb_cdc_tx_read_index =
                (uint8_t)((usb_cdc_tx_read_index + 1U) % USB_CDC_TX_QUEUE_DEPTH);
            usb_cdc_tx_count--;
        }
    }
}

/**
 * @brief Get the current read-only transport counters.
 * @return Address of the static statistics structure.
 */
const UsbCdcTransportStatistics *usb_cdc_transport_get_statistics(void)
{
    return &usb_cdc_statistics;
}
