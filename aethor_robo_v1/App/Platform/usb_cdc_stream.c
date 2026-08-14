/**
 * @file usb_cdc_stream.c
 * @brief Implements allocation-free USB CDC byte, line, and transmit queues.
 */

#include "usb_cdc_stream.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Copies a bounded message into static storage.
 */
static UsbCdcStreamStatus usb_cdc_stream_copy_message(UsbCdcStreamMessage *message,
                                                      const uint8_t *data,
                                                      uint16_t length)
{
    if ((message == NULL) || (data == NULL) || (length == 0U))
    {
        return USB_CDC_STREAM_STATUS_INVALID_ARGUMENT;
    }
    if (length > USB_CDC_STREAM_MESSAGE_CAPACITY)
    {
        return USB_CDC_STREAM_STATUS_MESSAGE_TOO_LONG;
    }

    memcpy(message->data, data, length);
    message->length = length;
    return USB_CDC_STREAM_STATUS_OK;
}

/**
 * @brief Appends one message to a task-context FIFO.
 */
static UsbCdcStreamStatus usb_cdc_stream_queue_fifo(
    UsbCdcStreamMessage *messages,
    uint8_t capacity,
    uint8_t *write_index,
    uint8_t *count,
    uint8_t *high_watermark,
    uint32_t *full_count,
    const uint8_t *data,
    uint16_t length)
{
    UsbCdcStreamStatus copy_status;

    if (*count >= capacity)
    {
        ++(*full_count);
        return USB_CDC_STREAM_STATUS_RESPONSE_QUEUE_FULL;
    }

    copy_status = usb_cdc_stream_copy_message(&messages[*write_index], data, length);
    if (copy_status != USB_CDC_STREAM_STATUS_OK)
    {
        return copy_status;
    }

    *write_index = (uint8_t)((*write_index + 1U) % capacity);
    ++(*count);
    if (*count > *high_watermark)
    {
        *high_watermark = *count;
    }
    return USB_CDC_STREAM_STATUS_OK;
}

/**
 * @brief Initializes an empty USB CDC stream around a physical transmitter.
 * @param stream Destination stream.
 * @param transmit_function Non-blocking physical transmit callback.
 */
void usb_cdc_stream_init(UsbCdcStream *stream,
                         UsbCdcStreamTransmitFunction transmit_function)
{
    if (stream == NULL)
    {
        return;
    }

    memset(stream, 0, sizeof(*stream));
    stream->transmit_function = transmit_function;
    stream->initialized = (uint8_t)(transmit_function != NULL);
}

/**
 * @brief Copies one USB receive packet from callback/ISR context.
 * @param stream Initialized stream.
 * @param data Packet bytes.
 * @param length Packet length.
 * @return OK or a whole-packet overflow/argument error.
 */
UsbCdcStreamStatus usb_cdc_stream_receive_isr(UsbCdcStream *stream,
                                              const uint8_t *data,
                                              uint32_t length)
{
    uint16_t write_index;
    uint16_t read_index;
    uint16_t ring_size = USB_CDC_STREAM_RX_CAPACITY + 1U;
    uint16_t free_space;
    uint32_t byte_index;

    if ((stream == NULL) || ((data == NULL) && (length != 0U)))
    {
        return USB_CDC_STREAM_STATUS_INVALID_ARGUMENT;
    }
    if (stream->initialized == 0U)
    {
        return USB_CDC_STREAM_STATUS_NOT_INITIALIZED;
    }
    if (length > USB_CDC_STREAM_RX_CAPACITY)
    {
        stream->dropped_byte_count += length;
        return USB_CDC_STREAM_STATUS_RX_OVERFLOW;
    }

    write_index = stream->rx_write_index;
    read_index = stream->rx_read_index;
    free_space = (uint16_t)((read_index + ring_size - write_index - 1U) % ring_size);
    if (length > free_space)
    {
        stream->dropped_byte_count += length;
        return USB_CDC_STREAM_STATUS_RX_OVERFLOW;
    }

    for (byte_index = 0U; byte_index < length; ++byte_index)
    {
        stream->rx_bytes[write_index] = data[byte_index];
        write_index = (uint16_t)((write_index + 1U) % ring_size);
    }
    stream->rx_write_index = write_index;
    stream->received_byte_count += length;
    return USB_CDC_STREAM_STATUS_OK;
}

/**
 * @brief Extracts one LF or CRLF terminated line in task context.
 * @param stream Initialized stream.
 * @param line Destination null-terminated line.
 * @param line_capacity Destination capacity.
 * @param line_length Destination line length excluding null.
 * @return OK, EMPTY, LINE_TOO_LONG, or an argument/size error.
 */
UsbCdcStreamStatus usb_cdc_stream_next_line(UsbCdcStream *stream,
                                            char *line,
                                            size_t line_capacity,
                                            uint16_t *line_length)
{
    uint16_t ring_size = USB_CDC_STREAM_RX_CAPACITY + 1U;

    if ((stream == NULL) || (line == NULL) || (line_length == NULL))
    {
        return USB_CDC_STREAM_STATUS_INVALID_ARGUMENT;
    }
    *line_length = 0U;
    if (stream->initialized == 0U)
    {
        return USB_CDC_STREAM_STATUS_NOT_INITIALIZED;
    }

    while (stream->rx_read_index != stream->rx_write_index)
    {
        uint8_t received_byte = stream->rx_bytes[stream->rx_read_index];
        stream->rx_read_index = (uint16_t)((stream->rx_read_index + 1U) % ring_size);

        if (received_byte == '\n')
        {
            if (stream->discarding_overlong_line != 0U)
            {
                stream->discarding_overlong_line = 0U;
                stream->line_length = 0U;
                ++stream->overlong_line_count;
                return USB_CDC_STREAM_STATUS_LINE_TOO_LONG;
            }

            if ((stream->line_length != 0U) &&
                (stream->line_buffer[stream->line_length - 1U] == '\r'))
            {
                --stream->line_length;
            }
            if (stream->line_length == 0U)
            {
                continue;
            }
            if (line_capacity <= stream->line_length)
            {
                stream->line_length = 0U;
                return USB_CDC_STREAM_STATUS_OUTPUT_TOO_SMALL;
            }

            memcpy(line, stream->line_buffer, stream->line_length);
            line[stream->line_length] = '\0';
            *line_length = stream->line_length;
            stream->line_length = 0U;
            return USB_CDC_STREAM_STATUS_OK;
        }

        if (stream->discarding_overlong_line != 0U)
        {
            continue;
        }
        if (stream->line_length >= (USB_CDC_STREAM_LINE_CAPACITY - 1U))
        {
            stream->discarding_overlong_line = 1U;
            continue;
        }

        stream->line_buffer[stream->line_length] = (char)received_byte;
        ++stream->line_length;
    }

    return USB_CDC_STREAM_STATUS_EMPTY;
}

/**
 * @brief Queues a non-droppable ACK, ERR, DONE, or event frame.
 * @param stream Initialized stream.
 * @param data Complete encoded protocol frame.
 * @param length Frame length.
 * @return OK or an explicit capacity/argument error.
 */
UsbCdcStreamStatus usb_cdc_stream_queue_high_priority(UsbCdcStream *stream,
                                                      const uint8_t *data,
                                                      uint16_t length)
{
    if ((stream == NULL) || (stream->initialized == 0U))
    {
        return USB_CDC_STREAM_STATUS_NOT_INITIALIZED;
    }
    return usb_cdc_stream_queue_fifo(stream->high_priority_messages,
                                     USB_CDC_STREAM_HIGH_PRIORITY_CAPACITY,
                                     &stream->high_priority_write_index,
                                     &stream->high_priority_count,
                                     &stream->high_priority_high_watermark,
                                     &stream->high_priority_queue_full_count,
                                     data,
                                     length);
}

/**
 * @brief Queues a non-droppable query response below state-changing results.
 */
UsbCdcStreamStatus usb_cdc_stream_queue_query(UsbCdcStream *stream,
                                              const uint8_t *data,
                                              uint16_t length)
{
    if ((stream == NULL) || (stream->initialized == 0U))
    {
        return USB_CDC_STREAM_STATUS_NOT_INITIALIZED;
    }
    return usb_cdc_stream_queue_fifo(stream->query_messages,
                                     USB_CDC_STREAM_QUERY_CAPACITY,
                                     &stream->query_write_index,
                                     &stream->query_count,
                                     &stream->query_high_watermark,
                                     &stream->query_queue_full_count,
                                     data,
                                     length);
}

/**
 * @brief Compatibility alias that queues a high-priority response.
 */
UsbCdcStreamStatus usb_cdc_stream_queue_response(UsbCdcStream *stream,
                                                 const uint8_t *data,
                                                 uint16_t length)
{
    return usb_cdc_stream_queue_high_priority(stream, data, length);
}

/**
 * @brief Stores the latest telemetry frame, replacing an older pending frame.
 * @param stream Initialized stream.
 * @param data Complete encoded telemetry frame.
 * @param length Frame length.
 * @return OK, REPLACED, or an argument/size error.
 */
UsbCdcStreamStatus usb_cdc_stream_queue_telemetry(UsbCdcStream *stream,
                                                  const uint8_t *data,
                                                  uint16_t length)
{
    UsbCdcStreamStatus copy_status;
    UsbCdcStreamStatus result = USB_CDC_STREAM_STATUS_OK;

    if ((stream == NULL) || (stream->initialized == 0U))
    {
        return USB_CDC_STREAM_STATUS_NOT_INITIALIZED;
    }

    if (stream->telemetry_count >= USB_CDC_STREAM_TELEMETRY_CAPACITY)
    {
        stream->telemetry_read_index = (uint8_t)((stream->telemetry_read_index + 1U) %
                                                 USB_CDC_STREAM_TELEMETRY_CAPACITY);
        --stream->telemetry_count;
        ++stream->telemetry_replaced_count;
        result = USB_CDC_STREAM_STATUS_REPLACED;
    }

    copy_status = usb_cdc_stream_copy_message(
        &stream->telemetry_messages[stream->telemetry_write_index],
        data,
        length);
    if (copy_status != USB_CDC_STREAM_STATUS_OK)
    {
        return copy_status;
    }

    stream->telemetry_write_index = (uint8_t)((stream->telemetry_write_index + 1U) %
                                              USB_CDC_STREAM_TELEMETRY_CAPACITY);
    ++stream->telemetry_count;
    if (stream->telemetry_count > stream->telemetry_high_watermark)
    {
        stream->telemetry_high_watermark = stream->telemetry_count;
    }
    return result;
}

/**
 * @brief Attempts one priority-ordered non-blocking transmit in task context.
 * @param stream Initialized stream.
 */
void usb_cdc_stream_service_tx(UsbCdcStream *stream)
{
    UsbCdcStreamTransmitResult transmit_result;
    uint8_t selected_queue;

    if ((stream == NULL) || (stream->initialized == 0U) ||
        (stream->tx_in_flight != 0U))
    {
        return;
    }

    if (stream->high_priority_count != 0U)
    {
        selected_queue = 1U;
        stream->active_message =
            stream->high_priority_messages[stream->high_priority_read_index];
    }
    else if (stream->query_count != 0U)
    {
        selected_queue = 2U;
        stream->active_message = stream->query_messages[stream->query_read_index];
    }
    else if (stream->telemetry_count != 0U)
    {
        selected_queue = 3U;
        stream->active_message = stream->telemetry_messages[stream->telemetry_read_index];
    }
    else
    {
        return;
    }

    transmit_result = stream->transmit_function(stream->active_message.data,
                                                stream->active_message.length);
    if (transmit_result == USB_CDC_STREAM_TRANSMIT_BUSY)
    {
        ++stream->transmit_busy_count;
        return;
    }
    if (transmit_result == USB_CDC_STREAM_TRANSMIT_ERROR)
    {
        ++stream->transmit_error_count;
        return;
    }

    stream->tx_in_flight = 1U;
    if (selected_queue == 1U)
    {
        stream->high_priority_read_index =
            (uint8_t)((stream->high_priority_read_index + 1U) %
                      USB_CDC_STREAM_HIGH_PRIORITY_CAPACITY);
        --stream->high_priority_count;
    }
    else if (selected_queue == 2U)
    {
        stream->query_read_index = (uint8_t)((stream->query_read_index + 1U) %
                                             USB_CDC_STREAM_QUERY_CAPACITY);
        --stream->query_count;
    }
    else
    {
        stream->telemetry_read_index = (uint8_t)((stream->telemetry_read_index + 1U) %
                                                 USB_CDC_STREAM_TELEMETRY_CAPACITY);
        --stream->telemetry_count;
    }
}

/**
 * @brief Releases the immutable in-flight buffer from the USB completion callback.
 * @param stream Initialized stream.
 */
void usb_cdc_stream_on_tx_complete_isr(UsbCdcStream *stream)
{
    if ((stream == NULL) || (stream->initialized == 0U))
    {
        return;
    }

    stream->tx_in_flight = 0U;
}
