/**
 * @file usb_cdc_transport.h
 * @brief Allocation-free USB CDC receive ring, line assembler, and transmit queue.
 */

#ifndef USB_CDC_TRANSPORT_H
#define USB_CDC_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_CDC_RX_RING_CAPACITY 512U
#define USB_CDC_LINE_CAPACITY 192U
#define USB_CDC_TX_QUEUE_DEPTH 8U
#define USB_CDC_TX_MESSAGE_CAPACITY 512U

/** @brief Results returned by the physical CDC transmit callback. */
typedef enum
{
    USB_CDC_TRANSMIT_OK = 0,
    USB_CDC_TRANSMIT_BUSY,
    USB_CDC_TRANSMIT_ERROR
} UsbCdcTransmitResult;

/** @brief Results returned by transport entry points. */
typedef enum
{
    USB_CDC_TRANSPORT_STATUS_OK = 0,
    USB_CDC_TRANSPORT_STATUS_INVALID_ARGUMENT,
    USB_CDC_TRANSPORT_STATUS_RX_OVERFLOW,
    USB_CDC_TRANSPORT_STATUS_NOT_INITIALIZED,
    USB_CDC_TRANSPORT_STATUS_TX_QUEUE_FULL,
    USB_CDC_TRANSPORT_STATUS_MESSAGE_TOO_LONG
} UsbCdcTransportStatus;

/** @brief Observable transport counters for diagnostics. */
typedef struct
{
    uint32_t received_byte_count;
    uint32_t dropped_byte_count;
    uint32_t overlong_line_count;
    uint32_t dropped_response_count;
    uint32_t transmit_busy_count;
    uint32_t transmit_error_count;
} UsbCdcTransportStatistics;

typedef UsbCdcTransmitResult (*UsbCdcTransmitFunction)(const uint8_t *data, uint16_t length);
typedef int (*UsbCdcLineHandler)(const char *line, char *response, size_t response_capacity);

void usb_cdc_transport_init(UsbCdcTransmitFunction transmit_function,
                            UsbCdcLineHandler line_handler);
UsbCdcTransportStatus usb_cdc_transport_receive(const uint8_t *data, uint32_t length);
UsbCdcTransportStatus usb_cdc_transport_queue_line(const char *line);
void usb_cdc_transport_service(void);
const UsbCdcTransportStatistics *usb_cdc_transport_get_statistics(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_TRANSPORT_H */
