/** @file usbd_cdc_if.h @brief USB declaration fake; the platform ordering suite performs no USB I/O. */
#ifndef SINGLE_STATIC_FAKE_USB_H
#define SINGLE_STATIC_FAKE_USB_H
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
uint8_t CDC_Transmit_HS(uint8_t *data, uint16_t length);
#endif
