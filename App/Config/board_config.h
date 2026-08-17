/**
 * @file board_config.h
 * @brief Freezes verified board identity and communication constants.
 */

#ifndef APP_CONFIG_BOARD_CONFIG_H
#define APP_CONFIG_BOARD_CONFIG_H

#include "arm_config.h"

#include <stdint.h>

#define BOARD_CONTROLLER_ID "aethor-controller-01"
#define BOARD_ARM_ID "arm-01"
#define BOARD_MCU_PART_NUMBER "STM32H723VGT6"
#define BOARD_FDCAN_NOMINAL_BITRATE (1000000UL)

/**
 * @brief Identifies the physical transport that carries the formal protocol.
 */
typedef enum
{
    BOARD_SERIAL_TRANSPORT_USB_CDC = 1U
} BoardSerialTransport;

#define BOARD_FORMAL_SERIAL_TRANSPORT BOARD_SERIAL_TRANSPORT_USB_CDC
#define BOARD_USB_CDC_VALIDATED (0U)

#if ARM_JOINT_COUNT != 7U
#error "The Aethor arm firmware contract requires exactly seven joints."
#endif

#endif
