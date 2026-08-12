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
#define BOARD_FORMAL_UART_BAUDRATE (921600UL)
#define BOARD_FORMAL_UART_DATA_BITS (8U)
#define BOARD_FORMAL_UART_STOP_BITS (1U)
#define BOARD_FORMAL_UART_PARITY_NONE (1U)
#define BOARD_FORMAL_UART_VALIDATED (0U)

#if ARM_JOINT_COUNT != 7U
#error "The Aethor arm firmware contract requires exactly seven joints."
#endif

#endif
