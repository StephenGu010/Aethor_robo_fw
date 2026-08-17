/**
 * @file platform_contract.h
 * @brief Freezes Phase 0 platform timing and future transport selection.
 */

#ifndef APP_PLATFORM_PLATFORM_CONTRACT_H
#define APP_PLATFORM_PLATFORM_CONTRACT_H

#include "board_config.h"

#define PLATFORM_CONTROL_PERIOD_US (4000UL)
#define PLATFORM_FDCAN_INSTANCE_INDEX (1U)
#define PLATFORM_FDCAN_NOMINAL_BITRATE (BOARD_FDCAN_NOMINAL_BITRATE)
#define PLATFORM_FORMAL_SERIAL_TRANSPORT (BOARD_FORMAL_SERIAL_TRANSPORT)
#define PLATFORM_FORMAL_LINK_VALIDATED (BOARD_USB_CDC_VALIDATED)

/**
 * @brief Identifies whether a platform capability is configuration-only or proven.
 */
typedef enum
{
    PLATFORM_CAPABILITY_NOT_VALIDATED = 0,
    PLATFORM_CAPABILITY_VALIDATED
} PlatformCapabilityStatus;

#endif
