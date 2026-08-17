/**
 * @file protocol_contract.h
 * @brief Freezes Phase 0 protocol capacities and channel semantics.
 */

#ifndef APP_PROTOCOL_PROTOCOL_CONTRACT_H
#define APP_PROTOCOL_PROTOCOL_CONTRACT_H

#define PROTOCOL_MAX_LINE_LENGTH (512U)
#define PROTOCOL_RECENT_RESULT_CAPACITY (32U)
#define PROTOCOL_BUSINESS_COMMAND_CAPACITY (8U)
#define PROTOCOL_VERSION_TEXT "aethor-arm-ascii-v1"

/**
 * @brief Identifies protocol availability during staged implementation.
 */
typedef enum
{
    PROTOCOL_STATUS_NOT_AVAILABLE = 0,
    PROTOCOL_STATUS_AVAILABLE
} ProtocolStatus;

#define PROTOCOL_PHASE0_STATUS (PROTOCOL_STATUS_NOT_AVAILABLE)

/**
 * @brief Separates non-droppable responses from replaceable telemetry.
 */
typedef enum
{
    PROTOCOL_CHANNEL_RESPONSE = 0,
    PROTOCOL_CHANNEL_TELEMETRY_DROP_OLDEST
} ProtocolChannel;

#endif
