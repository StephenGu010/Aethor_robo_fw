/** @file motor_persistence_app_port.h
 * @brief Fixed ID7 maintenance App/Core ownership port; no ordinary producer may borrow it.
 * Consumers require the integrated LCD single-static/POS build and the explicitly enabled
 * AETHOR_MOTOR_PERSISTENCE_TRANSPORT gate. Calls run under the existing App task critical
 * boundary; the final submitted hooks additionally run under Platform's IRQ mask. */
#ifndef APP_MOTOR_PERSISTENCE_APP_PORT_H
#define APP_MOTOR_PERSISTENCE_APP_PORT_H
#include <stdint.h>
#include "motor_parameter_persistence.h"
#include "../DebugUi/debug_ui_single_static_tx.h"
#if defined(AETHOR_MOTOR_PERSISTENCE_TRANSPORT) && AETHOR_MOTOR_PERSISTENCE_TRANSPORT
#if !AETHOR_ADRC_LCD_INTEGRATED || !AETHOR_DEBUG_UI_ALLOW_MOTION || !AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST
#error "Fixed persistence maintenance requires the integrated single-static/POS LCD build."
#endif
/** @brief Separates fixed parameter transfers from the independently proven DISABLE health request. */
typedef enum {
    AETHOR_PERSISTENCE_TRANSFER_NONE = 0,
    AETHOR_PERSISTENCE_TRANSFER_PARAMETER,
    AETHOR_PERSISTENCE_TRANSFER_DISABLE
} AethorPersistenceTransferKind;
/** @brief Retains one caller-owned immutable pair until one final admission or rejection. */
typedef struct {
    CanFrame frame;
    MotorPersistenceIdentity parameter_identity;
    DebugUiStaticTxIdentity disable_identity;
    AethorPersistenceTransferKind kind;
} AethorPersistenceTransfer;
/** @brief Returns the current ordinary-producer admission generation; zero never permits submission. */
uint32_t aethor_app_persistence_admission_generation(void);
/** @brief Final ordinary pop-to-HAL permit, rejecting an active/faulted maintenance lock or changed generation. */
uint8_t aethor_app_persistence_allow_ordinary(uint32_t admission_generation);
/** @brief Includes successful or failed maintenance awaiting explicit safe release and rediscovery. */
uint8_t aethor_app_persistence_locked(void);
/** @brief Supplies the whole atomic CDC callback word; any changed binding aborts the active owner.
 * Core must read it again at the final HAL callback, not only at the beginning of its control tick. */
void aethor_app_persistence_observe_link(uint32_t link_state, uint64_t now_us);
/** @brief Yields exactly one bounded parameter pair or selected DISABLE; never an enable/zero/motion frame. */
uint8_t aethor_app_persistence_next_transfer(uint64_t now_us, AethorPersistenceTransfer *transfer);
/** @brief Aborts an unaccepted immutable pair without requeue, fallthrough or Save retry. */
void aethor_app_persistence_transfer_rejected(const AethorPersistenceTransfer *transfer, uint64_t now_us);
/** @brief Returns the original outstanding parameter identity only when cancellation is required.
 * Platform owns cancellation latching and actual pending-debt lifetime; repeated polling is not a retry. */
uint8_t aethor_app_persistence_cancel_identity(MotorPersistenceIdentity *identity);
/** @brief Validates current owner/STOP/health/link before committing one parameter submission. */
uint8_t aethor_app_persistence_parameter_submitted(const MotorPersistenceIdentity *identity, uint64_t boundary_us);
/** @brief Validates the separate selected DISABLE identity immediately before its HAL submission. */
uint8_t aethor_app_persistence_disable_submitted(const DebugUiStaticTxIdentity *identity, uint64_t boundary_us);
/** @brief Consumes the unique parameter receipt and its software completion observation time. */
void aethor_app_persistence_parameter_transmitted(const MotorPersistenceIdentity *identity,
    uint8_t succeeded, uint64_t completed_observed_us, uint64_t now_us);
/** @brief Returns one only for its exact original maintenance DISABLE receipt, including late failures.
 * Other strict receipts must continue to the existing local static/POS consumer. */
uint8_t aethor_app_persistence_disable_transmitted(const DebugUiStaticTxIdentity *identity,
    uint8_t succeeded, uint16_t sof, uint64_t observed_us);
/** @brief Consumes fixed maintenance replies/selected health or quarantines AA before MotorBank.
 * Core provides the original strict RX identity/SOF when available. A zero identity cannot prove
 * DISABLE ordering. Ordinary unrelated feedback continues through the existing receive path. */
uint8_t aethor_app_persistence_receive(const CanFrame *frame, uint64_t arrival_lower_us,
    uint64_t observed_us, const DebugUiStaticTxIdentity *ordered_identity, uint16_t sof, uint8_t sof_valid);
#endif
#endif
