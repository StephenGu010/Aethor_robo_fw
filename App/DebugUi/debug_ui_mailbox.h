/** @file debug_ui_mailbox.h
 * @brief Bounded local transactions. Caller holds the existing App critical hooks
 * around each operation; no function calls RTOS, drivers, allocation or formatting.
 */
#ifndef APP_DEBUG_UI_DEBUG_UI_MAILBOX_H
#define APP_DEBUG_UI_DEBUG_UI_MAILBOX_H
#include <stdbool.h>
#include "debug_ui_contract.h"

/** @brief Reserved transaction: admission and completion retire only after both polls. */
typedef struct
{
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    uint8_t used;
    uint8_t taken;
    uint8_t admission_ready;
    uint8_t admission_consumed;
    uint8_t completion_ready;
    uint8_t completion_consumed;
} DebugUiMailboxTransaction;

/** @brief Single ordinary request, independent STOP, and four reserved result records. */
typedef struct
{
    DebugUiMailboxTransaction transactions[DEBUG_UI_RESULT_CAPACITY];
    DebugUiHealth health;
    uint32_t last_request_id;
    /** @brief Coalesced unaccepted STOP intents, independent of all retained identities. */
    uint8_t stop_intent_mask;
} DebugUiMailbox;

/** @brief Initializes all request/result/health records as empty. */
void debug_ui_mailbox_init(DebugUiMailbox *mailbox);
/** @brief Reserves both results, or explicitly returns STOP_LATCHED without a transaction. */
DebugUiReason debug_ui_mailbox_submit(DebugUiMailbox *mailbox, const DebugUiRequest *request);
/** @brief ArmControlTask consumes only the coalesced safety mask, never an accepted identity. */
uint8_t debug_ui_mailbox_take_stop_intent(DebugUiMailbox *mailbox);
/** @brief ProtocolTask takes the oldest matching pending request; STOP is separate. */
bool debug_ui_mailbox_take(DebugUiMailbox *mailbox, bool stop, DebugUiRequest *request);
/** @brief Publishes the unique reserved admission without reclaiming its transaction. */
bool debug_ui_mailbox_admit(DebugUiMailbox *mailbox, const DebugUiAdmission *admission);
/** @brief Publishes the unique reserved terminal; fails instead of losing an identity. */
bool debug_ui_mailbox_complete(DebugUiMailbox *mailbox, const DebugUiCompletion *completion);
/** @brief Consumes one retained admission; this is the UI acknowledgement. */
bool debug_ui_mailbox_poll_admission(DebugUiMailbox *mailbox, DebugUiAdmission *admission);
/** @brief Consumes one retained completion; this is the UI acknowledgement. */
bool debug_ui_mailbox_poll_completion(DebugUiMailbox *mailbox, DebugUiCompletion *completion);
/** @brief Returns occupied transaction count including unconsumed terminal records. */
uint8_t debug_ui_mailbox_result_count(const DebugUiMailbox *mailbox);
/** @brief Reports any unfinished request of the selected kind, including active work. */
bool debug_ui_mailbox_busy(const DebugUiMailbox *mailbox, bool stop);
/** @brief Reports pending request or unacknowledged health for ProtocolTask notification. */
bool debug_ui_mailbox_needs_service(const DebugUiMailbox *mailbox);
/** @brief Copies UI-owned health fields only, preserving the ProtocolTask acknowledgement. */
void debug_ui_mailbox_update_health(DebugUiMailbox *mailbox, const DebugUiHealth *health);
/** @brief ProtocolTask acknowledges a new healthy UI progress sample once. */
void debug_ui_mailbox_service_health(DebugUiMailbox *mailbox, uint64_t timestamp_us);
/** @brief Tests independent UI progress and ProtocolTask service ages, including clock rollback. */
bool debug_ui_mailbox_healthy(const DebugUiMailbox *mailbox, uint64_t timestamp_us);

#endif
