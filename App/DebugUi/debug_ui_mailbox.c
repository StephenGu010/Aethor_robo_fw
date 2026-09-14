/** @file debug_ui_mailbox.c
 * @brief Implements bounded request/result retention under caller-owned critical hooks.
 */
#include "debug_ui_mailbox.h"
#include <stddef.h>
#include <string.h>

/** @brief Compares all identity fields, never just a numeric request ID. */
static bool debug_ui_identity_equal(DebugUiIdentity left, DebugUiIdentity right)
{
    return (left.origin == right.origin) && (left.epoch == right.epoch) &&
           (left.request_id == right.request_id);
}

/** @brief Finds an exact retained transaction. */
static DebugUiMailboxTransaction *debug_ui_mailbox_find(
    DebugUiMailbox *mailbox, DebugUiIdentity identity)
{
    uint8_t slot_index;
    if (mailbox == NULL) { return NULL; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if ((transaction->used != 0U) &&
            debug_ui_identity_equal(transaction->request.identity, identity))
        { return transaction; }
    }
    return NULL;
}

/** @brief Releases storage only when UI consumed admission AND terminal. */
static void debug_ui_mailbox_retire(DebugUiMailboxTransaction *transaction)
{
    if ((transaction->admission_consumed != 0U) &&
        (transaction->completion_consumed != 0U))
    { memset(transaction, 0, sizeof(*transaction)); }
}

/** @brief Clears mailbox at application boot only. */
void debug_ui_mailbox_init(DebugUiMailbox *mailbox)
{
    if (mailbox != NULL) { memset(mailbox, 0, sizeof(*mailbox)); }
}

/** @brief Counts occupied records, including completed but unread work. */
uint8_t debug_ui_mailbox_result_count(const DebugUiMailbox *mailbox)
{
    uint8_t slot_index;
    uint8_t count = 0U;
    if (mailbox == NULL) { return 0U; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    { count += (uint8_t)(mailbox->transactions[slot_index].used != 0U); }
    return count;
}

/** @brief Keeps at most one unfinished ordinary request and one unfinished STOP. */
bool debug_ui_mailbox_busy(const DebugUiMailbox *mailbox, bool stop)
{
    uint8_t slot_index;
    if (mailbox == NULL) { return false; }
    if (stop && (mailbox->stop_intent_mask != 0U)) { return true; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        const DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if ((transaction->used != 0U) && (transaction->completion_ready == 0U) &&
            ((transaction->request.operation == DEBUG_UI_OPERATION_STOP) == stop))
        { return true; }
    }
    return false;
}

/** @brief Reserves outputs at submission so terminal publication cannot run out of room. */
DebugUiReason debug_ui_mailbox_submit(DebugUiMailbox *mailbox, const DebugUiRequest *request)
{
    uint8_t slot_index;
    uint8_t ordinary_count = 0U;
    bool stop;
    if ((mailbox == NULL) || (request == NULL) ||
        (request->identity.origin != DEBUG_UI_ORIGIN_LOCAL_UI) ||
        (request->identity.request_id == 0U) || (request->identity.epoch == 0U) ||
        (request->target_motor_id == 0U) || (request->target_motor_id > DEBUG_UI_MOTOR_COUNT) ||
        ((uint32_t)request->operation > (uint32_t)DEBUG_UI_OPERATION_STOP))
    { return DEBUG_UI_REASON_INVALID_ARGUMENT; }
    if (request->identity.request_id <= mailbox->last_request_id)
    { return DEBUG_UI_REASON_DUPLICATE; }
    stop = request->operation == DEBUG_UI_OPERATION_STOP;
    if (stop && (debug_ui_mailbox_busy(mailbox, true) ||
        (debug_ui_mailbox_result_count(mailbox) >= DEBUG_UI_RESULT_CAPACITY)))
    {
        mailbox->stop_intent_mask |= (uint8_t)(1U << (request->target_motor_id - 1U));
        mailbox->last_request_id = request->identity.request_id;
        return DEBUG_UI_REASON_STOP_LATCHED;
    }
    if (debug_ui_mailbox_busy(mailbox, stop)) { return DEBUG_UI_REASON_BUSY; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        if ((mailbox->transactions[slot_index].used != 0U) &&
            (mailbox->transactions[slot_index].request.operation != DEBUG_UI_OPERATION_STOP))
        { ++ordinary_count; }
    }
    if ((!stop && (ordinary_count >= DEBUG_UI_RESULT_CAPACITY - 1U)) ||
        (debug_ui_mailbox_result_count(mailbox) >= DEBUG_UI_RESULT_CAPACITY))
    { return DEBUG_UI_REASON_RESULT_BACKPRESSURE; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if (transaction->used == 0U)
        {
            memset(transaction, 0, sizeof(*transaction));
            transaction->request = *request;
            transaction->used = 1U;
            mailbox->last_request_id = request->identity.request_id;
            return DEBUG_UI_REASON_NONE;
        }
    }
    return DEBUG_UI_REASON_RESULT_BACKPRESSURE;
}

/** @brief Atomically takes the id-free safety intent under the caller's critical boundary. */
uint8_t debug_ui_mailbox_take_stop_intent(DebugUiMailbox *mailbox)
{
    uint8_t motor_mask;
    if (mailbox == NULL) { return 0U; }
    motor_mask = mailbox->stop_intent_mask;
    mailbox->stop_intent_mask = 0U;
    return motor_mask;
}

/** @brief Takes pending work without releasing its reserved admission/result slots. */
bool debug_ui_mailbox_take(DebugUiMailbox *mailbox, bool stop, DebugUiRequest *request)
{
    uint8_t slot_index;
    if ((mailbox == NULL) || (request == NULL)) { return false; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if ((transaction->used != 0U) && (transaction->taken == 0U) &&
            ((transaction->request.operation == DEBUG_UI_OPERATION_STOP) == stop))
        {
            *request = transaction->request;
            transaction->taken = 1U;
            return true;
        }
    }
    return false;
}

/** @brief Publishes exactly one matching admission. */
bool debug_ui_mailbox_admit(DebugUiMailbox *mailbox, const DebugUiAdmission *admission)
{
    DebugUiMailboxTransaction *transaction;
    if (admission == NULL) { return false; }
    transaction = debug_ui_mailbox_find(mailbox, admission->identity);
    if ((transaction == NULL) || (transaction->admission_ready != 0U)) { return false; }
    transaction->admission = *admission;
    transaction->admission_ready = 1U;
    return true;
}

/** @brief Publishes exactly one terminal into storage reserved at submit time. */
bool debug_ui_mailbox_complete(DebugUiMailbox *mailbox, const DebugUiCompletion *completion)
{
    DebugUiMailboxTransaction *transaction;
    if (completion == NULL) { return false; }
    transaction = debug_ui_mailbox_find(mailbox, completion->identity);
    if ((transaction == NULL) || (transaction->completion_ready != 0U)) { return false; }
    transaction->completion = *completion;
    transaction->completion_ready = 1U;
    return true;
}

/** @brief Consumes an admission while retaining an unread completion. */
bool debug_ui_mailbox_poll_admission(DebugUiMailbox *mailbox, DebugUiAdmission *admission)
{
    uint8_t slot_index;
    if ((mailbox == NULL) || (admission == NULL)) { return false; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if ((transaction->used != 0U) && (transaction->admission_ready != 0U) &&
            (transaction->admission_consumed == 0U))
        {
            *admission = transaction->admission;
            transaction->admission_consumed = 1U;
            debug_ui_mailbox_retire(transaction);
            return true;
        }
    }
    return false;
}

/** @brief Consumes a completion while retaining an unread admission. */
bool debug_ui_mailbox_poll_completion(DebugUiMailbox *mailbox, DebugUiCompletion *completion)
{
    uint8_t slot_index;
    if ((mailbox == NULL) || (completion == NULL)) { return false; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        DebugUiMailboxTransaction *transaction = &mailbox->transactions[slot_index];
        if ((transaction->used != 0U) && (transaction->completion_ready != 0U) &&
            (transaction->completion_consumed == 0U))
        {
            *completion = transaction->completion;
            transaction->completion_consumed = 1U;
            debug_ui_mailbox_retire(transaction);
            return true;
        }
    }
    return false;
}

/** @brief Reports pending ordinary/STOP requests or a new health notification. */
bool debug_ui_mailbox_needs_service(const DebugUiMailbox *mailbox)
{
    uint8_t slot_index;
    if (mailbox == NULL) { return false; }
    if (mailbox->stop_intent_mask != 0U) { return true; }
    if (mailbox->health.ui_sequence != mailbox->health.protocol_sequence) { return true; }
    for (slot_index = 0U; slot_index < DEBUG_UI_RESULT_CAPACITY; ++slot_index)
    {
        if ((mailbox->transactions[slot_index].used != 0U) &&
            (mailbox->transactions[slot_index].taken == 0U)) { return true; }
    }
    return false;
}

/** @brief Stores new UI progress only; repeated sequence cannot refresh a stalled UI. */
void debug_ui_mailbox_update_health(DebugUiMailbox *mailbox, const DebugUiHealth *health)
{
    if ((mailbox == NULL) || (health == NULL)) { return; }
    if (health->ui_sequence != mailbox->health.ui_sequence)
    {
        mailbox->health.ui_sequence = health->ui_sequence;
        mailbox->health.ui_timestamp_us = health->ui_timestamp_us;
    }
    mailbox->health.ui_valid = health->ui_valid;
    mailbox->health.input_valid = health->input_valid;
    mailbox->health.display_valid = health->display_valid;
    mailbox->health.display_error = health->display_error;
    mailbox->health.ui_diagnostics_valid = health->ui_diagnostics_valid;
    mailbox->health.adc_raw = health->adc_raw;
    mailbox->health.input_key = health->input_key;
    mailbox->health.input_age_ms = health->input_age_ms;
    mailbox->health.display_flush_last_us = health->display_flush_last_us;
    mailbox->health.display_dma_error_count = health->display_dma_error_count;
    mailbox->health.ui_stack_min_words = health->ui_stack_min_words;
}

/** @brief Records processing of fresh checked progress; repeated service is not progress. */
void debug_ui_mailbox_service_health(DebugUiMailbox *mailbox, uint64_t timestamp_us)
{
    DebugUiHealth *health;
    if (mailbox == NULL) { return; }
    health = &mailbox->health;
    if ((health->ui_sequence != health->protocol_sequence) &&
        (health->ui_valid != 0U) && (health->input_valid != 0U) &&
        (health->display_valid != 0U) && (health->display_error == 0U) &&
        (timestamp_us >= health->ui_timestamp_us) &&
        (timestamp_us - health->ui_timestamp_us <= DEBUG_UI_HEALTH_TIMEOUT_US))
    {
        health->protocol_sequence = health->ui_sequence;
        health->protocol_timestamp_us = timestamp_us;
        health->protocol_valid = 1U;
    }
}

/** @brief Both independently owned timestamps must be fresh and health flags valid. */
bool debug_ui_mailbox_healthy(const DebugUiMailbox *mailbox, uint64_t timestamp_us)
{
    const DebugUiHealth *health;
    if (mailbox == NULL) { return false; }
    health = &mailbox->health;
    return (health->ui_valid != 0U) && (health->protocol_valid != 0U) &&
        (health->input_valid != 0U) && (health->display_valid != 0U) &&
        (health->display_error == 0U) && (timestamp_us >= health->ui_timestamp_us) &&
        (timestamp_us >= health->protocol_timestamp_us) &&
        (timestamp_us - health->ui_timestamp_us <= DEBUG_UI_HEALTH_TIMEOUT_US) &&
        (timestamp_us - health->protocol_timestamp_us <= DEBUG_UI_HEALTH_TIMEOUT_US);
}
