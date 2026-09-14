/** @file debug_ui_mailbox_test_main.c
 * @brief Tests reserved results, identity isolation, STOP capacity and dual health.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "debug_ui_mailbox.h"

/** @brief Creates an immutable local request with a distinct monotonically increasing ID. */
static DebugUiRequest request_for(uint32_t request_id, DebugUiOperation operation)
{
    DebugUiRequest request;
    memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request.identity.epoch = 9U;
    request.identity.request_id = request_id;
    request.operation = operation;
    request.target_motor_id = 1U;
    return request;
}

/** @brief Completes the exact request while retaining both unread outputs. */
static void finish(DebugUiMailbox *mailbox, const DebugUiRequest *request)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    memset(&admission, 0, sizeof(admission));
    memset(&completion, 0, sizeof(completion));
    admission.identity = request->identity;
    admission.accepted = 1U;
    completion.identity = request->identity;
    assert(debug_ui_mailbox_admit(mailbox, &admission));
    assert(debug_ui_mailbox_complete(mailbox, &completion));
    assert(!debug_ui_mailbox_complete(mailbox, &completion));
}

/** @brief A full retained result ring still latches STOP without overwriting identities. */
static void test_p1_stop_intent_outlives_result_capacity(void)
{
    DebugUiMailbox mailbox;
    DebugUiRequest request;
    DebugUiRequest taken;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiMailboxTransaction retained[DEBUG_UI_RESULT_CAPACITY];
    uint32_t request_id;
    debug_ui_mailbox_init(&mailbox);
    for (request_id = 1U; request_id <= DEBUG_UI_RESULT_CAPACITY; ++request_id)
    {
        request = request_for(request_id, DEBUG_UI_OPERATION_STOP);
        assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
        assert(debug_ui_mailbox_take(&mailbox, true, &taken));
        finish(&mailbox, &taken);
    }
    memcpy(retained, mailbox.transactions, sizeof(retained));
    request = request_for(5U, DEBUG_UI_OPERATION_STOP);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_STOP_LATCHED);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_DUPLICATE);
    for (request_id = 6U; request_id < 10006U; ++request_id)
    {
        request = request_for(request_id, DEBUG_UI_OPERATION_STOP);
        request.target_motor_id = (uint8_t)(request_id % DEBUG_UI_MOTOR_COUNT + 1U);
        request.identity.epoch = request_id; /* Epoch changes do not alias retained identities. */
        assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_STOP_LATCHED);
    }
    assert(debug_ui_mailbox_busy(&mailbox, true));
    assert(debug_ui_mailbox_needs_service(&mailbox));
    assert(memcmp(retained, mailbox.transactions, sizeof(retained)) == 0);
    assert(debug_ui_mailbox_result_count(&mailbox) == DEBUG_UI_RESULT_CAPACITY);
    for (request_id = 1U; request_id <= DEBUG_UI_RESULT_CAPACITY; ++request_id)
    {
        assert(debug_ui_mailbox_poll_admission(&mailbox, &admission));
        assert(admission.identity.request_id == request_id);
        assert(debug_ui_mailbox_poll_completion(&mailbox, &completion));
        assert(completion.identity.request_id == request_id);
    }
    assert(!debug_ui_mailbox_poll_completion(&mailbox, &completion));
    assert(debug_ui_mailbox_take_stop_intent(&mailbox) == 0x7FU);
    assert(debug_ui_mailbox_take_stop_intent(&mailbox) == 0U);
    assert(!debug_ui_mailbox_busy(&mailbox, true));
    assert(!debug_ui_mailbox_needs_service(&mailbox));
    request = request_for(10006U, DEBUG_UI_OPERATION_STOP);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_take(&mailbox, true, &taken));
    memcpy(retained, mailbox.transactions, sizeof(retained));
    request = request_for(10007U, DEBUG_UI_OPERATION_STOP);
    request.target_motor_id = 3U;
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_STOP_LATCHED);
    assert(memcmp(retained, mailbox.transactions, sizeof(retained)) == 0);
    assert(debug_ui_mailbox_take_stop_intent(&mailbox) == 4U);
    assert(debug_ui_mailbox_busy(&mailbox, true));
    finish(&mailbox, &taken);
    assert(debug_ui_mailbox_poll_completion(&mailbox, &completion));
    assert(completion.identity.request_id == taken.identity.request_id);
    assert(debug_ui_mailbox_result_count(&mailbox) == 1U);
    assert(debug_ui_mailbox_poll_admission(&mailbox, &admission));
    assert(admission.identity.request_id == taken.identity.request_id);
    assert(debug_ui_mailbox_result_count(&mailbox) == 0U);
}

/** @brief Runs behavioral checks without a scheduler or hardware. */
int main(void)
{
    DebugUiMailbox mailbox;
    DebugUiRequest request = request_for(1U, DEBUG_UI_OPERATION_POS_MOVE);
    DebugUiRequest copied;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiHealth health;
    uint32_t request_id;
    debug_ui_mailbox_init(&mailbox);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_DUPLICATE);
    request = request_for(2U, DEBUG_UI_OPERATION_POS_MOVE);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_BUSY);
    request.operation = DEBUG_UI_OPERATION_STOP;
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_take(&mailbox, true, &copied));
    assert(copied.identity.request_id == 2U);
    finish(&mailbox, &copied);
    assert(debug_ui_mailbox_take(&mailbox, false, &copied));
    memset(&completion, 0, sizeof(completion));
    completion.identity = copied.identity;
    completion.identity.origin = DEBUG_UI_ORIGIN_USB;
    assert(!debug_ui_mailbox_complete(&mailbox, &completion));
    completion.identity = copied.identity;
    --completion.identity.epoch;
    assert(!debug_ui_mailbox_complete(&mailbox, &completion));
    finish(&mailbox, &copied);
    assert(debug_ui_mailbox_poll_completion(&mailbox, &completion));
    assert(debug_ui_mailbox_result_count(&mailbox) == 2U);
    assert(debug_ui_mailbox_poll_admission(&mailbox, &admission));
    assert(debug_ui_mailbox_result_count(&mailbox) == 1U);
    assert(debug_ui_mailbox_poll_admission(&mailbox, &admission));
    assert(debug_ui_mailbox_poll_completion(&mailbox, &completion));
    assert(debug_ui_mailbox_result_count(&mailbox) == 0U);
    for (request_id = 3U; request_id <= 5U; ++request_id)
    {
        request = request_for(request_id, DEBUG_UI_OPERATION_POS_MOVE);
        assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
        assert(debug_ui_mailbox_take(&mailbox, false, &copied));
        finish(&mailbox, &copied);
    }
    request = request_for(6U, DEBUG_UI_OPERATION_POS_MOVE);
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_RESULT_BACKPRESSURE);
    request.operation = DEBUG_UI_OPERATION_STOP;
    assert(debug_ui_mailbox_submit(&mailbox, &request) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_result_count(&mailbox) == 4U);
    assert(debug_ui_mailbox_take(&mailbox, true, &copied));
    finish(&mailbox, &copied);
    memset(&health, 0, sizeof(health));
    health.ui_timestamp_us = 100U;
    health.ui_sequence = 1U;
    health.ui_valid = health.input_valid = health.display_valid = 1U;
    debug_ui_mailbox_update_health(&mailbox, &health);
    assert(!debug_ui_mailbox_healthy(&mailbox, 100U));
    debug_ui_mailbox_service_health(&mailbox, 100U);
    assert(debug_ui_mailbox_healthy(&mailbox, 100U));
    assert(!debug_ui_mailbox_healthy(&mailbox, 99U));
    assert(!debug_ui_mailbox_healthy(&mailbox, 150101U));
    health.ui_timestamp_us = 150101U;
    ++health.ui_sequence;
    debug_ui_mailbox_update_health(&mailbox, &health);
    assert(!debug_ui_mailbox_healthy(&mailbox, 150101U));
    debug_ui_mailbox_service_health(&mailbox, 150101U);
    assert(debug_ui_mailbox_healthy(&mailbox, 150101U));
    health.display_valid = 0U;
    ++health.ui_sequence;
    debug_ui_mailbox_update_health(&mailbox, &health);
    assert(!debug_ui_mailbox_healthy(&mailbox, 150101U));
    test_p1_stop_intent_outlives_result_capacity();
    puts("debug_ui_mailbox: identity, reserved STOP/results, dual health PASS");
    return 0;
}
