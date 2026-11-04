/** @file Offline proxy isolation regression; no HAL, serial or motor transport is linked. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "DebugUi/debug_ui_mailbox.h"
/** @brief Build one immutable request in either the local or private high-bit domain. */
static DebugUiRequest make_request(uint32_t id, uint8_t remote, DebugUiOperation operation)
{
    DebugUiRequest request; memset(&request, 0, sizeof(request));
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI; request.identity.epoch = 1U;
    request.identity.request_id = id; request.target_motor_id = 7U;
    request.remote_proxy = remote; request.operation = operation; return request;
}
/** @brief Demonstrate remote completion cannot poison subsequent LCD IDs or steal LCD STOP receipts. */
int main(void)
{
    DebugUiMailbox mailbox; DebugUiRequest remote, local, taken;
    DebugUiAdmission admission, polled; DebugUiCompletion completion, terminal;
    debug_ui_mailbox_init(&mailbox);
    remote = make_request(0x80000001UL, 1U, DEBUG_UI_OPERATION_POS_MOVE);
    assert(debug_ui_mailbox_submit(&mailbox, &remote) == DEBUG_UI_REASON_NONE);
    assert(mailbox.last_request_id == 0U && mailbox.remote_last_request_id == 0x80000001UL);
    local = make_request(1U, 0U, DEBUG_UI_OPERATION_STOP);
    assert(debug_ui_mailbox_submit(&mailbox, &local) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_take(&mailbox, true, &taken) && !taken.remote_proxy);
    assert(debug_ui_mailbox_take(&mailbox, false, &taken) && taken.remote_proxy);
    memset(&admission, 0, sizeof(admission)); admission.identity = remote.identity; admission.accepted = 1U;
    memset(&completion, 0, sizeof(completion)); completion.identity = remote.identity;
    assert(debug_ui_mailbox_admit(&mailbox, &admission)); assert(debug_ui_mailbox_complete(&mailbox, &completion));
    assert(!debug_ui_mailbox_poll_admission(&mailbox, &polled));
    assert(!debug_ui_mailbox_poll_completion(&mailbox, &terminal));
    admission.identity = local.identity; completion.identity = local.identity;
    assert(debug_ui_mailbox_admit(&mailbox, &admission)); assert(debug_ui_mailbox_complete(&mailbox, &completion));
    assert(debug_ui_mailbox_poll_admission(&mailbox, &polled) && polled.identity.request_id == 1U);
    assert(debug_ui_mailbox_poll_completion(&mailbox, &terminal) && terminal.identity.request_id == 1U);
    local = make_request(2U, 0U, DEBUG_UI_OPERATION_POS_MOVE);
    assert(debug_ui_mailbox_submit(&mailbox, &local) == DEBUG_UI_REASON_NONE);
    assert(debug_ui_mailbox_submit(&mailbox, &remote) == DEBUG_UI_REASON_DUPLICATE);
    local = make_request(0x80000002UL, 0U, DEBUG_UI_OPERATION_STOP);
    assert(debug_ui_mailbox_submit(&mailbox, &local) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    local = make_request(3U, 1U, DEBUG_UI_OPERATION_STOP);
    assert(debug_ui_mailbox_submit(&mailbox, &local) == DEBUG_UI_REASON_INVALID_ARGUMENT);
    puts("REMOTE_UI_MAILBOX PASS: domains, STOP priority, LCD result isolation, duplicate rejection"); return 0;
}
