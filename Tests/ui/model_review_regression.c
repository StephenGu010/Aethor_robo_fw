/** @file model_review_regression.c
 * @brief Persistent active-target and result-less STOP latch regressions.
 * Fixtures are simulated; a zero exit verifies expectations, never hardware.
 */
#define main baseline_model_main
#include "../host/debug_ui_model_test_main.c"
#undef main
#include <stdlib.h>

/** @brief Return console failures without a Windows CRT assertion dialog. */
static void regression_check(int condition, const char *expression, unsigned line)
{
    if (!condition) { fprintf(stderr, "REGRESSION_FAIL line=%u %s\n", line, expression); exit(1); }
}
#undef assert
#define assert(condition) regression_check((condition) != 0, #condition, __LINE__)

/** @brief Request STOP for remote ID3 despite ID1 focus, lease and old submission. */
static void test_active_target(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned stale_target;
    for (stale_target = 0U; stale_target <= 1U; ++stale_target) {
        snapshot.active = 1U; snapshot.active_motor_mask = 4U;
        snapshot.target_motor_id = (uint8_t)stale_target;
        snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
        debug_ui_model_init(&model);
        model.submitted.target_motor_id = 1U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
        assert(debug_ui_model_take_request(&model, &request));
        assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 3U);
    }
    puts("ACTIVE_MASK_TARGET_PASS remote_ID3_selected_ID1_old_submission_ID1");
}

/** @brief Track a result-less latch until coherent cleanup and fresh disable evidence. */
static void test_stop_latch(void)
{
    unsigned outcome;
    for (outcome = 0U; outcome < 7U; ++outcome) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request;
        snapshot.active = 1U; snapshot.active_motor_mask = 5U;
        snapshot.target_motor_id = 1U;
        snapshot.motors[0].enabled = snapshot.motors[2].enabled = 1U;
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
        assert(debug_ui_model_take_request(&model, &request));
        assert(DEBUG_UI_REASON_STOP_LATCHED == 15);
        debug_ui_model_submit_failed(&model, &request, DEBUG_UI_REASON_STOP_LATCHED);
        assert(model.page == DEBUG_UI_PAGE_RUNNING && !model.stop_waiting && model.stop_requested);
        assert(!model.completion_seen);
        snapshot.active = 0U;
        snapshot.active_motor_mask = 0U;
        snapshot.motors[0].enabled = snapshot.motors[2].enabled = 0U;
        /* An older copied snapshot cannot establish latch completion. */
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_RUNNING);
        snapshot.timestamp_us += 10000U; snapshot.stop_pending = 1U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_RUNNING);
        key(&model, DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP, DEBUG_UI_KEY_CENTER, 1010U, 0U);
        assert(!debug_ui_model_take_request(&model, &request));
        snapshot.timestamp_us += 10000U; snapshot.stop_pending = 0U;
        snapshot.motors[0].feedback_age_ms = snapshot.motors[2].feedback_age_ms = 1U;
        if (outcome == 1U) snapshot.motors[2].enabled = 1U;
        if (outcome == 2U) snapshot.motors[2].feedback_valid = 0U;
        if (outcome == 3U) snapshot.motors[2].feedback_age_ms = 30U;
        if (outcome == 4U) {
            snapshot.motors[2].driver_state = 8U;
            snapshot.motors[2].fault_flags = 8U;
        }
        if (outcome == 5U) snapshot.motors[2].driver_state = 2U;
        if (outcome == 6U) snapshot.motors[2].fault_flags = 1U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(!model.stop_waiting && !model.stop_requested && !model.completion_seen);
        assert(model.page == (outcome == 0U ? DEBUG_UI_PAGE_RESULT : DEBUG_UI_PAGE_FAULT));
    }
    puts("STOP_LATCH_SNAPSHOT_PASS no_terminal_wait_multi_motor_fresh_disable_or_fault");
}

/** @brief A missed pending=1 frame can settle, while an unrelated terminal cannot settle STOP. */
static void test_latch_result_interleaving(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest stop_request;
    DebugUiCompletion completion;
    snapshot.active = 1U; snapshot.active_motor_mask = 4U; snapshot.target_motor_id = 3U;
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    snapshot.motors[2].enabled = 1U;
    debug_ui_model_init(&model);
    model.awaiting_result = 1U;
    model.submitted.target_motor_id = 3U;
    model.submitted.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    model.submitted.identity.epoch = snapshot.epoch;
    model.submitted.identity.request_id = 10U;
    model.next_request_id = 11U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &stop_request));
    debug_ui_model_submit_failed(&model, &stop_request, DEBUG_UI_REASON_STOP_LATCHED);
    assert(model.awaiting_result && !model.stop_waiting);
    memset(&completion, 0, sizeof(completion));
    completion.identity = model.submitted.identity;
    completion.operation = DEBUG_UI_OPERATION_POS_MOVE;
    completion.code = DEBUG_UI_CANCELLED;
    debug_ui_model_completion(&model, &completion);
    assert(!model.awaiting_result && model.page == DEBUG_UI_PAGE_RUNNING);
    assert(model.stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING);
    snapshot.timestamp_us += 100000U;
    snapshot.active = 0U; snapshot.active_motor_mask = 0U;
    snapshot.motors[2].enabled = 0U; snapshot.motors[2].feedback_age_ms = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED && model.page == DEBUG_UI_PAGE_RESULT);
    assert(model.completion.identity.request_id == 10U); /* No invented STOP terminal. */
    assert(model.snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    puts("STOP_LATCH_INTERLEAVING_PASS original_terminal_preserved_no_auto_unlock");
}

/** @brief Select independently runnable regressions so each RED is observable. */
int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "target") == 0 || strcmp(argv[1], "all") == 0) test_active_target();
    if (strcmp(argv[1], "latch") == 0 || strcmp(argv[1], "all") == 0) {
        test_stop_latch();
        test_latch_result_interleaving();
    }
    return 0;
}
