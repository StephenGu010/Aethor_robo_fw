/** @file load_trial_pages_model_test.c
 * @brief Offline navigation, metadata isolation and STOP-priority checks; no App endpoint is linked.
 */
#define main original_load_page_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main

/** @brief The setup's right-hand entry opens display-only metadata without emitting a request. */
static void test_load_entry_without_output(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.single_static_flow = 1U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_METADATA);
    assert(!debug_ui_model_take_request(&model, &request));
    assert(model.snapshot.authority == snapshot.authority && model.selected_motor == 0U);
}

/** @brief Release CENTER before the next independent metadata action. */
static void load_page_center(DebugUiModel *model, uint32_t timestamp)
{
    key(model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, timestamp, 0U);
    key(model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, timestamp + 1U, 0U);
}

/** @brief Coarse/fine edits clamp bounds, invalidate confirmation and remain isolated by ID. */
static void test_load_candidate_isolation_and_bounds(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest draft_before, reviewed_before, submitted_before, request;
    const DebugUiLoadMetadata *metadata;
    unsigned index;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_LOAD_METADATA;
    model.selected_motor = 2U;
    draft_before = model.draft; reviewed_before = model.reviewed; submitted_before = model.submitted;
    metadata = debug_ui_load_trials_candidate(&model.load_trials, 3U);
    assert(metadata->lever_mm == 250U && !metadata->confirmed);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(metadata->lever_mm == 260U);
    load_page_center(&model, 1001U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_NUMBER);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1003U, 0U);
    assert(metadata->lever_mm == 261U);
    for (index = 0U; index < 300U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_LEFT, 1010U + index, 0U);
    assert(metadata->lever_mm == 1U);
    for (index = 0U; index < 300U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_RIGHT, 1400U + index, 0U);
    assert(metadata->lever_mm == 2000U);
    load_page_center(&model, 1800U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_METADATA);
    model.focus = 1U;
    for (index = 0U; index < 1005U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_RIGHT, 1900U + index, 0U);
    assert(metadata->mass_g == 10000U);
    model.focus = 3U; load_page_center(&model, 3000U);
    assert(metadata->confirmed);
    model.focus = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 3002U, 0U);
    assert(metadata->mass_g == 9990U && !metadata->confirmed);
    assert(debug_ui_load_trials_candidate(&model.load_trials, 1U)->lever_mm == 250U);
    assert(debug_ui_load_trials_candidate(&model.load_trials, 1U)->mass_g == 0U);
    assert(!memcmp(&draft_before, &model.draft, sizeof(draft_before)));
    assert(!memcmp(&reviewed_before, &model.reviewed, sizeof(reviewed_before)));
    assert(!memcmp(&submitted_before, &model.submitted, sizeof(submitted_before)));
    assert(model.snapshot.authority == snapshot.authority && model.selected_motor == 2U);
    assert(!debug_ui_model_take_request(&model, &request));
}

/** @brief Each activity/cleanup form blocks edits and CENTER activity still produces STOP. */
static void test_load_busy_and_stop_priority(void)
{
    unsigned condition;
    for (condition = 0U; condition < 10U; ++condition) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        const DebugUiLoadMetadata *metadata;
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_LOAD_METADATA;
        model.snapshot.target_motor_id = 1U;
        model.snapshot.active_motor_mask = 1U;
        model.submitted.target_motor_id = 1U;
        if (condition == 0U) model.snapshot.active = 1U;
        else if (condition == 1U) model.snapshot.pending = 1U;
        else if (condition == 2U) model.awaiting_result = 1U;
        else if (condition == 3U) model.snapshot.stop_pending = 1U;
        else if (condition == 4U) model.snapshot.unconfirmed_disable_mask = 1U;
        else if (condition == 5U) model.stop_waiting = 1U;
        else if (condition == 6U) model.stop_requested = 1U;
        else if (condition == 7U) model.stop_ready = 1U;
        else if (condition == 8U) model.outgoing_ready = 1U;
        else model.stop_latch_state = DEBUG_UI_STOP_LATCH_UNCONFIRMED;
        metadata = debug_ui_load_trials_candidate(&model.load_trials, 1U);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
        assert(metadata->lever_mm == 250U && !metadata->confirmed);
        model.focus = 3U; load_page_center(&model, 1001U);
        assert(!metadata->confirmed);
        if (condition < 4U) assert(model.stop_ready || model.stop_requested);
    }
}

/** @brief History pagination and detail faces cannot change control selection or any request scope. */
static void test_load_history_read_only(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request, before;
    unsigned index;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    memset(&request, 0, sizeof(request));
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
    request.target_motor_id = 4U; request.hold_duration_ms = 1000U;
    request.identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI; request.identity.epoch = snapshot.epoch;
    for (index = 1U; index <= 18U; ++index) {
        request.identity.request_id = index;
        assert(debug_ui_load_trials_freeze(&model.load_trials, 4U));
        assert(debug_ui_load_trials_submit(&model.load_trials, &request));
    }
    assert(model.load_trials.count == 16U);
    before = model.submitted;
    model.selected_motor = 2U;
    model.page = DEBUG_UI_PAGE_LOAD_HISTORY;
    for (index = 0U; index < 20U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1000U + index, 0U);
    assert(model.focus == 15U && model.list_first == 11U);
    load_page_center(&model, 1100U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_DETAIL && model.load_history_offset == 15U);
    assert(debug_ui_load_trials_newest(&model.load_trials, 15U)->identity.request_id == 3U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1102U, 0U);
    assert(model.focus == 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1103U, 0U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_HISTORY && model.focus == 15U);
    assert(model.selected_motor == 2U && model.snapshot.authority == snapshot.authority);
    assert(!memcmp(&before, &model.submitted, sizeof(before)));
    assert(!debug_ui_model_take_request(&model, &request));
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1104U, 0U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_METADATA);
    debug_ui_load_trials_init(&model.load_trials);
    model.focus = 4U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1105U, 0U);
    assert(model.page == DEBUG_UI_PAGE_LOAD_HISTORY && model.focus == 0U && model.list_first == 0U);
}

/** @brief Execute only RAM model behavior without hardware access. */
int main(int argc, char **argv)
{
    if (argc == 1 || !strcmp(argv[1], "entry")) test_load_entry_without_output();
    if (argc == 1 || !strcmp(argv[1], "isolation")) test_load_candidate_isolation_and_bounds();
    if (argc == 1 || !strcmp(argv[1], "busy")) test_load_busy_and_stop_priority();
    if (argc == 1 || !strcmp(argv[1], "history")) test_load_history_read_only();
    puts("LOAD_TRIAL_PAGES_MODEL_OK");
    return 0;
}
