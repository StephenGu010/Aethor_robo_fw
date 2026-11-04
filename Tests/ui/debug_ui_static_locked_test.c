/** @file debug_ui_static_locked_test.c
 * @brief A motion-enabled build still rejects static HOLD when its independent gate defaults off.
 * All snapshots are synthetic and no application/transport is linked.
 */
#define main original_static_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main

/** @brief The closed static gate permits browsing while refusing every start/lock path. */
int main(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned index;
    assert(AETHOR_DEBUG_UI_ALLOW_MOTION && AETHOR_DEBUG_UI_ALLOW_MIT && !AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD);
    snapshot.static_load_enabled = 1U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_STATIC_HOLD) == DEBUG_UI_REASON_DISABLED);
    model.page = DEBUG_UI_PAGE_STATIC_HUB; model.focus = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_STATIC_LOAD);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1001U, 0U);
    assert(model.page == DEBUG_UI_PAGE_STATIC_ACTIONS);
    model.focus = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1002U, 0U);
    assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_DISABLED);
    assert(!model.static_locked && !debug_ui_model_take_request(&model, &request));
    puts("STATIC_UI_DEFAULT_LOCKED_OK browse=1 start_denied=1");
    return 0;
}
