/** @file debug_ui_readonly_test.c
 * @brief Compile-time M1 gate test even when a simulated snapshot claims motion.
 */
#include "debug_ui_model.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/** @brief No compile-time read-only build may queue motion or STOP. */
int main(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot;
    DebugUiRequest request;
    DebugUiInputEvent event;
    unsigned action_index;
    assert(!AETHOR_DEBUG_UI_ALLOW_MOTION);
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.motion_enabled = 1U;
    snapshot.active = 1U;
    snapshot.target_motor_id = 1U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, 0U, 1U, 1U);
    memset(&event, 0, sizeof(event));
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    event.key = DEBUG_UI_KEY_CENTER;
    debug_ui_model_event(&model, &event);
    event.type = DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP;
    debug_ui_model_event(&model, &event);
    assert(!debug_ui_model_take_request(&model, &request));
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_STOP) == DEBUG_UI_REASON_DISABLED);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    /* Every mutation is denied even if a synthetic snapshot claims permission. */
    snapshot.active = 0U;
    for (action_index = 0U; action_index <= DEBUG_UI_OPERATION_DISABLE; ++action_index) {
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, 0U, 1U, 1U);
        assert(!debug_ui_model_begin(&model, (DebugUiOperation)action_index, DEBUG_UI_MODE_MIT));
        assert(model.page == DEBUG_UI_PAGE_NOTICE && model.reason == DEBUG_UI_REASON_DISABLED);
        assert(!debug_ui_model_take_request(&model, &request));
    }
    /* All pages retain the compile-time STOP boundary, including new overlays. */
    snapshot.active = 1U;
    for (action_index = 0U; action_index < DEBUG_UI_PAGE_COUNT; ++action_index) {
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, 0U, 1U, 1U);
        model.page = (DebugUiPage)action_index;
        event.type = DEBUG_UI_INPUT_EVENT_PRESS;
        event.key = DEBUG_UI_KEY_CENTER;
        debug_ui_model_event(&model, &event);
        assert(!debug_ui_model_take_request(&model, &request));
    }
    puts("DEBUG_UI_COMPILETIME_READONLY_OK");
    return 0;
}
