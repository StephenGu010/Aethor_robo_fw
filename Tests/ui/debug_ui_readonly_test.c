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
    /* Blocked acquisition must still open an informative page without requests. */
    debug_ui_model_init(&model);
    snapshot.active = 0U;
    debug_ui_model_update(&model, &snapshot, 0U, 1U, 1U);
    model.focus = 2U;
    model.selected_motor = 6U;
    event.type = DEBUG_UI_INPUT_EVENT_PRESS;
    event.key = DEBUG_UI_KEY_RIGHT;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_PREPARE);
    assert(!debug_ui_model_take_request(&model, &request));
    event.key = DEBUG_UI_KEY_CENTER;
    debug_ui_model_event(&model, &event);
    assert(!debug_ui_model_take_request(&model, &request));
    event.key = DEBUG_UI_KEY_LEFT;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_OVERVIEW);
    event.key = DEBUG_UI_KEY_RIGHT;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_PREPARE);
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_DETAIL && model.selected_motor == 6U);
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_ACTIONS && model.focus == 0U);
    /* Every state-changing menu entry remains gated in a compile-time read-only build. */
    for (action_index = 0U; action_index < 7U; ++action_index) {
        model.focus = (uint8_t)action_index;
        debug_ui_model_event(&model, &event);
        assert(model.page == DEBUG_UI_PAGE_ACTIONS && model.reason == DEBUG_UI_REASON_DISABLED);
        assert(!debug_ui_model_take_request(&model, &request));
    }
    model.focus = 7U;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_REGISTERS);
    event.key = DEBUG_UI_KEY_CENTER;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_REGISTERS);
    event.key = DEBUG_UI_KEY_LEFT;
    debug_ui_model_event(&model, &event);
    assert(model.page == DEBUG_UI_PAGE_ACTIONS);
    assert(!debug_ui_model_take_request(&model, &request));
    puts("DEBUG_UI_COMPILETIME_READONLY_OK");
    return 0;
}
