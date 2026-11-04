/** @file static_gain_model_test.c
 * @brief Exercise gain selection through real key events without any motor transport.
 */
#define main existing_single_static_main
#include "debug_ui_model_test_main.c"
#undef main

/** @brief The new parameter entry must open without publishing a control request. */
int main(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    assert(debug_ui_static_gains_valid(200.0f, 0.30f));
    assert(!debug_ui_static_gains_valid(201.0f, 0.30f));
    debug_ui_model_init(&model);
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    snapshot.motors[6].single_static_profile.configured = 0U;
    snapshot.motors[6].single_static_lcd_owned = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.selected_motor = 6U;
    model.single_static_flow = 1U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    model.focus = 4U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(model.page != DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    assert(!debug_ui_model_take_request(&model, &request));
    assert(model.single_static_kp == 120U && model.single_static_kd_centi == 30U);
    model.single_static_kp = 100U; /* Exercise interior digit edits independently of the boot default. */
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1002U, 0U);
    assert(model.gain_editing);
    model.gain_digit = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1003U, 0U);
    assert(model.gain_edit_value == 110U && model.single_static_kp == 100U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 1004U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_LEFT, 1904U, 900U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_LEFT, 1905U, 0U);
    assert(!model.gain_editing && model.single_static_kp == 100U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1905U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1906U, 0U);
    model.gain_digit = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1907U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1908U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1909U, 0U);
    assert(model.single_static_kp == 110U && !model.gain_editing);
    assert(!debug_ui_model_take_request(&model, &request));
    model.focus = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1910U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1911U, 0U);
    model.gain_digit = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1912U, 0U);
    assert(model.gain_edit_value == 40U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1913U, 0U);
    assert(model.gain_edit_value == 20U);
    model.snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1914U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1915U, 0U);
    assert(!model.gain_editing && model.single_static_kd_centi == 30U);
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    assert(!debug_ui_model_take_request(&model, &request));
    debug_ui_model_init(&model);
    assert(model.single_static_kp == 120U && model.single_static_kd_centi == 30U);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE; model.selected_motor = 6U;
    model.page = DEBUG_UI_PAGE_SINGLE_STATIC_GAINS; model.focus = 0U;
    model.gain_editing = 1U; model.gain_digit = 0U; model.gain_edit_value = 100U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 2000U, 0U);
    assert(model.gain_edit_value == 80U);
    model.gain_digit = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 2001U, 0U);
    assert(model.gain_edit_value == 90U);
    model.gain_digit = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 2002U, 0U);
    assert(model.gain_edit_value == 99U);
    model.gain_digit = 0U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 2003U, 0U);
    assert(model.gain_edit_value == 199U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 2004U, 0U);
    assert(model.gain_edit_value == 200U);
    model.gain_digit = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 2005U, 0U);
    assert(model.gain_edit_value == 200U);
    model.gain_digit = 0U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 2006U, 0U);
    assert(model.gain_edit_value == 100U);
    puts("STATIC_GAIN_ENTRY_PASS defaults=1 scratch=1 cancel=1 save=1 bounds=1 ownership=1 reset=1 no_tx=1");
    return 0;
}
