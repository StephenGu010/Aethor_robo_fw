/** @file pos_load_digit_cases.inc
 * @brief Exercise approved decimal editing through production model input events.
 */

/** @brief Send an ordinary key press and release without creating any motor command. */
static void digit_key(DebugUiModel *model, DebugUiKey key)
{
    static uint64_t timestamp = 7000ULL;
    workflow_key(model, DEBUG_UI_INPUT_EVENT_PRESS, key, timestamp++, 0U);
    workflow_key(model, DEBUG_UI_INPUT_EVENT_RELEASE, key, timestamp++, 0U);
}

/** @brief Read display degrees while preserving the production output/flange transform. */
static float digit_angle(const DebugUiModel *model)
{
    return debug_ui_radians_to_degrees(debug_ui_pos_load_display(&model->snapshot.motors[6], model->draft.delta_rad));
}

/** @brief Cover direct place edits, sign, legal wrap, cancel and zero command emission. */
static void pos_load_digit_cases(void)
{
    DebugUiModel model;
    unsigned index;
    float saved;
    flange_fixture(512.0F, 0.0F);
    flange_acquire();
    debug_ui_model_init(&model);
    model.selected_motor = 6U; model.page = DEBUG_UI_PAGE_STATIC_HUB; model.focus = 2U;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    model.focus = 1U;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    digit_key(&model, DEBUG_UI_KEY_UP); /* ones: 001.0 -> 002.0 */
    assert(fabsf(digit_angle(&model) - 2.0F) < 0.001F);
    digit_key(&model, DEBUG_UI_KEY_LEFT);
    digit_key(&model, DEBUG_UI_KEY_UP); /* tens */
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_NUMBER);
    assert(fabsf(digit_angle(&model) - 12.0F) < 0.001F);
    digit_key(&model, DEBUG_UI_KEY_LEFT);
    digit_key(&model, DEBUG_UI_KEY_UP); /* hundreds */
    assert(fabsf(digit_angle(&model) - 112.0F) < 0.001F);
    digit_key(&model, DEBUG_UI_KEY_UP); /* 212..912 invalid; wraps to 012 */
    assert(fabsf(digit_angle(&model) - 12.0F) < 0.001F);
    digit_key(&model, DEBUG_UI_KEY_LEFT);
    digit_key(&model, DEBUG_UI_KEY_DOWN); /* independent minus */
    assert(fabsf(digit_angle(&model) + 12.0F) < 0.001F);
    for (index = 0U; index < 4U; ++index) digit_key(&model, DEBUG_UI_KEY_RIGHT);
    digit_key(&model, DEBUG_UI_KEY_DOWN); /* tenths 0 -> 9, no borrowing */
    assert(fabsf(digit_angle(&model) + 12.9F) < 0.001F);
    for (index = 0U; index < 60U; ++index) {
        digit_key(&model, DEBUG_UI_KEY_UP);
        assert(fabsf(digit_angle(&model)) <= 180.0001F && !model.outgoing_ready);
    }
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    saved = model.draft.delta_rad;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    digit_key(&model, DEBUG_UI_KEY_UP);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, 100000ULL, 0U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_LEFT, 900000ULL, 800U);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_LEFT, 950000ULL, 850U);
    assert(model.draft.delta_rad == saved);
    workflow_key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_LEFT, 951000ULL, 851U);
    assert(model.page == DEBUG_UI_PAGE_POS_LOAD_SETUP && model.draft.delta_rad == saved);
    model.focus = 3U;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    digit_key(&model, DEBUG_UI_KEY_DOWN); /* seconds ones 1 -> 0 invalid -> 9 */
    assert(model.draft.hold_duration_ms == 9000U);
    for (index = 0U; index < 100U; ++index) {
        digit_key(&model, DEBUG_UI_KEY_LEFT);
        digit_key(&model, DEBUG_UI_KEY_UP);
        assert(model.draft.hold_duration_ms >= 1000U && model.draft.hold_duration_ms <= 60000U);
        assert(model.draft.hold_duration_ms % 1000U == 0U && !model.outgoing_ready);
    }
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    model.focus = 2U;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    for (index = 0U; index < 200U; ++index) {
        DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(&model.snapshot.motors[6]);
        digit_key(&model, DEBUG_UI_KEY_RIGHT);
        digit_key(&model, DEBUG_UI_KEY_DOWN);
        assert(model.pos_digit_cursor >= 1U && model.pos_digit_cursor <= 4U);
        assert(model.draft.speed_rad_s > 0.0F && model.draft.speed_rad_s <= bounds.max_speed_rad_s);
        assert(!model.outgoing_ready);
    }
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    model.focus = 1U;
    model.draft.delta_rad = debug_ui_degrees_to_radians(180.0F) * debug_ui_pos_load_scale(&model.snapshot.motors[6]);
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    digit_key(&model, DEBUG_UI_KEY_RIGHT);
    digit_key(&model, DEBUG_UI_KEY_UP); /* 180.1..180.9 are forbidden. */
    assert(model.pos_digit_value == 1800U && fabsf(digit_angle(&model)) <= 180.0001F);
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    model.draft.delta_rad = 0.0F;
    digit_key(&model, DEBUG_UI_KEY_CENTER);
    for (index = 0U; index < 3U; ++index) digit_key(&model, DEBUG_UI_KEY_LEFT);
    digit_key(&model, DEBUG_UI_KEY_DOWN);
    assert(model.pos_digit_negative && model.pos_digit_value == 0U);
    for (index = 0U; index < 4U; ++index) digit_key(&model, DEBUG_UI_KEY_RIGHT);
    digit_key(&model, DEBUG_UI_KEY_UP);
    assert(fabsf(digit_angle(&model) + 0.1F) < 0.001F && !model.outgoing_ready);
    puts("POS_DIGIT_EDITOR_PASS places=4 sign=1 negative_zero=1 exact180=1 wrap=1 bounds=1 cancel_repeat=1 positive_speed=1 no_commands=1");
}
