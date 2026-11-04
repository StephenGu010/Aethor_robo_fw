/** @file debug_ui_static_gain_pages.inc
 * @brief Idle-only ID7 gain draft editor; never emits a control request.
 */
/** @brief Reject changes when any owner, preparation, motion or cleanup remains. */
static uint8_t single_gain_edit_allowed(const DebugUiModel *model)
{
    return (uint8_t)(model->selected_motor == 6U && !single_flow_busy(model) &&
        !model->single_static_configured && !single_any_lcd_profile(model) &&
        model->snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE);
}
/** @brief Route the parameter subpage with scratch editing and explicit cancellation. */
static void single_gain_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    if (!single_gain_edit_allowed(model)) {
        model->gain_editing = model->gain_left_down = 0U;
        model->reason = DEBUG_UI_REASON_BUSY;
        navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        return;
    }
    if (model->gain_left_down == 2U && event->key == DEBUG_UI_KEY_LEFT) {
        if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE) model->gain_left_down = 0U;
        return;
    }
    if (model->gain_editing) {
        if (event->key == DEBUG_UI_KEY_LEFT) {
            if (event->type == DEBUG_UI_INPUT_EVENT_PRESS) model->gain_left_down = 1U;
            if (event->type == DEBUG_UI_INPUT_EVENT_HOLD && event->held_ms >= DEBUG_UI_REVIEW_HOLD_MS) {
                model->gain_editing = 0U; model->gain_left_down = 2U;
            }
            if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE && model->gain_left_down == 1U) {
                model->gain_digit = (uint8_t)((model->gain_digit + 2U) % 3U);
                model->gain_left_down = 0U;
            }
            return;
        }
        if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return;
        if (event->key == DEBUG_UI_KEY_RIGHT) model->gain_digit = (uint8_t)((model->gain_digit + 1U) % 3U);
        if (event->key == DEBUG_UI_KEY_UP || event->key == DEBUG_UI_KEY_DOWN) {
            static const uint16_t places[3] = {100U, 10U, 1U};
            uint16_t place = places[model->gain_digit];
            uint16_t digit = (model->gain_edit_value / place) % 10U;
            uint16_t remainder = model->gain_edit_value - digit * place;
            unsigned attempt;
            for (attempt = 0U; attempt < 10U; ++attempt) {
                uint16_t candidate;
                digit = (uint16_t)((digit + (event->key == DEBUG_UI_KEY_UP ? 1U : 9U)) % 10U);
                candidate = remainder + digit * place;
                /* Crossing the hundreds boundary lands on a valid endpoint, keeping 80..99 reachable. */
                if (model->focus == 0U && place == 100U) {
                    if (candidate < 80U) candidate = 80U;
                    if (candidate > DEBUG_UI_STATIC_KP_MAX) candidate = DEBUG_UI_STATIC_KP_MAX;
                }
                if ((model->focus == 0U && candidate >= 80U && candidate <= DEBUG_UI_STATIC_KP_MAX) ||
                    (model->focus == 1U && candidate >= 20U && candidate <= 40U)) {
                    model->gain_edit_value = candidate; break;
                }
            }
        }
        if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS) {
            if (model->focus == 0U) model->single_static_kp = model->gain_edit_value;
            else model->single_static_kd_centi = model->gain_edit_value;
            model->gain_editing = model->static_locked = 0U;
            model->review_pressed = model->review_released = 0U;
            model->confirm_elapsed_ms = 0U;
            model->center_consumed = 1U;
        }
        return;
    }
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return;
    if (event->key == DEBUG_UI_KEY_UP) move_focus(model, -1, 4U);
    if (event->key == DEBUG_UI_KEY_DOWN) move_focus(model, 1, 4U);
    if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS) {
        if (model->focus < 2U) {
            model->gain_editing = 1U; model->gain_digit = model->gain_left_down = 0U;
            model->gain_edit_value = model->focus == 0U ? model->single_static_kp : model->single_static_kd_centi;
        } else if (model->focus == 2U) {
            model->single_static_kp = DEBUG_UI_STATIC_KP_DEFAULT;
            model->single_static_kd_centi = DEBUG_UI_STATIC_KD_CENTI_DEFAULT;
            model->static_locked = model->review_pressed = model->review_released = 0U;
            model->confirm_elapsed_ms = 0U;
        } else navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        model->center_consumed = 1U;
    }
}
