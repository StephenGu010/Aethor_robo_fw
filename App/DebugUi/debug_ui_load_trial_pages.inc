/** @file debug_ui_load_trial_pages.inc
 * @brief Display-only metadata editing and RAM-history navigation, included by the model.
 * No request, permission, profile, control parameter or motor-selection mutation is permitted here.
 */

/** @brief Block display-only edits using the guided activity/cleanup gate. */
static uint8_t load_page_editable(const DebugUiModel *model)
{
    return (uint8_t)(model != NULL && model->selected_motor < DEBUG_UI_MOTOR_COUNT && !single_flow_busy(model));
}

/** @brief Saturate one user-entered dimension before passing it through the ledger validator. */
static void load_page_adjust(DebugUiModel *model, uint8_t field, int delta)
{
    uint8_t motor_id = (uint8_t)(model->selected_motor + 1U);
    const DebugUiLoadMetadata *metadata = debug_ui_load_trials_candidate(&model->load_trials, motor_id);
    int value, minimum, maximum;
    if (metadata == NULL || !load_page_editable(model)) return;
    value = field == 0U ? metadata->lever_mm : metadata->mass_g;
    minimum = field == 0U ? (int)DEBUG_UI_LOAD_LEVER_MIN_MM : 0;
    maximum = field == 0U ? (int)DEBUG_UI_LOAD_LEVER_MAX_MM : (int)DEBUG_UI_LOAD_MASS_MAX_G;
    value += delta;
    if (value < minimum) value = minimum;
    if (value > maximum) value = maximum;
    (void)debug_ui_load_trials_set(&model->load_trials, motor_id,
        field == 0U ? (uint16_t)value : metadata->lever_mm,
        field == 1U ? (uint16_t)value : metadata->mass_g);
}

/** @brief Restore a valid list viewport after RAM history is cleared or reduced at a boot boundary. */
static void load_page_history_clamp(DebugUiModel *model)
{
    uint8_t count = model->load_trials.count;
    if (count == 0U) { model->focus = model->list_first = 0U; return; }
    if (model->focus >= count) model->focus = (uint8_t)(count - 1U);
    if (count <= DEBUG_UI_VISIBLE_MOTORS) model->list_first = 0U;
    else if (model->list_first > count - DEBUG_UI_VISIBLE_MOTORS)
        model->list_first = (uint8_t)(count - DEBUG_UI_VISIBLE_MOTORS);
    if (model->focus < model->list_first) model->list_first = model->focus;
    if (model->focus >= model->list_first + DEBUG_UI_VISIBLE_MOTORS)
        model->list_first = (uint8_t)(model->focus + 1U - DEBUG_UI_VISIBLE_MOTORS);
}

/** @brief Handle only display pages after global STOP routing and before guided lifecycle routing. */
static uint8_t load_page_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    int delta;
    uint8_t activation = (uint8_t)(event->key == DEBUG_UI_KEY_CENTER || event->key == DEBUG_UI_KEY_RIGHT);
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return 0U;
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP && model->focus == 2U && event->key == DEBUG_UI_KEY_RIGHT) {
        model->load_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
        navigate(model, DEBUG_UI_PAGE_LOAD_METADATA);
        return 1U;
    }
    if (model->page < DEBUG_UI_PAGE_LOAD_METADATA || model->page > DEBUG_UI_PAGE_LOAD_DETAIL) return 0U;
    if (event->type == DEBUG_UI_INPUT_EVENT_REPEAT && event->key == DEBUG_UI_KEY_CENTER) return 1U;
    if (model->page == DEBUG_UI_PAGE_LOAD_METADATA) {
        if (event->key == DEBUG_UI_KEY_UP) { if (model->focus > 0U) --model->focus; }
        else if (event->key == DEBUG_UI_KEY_DOWN) { if (model->focus < 5U) ++model->focus; }
        else if (model->focus < 2U && load_page_editable(model)) {
            if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_RIGHT)
                load_page_adjust(model, model->focus, event->key == DEBUG_UI_KEY_LEFT ? -10 : 10);
            else if (event->key == DEBUG_UI_KEY_CENTER) {
                model->load_edit_field = model->focus;
                navigate(model, DEBUG_UI_PAGE_LOAD_NUMBER);
            }
        } else if (model->focus == 3U && event->key == DEBUG_UI_KEY_CENTER && load_page_editable(model))
            (void)debug_ui_load_trials_confirm(&model->load_trials, (uint8_t)(model->selected_motor + 1U));
        else if (model->focus == 4U && activation) {
            navigate(model, DEBUG_UI_PAGE_LOAD_HISTORY);
            load_page_history_clamp(model);
        }
        else if (model->focus == 5U && (activation || event->key == DEBUG_UI_KEY_LEFT))
            navigate(model, model->load_return_page == DEBUG_UI_PAGE_POS_LOAD_SETUP ?
                DEBUG_UI_PAGE_POS_LOAD_SETUP : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        return 1U;
    }
    if (model->page == DEBUG_UI_PAGE_LOAD_NUMBER) {
        if (event->key == DEBUG_UI_KEY_CENTER) navigate(model, DEBUG_UI_PAGE_LOAD_METADATA);
        else if (load_page_editable(model)) {
            delta = event->key == DEBUG_UI_KEY_UP ? 1 : (event->key == DEBUG_UI_KEY_DOWN ? -1 :
                (event->key == DEBUG_UI_KEY_LEFT ? -10 : (event->key == DEBUG_UI_KEY_RIGHT ? 10 : 0)));
            if (delta != 0) load_page_adjust(model, model->load_edit_field, delta);
        }
        return 1U;
    }
    if (model->page == DEBUG_UI_PAGE_LOAD_HISTORY) {
        load_page_history_clamp(model);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_LOAD_METADATA);
        else if (event->key == DEBUG_UI_KEY_UP || event->key == DEBUG_UI_KEY_DOWN)
            move_focus(model, event->key == DEBUG_UI_KEY_UP ? -1 : 1, model->load_trials.count);
        else if (activation && model->load_trials.count != 0U && model->focus < model->load_trials.count) {
            model->load_history_offset = model->focus;
            model->page_focus[DEBUG_UI_PAGE_LOAD_DETAIL] = 0U;
            navigate(model, DEBUG_UI_PAGE_LOAD_DETAIL);
        }
        return 1U;
    }
    if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER)
        navigate(model, DEBUG_UI_PAGE_LOAD_HISTORY);
    else if (event->key == DEBUG_UI_KEY_UP || event->key == DEBUG_UI_KEY_DOWN || event->key == DEBUG_UI_KEY_RIGHT)
        model->focus = (uint8_t)(model->focus == 0U ? 1U : 0U);
    return 1U;
}
