/** @file debug_ui_pos_load_pages.inc
 * @brief Independent allocation-free POS load setup and result navigation; included by model.
 */

/** @brief Keep failed cleanup locked while exposing the existing explicit STOP and return navigation. */
static void pos_load_open_cleanup_check(DebugUiModel *model)
{
    uint8_t motor_mask = model->snapshot.unconfirmed_disable_mask;
    uint8_t target;
    if (!motor_mask) motor_mask = model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ?
        model->stop_motor_mask : model->completion.motor_mask;
    target = first_motor(motor_mask);
    if (target) model->selected_motor = (uint8_t)(target - 1U);
    model->pos_load_flow = 1U;
    model->prepare_return_page = DEBUG_UI_PAGE_STATIC_HUB;
    model->reason = DEBUG_UI_REASON_NOT_DISABLED;
    navigate(model, DEBUG_UI_PAGE_PREPARE);
}

/** @brief Build a fresh bounded draft only after prior cleanup is confirmed, without issuing a command. */
static void pos_load_setup(DebugUiModel *model)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
    float default_step = debug_ui_degrees_to_radians(1.0f) * debug_ui_pos_load_scale(motor);
    if (debug_ui_model_result_requires_check(model)) {
        pos_load_open_cleanup_check(model);
        return;
    }
    memset(&model->draft, 0, sizeof(model->draft));
    model->single_static_flow = 0U;
    model->pos_load_flow = 1U;
    model->pos_load_result_detail = 0U;
    model->pos_load_check_pending = 0U;
    model->draft.operation = DEBUG_UI_OPERATION_POS_LOAD_TEST;
    model->draft.requested_mode = DEBUG_UI_MODE_POS_VEL;
    model->draft.target_motor_id = motor->motor_id;
    model->draft.reference_generation = motor->reference_generation;
    model->draft.delta_rad = finite_value(bounds.max_delta_rad) && bounds.max_delta_rad > 0.0f ?
        bounded(default_step, 0.0f, debug_ui_pos_load_edit_limit(motor)) : 0.0f;
    model->draft.speed_rad_s = finite_value(bounds.max_speed_rad_s) && bounds.max_speed_rad_s > 0.0f ?
        bounded(default_step, 0.0f, bounds.max_speed_rad_s) : 0.0f;
    model->draft.hold_duration_ms = 1000U;
    model->edit_reference = motor->reference_generation;
    model->edit_epoch = model->snapshot.epoch;
    model->reason = DEBUG_UI_REASON_NONE;
    navigate(model, DEBUG_UI_PAGE_POS_LOAD_SETUP);
    model->focus = model->list_first = 0U;
}

/** @brief Apply one bounded coarse/fine edit; running and cleanup cannot mutate any parameter. */
static void pos_load_adjust(DebugUiModel *model, uint8_t field, int direction, uint8_t fine)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
    float angle_limit = debug_ui_pos_load_edit_limit(motor);
    float minimum_speed = debug_ui_degrees_to_radians(0.1f) * debug_ui_pos_load_scale(motor);
    float step = debug_ui_degrees_to_radians(fine ? 0.1f : 1.0f) *
        debug_ui_pos_load_scale(motor) * (float)direction;
    if (single_flow_busy(model)) return;
    if (field == 0U) {
        uint8_t previous = model->selected_motor;
        if (direction < 0 && model->selected_motor > 0U) --model->selected_motor;
        if (direction > 0 && model->selected_motor + 1U < DEBUG_UI_MOTOR_COUNT) ++model->selected_motor;
        if (previous != model->selected_motor) { pos_load_setup(model); model->focus = 0U; }
    } else if (field == 1U && finite_value(angle_limit) && angle_limit > 0.0f)
        model->draft.delta_rad = bounded(model->draft.delta_rad + step, -angle_limit, angle_limit);
    else if (field == 2U && finite_value(bounds.max_speed_rad_s) && bounds.max_speed_rad_s > 0.0f)
        model->draft.speed_rad_s = bounded(model->draft.speed_rad_s + step,
            bounds.max_speed_rad_s < minimum_speed ? bounds.max_speed_rad_s : minimum_speed, bounds.max_speed_rad_s);
    else if (field == 3U) {
        int32_t duration = (int32_t)model->draft.hold_duration_ms + direction * 1000;
        if (duration < 1000) duration = 1000;
        if (duration > 60000) duration = 60000;
        model->draft.hold_duration_ms = (uint32_t)duration;
    }
}

/** @brief Return legal display-unit magnitude, rounded inward except float conversion noise. */
static uint16_t pos_digit_maximum(const DebugUiModel *model)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
    float maximum;
    if (model->edit_field == 3U) return 60U;
    maximum = model->edit_field == 1U ? debug_ui_pos_load_edit_limit(motor) : bounds.max_speed_rad_s;
    if (model->edit_field == 1U && bounds.max_delta_rad < maximum) maximum = bounds.max_delta_rad;
    maximum = debug_ui_radians_to_degrees(debug_ui_pos_load_display(motor, maximum)) * 10.0f;
    if (!finite_value(maximum) || maximum <= 0.0f) return 0U;
    if (maximum > 9999.0f) maximum = 9999.0f;
    return (uint16_t)(maximum + 0.001f);
}

/** @brief Publish only a legal integer edit into the existing bounded request draft. */
static void pos_digit_publish(DebugUiModel *model)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_pos_load_bounds(motor);
    float value;
    if (model->pos_digit_value > pos_digit_maximum(model) ||
        (model->edit_field != 1U && model->pos_digit_value == 0U)) return;
    if (model->edit_field == 3U) {
        model->draft.hold_duration_ms = (uint32_t)model->pos_digit_value * 1000U;
        return;
    }
    value = debug_ui_degrees_to_radians((float)model->pos_digit_value * 0.1f) * debug_ui_pos_load_scale(motor);
    if (model->edit_field == 1U) {
        float maximum = debug_ui_pos_load_edit_limit(motor);
        if (bounds.max_delta_rad < maximum) maximum = bounds.max_delta_rad;
        value = bounded(value, 0.0f, maximum);
        model->draft.delta_rad = model->pos_digit_negative ? -value : value;
    } else model->draft.speed_rad_s = bounded(value, 0.0f, bounds.max_speed_rad_s);
}

/** @brief Freeze cancel backup and initialize allocation-free decimal state in display units. */
static void pos_digit_begin(DebugUiModel *model)
{
    float value = model->edit_field == 3U ? (float)(model->draft.hold_duration_ms / 1000U) :
        debug_ui_radians_to_degrees(debug_ui_pos_load_display(selected(model),
            model->edit_field == 1U ? model->draft.delta_rad : model->draft.speed_rad_s)) * 10.0f;
    uint16_t maximum = pos_digit_maximum(model);
    model->number_backup = model->draft;
    model->pos_digit_negative = (uint8_t)(model->edit_field == 1U && value < 0.0f);
    if (value < 0.0f) value = -value;
    if (!finite_value(value)) value = 0.0f;
    if (value > (float)maximum) value = (float)maximum;
    model->pos_digit_value = (uint16_t)(value + 0.5f);
    model->pos_digit_cursor = 3U;
    model->pos_digit_left_down = 0U;
    pos_digit_publish(model);
}

/** @brief Cycle only the selected digit; skip illegal candidates without carrying into other places. */
static void pos_digit_adjust(DebugUiModel *model, int direction)
{
    static const uint16_t decimal_places[] = {0U, 1000U, 100U, 10U, 1U};
    uint16_t place, digit, remaining, maximum = pos_digit_maximum(model);
    unsigned attempt;
    if (model->pos_digit_cursor == 0U) {
        if (model->edit_field == 1U) model->pos_digit_negative = (uint8_t)!model->pos_digit_negative;
        pos_digit_publish(model);
        return;
    }
    if (model->pos_digit_cursor > 4U) return;
    place = model->edit_field == 3U ? (model->pos_digit_cursor == 2U ? 10U : 1U) :
        decimal_places[model->pos_digit_cursor];
    digit = (uint16_t)((model->pos_digit_value / place) % 10U);
    remaining = (uint16_t)(model->pos_digit_value - digit * place);
    for (attempt = 0U; attempt < 10U; ++attempt) {
        uint16_t candidate;
        digit = (uint16_t)((digit + (direction > 0 ? 1U : 9U)) % 10U);
        candidate = (uint16_t)(remaining + digit * place);
        if (candidate <= maximum && (model->edit_field == 1U || candidate > 0U)) {
            model->pos_digit_value = candidate;
            pos_digit_publish(model);
            break;
        }
    }
}

/** @brief Select a legal place without changing the number or moving any motor. */
static void pos_digit_move(DebugUiModel *model, int direction)
{
    uint8_t first = model->edit_field == 1U ? 0U : (model->edit_field == 3U ? 2U : 1U);
    uint8_t last = model->edit_field == 3U ? 3U : 4U;
    if (direction > 0) model->pos_digit_cursor = model->pos_digit_cursor >= last ? first : (uint8_t)(model->pos_digit_cursor + 1U);
    else model->pos_digit_cursor = model->pos_digit_cursor <= first ? last : (uint8_t)(model->pos_digit_cursor - 1U);
}

/** @brief Route setup, numeric edits and retained results after global STOP has priority. */
static uint8_t pos_load_page_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    int direction;
    if (model->pos_digit_left_down == 2U && event->key == DEBUG_UI_KEY_LEFT) {
        if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE) model->pos_digit_left_down = 0U;
        return 1U;
    }
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT &&
        !(model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER && event->key == DEBUG_UI_KEY_LEFT &&
          (event->type == DEBUG_UI_INPUT_EVENT_RELEASE || event->type == DEBUG_UI_INPUT_EVENT_HOLD))) return 0U;
    if (model->page != DEBUG_UI_PAGE_POS_LOAD_SETUP && model->page != DEBUG_UI_PAGE_POS_LOAD_NUMBER &&
        model->page != DEBUG_UI_PAGE_POS_LOAD_RESULT) return 0U;
    if (event->type == DEBUG_UI_INPUT_EVENT_REPEAT && event->key == DEBUG_UI_KEY_CENTER) return 1U;
    if (model->page == DEBUG_UI_PAGE_POS_LOAD_RESULT) {
        if (event->key == DEBUG_UI_KEY_UP || event->key == DEBUG_UI_KEY_DOWN || event->key == DEBUG_UI_KEY_RIGHT)
            model->pos_load_result_detail = (uint8_t)!model->pos_load_result_detail;
        else if (event->key == DEBUG_UI_KEY_CENTER || event->key == DEBUG_UI_KEY_LEFT) {
            if (debug_ui_model_result_requires_check(model)) {
                model->prepare_return_page = DEBUG_UI_PAGE_STATIC_HUB;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
            } else { model->completion_seen = 0U; model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE; pos_load_setup(model); }
        }
        return 1U;
    }
    if (debug_ui_model_result_requires_check(model)) {
        pos_load_open_cleanup_check(model);
        return 1U;
    }
    if (single_flow_busy(model)) return 1U;
    if (model->page == DEBUG_UI_PAGE_POS_LOAD_NUMBER) {
        if (event->key == DEBUG_UI_KEY_CENTER) navigate(model, DEBUG_UI_PAGE_POS_LOAD_SETUP);
        else if (event->key == DEBUG_UI_KEY_LEFT) {
            if (event->type == DEBUG_UI_INPUT_EVENT_PRESS) model->pos_digit_left_down = 1U;
            else if (model->pos_digit_left_down && event->held_ms >= 800U) {
                model->draft = model->number_backup;
                model->pos_digit_left_down = event->type == DEBUG_UI_INPUT_EVENT_RELEASE ? 0U : 2U;
                navigate(model, DEBUG_UI_PAGE_POS_LOAD_SETUP);
            } else if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE && model->pos_digit_left_down) {
                model->pos_digit_left_down = 0U;
                pos_digit_move(model, -1);
            }
        }
        else if (event->key == DEBUG_UI_KEY_RIGHT && event->type == DEBUG_UI_INPUT_EVENT_PRESS) pos_digit_move(model, 1);
        else if (event->key == DEBUG_UI_KEY_UP || event->key == DEBUG_UI_KEY_DOWN)
            pos_digit_adjust(model, event->key == DEBUG_UI_KEY_UP ? 1 : -1);
        return 1U;
    }
    direction = event->key == DEBUG_UI_KEY_LEFT ? -1 : (event->key == DEBUG_UI_KEY_RIGHT ? 1 : 0);
    if (event->key == DEBUG_UI_KEY_UP) { if (model->focus > 0U) --model->focus; }
    else if (event->key == DEBUG_UI_KEY_DOWN) { if (model->focus < 6U) ++model->focus; }
    else if (model->focus < 4U) {
        if (direction) pos_load_adjust(model, model->focus, direction, 0U);
        else if (event->key == DEBUG_UI_KEY_CENTER && model->focus > 0U) {
            model->edit_field = model->focus;
            pos_digit_begin(model);
            navigate(model, DEBUG_UI_PAGE_POS_LOAD_NUMBER);
        }
    } else if (model->focus == 4U && (event->key == DEBUG_UI_KEY_CENTER || event->key == DEBUG_UI_KEY_RIGHT)) {
        model->load_return_page = DEBUG_UI_PAGE_POS_LOAD_SETUP;
        navigate(model, DEBUG_UI_PAGE_LOAD_METADATA);
    } else if (model->focus == 5U && event->key == DEBUG_UI_KEY_CENTER) {
        model->edit_reference = selected(model)->reference_generation;
        model->edit_epoch = model->snapshot.epoch;
        model->draft.reference_generation = model->edit_reference;
        enter_review(model);
    } else if (model->focus == 6U && (event->key == DEBUG_UI_KEY_RIGHT || event->key == DEBUG_UI_KEY_CENTER)) {
        model->prepare_return_page = DEBUG_UI_PAGE_POS_LOAD_SETUP;
        navigate(model, DEBUG_UI_PAGE_PREPARE);
    } else if (model->focus == 6U && event->key == DEBUG_UI_KEY_LEFT) {
        model->pos_load_flow = 0U;
        navigate(model, DEBUG_UI_PAGE_STATIC_HUB);
    }
    return 1U;
}
