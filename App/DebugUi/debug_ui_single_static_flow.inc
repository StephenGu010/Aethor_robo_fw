/** @file debug_ui_single_static_flow.inc
 * @brief Guided single-axis LCD lifecycle, included by the allocation-free model.
 * Only immutable mailbox requests leave this unit; configuration never drives a motor.
 */
/** @brief Classify lifecycle operations without broadening motion-operation helpers. */
static uint8_t single_lifecycle_operation(DebugUiOperation operation)
{
    return (uint8_t)(operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ||
        operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK || operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT);
}
/** @brief A local draft or navigation must never bypass pending cleanup evidence. */
static uint8_t single_flow_busy(const DebugUiModel *model)
{
    return (uint8_t)(model->snapshot.active || model->snapshot.pending || model->snapshot.stop_pending ||
        model->snapshot.unconfirmed_disable_mask || model->awaiting_result || model->outgoing_ready ||
        model->stop_ready || model->stop_waiting || model->stop_requested || model->single_static_wait_snapshot ||
        model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING ||
        model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED);
}
/** @brief Detect actual LCD-owned RAM installation before permitting locked-build browsing exits. */
static uint8_t single_any_lcd_profile(const DebugUiModel *model)
{
    unsigned index;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if (model->snapshot.motors[index].single_static_lcd_owned) return 1U;
    return 0U;
}
#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Fault withdrawal belongs only to this idle guided session's unchanged selected LCD installation. */
static uint8_t single_fault_relock_owned(const DebugUiModel *model, const DebugUiMotorView *motor)
{
    unsigned index;
    if (!model->single_static_flow || !model->single_static_owned_reference || !model->single_static_owned_evidence ||
        single_flow_busy(model) ||
        motor->motor_id != model->selected_motor + 1U || model->snapshot.target_motor_id != motor->motor_id ||
        !motor->single_static_lcd_owned || !motor->single_static_profile.configured ||
        model->single_static_owned_reference != motor->reference_generation) return 0U;
    /* STOP invalidates HOLD evidence, while the unchanged installation reference still scopes withdrawal.
     * The new withdrawal request freezes current evidence; no old HOLD/configuration is renewed. */
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if (index + 1U != motor->motor_id && model->snapshot.motors[index].single_static_lcd_owned) return 0U;
    return 1U;
}
#endif
/** @brief Navigation and its visible LEFT hint share exact no-ownership, no-preflight and no-cleanup guards. */
uint8_t debug_ui_model_single_static_can_cancel_handoff(const DebugUiModel *model)
{
    return (uint8_t)(model && model->single_static_flow && model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK &&
        !model->single_static_configured && !model->single_static_preflight &&
        !single_any_lcd_profile(model) && !single_flow_busy(model));
}
/** @brief Freeze one configuration from the reviewed ID, seconds and discovery evidence. */
static void single_config_review(DebugUiModel *model)
{
    model->reason = single_intent_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE);
    if (model->reason != DEBUG_UI_REASON_NONE) { notice(model); return; }
    memset(&model->reviewed, 0, sizeof(model->reviewed));
    model->reviewed.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE;
    model->reviewed.target_motor_id = selected(model)->motor_id;
    model->reviewed.reference_generation = selected(model)->reference_generation;
    model->reviewed.single_static_evidence_generation = selected(model)->single_static_evidence_generation;
    model->reviewed.hold_duration_ms = (uint32_t)model->single_static_seconds * 1000U;
    model->reviewed.torque_ff_nm = debug_ui_model_single_static_feedforward(model);
    if (model->selected_motor == 6U) {
        model->reviewed.kp = (float)model->single_static_kp;
        model->reviewed.kd = (float)model->single_static_kd_centi / 100.0f;
    }
    model->edit_epoch = model->snapshot.epoch;
    model->edit_reference = selected(model)->reference_generation;
    model->review_pressed = model->review_released = 0U;
    model->confirm_elapsed_ms = 0U;
    model->single_static_config_cancelled = 0U;
    model->center_consumed = 1U;
    navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW);
}
/** @brief Publish one immutable lifecycle request and keep its own synchronization page. */
static void single_submit_lifecycle(DebugUiModel *model, const DebugUiRequest *request)
{
    model->submitted = *request;
    model->single_static_lifecycle_cancelled = 0U;
    identify(model, &model->submitted);
    model->outgoing = model->submitted;
    model->outgoing_ready = model->awaiting_result = 1U;
    model->admission_seen = model->completion_seen = 0U;
    model->single_static_snapshot_generation = model->snapshot.generation;
    model->center_consumed = 1U;
    model->review_pressed = model->review_released = 0U;
}
/** @brief Freeze one explicit phase and publish only its separately named selected DISABLE preflight. */
static void single_preflight_begin(DebugUiModel *model, const DebugUiRequest *continuation, uint8_t phase)
{
    DebugUiRequest request = *continuation;
    model->reason = single_intent_gate(model, phase == 3U ? DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD : continuation->operation);
    if (model->reason != DEBUG_UI_REASON_NONE) { notice(model); return; }
    model->single_static_continuation = *continuation;
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT;
    request.single_static_preflight_purpose = phase == 2U ? DEBUG_UI_SINGLE_STATIC_PREFLIGHT_LOCK :
        DEBUG_UI_SINGLE_STATIC_PREFLIGHT_NONE;
    request.requested_mode = DEBUG_UI_MODE_MIT;
    request.single_static_evidence_generation = model->snapshot.motors[request.target_motor_id - 1U].single_static_evidence_generation;
    model->single_static_preflight = phase;
    single_submit_lifecycle(model, &request);
    navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT);
}
/** @brief Request application-owned relocking before exit, editing or reacquiring REMOTE. */
static void single_relock(DebugUiModel *model, DebugUiPage destination)
{
    DebugUiRequest request;
    const DebugUiMotorView *relock_motor;
    uint8_t relock_motor_id = selected(model)->motor_id;
    if (model->snapshot.authority != DEBUG_UI_AUTHORITY_REMOTE && model->snapshot.target_motor_id > 0U &&
        model->snapshot.target_motor_id <= DEBUG_UI_MOTOR_COUNT) relock_motor_id = model->snapshot.target_motor_id;
    model->single_static_relock_motor_id = relock_motor_id;
    relock_motor = &model->snapshot.motors[relock_motor_id > 0U && relock_motor_id <= DEBUG_UI_MOTOR_COUNT ? relock_motor_id - 1U : 0U];
    if (single_flow_busy(model)) {
        request_stop_with_recovery_target(model, selected(model)->motor_id);
        return;
    }
    model->single_static_relock_destination = destination;
    if (destination != DEBUG_UI_PAGE_SINGLE_STATIC_SETUP) model->single_static_lock_then_configure = 0U;
    model->single_static_lock_owned = relock_motor->single_static_lcd_owned;
    model->static_locked = 0U;
    model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
    memset(&model->draft, 0, sizeof(model->draft));
    memset(&model->reviewed, 0, sizeof(model->reviewed));
    if (!model->single_static_configured && !single_any_lcd_profile(model) &&
        !model->single_static_lock_then_configure) {
        /* Pure browsing owns no configuration or authority, including an old relock/result page. */
        model->single_static_configured = model->single_static_lock_then_configure = 0U;
        model->selected_motor = model->single_static_pending_motor;
        model->single_static_seconds = model->single_static_pending_seconds;
        if (destination == DEBUG_UI_PAGE_STATIC_HUB) model->single_static_flow = 0U;
        navigate(model, destination);
        return;
    }
    model->reason = single_intent_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
    if (model->reason != DEBUG_UI_REASON_NONE) { navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK); return; }
    if (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && model->page != DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK) {
        /* Fault exit visibly waits for a separate CENTER edge before the one selected DISABLE. */
        navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        return;
    }
    memset(&request, 0, sizeof(request));
    request.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK;
    request.target_motor_id = relock_motor_id;
    request.reference_generation = relock_motor->reference_generation;
    request.single_static_evidence_generation = relock_motor->single_static_evidence_generation;
    if (!model->single_static_configured && !single_any_lcd_profile(model)) {
        /* An explicit ordinary handoff is RAM-only; never DISABLE a different prior local target. */
        model->reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK);
        if (model->reason == DEBUG_UI_REASON_NONE) single_submit_lifecycle(model, &request);
        navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
    } else if (relock_motor_id == selected(model)->motor_id) single_preflight_begin(model, &request, 2U);
    else { model->reason = DEBUG_UI_REASON_NOT_ARMED; navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK); }
}
/** @brief Revoke pose before applying an ID/time edit; installed profiles must first be relocked. */
static void single_edit_setup(DebugUiModel *model, int direction)
{
    uint8_t motor = model->selected_motor, seconds = model->single_static_seconds;
    if (model->focus == 0U) {
        if (direction < 0 && motor > 0U) --motor;
        if (direction > 0 && motor + 1U < DEBUG_UI_MOTOR_COUNT) ++motor;
    } else if (model->focus == 1U) {
        if (direction < 0 && seconds > 1U) --seconds;
        if (direction > 0 && seconds < 60U) ++seconds;
    }
    if (motor == model->selected_motor && seconds == model->single_static_seconds) return;
    if (single_flow_busy(model)) { request_stop_with_recovery_target(model, selected(model)->motor_id); return; }
    if (motor != model->selected_motor)
        model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
    model->single_static_pending_motor = motor;
    model->single_static_pending_seconds = seconds;
    revoke_static_selection(model);
    if (model->single_static_configured || selected(model)->single_static_lcd_owned ||
        model->snapshot.authority != DEBUG_UI_AUTHORITY_REMOTE) {
        single_relock(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
    } else {
        model->selected_motor = motor;
        model->single_static_seconds = seconds;
    }
}
/** @brief Match the backend's monotonic nonzero evidence increment after installation. */
static uint32_t single_next_evidence(uint32_t generation)
{
    ++generation;
    return generation == 0U ? 1U : generation;
}
/** @brief Bind completion plus a newer coherent snapshot before changing visible ownership. */
static void single_flow_update(DebugUiModel *model)
{
    const DebugUiMotorView *motor = selected(model);
    const DebugUiMotorView *relock_motor = &model->snapshot.motors[
        model->submitted.target_motor_id > 0U && model->submitted.target_motor_id <= DEBUG_UI_MOTOR_COUNT ?
        model->submitted.target_motor_id - 1U : (model->selected_motor < DEBUG_UI_MOTOR_COUNT ? model->selected_motor : 0U)];
    if (!model->single_static_flow) return;
    if (model->single_static_preflight) {
        uint8_t phase = model->single_static_preflight;
        const DebugUiMotorView *prepared = &model->snapshot.motors[model->submitted.target_motor_id - 1U];
        if (model->stop_requested || model->stop_waiting || model->single_static_lifecycle_cancelled) {
            model->single_static_preflight = 0U;
            return;
        }
        model->page = DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT;
        if (model->awaiting_result || !model->completion_seen || !model->admission_seen ||
            model->snapshot.active || model->snapshot.pending || model->snapshot.stop_pending ||
            model->snapshot.retained_result_count || model->snapshot.generation == model->single_static_snapshot_generation ||
            model->snapshot.timestamp_us < model->completion.timestamp_us) return;
        model->single_static_preflight = 0U;
        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
        if (model->snapshot.epoch == model->submitted.identity.epoch &&
            prepared->motor_id == model->submitted.target_motor_id &&
            prepared->reference_generation == model->submitted.reference_generation &&
            prepared->single_static_evidence_generation == model->submitted.single_static_evidence_generation &&
            prepared->feedback_timestamp_us >= model->completion.feedback_timestamp_us &&
            model->completion.feedback_timestamp_us > model->submitted.created_at_us &&
            (phase == 2U || prepared->motor_id == selected(model)->motor_id)) {
            DebugUiOperation operation = phase == 3U ? DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD : model->single_static_continuation.operation;
            model->reason = debug_ui_model_gate(model, operation);
            if (model->reason == DEBUG_UI_REASON_NONE && phase == 4U)
                model->reason = static_request_valid(model, &model->single_static_continuation);
            if (model->reason == DEBUG_UI_REASON_NONE) {
                if (phase == 3U) {
                    if (debug_ui_model_begin(model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT))
                        navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_READY);
                } else {
                    single_submit_lifecycle(model, &model->single_static_continuation);
                    model->center_consumed = model->center_down;
                    if (phase == 4U) {
                        (void)debug_ui_load_trials_submit(&model->load_trials, &model->submitted);
                        navigate(model, DEBUG_UI_PAGE_RUNNING);
                    } else navigate(model, phase == 1U ? DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW : DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
                }
                return;
            }
        }
        model->static_locked = 0U;
        if (phase == 2U) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
        else { notice(model); model->notice_return_page = phase == 1U ? DEBUG_UI_PAGE_SINGLE_STATIC_SETUP : DEBUG_UI_PAGE_SINGLE_STATIC_READY; }
        return;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW && !model->awaiting_result &&
        model->single_static_wait_snapshot == 0U && !model->single_static_config_cancelled &&
        (model->edit_epoch != model->snapshot.epoch || model->reviewed.target_motor_id != motor->motor_id ||
         model->reviewed.reference_generation != motor->reference_generation ||
         model->reviewed.single_static_evidence_generation != motor->single_static_evidence_generation)) {
        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
        model->review_pressed = model->review_released = 0U;
        model->confirm_elapsed_ms = 0U;
        notice(model); model->notice_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
    }
    if (model->single_static_wait_snapshot && model->completion_seen &&
        !model->stop_requested && !model->stop_waiting && !model->stop_ready &&
        model->stop_latch_state != DEBUG_UI_STOP_LATCH_WAITING &&
        model->snapshot.generation != model->single_static_snapshot_generation &&
        model->snapshot.timestamp_us >= model->completion.timestamp_us &&
        model->now_us >= model->snapshot.timestamp_us &&
        model->now_us - model->snapshot.timestamp_us < DEBUG_UI_HEALTH_TIMEOUT_US) {
        if (model->single_static_wait_snapshot == 1U && !model->single_static_config_cancelled &&
            model->completion.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE &&
            model->snapshot.epoch == model->submitted.identity.epoch &&
            motor->motor_id == model->submitted.target_motor_id &&
            motor->single_static_lcd_owned && motor->single_static_profile.configured &&
            motor->single_static_profile.minimum_ms == DEBUG_UI_SINGLE_STATIC_MIN_MS &&
            motor->single_static_profile.maximum_ms == model->submitted.hold_duration_ms &&
            motor->single_static_profile.torque_ff_nm == model->submitted.torque_ff_nm &&
            motor->single_static_evidence_generation == single_next_evidence(model->submitted.single_static_evidence_generation) &&
            motor->reference_generation == model->submitted.reference_generation + 1U && motor->reference_generation != 0U) {
            model->single_static_wait_snapshot = 0U;
            model->single_static_configured = 1U;
            model->single_static_owned_reference = motor->reference_generation;
            model->single_static_owned_evidence = motor->single_static_evidence_generation;
            model->reason = DEBUG_UI_REASON_NONE;
            navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        } else if (model->single_static_wait_snapshot == 2U &&
            model->completion.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK &&
            relock_motor->motor_id == model->submitted.target_motor_id &&
            !relock_motor->single_static_lcd_owned && (!model->single_static_lock_owned || !relock_motor->single_static_profile.configured) &&
            model->snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE &&
            !model->snapshot.active && !model->snapshot.pending && !model->snapshot.stop_pending &&
            !model->snapshot.unconfirmed_disable_mask) {
            DebugUiPage destination = model->single_static_relock_destination;
            model->single_static_wait_snapshot = model->single_static_configured = 0U;
            model->single_static_owned_reference = model->single_static_owned_evidence = 0U;
            model->selected_motor = model->single_static_pending_motor;
            model->single_static_seconds = model->single_static_pending_seconds;
            model->reason = DEBUG_UI_REASON_NONE;
            if (destination == DEBUG_UI_PAGE_STATIC_HUB) model->single_static_flow = 0U;
            navigate(model, destination);
            if (model->single_static_flow && destination == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP &&
                model->single_static_lock_then_configure) {
                model->single_static_lock_then_configure = 0U;
                single_config_review(model);
            }
        } else if (model->single_static_wait_snapshot == 3U &&
            model->completion.operation == DEBUG_UI_OPERATION_ACQUIRE &&
            model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
            model->snapshot.target_motor_id == motor->motor_id) {
            model->single_static_wait_snapshot = 0U;
            navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_READY);
        }
    }
    if (model->single_static_configured && !model->awaiting_result && !model->single_static_wait_snapshot &&
        !model->snapshot.active && !model->snapshot.pending && !model->snapshot.stop_pending &&
        (!motor->single_static_profile.configured || !motor->single_static_lcd_owned ||
         model->single_static_owned_reference != motor->reference_generation ||
         model->single_static_owned_evidence != motor->single_static_evidence_generation)) {
        model->single_static_configured = model->static_locked = 0U;
        model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
        if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_READY || model->page == DEBUG_UI_PAGE_REVIEW) {
            notice(model); model->notice_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP;
        }
    }
}
/** @brief Edit a signed decimal digit only while unconfigured, idle and remotely owned. */
static void single_edit_feedforward(DebugUiModel *model, const DebugUiInputEvent *event)
{
    static const uint16_t increments[4] = {0U, 100U, 10U, 1U};
    int direction = event->key == DEBUG_UI_KEY_UP ? 1 : (event->key == DEBUG_UI_KEY_DOWN ? -1 : 0);
    if (single_flow_busy(model) || model->single_static_configured || selected(model)->single_static_lcd_owned ||
        model->snapshot.authority != DEBUG_UI_AUTHORITY_REMOTE) return;
    if (event->key == DEBUG_UI_KEY_LEFT) {
        if (event->type == DEBUG_UI_INPUT_EVENT_PRESS) model->single_static_ff_left_down = 1U;
        else if (model->single_static_ff_left_down && event->held_ms >= 800U) {
            model->single_static_ff_centi = model->single_static_ff_saved_centi;
            model->single_static_ff_negative = model->single_static_ff_saved_negative;
            model->single_static_ff_editing = 0U;
            model->single_static_ff_left_down = event->type == DEBUG_UI_INPUT_EVENT_RELEASE ? 0U : 2U;
        } else if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE && model->single_static_ff_left_down) {
            model->single_static_ff_left_down = 0U;
            model->single_static_ff_digit = model->single_static_ff_digit ? model->single_static_ff_digit - 1U : 3U;
        }
        return;
    }
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return;
    if (event->key == DEBUG_UI_KEY_RIGHT && event->type == DEBUG_UI_INPUT_EVENT_PRESS)
        model->single_static_ff_digit = (uint8_t)((model->single_static_ff_digit + 1U) % 4U);
    if (direction) {
        if (!model->single_static_ff_digit) model->single_static_ff_negative ^= 1U;
        else {
            uint16_t place = increments[model->single_static_ff_digit];
            uint16_t digit = (uint16_t)((model->single_static_ff_centi / place) % 10U);
            uint16_t remaining = (uint16_t)(model->single_static_ff_centi - digit * place);
            unsigned attempt;
            for (attempt = 0U; attempt < 10U; ++attempt) {
                uint16_t candidate;
                digit = (uint16_t)((digit + (direction > 0 ? 1U : 9U)) % 10U);
                candidate = (uint16_t)(remaining + digit * place);
                if (debug_ui_single_static_feedforward_valid((float)candidate * 0.01f,
                    selected(model)->ranges_verified ? selected(model)->torque_max_nm : 0.0f)) {
                    model->single_static_ff_centi = candidate;
                    break;
                }
            }
        }
    }
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS) {
        model->single_static_ff_editing = 0U;
        model->center_consumed = 1U;
    }
}
#include "debug_ui_static_gain_pages.inc"
/** @brief Route only guided pages, preserving the existing seven-axis and ordinary editors. */
static uint8_t single_flow_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    int direction;
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_GAINS) { single_gain_event(model, event); return 1U; }
    if (model->single_static_ff_left_down == 2U && event->key == DEBUG_UI_KEY_LEFT) {
        if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE) model->single_static_ff_left_down = 0U;
        return 1U;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP && model->single_static_ff_editing) {
        single_edit_feedforward(model, event);
        return 1U;
    }
    /* Compatibility pages cannot become an alternate exit or ID selector for an owned guided session. */
    if (model->single_static_flow && (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_LOAD ||
        model->page == DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS || model->page == DEBUG_UI_PAGE_SINGLE_STATIC_IDS)) {
        model->static_locked = 0U;
        model->review_pressed = model->review_released = 0U;
        model->confirm_elapsed_ms = 0U;
        if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && event->key == DEBUG_UI_KEY_LEFT)
            single_relock(model, DEBUG_UI_PAGE_STATIC_HUB);
        else navigate(model, model->single_static_configured ? DEBUG_UI_PAGE_SINGLE_STATIC_READY : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        return 1U;
    }
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW) {
        if (model->awaiting_result || model->single_static_wait_snapshot) return 1U;
        if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && event->key == DEBUG_UI_KEY_LEFT) {
            model->review_pressed = model->review_released = 0U;
            model->confirm_elapsed_ms = 0U;
            model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
            navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        } else if (event->key == DEBUG_UI_KEY_CENTER && !model->center_consumed) {
            if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && model->review_released) {
                model->review_pressed = 1U; model->review_pressed_ms = event->timestamp_ms;
            } else if (event->type == DEBUG_UI_INPUT_EVENT_HOLD && model->review_pressed && model->center_down) {
                uint32_t elapsed = event->timestamp_ms - model->review_pressed_ms;
                model->confirm_elapsed_ms = elapsed < event->held_ms ? elapsed : event->held_ms;
                if (model->confirm_elapsed_ms >= DEBUG_UI_REVIEW_HOLD_MS) {
                    model->reason = single_intent_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE);
                    if (model->reason == DEBUG_UI_REASON_NONE &&
                        (model->edit_epoch != model->snapshot.epoch || model->reviewed.target_motor_id != selected(model)->motor_id ||
                         model->reviewed.reference_generation != selected(model)->reference_generation ||
                         model->reviewed.single_static_evidence_generation != selected(model)->single_static_evidence_generation))
                        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
                    if (model->reason == DEBUG_UI_REASON_NONE) single_preflight_begin(model, &model->reviewed, 1U);
                    else { notice(model); model->notice_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; }
                }
            }
        }
        return 1U;
    }
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return 0U;
    if (event->type == DEBUG_UI_INPUT_EVENT_REPEAT && event->key == DEBUG_UI_KEY_CENTER) return 1U;
    direction = event->key == DEBUG_UI_KEY_LEFT ? -1 : (event->key == DEBUG_UI_KEY_RIGHT ? 1 : 0);
    if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP) {
        if (model->single_static_ff_editing) { single_edit_feedforward(model, event); return 1U; }
        if (event->key == DEBUG_UI_KEY_CENTER && model->focus == 4U) {
            if (single_gain_edit_allowed(model)) { navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_GAINS); model->focus = 0U; }
            else { model->reason = DEBUG_UI_REASON_BUSY; notice(model); model->notice_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; }
            model->center_consumed = 1U;
            return 1U;
        }
        if (event->key == DEBUG_UI_KEY_CENTER && model->focus == 3U) {
            if (model->single_static_configured || selected(model)->single_static_lcd_owned ||
                model->snapshot.authority != DEBUG_UI_AUTHORITY_REMOTE) single_relock(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
            else if (!single_flow_busy(model)) {
                model->single_static_ff_editing = 1U; model->single_static_ff_digit = 0U;
                model->single_static_ff_saved_centi = model->single_static_ff_centi;
                model->single_static_ff_saved_negative = model->single_static_ff_negative;
                model->single_static_ff_left_down = 0U;
            }
            model->center_consumed = 1U;
            return 1U;
        }
        if (event->key == DEBUG_UI_KEY_UP) move_focus(model, -1, 5U);
        if (event->key == DEBUG_UI_KEY_DOWN) move_focus(model, 1, 5U);
        if (direction && model->focus < 2U) single_edit_setup(model, direction);
        if (event->key == DEBUG_UI_KEY_LEFT && model->focus == 2U) single_relock(model, DEBUG_UI_PAGE_STATIC_HUB);
        if (event->key == DEBUG_UI_KEY_CENTER && model->focus == 2U) {
            if (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !model->snapshot.single_static_load_enabled) {
                model->reason = DEBUG_UI_REASON_DISABLED;
                notice(model);
            } else if (model->single_static_configured || selected(model)->single_static_lcd_owned ||
                model->snapshot.authority != DEBUG_UI_AUTHORITY_REMOTE) {
                model->single_static_lock_then_configure = 1U;
                single_relock(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
            } else single_config_review(model);
        }
    } else if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_READY) {
        if (event->key == DEBUG_UI_KEY_LEFT) single_relock(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        if (event->key == DEBUG_UI_KEY_CENTER && !model->single_static_wait_snapshot) {
            if (!model->single_static_configured) { model->reason = DEBUG_UI_REASON_UNCALIBRATED; notice(model); }
            else if (model->snapshot.authority == DEBUG_UI_AUTHORITY_REMOTE)
                (void)debug_ui_model_begin(model, DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_MODE_UNKNOWN);
            else if (!model->static_locked) {
                DebugUiRequest capture;
                memset(&capture, 0, sizeof(capture));
                capture.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
                capture.target_motor_id = selected(model)->motor_id;
                capture.reference_generation = selected(model)->reference_generation;
                single_preflight_begin(model, &capture, 3U);
            } else enter_review(model);
        }
    } else if (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK) {
        if (event->key == DEBUG_UI_KEY_LEFT && debug_ui_model_single_static_can_cancel_handoff(model)) {
            model->single_static_lock_then_configure = 0U;
            single_relock(model, DEBUG_UI_PAGE_STATIC_HUB);
        }
        if (event->key == DEBUG_UI_KEY_RIGHT)
            request_stop_with_recovery_target(model, model->single_static_relock_motor_id);
        if (event->key == DEBUG_UI_KEY_CENTER && !model->awaiting_result && !model->single_static_wait_snapshot)
            single_relock(model, model->single_static_relock_destination);
    } else if (model->single_static_flow && (model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT ||
        model->page == DEBUG_UI_PAGE_RESULT || model->page == DEBUG_UI_PAGE_FAULT)) {
        if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER) {
            single_relock(model, event->key == DEBUG_UI_KEY_LEFT ? DEBUG_UI_PAGE_STATIC_HUB : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        } else if (event->key == DEBUG_UI_KEY_UP) model->static_result_detail = (uint8_t)((model->static_result_detail + 2U) % 3U);
        else if (event->key == DEBUG_UI_KEY_DOWN) model->static_result_detail = (uint8_t)((model->static_result_detail + 1U) % 3U);
    } else return 0U;
    if (event->key == DEBUG_UI_KEY_CENTER) model->center_consumed = 1U;
    return 1U;
}
