/** @file debug_ui_model.c
 * @brief Pure C UI state machine; no motor, queue, HAL, RTOS or LVGL access.
 * Display previews are advisory. The control owner recomputes the relative
 * target against fresh feedback and revalidates every submitted operation.
 */
#include "debug_ui_model.h"
#include "debug_ui_pos_load_units.h"
#include "debug_ui_single_static_bounds.h"
#include <float.h>
#include <stdio.h>
#include <string.h>
#include "debug_ui_load_trials.inc"

/** @brief Portable finite check for ARMCC5 and host C without fast-math. */
static uint8_t finite_value(float value)
{
    return (uint8_t)((value == value) && value <= FLT_MAX && value >= -FLT_MAX);
}
/** @brief Obtain single static display bounds without mutating ordinary motor profiles. */
static DebugUiMotorProfile single_model_bounds(const DebugUiMotorView *motor)
{
    return debug_ui_single_static_bounds(&motor->profile, motor->motor_id,
        motor->ranges_verified ? motor->position_max_rad : 0.0f, motor->single_static_native_per_output);
}
/** @brief Compare the complete provenance key, never only request_id. */
static uint8_t same_identity(const DebugUiIdentity *left, const DebugUiIdentity *right)
{
    return (uint8_t)(left->request_id != 0U && left->origin == right->origin &&
                    left->epoch == right->epoch && left->request_id == right->request_id);
}
/** @brief Classify the bounded MIT operations. */
static uint8_t is_mit(DebugUiOperation operation)
{
    return (uint8_t)(operation == DEBUG_UI_OPERATION_MIT_HOLD ||
                     operation == DEBUG_UI_OPERATION_MIT_MOVE || debug_ui_is_static_operation(operation));
}
/** @brief Return editable parameter count, excluding preview. */
uint8_t debug_ui_model_parameter_count(const DebugUiModel *model)
{
    if (model == NULL) return 0U;
    if (debug_ui_is_static_operation(model->draft.operation)) return 1U;
    if (model->draft.operation == DEBUG_UI_OPERATION_POS_MOVE) return 2U;
    if (model->draft.operation == DEBUG_UI_OPERATION_MIT_MOVE) return 5U;
    return model->draft.operation == DEBUG_UI_OPERATION_MIT_HOLD ? 3U : 0U;
}
/** @brief Map visible parameter row to the stable request field. */
uint8_t debug_ui_model_parameter_field(const DebugUiModel *model, uint8_t index)
{
    if (index >= debug_ui_model_parameter_count(model)) return 255U;
    if (debug_ui_is_static_operation(model->draft.operation)) return 4U;
    return (uint8_t)(index + (model->draft.operation == DEBUG_UI_OPERATION_MIT_HOLD ? 2U : 0U));
}
/** @brief Share the failed-disable acknowledgement contract with the view. */
uint8_t debug_ui_model_result_requires_check(const DebugUiModel *model)
{
    if (model == NULL) return 0U;
    return (uint8_t)(model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ||
        model->snapshot.unconfirmed_disable_mask != 0U ||
        (model->completion_seen && !model->completion.disabled_confirmed &&
         (model->completion.operation == DEBUG_UI_OPERATION_STOP ||
          model->completion.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST ||
          model->completion.operation == DEBUG_UI_OPERATION_POS_MOVE || is_mit(model->completion.operation))));
}
/** @brief Restore each page focus and viewport. */
static void navigate(DebugUiModel *model, DebugUiPage page)
{
    model->page_focus[model->page] = model->focus;
    model->page_first[model->page] = model->list_first;
    model->page = page;
    model->focus = model->page_focus[page];
    model->list_first = model->page_first[page];
    if (page == DEBUG_UI_PAGE_MOTORS || page == DEBUG_UI_PAGE_SINGLE_STATIC_IDS) {
        model->focus = model->selected_motor;
        if (model->focus < model->list_first) model->list_first = model->focus;
        if (model->focus >= model->list_first + DEBUG_UI_VISIBLE_MOTORS)
            model->list_first = (uint8_t)(model->focus + 1U - DEBUG_UI_VISIBLE_MOTORS);
    }
}
/** @brief Open bounded read-only tabs while retaining the immediate navigation parent. */
static void open_diagnostics(DebugUiModel *model, DebugUiPage parent)
{
    model->diagnostic_parent_page = parent;
    if (model->diagnostic_page >= 3U) model->diagnostic_page = 0U;
    navigate(model, DEBUG_UI_PAGE_DIAGNOSTICS);
}
/** @brief Clamp selection at list edges and retain five visible rows. */
static void move_focus(DebugUiModel *model, int direction, uint8_t count)
{
    if (direction < 0 && model->focus > 0U) --model->focus;
    if (direction > 0 && model->focus + 1U < count) ++model->focus;
    if (model->focus < model->list_first) model->list_first = model->focus;
    if (model->focus >= model->list_first + DEBUG_UI_VISIBLE_MOTORS)
        model->list_first = (uint8_t)(model->focus + 1U - DEBUG_UI_VISIBLE_MOTORS);
}
/** @brief Preserve the source of a refusal overlay. */
static void notice(DebugUiModel *model)
{
    model->notice_return_page = model->page;
    navigate(model, DEBUG_UI_PAGE_NOTICE);
}
/** @brief A revoked draft must be rebuilt from its stable parent menu. */
static void revoke_draft(DebugUiModel *model)
{
    DebugUiPage parent = model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
        (model->single_static_flow ? (model->single_static_configured ? DEBUG_UI_PAGE_SINGLE_STATIC_READY :
         DEBUG_UI_PAGE_SINGLE_STATIC_SETUP) : DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS) :
        (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? DEBUG_UI_PAGE_STATIC_ACTIONS :
        (debug_ui_model_parameter_count(model) != 0U ? DEBUG_UI_PAGE_ACTIONS : model->review_return_page));
    if (model->draft.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST) parent = DEBUG_UI_PAGE_POS_LOAD_SETUP;
    if (model->pos_load_check_pending == 1U && model->draft.operation == DEBUG_UI_OPERATION_ACQUIRE) {
        /* An invalidated ownership review must never turn the POS editor into an ACQUIRE editor. */
        model->draft = model->pos_load_saved_draft;
        model->pos_load_check_pending = 0U;
        parent = DEBUG_UI_PAGE_PREPARE;
    }
    notice(model);
    model->notice_return_page = parent;
    model->review_pressed = model->review_released = 0U;
    model->confirm_elapsed_ms = 0U;
}
/** @brief Declare shared confirmation before draft creation. */
static void enter_review(DebugUiModel *model);
/** @brief Guided intent defers selected feedback only; submission still requires real preflight evidence. */
static DebugUiReason single_intent_gate(const DebugUiModel *model, DebugUiOperation operation);
/** @brief Return a value-copy motor only if the selected row exists. */
static const DebugUiMotorView *selected(const DebugUiModel *model)
{
    return &model->snapshot.motors[model->selected_motor < DEBUG_UI_MOTOR_COUNT ?
                                  model->selected_motor : 0U];
}
/** @brief A changed single-axis selection discards all frozen targets and preview state. */
static void revoke_static_selection(DebugUiModel *model)
{
    model->static_locked = model->static_result_retained = 0U;
    model->confirm_elapsed_ms = 0U;
    model->review_pressed = model->review_released = 0U;
    memset(&model->draft, 0, sizeof(model->draft));
    memset(&model->reviewed, 0, sizeof(model->reviewed));
    model->draft.operation = DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD;
}
/** @brief Return the first physical ID in a bounded seven-motor mask. */
static uint8_t first_motor(uint8_t mask)
{
    uint8_t index;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        if ((mask & (1U << index)) != 0U) return (uint8_t)(index + 1U);
    return 0U;
}

/** @brief Prefer activity/cleanup evidence over a browsing row or an old submission. */
uint8_t debug_ui_model_active_target(const DebugUiModel *model)
{
    uint8_t target;
    if (model == NULL) return 0U;
    target = first_motor(model->snapshot.active_motor_mask);
    if (target != 0U) return target;
    target = first_motor(model->snapshot.unconfirmed_disable_mask);
    if (target != 0U) return target;
    if (model->snapshot.active || model->snapshot.pending || model->snapshot.stop_pending) {
        target = model->snapshot.target_motor_id;
        if (target > 0U && target <= DEBUG_UI_MOTOR_COUNT) return target;
    }
    if (model->stop_requested) {
        target = first_motor(model->stop_motor_mask);
        if (target != 0U) return target;
    }
    if (model->awaiting_result || model->outgoing_ready) {
        target = model->submitted.target_motor_id;
        if (target > 0U && target <= DEBUG_UI_MOTOR_COUNT) return target;
    }
    return 0U;
}
/** @brief Allocate a nonzero ID; wrap is still separated by application epochs. */
static void identify(DebugUiModel *model, DebugUiRequest *request)
{
    request->identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    request->identity.epoch = model->snapshot.epoch;
    request->identity.request_id = model->next_request_id++;
    if (model->next_request_id == 0U) model->next_request_id = 1U;
    request->created_at_us = model->now_us;
}
/** @brief Latch STOP for activity, or an explicitly selected idle recovery target. */
static void request_stop_with_recovery_target(DebugUiModel *model, uint8_t recovery_target)
{
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    (void)recovery_target;
    model->reason = DEBUG_UI_REASON_DISABLED;
#else
    uint8_t target;
    uint8_t static_group_pending = (uint8_t)(model->submitted.operation == DEBUG_UI_OPERATION_STATIC_HOLD &&
        (model->awaiting_result || model->outgoing_ready));
    if (!model->snapshot.motion_enabled) {
        model->reason = DEBUG_UI_REASON_DISABLED;
        return;
    }
    if (model->stop_requested) return;
    if (model->pos_load_check_pending == 1U) model->pos_load_check_pending = 2U;
    target = debug_ui_model_active_target(model);
    if (target == 0U && recovery_target > 0U && recovery_target <= DEBUG_UI_MOTOR_COUNT)
        target = recovery_target;
    /* Never introduce an unrelated selected motor into a remote STOP mask. */
    if (target == 0U) { model->reason = DEBUG_UI_REASON_NOT_READY; return; }
    if (model->single_static_flow &&
        (model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ||
         model->single_static_wait_snapshot == 1U)) {
        model->single_static_config_cancelled = 1U;
        model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
        model->single_static_wait_snapshot = model->single_static_configured = 0U;
        model->static_locked = 0U;
    }
    if (model->single_static_flow) {
        model->single_static_preflight = 0U;
        model->single_static_wait_snapshot = 0U;
        model->single_static_lifecycle_cancelled = 1U;
        model->single_static_lock_then_configure = 0U;
    }
    if (model->outgoing_ready) {
        model->outgoing_ready = 0U;
        model->awaiting_result = 0U; /* Unsent motion cannot follow its STOP. */
    }
    memset(&model->outgoing_stop, 0, sizeof(model->outgoing_stop));
    model->outgoing_stop.operation = DEBUG_UI_OPERATION_STOP;
    model->outgoing_stop.target_motor_id = target;
    model->stop_motor_mask = (uint8_t)((model->snapshot.active_motor_mask & 0x7FU) |
        model->snapshot.unconfirmed_disable_mask | (1U << (target - 1U)));
    if (static_group_pending)
        model->stop_motor_mask |= 0x7FU;
    if (model->stop_latch_state != DEBUG_UI_STOP_LATCH_UNCONFIRMED)
        model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
    identify(model, &model->outgoing_stop);
    model->stop_ready = 1U;
    model->stop_requested = 1U;
    model->stop_waiting = 1U;
    model->page = DEBUG_UI_PAGE_RUNNING;
#endif
}
/** @brief Ordinary STOP must only target actual activity, never an unrelated browsing row. */
static void request_stop(DebugUiModel *model)
{
    request_stop_with_recovery_target(model, 0U);
}
/** @brief Validate calibrated limits and defaults, rejecting NaN and empty ranges. */
static uint8_t profile_valid(const DebugUiMotorProfile *profile, uint8_t mit)
{
    DebugUiMotorProfile bounds = debug_ui_profile_bounds(profile, mit);
    profile = &bounds;
    if (!finite_value(profile->position_min_rad) || !finite_value(profile->position_max_rad) ||
        !finite_value(profile->max_delta_rad) || !finite_value(profile->max_speed_rad_s) ||
        profile->position_min_rad >= profile->position_max_rad ||
        profile->max_delta_rad <= 0.0f || profile->max_speed_rad_s <= 0.0f) return 0U;
    if (!mit) return profile->pos_valid;
    return (uint8_t)(profile->mit_valid && finite_value(profile->default_kp) &&
        finite_value(profile->default_kd) && finite_value(profile->kp_min) &&
        finite_value(profile->kp_max) && finite_value(profile->kd_min) &&
        finite_value(profile->kd_max) && profile->kp_min > 0.0f && profile->kd_min > 0.0f &&
        profile->default_kp >= profile->kp_min && profile->default_kp <= profile->kp_max &&
        profile->default_kd >= profile->kd_min && profile->default_kd <= profile->kd_max &&
        profile->hold_min_ms >= 100U && profile->hold_max_ms >= profile->hold_min_ms &&
        profile->hold_max_ms <= 1000U);
}

/** @brief Intersect the existing per-axis HOLD limits; never invent a longer duration. */
uint8_t debug_ui_model_static_duration_bounds(const DebugUiModel *model,
                                             uint32_t *minimum_ms, uint32_t *maximum_ms)
{
    unsigned index;
    uint32_t minimum = 100U, maximum = 1000U;
    if (model == NULL || minimum_ms == NULL || maximum_ms == NULL) return 0U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        const DebugUiMotorProfile *profile = &model->snapshot.motors[index].profile;
        if (!profile_valid(profile, 1U)) return 0U;
        if (profile->hold_min_ms > minimum) minimum = profile->hold_min_ms;
        if (profile->hold_max_ms < maximum) maximum = profile->hold_max_ms;
    }
    if (minimum > maximum) return 0U;
    *minimum_ms = minimum; *maximum_ms = maximum;
    return 1U;
}

/** @brief Validate the independent selected-axis long profile without extending MIT limits. */
uint8_t debug_ui_model_single_static_duration_bounds(const DebugUiModel *model,
                                                    uint32_t *minimum_ms, uint32_t *maximum_ms)
{
    const DebugUiSingleStaticProfile *profile;
    if (model == NULL || minimum_ms == NULL || maximum_ms == NULL ||
        model->selected_motor >= DEBUG_UI_MOTOR_COUNT) return 0U;
    profile = &selected(model)->single_static_profile;
    if (!profile->configured || profile->minimum_ms < DEBUG_UI_SINGLE_STATIC_MIN_MS ||
        profile->maximum_ms > DEBUG_UI_SINGLE_STATIC_MAX_MS || profile->minimum_ms > profile->maximum_ms)
        return 0U;
    *minimum_ms = profile->minimum_ms;
    *maximum_ms = profile->maximum_ms;
    return 1U;
}

/** @brief Dispatch duration bounds by static operation; ordinary paths retain their old cap. */
static uint8_t static_duration_bounds(const DebugUiModel *model, DebugUiOperation operation,
                                     uint32_t *minimum_ms, uint32_t *maximum_ms)
{
    return operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
        debug_ui_model_single_static_duration_bounds(model, minimum_ms, maximum_ms) :
        debug_ui_model_static_duration_bounds(model, minimum_ms, maximum_ms);
}

#if AETHOR_DEBUG_UI_ALLOW_MOTION
/** @brief Gate exactly the requested static selection without changing the browsing target. */
static DebugUiReason static_axis_gate(const DebugUiModel *model, DebugUiOperation operation, uint8_t defer_feedback)
{
    unsigned index;
    uint32_t minimum, maximum;
    uint8_t single_axis = (uint8_t)(operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
    if (single_axis ? (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !model->snapshot.single_static_load_enabled) :
        (!AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || !model->snapshot.static_load_enabled))
        return DEBUG_UI_REASON_DISABLED;
    if (single_axis && model->selected_motor >= DEBUG_UI_MOTOR_COUNT) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (model->snapshot.unconfirmed_disable_mask != 0U) return DEBUG_UI_REASON_NOT_DISABLED;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        const DebugUiMotorView *motor = &model->snapshot.motors[index];
        DebugUiMotorProfile bounds = single_axis ? single_model_bounds(motor) : motor->profile;
        if (single_axis && index != model->selected_motor) continue;
        if (motor->motor_id != index + 1U || !motor->identity_verified ||
            !motor->ranges_verified || !motor->mode_verified || motor->actual_mode != DEBUG_UI_MODE_MIT ||
            motor->reference_generation == 0U || motor->fault_flags != 0U || model->snapshot.arm_fault)
            return DEBUG_UI_REASON_NOT_READY;
        if (!defer_feedback && (!motor->feedback_seen || !motor->feedback_valid || motor->feedback_age_ms >= 150U ||
            !finite_value(motor->position_rad) || !finite_value(motor->velocity_rad_s) ||
            !finite_value(motor->torque_nm) || !finite_value(motor->mos_temperature_c) ||
            !finite_value(motor->rotor_temperature_c))) return DEBUG_UI_REASON_STALE_FEEDBACK;
        if (!defer_feedback && (motor->enabled || motor->driver_state != 0U)) return DEBUG_UI_REASON_NOT_DISABLED;
        if (!defer_feedback && (motor->velocity_rad_s > 0.05f || motor->velocity_rad_s < -0.05f))
            return DEBUG_UI_REASON_NOT_READY;
        if (!profile_valid(&motor->profile, 1U)) return DEBUG_UI_REASON_UNCALIBRATED;
        if (!defer_feedback && (motor->position_rad < bounds.mit_position_min_rad ||
            motor->position_rad > bounds.mit_position_max_rad)) return DEBUG_UI_REASON_OUT_OF_RANGE;
    }
    return static_duration_bounds(model, operation, &minimum, &maximum) ?
        DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_UNCALIBRATED;
}
#endif

/** @brief Validate immutable output-axis targets against every current reference and pose. */
static DebugUiReason static_request_valid(const DebugUiModel *model, const DebugUiRequest *request)
{
    unsigned index;
    uint32_t minimum, maximum;
    float tolerance = debug_ui_degrees_to_radians(0.5f);
    if (!model->static_locked || model->edit_epoch != model->snapshot.epoch)
        return DEBUG_UI_REASON_OLD_EPOCH;
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (model->selected_motor >= DEBUG_UI_MOTOR_COUNT || request->target_motor_id != model->selected_motor + 1U))
        return DEBUG_UI_REASON_OLD_EPOCH;
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && request->target_motor_id == 7U &&
        (selected(model)->single_static_gains.kp != 0.0f || selected(model)->single_static_gains.kd != 0.0f) &&
        (!debug_ui_static_gains_valid(request->kp, request->kd) ||
         request->kp != selected(model)->single_static_gains.kp || request->kd != selected(model)->single_static_gains.kd))
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (!static_duration_bounds(model, request->operation, &minimum, &maximum))
        return DEBUG_UI_REASON_UNCALIBRATED;
    if (request->hold_duration_ms < minimum || request->hold_duration_ms > maximum)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (!finite_value(request->torque_ff_nm) ||
        (request->operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && request->torque_ff_nm != 0.0f))
        return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        (!debug_ui_single_static_feedforward_valid(request->torque_ff_nm, selected(model)->torque_max_nm) ||
         request->torque_ff_nm != selected(model)->single_static_profile.torque_ff_nm))
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        const DebugUiMotorView *motor = &model->snapshot.motors[index];
        DebugUiMotorProfile bounds = request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ? single_model_bounds(motor) : motor->profile;
        float position = request->static_positions_rad[index];
        float error = position - motor->position_rad;
        if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && index + 1U != request->target_motor_id) continue;
        if (request->static_references[index] != motor->reference_generation)
            return DEBUG_UI_REASON_OLD_EPOCH;
        if (!finite_value(position)) return DEBUG_UI_REASON_INVALID_ARGUMENT;
        if (position < bounds.mit_position_min_rad || position > bounds.mit_position_max_rad ||
            error > tolerance || error < -tolerance) return DEBUG_UI_REASON_OUT_OF_RANGE;
    }
    return DEBUG_UI_REASON_NONE;
}

#include "debug_ui_single_static_flow.inc"
#include "debug_ui_load_trial_pages.inc"

/** @brief Reset to a locked overview without inferring any feedback. */
void debug_ui_model_init(DebugUiModel *model)
{
    if (model == NULL) return;
    memset(model, 0, sizeof(*model));
    model->page = DEBUG_UI_PAGE_OVERVIEW;
    model->diagnostic_return_page = model->diagnostic_parent_page = DEBUG_UI_PAGE_OVERVIEW;
    model->reason = DEBUG_UI_REASON_NOT_READY;
    model->next_request_id = 1U;
    model->single_static_seconds = model->single_static_pending_seconds = 1U;
    model->single_static_kp = DEBUG_UI_STATIC_KP_DEFAULT;
    model->single_static_kd_centi = DEBUG_UI_STATIC_KD_CENTI_DEFAULT;
    debug_ui_load_trials_init(&model->load_trials);
}

/** @brief Check compile-time, ownership, health and commissioned profile gates. */
static DebugUiReason model_gate(const DebugUiModel *model, DebugUiOperation operation, uint8_t defer_feedback)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    const DebugUiSnapshot *snapshot;
    const DebugUiMotorView *motor;
    uint8_t deferred_pos_feedback;
    uint8_t selected_motor_index;
#endif
    if (model == NULL) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if ((unsigned)operation > DEBUG_UI_OPERATION_POS_LOAD_TEST) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (operation == DEBUG_UI_OPERATION_STOP)
        return AETHOR_DEBUG_UI_ALLOW_MOTION && model->snapshot.motion_enabled ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_DISABLED;
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    (void)defer_feedback;
    return DEBUG_UI_REASON_DISABLED;
#else
    snapshot = &model->snapshot;
    selected_motor_index = model->selected_motor;
    if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK && model->single_static_flow &&
        model->single_static_relock_motor_id > 0U && model->single_static_relock_motor_id <= DEBUG_UI_MOTOR_COUNT)
        selected_motor_index = (uint8_t)(model->single_static_relock_motor_id - 1U);
    motor = &snapshot->motors[selected_motor_index < DEBUG_UI_MOTOR_COUNT ? selected_motor_index : 0U];
    if (!model->snapshot_seen || model->now_us < snapshot->timestamp_us ||
        model->now_us - snapshot->timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US)
        return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (!snapshot->motion_enabled || !snapshot->bench_profile) return DEBUG_UI_REASON_DISABLED;
    if (operation == DEBUG_UI_OPERATION_POS_LOAD_TEST &&
        (!AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST || !snapshot->pos_load_enabled)) return DEBUG_UI_REASON_DISABLED;
    if (is_mit(operation) && (!AETHOR_DEBUG_UI_ALLOW_MIT || !snapshot->mit_enabled))
        return DEBUG_UI_REASON_DISABLED;
    if (operation == DEBUG_UI_OPERATION_STATIC_HOLD &&
        (!AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || !snapshot->static_load_enabled)) return DEBUG_UI_REASON_DISABLED;
    if ((operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD || operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) &&
        (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !snapshot->single_static_load_enabled)) return DEBUG_UI_REASON_DISABLED;
    if (!model->input_valid || !model->display_valid || !snapshot->health.input_valid ||
        !snapshot->health.display_valid || !snapshot->health.ui_valid || !snapshot->health.protocol_valid ||
        model->now_us < snapshot->health.ui_timestamp_us ||
        model->now_us < snapshot->health.protocol_timestamp_us ||
        model->now_us - snapshot->health.ui_timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US ||
        model->now_us - snapshot->health.protocol_timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US)
        return DEBUG_UI_REASON_UNHEALTHY;
    if (snapshot->active || snapshot->pending || snapshot->stop_pending || model->awaiting_result ||
        model->outgoing_ready || model->stop_ready || model->stop_waiting ||
        model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING) return DEBUG_UI_REASON_BUSY;
    if (snapshot->retained_result_count != 0U) return DEBUG_UI_REASON_RESULT_BACKPRESSURE;
    if (single_lifecycle_operation(operation)) {
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
            (!AETHOR_DEBUG_UI_ALLOW_MIT || !snapshot->mit_enabled)) return DEBUG_UI_REASON_DISABLED;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE &&
            (!AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || !snapshot->single_static_load_enabled ||
             !AETHOR_DEBUG_UI_ALLOW_MIT || !snapshot->mit_enabled)) return DEBUG_UI_REASON_DISABLED;
        if (snapshot->unconfirmed_disable_mask || model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED)
            return DEBUG_UI_REASON_NOT_DISABLED;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE && snapshot->authority != DEBUG_UI_AUTHORITY_REMOTE)
            return DEBUG_UI_REASON_NOT_ARMED;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK && snapshot->authority != DEBUG_UI_AUTHORITY_REMOTE &&
            ((snapshot->authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED || snapshot->target_motor_id != motor->motor_id) &&
             !(snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT && single_fault_relock_owned(model, motor))))
            return DEBUG_UI_REASON_NOT_ARMED;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT && snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
            (model->single_static_preflight != 2U || !single_fault_relock_owned(model, motor)))
            return DEBUG_UI_REASON_NOT_ARMED;
        if (motor->motor_id != selected_motor_index + 1U || !motor->reference_generation ||
            !motor->single_static_evidence_generation) return DEBUG_UI_REASON_NOT_READY;
        if (!defer_feedback && (!motor->feedback_seen || !motor->feedback_valid || motor->feedback_age_ms >= 150U ||
            !finite_value(motor->position_rad) || !finite_value(motor->velocity_rad_s) ||
            !finite_value(motor->torque_nm) || !finite_value(motor->mos_temperature_c) ||
            !finite_value(motor->rotor_temperature_c))) return DEBUG_UI_REASON_STALE_FEEDBACK;
        if (!defer_feedback && (motor->enabled || motor->driver_state != 0U)) return DEBUG_UI_REASON_NOT_DISABLED;
        if (motor->fault_flags || snapshot->arm_fault || (!defer_feedback && (motor->velocity_rad_s > 0.05f ||
            motor->velocity_rad_s < -0.05f))) return DEBUG_UI_REASON_NOT_READY;
        if (!defer_feedback && operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK &&
            snapshot->authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT) {
            /* Match the existing 3.5 Nm / 55 C executor guards at the final fault-withdrawal snapshot. */
            if (motor->torque_nm > 3.5f || motor->torque_nm < -3.5f) return DEBUG_UI_REASON_TORQUE_LIMIT;
            if (motor->mos_temperature_c > 55.0f || motor->rotor_temperature_c > 55.0f)
                return DEBUG_UI_REASON_TEMPERATURE_LIMIT;
            if (motor->position_rad < single_model_bounds(motor).mit_position_min_rad ||
                motor->position_rad > single_model_bounds(motor).mit_position_max_rad) return DEBUG_UI_REASON_OUT_OF_RANGE;
        }
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE) {
            if (!motor->identity_verified || !motor->mode_verified || !motor->ranges_verified ||
                motor->actual_mode != DEBUG_UI_MODE_MIT) return DEBUG_UI_REASON_NOT_READY;
            if (!profile_valid(&motor->profile, 1U)) return DEBUG_UI_REASON_UNCALIBRATED;
            if (!debug_ui_single_static_feedforward_valid(debug_ui_model_single_static_feedforward(model), motor->torque_max_nm))
                return DEBUG_UI_REASON_OUT_OF_RANGE;
            if (model->single_static_seconds < 1U || model->single_static_seconds > 60U ||
                (!defer_feedback && (motor->position_rad < single_model_bounds(motor).mit_position_min_rad ||
                motor->position_rad > single_model_bounds(motor).mit_position_max_rad))) return DEBUG_UI_REASON_OUT_OF_RANGE;
        }
        return DEBUG_UI_REASON_NONE;
    }
    if (model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED &&
        operation != DEBUG_UI_OPERATION_DISABLE && operation != DEBUG_UI_OPERATION_CLEAR_FAULT)
        return DEBUG_UI_REASON_NOT_DISABLED;
    if ((operation == DEBUG_UI_OPERATION_ACQUIRE || operation == DEBUG_UI_OPERATION_RELEASE) &&
        snapshot->unconfirmed_disable_mask != 0U) return DEBUG_UI_REASON_NOT_DISABLED;
    if (operation == DEBUG_UI_OPERATION_RELEASE && snapshot->target_motor_id != 0U &&
        snapshot->authority != DEBUG_UI_AUTHORITY_REMOTE &&
        motor->motor_id != snapshot->target_motor_id) return DEBUG_UI_REASON_NOT_ARMED;
    if (operation != DEBUG_UI_OPERATION_ACQUIRE && operation != DEBUG_UI_OPERATION_RELEASE &&
        snapshot->authority != DEBUG_UI_AUTHORITY_LOCAL_ARMED) return DEBUG_UI_REASON_NOT_ARMED;
    if (operation == DEBUG_UI_OPERATION_ACQUIRE &&
        snapshot->authority != DEBUG_UI_AUTHORITY_REMOTE &&
        snapshot->authority != DEBUG_UI_AUTHORITY_LOCAL_FAULT) return DEBUG_UI_REASON_NOT_ARMED;
    if (debug_ui_is_static_operation(operation)) return static_axis_gate(model, operation, defer_feedback);
    if (operation == DEBUG_UI_OPERATION_POS_LOAD_TEST) {
        if (snapshot->unconfirmed_disable_mask) return DEBUG_UI_REASON_NOT_DISABLED;
        if (snapshot->target_motor_id != motor->motor_id || motor->actual_mode != DEBUG_UI_MODE_POS_VEL)
            return DEBUG_UI_REASON_NOT_READY;
        /* Review intent defers standard feedback to the executor's real DISABLE preflight.
         * Retained finite values cannot manufacture freshness or an ENABLE acknowledgement. */
        if (!finite_value(motor->position_rad) || !finite_value(motor->torque_nm) ||
            !finite_value(motor->mos_temperature_c) || !finite_value(motor->rotor_temperature_c))
            return DEBUG_UI_REASON_NOT_READY;
        if (motor->driver_state != 0U) return DEBUG_UI_REASON_NOT_DISABLED;
        if (motor->torque_nm > 3.5f || motor->torque_nm < -3.5f) return DEBUG_UI_REASON_TORQUE_LIMIT;
        if (motor->mos_temperature_c > 55.0f || motor->rotor_temperature_c > 55.0f)
            return DEBUG_UI_REASON_TEMPERATURE_LIMIT;
    }
    /* Intent only: each motion/mode change obtains a new disabled ACK in the executor. */
    deferred_pos_feedback = (uint8_t)((motor->actual_mode == DEBUG_UI_MODE_POS_VEL ||
        motor->actual_mode == DEBUG_UI_MODE_MIT) &&
        (operation == DEBUG_UI_OPERATION_POS_MOVE || operation == DEBUG_UI_OPERATION_POS_LOAD_TEST || is_mit(operation) ||
         operation == DEBUG_UI_OPERATION_SET_MODE ||
         ((operation == DEBUG_UI_OPERATION_ACQUIRE || operation == DEBUG_UI_OPERATION_RELEASE) &&
          snapshot->unconfirmed_disable_mask == 0U &&
          ((motor->actual_mode == DEBUG_UI_MODE_POS_VEL && motor->profile.pos_valid) ||
           (motor->actual_mode == DEBUG_UI_MODE_MIT && AETHOR_DEBUG_UI_ALLOW_MIT && motor->profile.mit_valid)))));
    if (!deferred_pos_feedback && (!motor->feedback_valid || !motor->feedback_seen ||
        motor->feedback_age_ms >= 150U || !finite_value(motor->position_rad) ||
        !finite_value(motor->velocity_rad_s)))
        return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (motor->motor_id != model->selected_motor + 1U || !motor->identity_verified ||
        !motor->ranges_verified || !motor->mode_verified || motor->actual_mode == DEBUG_UI_MODE_UNKNOWN)
        return DEBUG_UI_REASON_NOT_READY;
    if (operation == DEBUG_UI_OPERATION_DISABLE) return DEBUG_UI_REASON_NONE;
    if (motor->enabled || ((operation == DEBUG_UI_OPERATION_ACQUIRE ||
        operation == DEBUG_UI_OPERATION_RELEASE) && motor->driver_state != 0U))
        return DEBUG_UI_REASON_NOT_DISABLED;
    /* Match the executor's 0.05 rad/s gate; +/-200 rad/s 12-bit feedback has +/-0.04884 zero bins. */
    if (!finite_value(motor->velocity_rad_s) || motor->velocity_rad_s > 0.05f || motor->velocity_rad_s < -0.05f)
        return DEBUG_UI_REASON_NOT_READY;
    if ((motor->fault_flags != 0U || (snapshot->arm_fault != 0U && operation != DEBUG_UI_OPERATION_RELEASE)) &&
        operation != DEBUG_UI_OPERATION_CLEAR_FAULT) return DEBUG_UI_REASON_NOT_READY;
    if (!profile_valid(&motor->profile, is_mit(operation)) &&
        !(operation == DEBUG_UI_OPERATION_RELEASE && profile_valid(&motor->profile, 1U)))
        return DEBUG_UI_REASON_UNCALIBRATED;
    return DEBUG_UI_REASON_NONE;
#endif
}

/** @brief Public execution gate always consumes actual fresh selected feedback. */
DebugUiReason debug_ui_model_gate(const DebugUiModel *model, DebugUiOperation operation)
{
    return model_gate(model, operation, 0U);
}

/** @brief Keep human review valid while deferring only the explicit selected-axis sample collection. */
static DebugUiReason single_intent_gate(const DebugUiModel *model, DebugUiOperation operation)
{
    return model_gate(model, operation, (uint8_t)(model->single_static_flow &&
        (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE || operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK ||
         operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)));
}

/** @brief Check edited parameters again against the latest copied feedback. */
static DebugUiReason validate_request(const DebugUiModel *model, const DebugUiRequest *request)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_profile_bounds(&motor->profile, is_mit(request->operation));
    const DebugUiMotorProfile *profile = &bounds;
    DebugUiReason gate = single_intent_gate(model, request->operation);
    float target;
    if (gate != DEBUG_UI_REASON_NONE) return gate;
    if (request->operation == DEBUG_UI_OPERATION_POS_LOAD_TEST) {
        if (motor->motor_id == 7U && !debug_ui_pos_load_flange(motor)) return DEBUG_UI_REASON_UNCALIBRATED;
        bounds = debug_ui_pos_load_bounds(motor);
    }
    if (debug_ui_is_static_operation(request->operation)) return static_request_valid(model, request);
    if (request->operation == DEBUG_UI_OPERATION_SET_MODE) {
        if (request->requested_mode != DEBUG_UI_MODE_MIT && request->requested_mode != DEBUG_UI_MODE_POS_VEL)
            return DEBUG_UI_REASON_INVALID_ARGUMENT;
        if (request->requested_mode == DEBUG_UI_MODE_MIT &&
            (!AETHOR_DEBUG_UI_ALLOW_MIT || !model->snapshot.mit_enabled || !profile_valid(profile, 1U)))
            return DEBUG_UI_REASON_UNCALIBRATED;
    }
    if (request->operation != DEBUG_UI_OPERATION_POS_MOVE && request->operation != DEBUG_UI_OPERATION_POS_LOAD_TEST && !is_mit(request->operation))
        return DEBUG_UI_REASON_NONE;
    if (!finite_value(request->delta_rad) || !finite_value(request->speed_rad_s) ||
        !finite_value(request->torque_ff_nm) || request->torque_ff_nm != 0.0f)
        return DEBUG_UI_REASON_INVALID_ARGUMENT;
    target = motor->position_rad + request->delta_rad;
    if (request->delta_rad < -profile->max_delta_rad || request->delta_rad > profile->max_delta_rad ||
        request->speed_rad_s <= 0.0f || request->speed_rad_s > profile->max_speed_rad_s)
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (request->operation == DEBUG_UI_OPERATION_POS_LOAD_TEST &&
        (request->requested_mode != DEBUG_UI_MODE_POS_VEL || request->delta_rad == 0.0f ||
         request->hold_duration_ms < 1000U || request->hold_duration_ms > 60000U ||
         request->hold_duration_ms % 1000U != 0U || request->kp != 0.0f || request->kd != 0.0f))
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    /* An expired position is neither a preview nor a reference for a local motion command. */
    if ((motor->feedback_valid && motor->feedback_seen && motor->feedback_age_ms < 150U) &&
        (!finite_value(target) || target < profile->position_min_rad || target > profile->position_max_rad))
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    if (is_mit(request->operation) && (!finite_value(request->kp) || !finite_value(request->kd) ||
        request->kp < profile->kp_min || request->kp > profile->kp_max ||
        request->kd < profile->kd_min || request->kd > profile->kd_max ||
        request->hold_duration_ms < profile->hold_min_ms || request->hold_duration_ms > profile->hold_max_ms))
        return DEBUG_UI_REASON_OUT_OF_RANGE;
    return DEBUG_UI_REASON_NONE;
}

/** @brief Resolve a result-less STOP only from newer coherent cleanup/feedback evidence. */
static void update_stop_latch(DebugUiModel *model)
{
    unsigned index;
    uint8_t confirmed = (uint8_t)(model->stop_motor_mask != 0U);
    const DebugUiSnapshot *snapshot = &model->snapshot;
    if (model->stop_latch_state != DEBUG_UI_STOP_LATCH_WAITING ||
        snapshot->timestamp_us <= model->outgoing_stop.created_at_us ||
        model->now_us < snapshot->timestamp_us ||
        model->now_us - snapshot->timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US) return;
    model->stop_motor_mask |= (uint8_t)(snapshot->active_motor_mask & 0x7FU);
    if (snapshot->active || snapshot->pending || snapshot->stop_pending) return;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        const DebugUiMotorView *motor = &snapshot->motors[index];
        uint64_t age_us = (uint64_t)motor->feedback_age_ms * 1000ULL;
        if ((model->stop_motor_mask & (1U << index)) == 0U) continue;
        /* Raw feedback state 0 is DISABLED; enabled==0 also includes fault states. */
        if (motor->motor_id != index + 1U || !motor->feedback_seen || !motor->feedback_valid ||
            motor->feedback_age_ms >= 150U || motor->enabled ||
            motor->driver_state != 0U || motor->fault_flags != 0U ||
            age_us >= snapshot->timestamp_us - model->outgoing_stop.created_at_us)
            confirmed = 0U;
    }
    model->stop_latch_state = confirmed ? DEBUG_UI_STOP_LATCH_CONFIRMED : DEBUG_UI_STOP_LATCH_UNCONFIRMED;
    model->stop_requested = 0U;
    model->reason = confirmed ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_NOT_DISABLED;
}

/** @brief Keep failed cleanup visible after the last outstanding request has settled. */
static DebugUiPage settled_page(const DebugUiModel *model)
{
    if (model->pos_load_flow && model->submitted.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST &&
        (model->completion_seen || model->stop_latch_state != DEBUG_UI_STOP_LATCH_NONE))
        return DEBUG_UI_PAGE_POS_LOAD_RESULT;
    if (debug_ui_is_static_operation(model->submitted.operation) && model->completion_seen && !model->static_result_retained)
        return model->submitted.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            DEBUG_UI_PAGE_SINGLE_STATIC_RESULT : DEBUG_UI_PAGE_STATIC_RESULT;
    if (model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED) return DEBUG_UI_PAGE_FAULT;
    if (model->stop_latch_state == DEBUG_UI_STOP_LATCH_CONFIRMED) return DEBUG_UI_PAGE_RESULT;
    if (!model->completion_seen) return DEBUG_UI_PAGE_DETAIL;
    return model->completion.code == DEBUG_UI_FAILED ||
        (!model->completion.disabled_confirmed && (model->completion.operation == DEBUG_UI_OPERATION_STOP ||
         model->completion.operation == DEBUG_UI_OPERATION_POS_MOVE || is_mit(model->completion.operation))) ?
        DEBUG_UI_PAGE_FAULT : DEBUG_UI_PAGE_RESULT;
}

/** @brief Copy a coherent snapshot and revoke stale or changed draft state. */
void debug_ui_model_update(DebugUiModel *model, const DebugUiSnapshot *snapshot,
                           uint64_t now_us, uint8_t input_valid, uint8_t display_valid)
{
    uint8_t editing;
    if (model == NULL) return;
    model->now_us = now_us;
    model->input_valid = input_valid;
    model->display_valid = display_valid;
    if (snapshot != NULL) {
        model->snapshot = *snapshot;
        model->snapshot_seen = 1U;
        update_stop_latch(model);
        debug_ui_load_trials_observe(&model->load_trials, snapshot, now_us);
    } else if (model->snapshot_seen && now_us - model->snapshot.timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US) {
        unsigned index;
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
            model->snapshot.motors[index].feedback_valid = 0U;
    }
    editing = (uint8_t)(model->page == DEBUG_UI_PAGE_EDIT || model->page == DEBUG_UI_PAGE_NUMBER || model->page == DEBUG_UI_PAGE_REVIEW);
    if (model->static_locked && !model->single_static_preflight && !model->awaiting_result && !model->snapshot.active && !model->snapshot.pending) {
        DebugUiReason static_reason = single_intent_gate(model, model->draft.operation);
        if (static_reason == DEBUG_UI_REASON_NONE) static_reason = static_request_valid(model, &model->draft);
        if (static_reason != DEBUG_UI_REASON_NONE) {
            model->static_locked = 0U;
            if (debug_ui_is_static_operation(model->draft.operation)) model->reason = static_reason;
        }
    }
    if (editing) {
        DebugUiReason reason = single_intent_gate(model, model->draft.operation);
        if (debug_ui_is_static_operation(model->draft.operation)) {
            if (reason == DEBUG_UI_REASON_NONE) reason = static_request_valid(model, &model->draft);
        } else if (model->edit_epoch != model->snapshot.epoch ||
            model->edit_reference != selected(model)->reference_generation || model->draft.target_motor_id != selected(model)->motor_id)
            reason = DEBUG_UI_REASON_OLD_EPOCH;
        if (reason != DEBUG_UI_REASON_NONE) {
            model->reason = reason;
            model->confirm_elapsed_ms = 0U;
            revoke_draft(model);
        }
    }
    if (model->snapshot.active || model->snapshot.pending || model->snapshot.stop_pending ||
        model->awaiting_result || model->stop_waiting || model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING) {
        if (!model->single_static_flow || !single_lifecycle_operation(model->submitted.operation) ||
            model->snapshot.active || model->snapshot.pending || model->snapshot.stop_pending || model->stop_requested)
            model->page = DEBUG_UI_PAGE_RUNNING;
        if ((!input_valid || !display_valid) &&
            (model->snapshot.active_identity.origin == DEBUG_UI_ORIGIN_LOCAL_UI || model->awaiting_result))
            request_stop(model);
    } else if (model->page == DEBUG_UI_PAGE_RUNNING) {
        model->page = settled_page(model);
        model->stop_requested = 0U;
    }
    single_flow_update(model);
}

/** @brief Create a bounded draft after gates; never publish from this function. */
uint8_t debug_ui_model_begin(DebugUiModel *model, DebugUiOperation operation,
                             DebugUiMode requested_mode)
{
    DebugUiMotorProfile bounds;
    const DebugUiMotorProfile *profile;
    if (model == NULL || ((unsigned)operation > DEBUG_UI_OPERATION_DISABLE && !debug_ui_is_static_operation(operation))) return 0U;
    model->reason = debug_ui_model_gate(model, operation);
    if (model->reason != DEBUG_UI_REASON_NONE) { notice(model); return 0U; }
    bounds = debug_ui_profile_bounds(&selected(model)->profile, is_mit(operation));
    profile = &bounds;
    memset(&model->draft, 0, sizeof(model->draft));
    model->static_locked = 0U;
    model->static_result_retained = 0U;
    model->draft.operation = operation;
    model->draft.requested_mode = requested_mode;
    model->draft.target_motor_id = selected(model)->motor_id;
    model->draft.reference_generation = selected(model)->reference_generation;
    model->draft.delta_rad = operation == DEBUG_UI_OPERATION_MIT_HOLD ? 0.0f :
        (profile->max_delta_rad < debug_ui_degrees_to_radians(1.0f) ? profile->max_delta_rad : debug_ui_degrees_to_radians(1.0f));
    model->draft.speed_rad_s = profile->max_speed_rad_s < debug_ui_degrees_to_radians(10.0f) ?
        profile->max_speed_rad_s : debug_ui_degrees_to_radians(10.0f);
    model->draft.kp = profile->default_kp;
    model->draft.kd = profile->default_kd;
    model->draft.hold_duration_ms = profile->hold_min_ms;
    model->edit_reference = selected(model)->reference_generation;
    model->edit_epoch = model->snapshot.epoch;
    if (debug_ui_is_static_operation(operation)) {
        unsigned index;
        uint32_t minimum, maximum;
        (void)static_duration_bounds(model, operation, &minimum, &maximum);
        model->draft.hold_duration_ms = minimum;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && model->single_static_flow &&
            model->single_static_configured && (uint32_t)model->single_static_seconds * 1000U >= minimum &&
            (uint32_t)model->single_static_seconds * 1000U <= maximum)
            model->draft.hold_duration_ms = (uint32_t)model->single_static_seconds * 1000U;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
            model->draft.single_static_evidence_generation = selected(model)->single_static_evidence_generation;
            model->draft.torque_ff_nm = selected(model)->single_static_profile.torque_ff_nm;
        }
        if (operation == DEBUG_UI_OPERATION_STATIC_HOLD)
            model->draft.target_motor_id = model->snapshot.target_motor_id > 0U && model->snapshot.target_motor_id <= DEBUG_UI_MOTOR_COUNT ?
                model->snapshot.target_motor_id : selected(model)->motor_id;
        model->draft.reference_generation = model->snapshot.motors[model->draft.target_motor_id - 1U].reference_generation;
        model->draft.delta_rad = model->draft.speed_rad_s = model->draft.kp = model->draft.kd = 0.0f;
        if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && model->draft.target_motor_id == 7U) {
            model->draft.kp = selected(model)->single_static_gains.kp;
            model->draft.kd = selected(model)->single_static_gains.kd;
        }
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
            if (operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && index + 1U != model->draft.target_motor_id) continue;
            model->draft.static_positions_rad[index] = model->snapshot.motors[index].position_rad;
            model->draft.static_references[index] = model->snapshot.motors[index].reference_generation;
        }
        model->static_locked = 1U;
    }
    model->edit_field = operation == DEBUG_UI_OPERATION_MIT_HOLD ? 2U : 0U;
    /* Opening a recovery review is not evidence that cleanup succeeded. */
    if (model->stop_latch_state != DEBUG_UI_STOP_LATCH_UNCONFIRMED) {
        model->completion_seen = 0U;
        model->stop_requested = 0U;
        model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
    }
    if (debug_ui_model_parameter_count(model)) {
        navigate(model, DEBUG_UI_PAGE_EDIT);
        model->focus = model->list_first = 0U;
    } else enter_review(model);
    return 1U;
}

/** @brief Saturate an editable value to commissioned bounds. */
static float bounded(float value, float minimum, float maximum)
{
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}
/** @brief Edit POS/MIT output angles by one degree and speeds by one degree per second. */
static void edit_value(DebugUiModel *model, int direction)
{
    DebugUiMotorProfile bounds = debug_ui_profile_bounds(&selected(model)->profile, is_mit(model->draft.operation));
    const DebugUiMotorProfile *profile = &bounds;
    float step = (float)direction;
    if (model->draft.operation != DEBUG_UI_OPERATION_POS_MOVE && !is_mit(model->draft.operation)) return;
    if (model->edit_field == 0U && model->draft.operation != DEBUG_UI_OPERATION_MIT_HOLD)
        model->draft.delta_rad = bounded(model->draft.delta_rad + step * debug_ui_degrees_to_radians(1.0f),
                                         -profile->max_delta_rad, profile->max_delta_rad);
    else if (model->edit_field == 1U)
        model->draft.speed_rad_s = bounded(model->draft.speed_rad_s + step * debug_ui_degrees_to_radians(1.0f),
                                            profile->max_speed_rad_s < debug_ui_degrees_to_radians(1.0f) ?
                                            profile->max_speed_rad_s : debug_ui_degrees_to_radians(1.0f), profile->max_speed_rad_s);
    else if (model->edit_field == 2U)
        model->draft.kp = bounded(model->draft.kp + step * 0.1f, profile->kp_min, profile->kp_max);
    else if (model->edit_field == 3U)
        model->draft.kd = bounded(model->draft.kd + step * 0.01f, profile->kd_min, profile->kd_max);
    else if (model->edit_field == 4U) {
        uint32_t minimum = profile->hold_min_ms, maximum = profile->hold_max_ms;
        int32_t duration = (int32_t)model->draft.hold_duration_ms + direction *
            (model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ? 1000 : 100);
        if (debug_ui_is_static_operation(model->draft.operation) &&
            !static_duration_bounds(model, model->draft.operation, &minimum, &maximum)) return;
        if (duration < (int32_t)minimum) duration = (int32_t)minimum;
        if (duration > (int32_t)maximum) duration = (int32_t)maximum;
        model->draft.hold_duration_ms = (uint32_t)duration;
    }
}
/** @brief Freeze a validated draft; the entering press cannot submit it. */
static void enter_review(DebugUiModel *model)
{
    model->reason = validate_request(model, &model->draft);
    if (model->reason != DEBUG_UI_REASON_NONE) { notice(model); return; }
    model->review_return_page = model->page;
    model->reviewed = model->draft;
    if (model->reviewed.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
        (void)debug_ui_load_trials_freeze(&model->load_trials, model->reviewed.target_motor_id);
    navigate(model, DEBUG_UI_PAGE_REVIEW);
    model->review_released = model->review_pressed = 0U;
    model->confirm_elapsed_ms = 0U;
    model->center_consumed = 1U;
}
/** @brief Commit the immutable reviewed action exactly once. */
static void submit_review(DebugUiModel *model)
{
    if (model->edit_epoch != model->snapshot.epoch ||
        (!debug_ui_is_static_operation(model->reviewed.operation) && (
        model->edit_reference != selected(model)->reference_generation ||
        model->reviewed.target_motor_id != selected(model)->motor_id))) {
        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
        model->review_pressed = model->review_released = 0U;
        model->confirm_elapsed_ms = 0U;
        revoke_draft(model);
        return;
    }
    model->reason = validate_request(model, &model->reviewed);
    if (model->reason != DEBUG_UI_REASON_NONE) return;
    if (model->single_static_flow && model->reviewed.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
        single_preflight_begin(model, &model->reviewed, 4U);
        return;
    }
    model->submitted = model->reviewed;
    model->single_static_lifecycle_cancelled = 0U;
    identify(model, &model->submitted);
    (void)debug_ui_load_trials_submit(&model->load_trials, &model->submitted);
    model->outgoing = model->submitted;
    model->outgoing_ready = model->awaiting_result = 1U;
    model->admission_seen = model->completion_seen = 0U;
    model->center_consumed = 1U;
    model->page = DEBUG_UI_PAGE_RUNNING;
}

#include "debug_ui_pos_load_pages.inc"

/** @brief Dispatch one input event with running STOP ahead of navigation gates. */
void debug_ui_model_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    uint8_t activation;
    int direction;
    if (model == NULL || event == NULL) return;
    if (event->type == DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP) {
        if (event->key == DEBUG_UI_KEY_CENTER) {
            if (model->single_static_preflight)
                request_stop_with_recovery_target(model, model->submitted.target_motor_id);
            else if (model->snapshot.active || model->snapshot.pending || model->awaiting_result) request_stop(model);
        }
        return;
    }
    if (model->single_static_preflight && event->type == DEBUG_UI_INPUT_EVENT_PRESS &&
        event->key == DEBUG_UI_KEY_LEFT) {
        request_stop_with_recovery_target(model, model->submitted.target_motor_id);
        return;
    }
    /* A delivered, debounced CENTER edge is a STOP intent even when the input
     * queue reports overflow/invalid health. It cannot arm or submit motion. */
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS &&
        model->single_static_flow && (model->single_static_wait_snapshot != 0U || model->single_static_preflight)) {
        uint8_t recovery_target = model->submitted.target_motor_id;
        if (recovery_target == 0U || recovery_target > DEBUG_UI_MOTOR_COUNT)
            recovery_target = model->single_static_relock_motor_id > 0U ?
                model->single_static_relock_motor_id : selected(model)->motor_id;
        request_stop_with_recovery_target(model, recovery_target);
        model->center_down = model->center_consumed = 1U;
        return;
    }
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS &&
        (model->snapshot.active || model->snapshot.pending || model->awaiting_result || model->snapshot.stop_pending)) {
        request_stop(model);
        model->center_down = model->center_consumed = 1U;
        return;
    }
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_RELEASE) {
        model->center_down = model->center_consumed = 0U;
        model->review_released = 1U;
        model->review_pressed = 0U;
        model->confirm_elapsed_ms = 0U;
        return;
    }
    if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE && model->page == DEBUG_UI_PAGE_REVIEW && !model->center_down) {
        model->center_consumed = 0U;
        model->review_released = 1U;
        return;
    }
    if (!model->input_valid || event->key == DEBUG_UI_KEY_UNKNOWN) return;
    if (event->key == DEBUG_UI_KEY_CENTER && event->type == DEBUG_UI_INPUT_EVENT_PRESS) {
        if (model->center_down || model->center_consumed) return;
        model->center_down = 1U;
        if (model->snapshot.active || model->snapshot.pending || model->awaiting_result || model->snapshot.stop_pending) {
            request_stop(model);
            model->center_consumed = 1U;
            return;
        }
    }
    if (model->page == DEBUG_UI_PAGE_RUNNING) return;
    if (pos_load_page_event(model, event)) return;
    if (load_page_event(model, event)) return;
    if (single_flow_event(model, event)) return;
    if (model->page == DEBUG_UI_PAGE_REVIEW) {
        if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && event->key == DEBUG_UI_KEY_LEFT) {
            if (model->pos_load_check_pending == 1U && model->reviewed.operation == DEBUG_UI_OPERATION_ACQUIRE) {
                model->draft = model->pos_load_saved_draft;
                model->pos_load_check_pending = 0U;
            }
            navigate(model, model->review_return_page);
            model->confirm_elapsed_ms = 0U;
            model->review_pressed = model->review_released = 0U;
        } else if (event->key == DEBUG_UI_KEY_CENTER && !model->center_consumed) {
            if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && model->review_released) {
                model->review_pressed = 1U;
                model->review_pressed_ms = event->timestamp_ms;
            } else if (event->type == DEBUG_UI_INPUT_EVENT_HOLD && model->review_pressed && model->center_down) {
                uint32_t elapsed = event->timestamp_ms - model->review_pressed_ms;
                model->confirm_elapsed_ms = elapsed < event->held_ms ? elapsed : event->held_ms;
                if (model->confirm_elapsed_ms >= DEBUG_UI_REVIEW_HOLD_MS) submit_review(model);
            }
        }
        return;
    }
    if (event->type != DEBUG_UI_INPUT_EVENT_PRESS && event->type != DEBUG_UI_INPUT_EVENT_REPEAT) return;
    if (event->type == DEBUG_UI_INPUT_EVENT_REPEAT && event->key == DEBUG_UI_KEY_CENTER) return;
    activation = (uint8_t)(event->key == DEBUG_UI_KEY_CENTER || event->key == DEBUG_UI_KEY_RIGHT);
    direction = event->key == DEBUG_UI_KEY_UP ? -1 : (event->key == DEBUG_UI_KEY_DOWN ? 1 : 0);
    switch (model->page) {
    case DEBUG_UI_PAGE_OVERVIEW:
        /* The horizontal home carousel alone uses LEFT/RIGHT selection and CENTER/DOWN entry. */
        direction = event->key == DEBUG_UI_KEY_LEFT ? -1 : (event->key == DEBUG_UI_KEY_RIGHT ? 1 : 0);
        activation = (uint8_t)(event->type == DEBUG_UI_INPUT_EVENT_PRESS &&
            (event->key == DEBUG_UI_KEY_CENTER || event->key == DEBUG_UI_KEY_DOWN));
        if (direction) model->focus = (uint8_t)((model->focus + 4 + direction) % 4);
        if (activation) {
            if (model->focus == 0U) navigate(model, DEBUG_UI_PAGE_MOTORS);
            else if (model->focus == 1U) {
                model->diagnostic_return_page = DEBUG_UI_PAGE_OVERVIEW;
                open_diagnostics(model, DEBUG_UI_PAGE_OVERVIEW);
            } else if (model->focus == 3U) navigate(model, DEBUG_UI_PAGE_STATIC_HUB);
            else {
                model->prepare_return_page = DEBUG_UI_PAGE_OVERVIEW;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
                if (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED)
                    (void)debug_ui_model_begin(model, DEBUG_UI_OPERATION_RELEASE, DEBUG_UI_MODE_UNKNOWN);
            }
        }
        break;
    case DEBUG_UI_PAGE_PREPARE:
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, model->prepare_return_page);
        else if (event->key == DEBUG_UI_KEY_CENTER) {
            if (model->snapshot.unconfirmed_disable_mask != 0U) request_stop(model);
            else if (model->pos_load_flow && debug_ui_model_result_requires_check(model))
                request_stop_with_recovery_target(model,
                    first_motor(model->stop_latch_state == DEBUG_UI_STOP_LATCH_UNCONFIRMED ?
                        model->stop_motor_mask : model->completion.motor_mask));
            else if (model->pos_load_flow && model->prepare_return_page == DEBUG_UI_PAGE_POS_LOAD_SETUP) {
                if (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED) {
                    if (model->snapshot.target_motor_id == selected(model)->motor_id)
                        navigate(model, DEBUG_UI_PAGE_POS_LOAD_SETUP);
                    else { model->reason = DEBUG_UI_REASON_NOT_ARMED; notice(model); }
                } else {
                    /* Ownership keeps its existing confirmation; only its return destination is shortened. */
                    model->pos_load_saved_draft = model->draft;
                    model->pos_load_check_boot_id = model->snapshot.boot_id;
                    if (debug_ui_model_begin(model, DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_MODE_UNKNOWN))
                        model->pos_load_check_pending = 1U;
                    else model->draft = model->pos_load_saved_draft;
                }
            }
            else if (!debug_ui_model_begin(model, model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED ?
                     DEBUG_UI_OPERATION_RELEASE : DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_MODE_UNKNOWN) &&
                     model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT &&
                     model->snapshot.target_motor_id == selected(model)->motor_id &&
                     (model->reason == DEBUG_UI_REASON_NOT_READY || model->reason == DEBUG_UI_REASON_NOT_DISABLED ||
                      model->reason == DEBUG_UI_REASON_STALE_FEEDBACK))
                request_stop_with_recovery_target(model, selected(model)->motor_id);
        }
        break;
    case DEBUG_UI_PAGE_MOTORS:
        move_focus(model, direction, DEBUG_UI_MOTOR_COUNT);
        if (model->selected_motor != model->focus && model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
            revoke_static_selection(model);
        model->selected_motor = model->focus;
        if (activation) navigate(model, DEBUG_UI_PAGE_ACTIONS);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_OVERVIEW);
        break;
    case DEBUG_UI_PAGE_DETAIL:
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_ACTIONS);
        break;
    case DEBUG_UI_PAGE_ACTIONS:
        move_focus(model, direction, 6U);
        if (activation) {
            if (model->focus == 0U) navigate(model, DEBUG_UI_PAGE_DETAIL);
            else if (model->focus == 4U) navigate(model, DEBUG_UI_PAGE_MODES);
            else if (model->focus == 5U) navigate(model, DEBUG_UI_PAGE_RECOVERY);
            else (void)debug_ui_model_begin(model, model->focus == 1U ? DEBUG_UI_OPERATION_POS_MOVE :
                (model->focus == 2U ? DEBUG_UI_OPERATION_MIT_MOVE : DEBUG_UI_OPERATION_MIT_HOLD),
                model->focus == 1U ? DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT);
        }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_MOTORS);
        break;
    case DEBUG_UI_PAGE_MODES:
        move_focus(model, direction, 2U);
        if (activation) (void)debug_ui_model_begin(model, DEBUG_UI_OPERATION_SET_MODE,
            model->focus == 0U ? DEBUG_UI_MODE_POS_VEL : DEBUG_UI_MODE_MIT);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_ACTIONS);
        break;
    case DEBUG_UI_PAGE_RECOVERY:
        move_focus(model, direction, 4U);
        if (activation) {
            if (model->focus == 2U) navigate(model, DEBUG_UI_PAGE_REGISTERS);
            else if (model->focus == 3U) {
                model->prepare_return_page = DEBUG_UI_PAGE_RECOVERY;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
            } else (void)debug_ui_model_begin(model, model->focus == 0U ?
                DEBUG_UI_OPERATION_CLEAR_FAULT : DEBUG_UI_OPERATION_DISABLE, DEBUG_UI_MODE_UNKNOWN);
        }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_ACTIONS);
        break;
    case DEBUG_UI_PAGE_REGISTERS:
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_RECOVERY);
        break;
    case DEBUG_UI_PAGE_EDIT:
        move_focus(model, direction, (uint8_t)(debug_ui_model_parameter_count(model) + 1U));
        if (activation) {
            if (model->focus == debug_ui_model_parameter_count(model)) enter_review(model);
            else {
                model->edit_field = debug_ui_model_parameter_field(model, model->focus);
                model->number_backup = model->draft;
                navigate(model, DEBUG_UI_PAGE_NUMBER);
            }
        }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, model->draft.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ?
            DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS : (model->draft.operation == DEBUG_UI_OPERATION_STATIC_HOLD ?
            DEBUG_UI_PAGE_STATIC_ACTIONS : DEBUG_UI_PAGE_ACTIONS));
        break;
    case DEBUG_UI_PAGE_NUMBER:
        if (direction) edit_value(model, -direction);
        if (event->key == DEBUG_UI_KEY_LEFT) {
            model->draft = model->number_backup;
            navigate(model, DEBUG_UI_PAGE_EDIT);
        } else if (event->key == DEBUG_UI_KEY_CENTER) navigate(model, DEBUG_UI_PAGE_EDIT);
        break;
    case DEBUG_UI_PAGE_DIAGNOSTIC_MENU:
        move_focus(model, direction, 3U);
        if (activation) { model->diagnostic_page = model->focus; open_diagnostics(model, DEBUG_UI_PAGE_DIAGNOSTIC_MENU); }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, model->diagnostic_return_page);
        break;
    case DEBUG_UI_PAGE_DIAGNOSTICS:
        if (direction || event->key == DEBUG_UI_KEY_RIGHT) {
            if (model->diagnostic_page >= 3U) model->diagnostic_page = 0U;
            model->diagnostic_page = (uint8_t)((model->diagnostic_page + (direction < 0 ? 2U : 1U)) % 3U);
        }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, model->diagnostic_parent_page);
        break;
    case DEBUG_UI_PAGE_STATIC_HUB:
        move_focus(model, direction, 3U);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_OVERVIEW);
        else if (activation && model->focus == 2U) pos_load_setup(model);
        else if (activation && model->focus == 1U) {
            model->pos_load_flow = 0U;
            model->single_static_flow = 0U;
            navigate(model, DEBUG_UI_PAGE_STATIC_LOAD);
        } else if (activation) {
            model->pos_load_flow = 0U;
            revoke_static_selection(model);
            model->single_static_flow = 1U;
            model->single_static_ff_centi = model->single_static_ff_negative = model->single_static_ff_editing = 0U;
            model->single_static_configured = model->single_static_config_cancelled = 0U;
            model->single_static_wait_snapshot = model->single_static_lock_then_configure = 0U;
            model->single_static_pending_motor = model->selected_motor;
            model->single_static_pending_seconds = model->single_static_seconds;
            navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_SETUP);
        }
        break;
    case DEBUG_UI_PAGE_STATIC_LOAD:
        if (activation) navigate(model, DEBUG_UI_PAGE_STATIC_ACTIONS);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_STATIC_HUB);
        break;
    case DEBUG_UI_PAGE_STATIC_ACTIONS:
        move_focus(model, direction, 6U);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_STATIC_LOAD);
        if (activation) {
            if (model->focus == 0U) {
                model->prepare_return_page = DEBUG_UI_PAGE_STATIC_ACTIONS;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
            } else if (model->focus == 1U) {
                if (debug_ui_model_begin(model, DEBUG_UI_OPERATION_STATIC_HOLD, DEBUG_UI_MODE_MIT))
                    navigate(model, DEBUG_UI_PAGE_STATIC_ACTIONS);
            } else if (model->focus == 2U || model->focus == 3U) {
                model->reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_STATIC_HOLD);
                if (model->reason == DEBUG_UI_REASON_NONE && model->draft.operation != DEBUG_UI_OPERATION_STATIC_HOLD)
                    model->reason = DEBUG_UI_REASON_OLD_EPOCH;
                if (model->reason == DEBUG_UI_REASON_NONE) model->reason = static_request_valid(model, &model->draft);
                if (model->reason != DEBUG_UI_REASON_NONE) notice(model);
                else if (model->focus == 2U) { navigate(model, DEBUG_UI_PAGE_EDIT); model->focus = 0U; model->list_first = 0U; }
                else enter_review(model);
            } else if (model->focus == 4U) request_stop(model);
            else { model->static_result_retained = 1U; navigate(model, DEBUG_UI_PAGE_STATIC_RESULT); }
        }
        break;
    case DEBUG_UI_PAGE_STATIC_RESULT:
    case DEBUG_UI_PAGE_SINGLE_STATIC_RESULT:
        if (direction < 0) model->static_result_detail = (uint8_t)((model->static_result_detail + 2U) % 3U);
        if (direction > 0) model->static_result_detail = (uint8_t)((model->static_result_detail + 1U) % 3U);
        if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER) {
            model->static_result_retained = 1U;
            navigate(model, model->page == DEBUG_UI_PAGE_SINGLE_STATIC_RESULT ?
                DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS : DEBUG_UI_PAGE_STATIC_ACTIONS);
        }
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_LOAD:
        if (activation) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_DIAGNOSTIC_MENU);
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_IDS:
        move_focus(model, direction, DEBUG_UI_MOTOR_COUNT);
        if (model->selected_motor != model->focus) {
            model->selected_motor = model->focus;
            revoke_static_selection(model);
        }
        if (activation || event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS);
        break;
    case DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS:
        move_focus(model, direction, 7U);
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_LOAD);
        if (activation) {
            if (model->focus == 0U) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_IDS);
            else if (model->focus == 1U) {
                model->prepare_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
            } else if (model->focus == 2U) {
                if (debug_ui_model_begin(model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT))
                    navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS);
            } else if (model->focus == 3U || model->focus == 4U) {
                model->reason = debug_ui_model_gate(model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD);
                if (model->reason == DEBUG_UI_REASON_NONE && model->draft.operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD)
                    model->reason = DEBUG_UI_REASON_OLD_EPOCH;
                if (model->reason == DEBUG_UI_REASON_NONE) model->reason = static_request_valid(model, &model->draft);
                if (model->reason != DEBUG_UI_REASON_NONE) notice(model);
                else if (model->focus == 3U) { navigate(model, DEBUG_UI_PAGE_EDIT); model->focus = model->list_first = 0U; }
                else enter_review(model);
            } else if (model->focus == 5U) request_stop(model);
            else { model->static_result_retained = 1U; navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RESULT); }
        }
        break;
    case DEBUG_UI_PAGE_NOTICE:
        if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER)
            navigate(model, model->notice_return_page);
        break;
    case DEBUG_UI_PAGE_RESULT:
    case DEBUG_UI_PAGE_FAULT:
        if (event->key == DEBUG_UI_KEY_RIGHT) {
            model->diagnostic_return_page = model->page;
            open_diagnostics(model, model->page);
        }
        if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER) {
            uint8_t unconfirmed = debug_ui_model_result_requires_check(model);
            if (unconfirmed && event->key == DEBUG_UI_KEY_CENTER) {
                model->prepare_return_page = DEBUG_UI_PAGE_ACTIONS;
                navigate(model, DEBUG_UI_PAGE_PREPARE);
            } else navigate(model, DEBUG_UI_PAGE_ACTIONS);
            if (!unconfirmed) {
                model->completion_seen = 0U;
                model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
            }
        }
        break;
    default: break;
    }
    if (event->key == DEBUG_UI_KEY_CENTER) model->center_consumed = 1U;
}

/** @brief Consume independent STOP before the ordinary one-shot request slot. */
uint8_t debug_ui_model_take_request(DebugUiModel *model, DebugUiRequest *request)
{
    if (model == NULL || request == NULL) return 0U;
    if (model->stop_ready) { *request = model->outgoing_stop; model->stop_ready = 0U; return 1U; }
    if (model->outgoing_ready) { *request = model->outgoing; model->outgoing_ready = 0U; return 1U; }
    return 0U;
}

/** @brief Separate result-less STOP latching from ordinary submission refusal. */
void debug_ui_model_submit_failed(DebugUiModel *model, const DebugUiRequest *request,
                                  DebugUiReason reason)
{
    if (model == NULL || request == NULL) return;
    debug_ui_load_trials_refused(&model->load_trials, request, reason);
    model->reason = reason;
    if (request->operation == DEBUG_UI_OPERATION_STOP && reason == DEBUG_UI_REASON_STOP_LATCHED &&
        same_identity(&request->identity, &model->outgoing_stop.identity)) {
        model->stop_waiting = 0U; /* This identity has no admission or terminal. */
        model->stop_latch_state = DEBUG_UI_STOP_LATCH_WAITING;
        model->page = DEBUG_UI_PAGE_RUNNING;
        return;
    }
    if (model->single_static_flow && single_lifecycle_operation(request->operation) &&
        same_identity(&request->identity, &model->submitted.identity)) {
        uint8_t phase = model->single_static_preflight;
        model->awaiting_result = model->single_static_wait_snapshot = 0U;
        model->single_static_preflight = 0U;
        if (request->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK || phase == 2U) model->page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
        else { notice(model); model->notice_return_page = phase >= 3U ? DEBUG_UI_PAGE_SINGLE_STATIC_READY : DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; }
        return;
    }
    if (request->operation != DEBUG_UI_OPERATION_STOP) model->awaiting_result = 0U;
    else { model->stop_waiting = 0U; model->stop_requested = 0U; }
    model->page = DEBUG_UI_PAGE_FAULT;
}

/** @brief Record admission separately from a matching terminal result. */
void debug_ui_model_admission(DebugUiModel *model, const DebugUiAdmission *admission)
{
    if (model == NULL || admission == NULL || admission->operation != model->submitted.operation ||
        !same_identity(&admission->identity, &model->submitted.identity)) return;
    model->admission_seen = admission->accepted;
    model->reason = admission->reason;
}

/** @brief Match complete identity and preserve outstanding STOP cleanup. */
void debug_ui_model_completion(DebugUiModel *model, const DebugUiCompletion *completion)
{
    uint8_t submitted_match, stop_match;
    if (model == NULL || completion == NULL) return;
    submitted_match = same_identity(&completion->identity, &model->submitted.identity);
    /* Lifecycle terminals are consumed once; duplicate ACKs cannot reopen a finished flow. */
    if (submitted_match && (single_lifecycle_operation(model->submitted.operation) ||
        (model->pos_load_flow && model->submitted.operation == DEBUG_UI_OPERATION_ACQUIRE)) && !model->awaiting_result)
        submitted_match = 0U;
    stop_match = (uint8_t)(same_identity(&completion->identity, &model->outgoing_stop.identity) &&
        completion->operation == DEBUG_UI_OPERATION_STOP);
    if (submitted_match && (model->submitted.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST || debug_ui_is_static_operation(model->submitted.operation) ||
        single_lifecycle_operation(model->submitted.operation) ||
        ((model->single_static_flow || model->pos_load_flow) && model->submitted.operation == DEBUG_UI_OPERATION_ACQUIRE))) {
        uint8_t expected_mask = model->submitted.operation == DEBUG_UI_OPERATION_STATIC_HOLD ? 0x7FU :
            (model->submitted.target_motor_id > 0U && model->submitted.target_motor_id <= DEBUG_UI_MOTOR_COUNT ?
             (uint8_t)(1U << (model->submitted.target_motor_id - 1U)) : 0U);
        /* ACQUIRE historically reports no motor execution scope; configuration/HOLD report their selected mask. */
        if (model->submitted.operation == DEBUG_UI_OPERATION_ACQUIRE) expected_mask = 0U;
        if (completion->operation != model->submitted.operation || completion->motor_mask != expected_mask)
            submitted_match = 0U;
    }
    if (!submitted_match && !stop_match) return;
    debug_ui_load_trials_completion(&model->load_trials, completion, &model->outgoing_stop);
    if (submitted_match && model->pos_load_flow && model->submitted.operation == DEBUG_UI_OPERATION_ACQUIRE &&
        model->pos_load_check_pending == 2U) {
        /* A cancelled ownership receipt may retire its wait, but never replace the STOP result or reopen setup. */
        model->awaiting_result = model->pos_load_check_pending = 0U;
        model->page = model->stop_requested || model->stop_waiting || model->snapshot.stop_pending ||
            model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING ? DEBUG_UI_PAGE_RUNNING : settled_page(model);
        return;
    }
    if (submitted_match && model->single_static_flow && model->single_static_lifecycle_cancelled &&
        (single_lifecycle_operation(completion->operation) || completion->operation == DEBUG_UI_OPERATION_ACQUIRE)) {
        /* Preserve the STOP receipt in either delivery order and never reopen a cancelled synchronization. */
        model->awaiting_result = model->single_static_wait_snapshot = 0U;
        model->page = model->stop_requested || model->stop_waiting ||
            model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING ? DEBUG_UI_PAGE_RUNNING : settled_page(model);
        return;
    }
    model->completion = *completion;
    model->completion_seen = 1U;
    if (submitted_match) model->awaiting_result = 0U;
    if (stop_match) {
        model->stop_waiting = 0U;
        model->stop_requested = 0U;
        if (completion->disabled_confirmed)
            model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
    }
    model->reason = completion->reason;
    if (submitted_match && model->pos_load_flow && model->submitted.operation == DEBUG_UI_OPERATION_ACQUIRE) {
        uint8_t check_pending = model->pos_load_check_pending;
        model->pos_load_check_pending = 0U;
        if (check_pending == 1U && completion->code == DEBUG_UI_COMPLETED && completion->reason == DEBUG_UI_REASON_NONE &&
            model->prepare_return_page == DEBUG_UI_PAGE_POS_LOAD_SETUP &&
            model->pos_load_check_boot_id == model->snapshot.boot_id &&
            (model->submitted.identity.epoch == model->snapshot.epoch ||
             (model->snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_ARMED &&
              model->snapshot.target_motor_id == model->submitted.target_motor_id &&
              model->snapshot.epoch == (model->submitted.identity.epoch == UINT32_MAX ?
                  1U : model->submitted.identity.epoch + 1U))) &&
            model->pos_load_saved_draft.operation == DEBUG_UI_OPERATION_POS_LOAD_TEST &&
            model->pos_load_saved_draft.target_motor_id == selected(model)->motor_id &&
            model->pos_load_saved_draft.reference_generation == selected(model)->reference_generation &&
            selected(model)->mode_verified && selected(model)->actual_mode == DEBUG_UI_MODE_POS_VEL &&
            !model->snapshot.active && !model->snapshot.pending && !model->snapshot.unconfirmed_disable_mask &&
            !model->snapshot.arm_fault && !model->stop_requested && !model->stop_waiting && !model->snapshot.stop_pending &&
            model->stop_latch_state == DEBUG_UI_STOP_LATCH_NONE) {
            model->draft = model->pos_load_saved_draft;
            model->completion_seen = 0U;
            navigate(model, DEBUG_UI_PAGE_POS_LOAD_SETUP);
            return;
        }
    }
    if (submitted_match && model->single_static_flow && completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT) {
        if (completion->code != DEBUG_UI_COMPLETED || completion->reason != DEBUG_UI_REASON_NONE ||
            !completion->disabled_confirmed || completion->stage != DEBUG_UI_STAGE_DISABLE ||
            completion->feedback_timestamp_us <= model->submitted.created_at_us) {
            uint8_t phase = model->single_static_preflight;
            model->single_static_preflight = 0U;
            model->static_locked = 0U;
            if (model->reason == DEBUG_UI_REASON_NONE) model->reason = DEBUG_UI_REASON_NOT_DISABLED;
            if (phase == 2U) navigate(model, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK);
            else { notice(model); model->notice_return_page = phase == 1U ? DEBUG_UI_PAGE_SINGLE_STATIC_SETUP : DEBUG_UI_PAGE_SINGLE_STATIC_READY; }
        } else model->page = DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT;
        return;
    }
    if (submitted_match && model->single_static_flow && single_lifecycle_operation(completion->operation)) {
        if (model->single_static_config_cancelled && completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE) {
            model->single_static_wait_snapshot = model->single_static_configured = 0U;
            model->page = DEBUG_UI_PAGE_RUNNING;
            return;
        }
        if (completion->code == DEBUG_UI_COMPLETED && completion->reason == DEBUG_UI_REASON_NONE) {
            model->single_static_wait_snapshot = completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ? 1U : 2U;
            model->page = completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE ?
                DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW : DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
        } else {
            model->single_static_wait_snapshot = 0U;
            if (completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_LOCK) model->page = DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK;
            else { notice(model); model->notice_return_page = DEBUG_UI_PAGE_SINGLE_STATIC_SETUP; }
        }
        if (model->stop_waiting || model->stop_requested) model->page = DEBUG_UI_PAGE_RUNNING;
        return;
    }
    if (submitted_match && model->single_static_flow && completion->operation == DEBUG_UI_OPERATION_ACQUIRE &&
        completion->code == DEBUG_UI_COMPLETED && completion->reason == DEBUG_UI_REASON_NONE &&
        !model->stop_requested && !model->stop_waiting && model->stop_latch_state != DEBUG_UI_STOP_LATCH_WAITING) {
        model->single_static_snapshot_generation = model->snapshot.generation;
        model->single_static_wait_snapshot = 3U;
        model->page = DEBUG_UI_PAGE_SINGLE_STATIC_READY;
        return;
    }
    model->page = settled_page(model);
    if (model->awaiting_result || model->stop_waiting || model->stop_latch_state == DEBUG_UI_STOP_LATCH_WAITING)
        model->page = DEBUG_UI_PAGE_RUNNING;
}

/** @brief Convert internal output-shaft radians for display only. */
float debug_ui_radians_to_degrees(float radians) { return radians * (180.0f / 3.14159265358979323846f); }
/** @brief Convert edited output-shaft degrees into internal radians. */
float debug_ui_degrees_to_radians(float degrees) { return degrees * (3.14159265358979323846f / 180.0f); }

/** @brief Render unknown as -- and finite stale feedback with an explicit marker. */
void debug_ui_format_value(char *buffer, size_t capacity, float value,
                           uint8_t seen, uint8_t valid, const char *unit)
{
    if (buffer == NULL || capacity == 0U) return;
    if (unit == NULL) unit = "";
    if (!seen || !finite_value(value)) (void)snprintf(buffer, capacity, "-- %s", unit);
    else (void)snprintf(buffer, capacity, "%s%.1f %s", valid ? "" : "过期 ", (double)value, unit);
    buffer[capacity - 1U] = '\0';
}

/** @brief Return the static Chinese explanation for a bounded gate reason. */
const char *debug_ui_reason_text(DebugUiReason reason)
{
    static const char *const texts[] = { "就绪", "动作忙", "只读配置", "参数无效",
        "参数未标定", "请求过期", "参考已变化", "未获本地控制", "反馈过期",
        "尚未失能", "输入或显示异常", "超出范围", "结果待处理", "重复请求", "尚未就绪", "停止已锁存",
        "转矩超限", "速度超限", "温度超限", "反馈异常" };
    return (unsigned)reason < sizeof(texts) / sizeof(texts[0]) ? texts[reason] : "未知原因";
}
