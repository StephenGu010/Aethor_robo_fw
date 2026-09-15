/** @file debug_ui_model.c
 * @brief Pure C UI state machine; no motor, queue, HAL, RTOS or LVGL access.
 * Display previews are advisory. The control owner recomputes the relative
 * target against fresh feedback and revalidates every submitted operation.
 */
#include "debug_ui_model.h"
#include <float.h>
#include <stdio.h>
#include <string.h>

/** @brief Portable finite check for ARMCC5 and host C without fast-math. */
static uint8_t finite_value(float value)
{
    return (uint8_t)((value == value) && value <= FLT_MAX && value >= -FLT_MAX);
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
                     operation == DEBUG_UI_OPERATION_MIT_MOVE);
}
/** @brief Return editable parameter count, excluding preview. */
uint8_t debug_ui_model_parameter_count(const DebugUiModel *model)
{
    if (model == NULL) return 0U;
    if (model->draft.operation == DEBUG_UI_OPERATION_POS_MOVE) return 2U;
    if (model->draft.operation == DEBUG_UI_OPERATION_MIT_MOVE) return 5U;
    return model->draft.operation == DEBUG_UI_OPERATION_MIT_HOLD ? 3U : 0U;
}
/** @brief Map visible parameter row to the stable request field. */
uint8_t debug_ui_model_parameter_field(const DebugUiModel *model, uint8_t index)
{
    if (index >= debug_ui_model_parameter_count(model)) return 255U;
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
    if (page == DEBUG_UI_PAGE_MOTORS) {
        model->focus = model->selected_motor;
        if (model->focus < model->list_first) model->list_first = model->focus;
        if (model->focus >= model->list_first + DEBUG_UI_VISIBLE_MOTORS)
            model->list_first = (uint8_t)(model->focus + 1U - DEBUG_UI_VISIBLE_MOTORS);
    }
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
    DebugUiPage parent = debug_ui_model_parameter_count(model) != 0U ?
        DEBUG_UI_PAGE_ACTIONS : model->review_return_page;
    notice(model);
    model->notice_return_page = parent;
    model->review_pressed = model->review_released = 0U;
    model->confirm_elapsed_ms = 0U;
}
/** @brief Declare shared confirmation before draft creation. */
static void enter_review(DebugUiModel *model);
/** @brief Return a value-copy motor only if the selected row exists. */
static const DebugUiMotorView *selected(const DebugUiModel *model)
{
    return &model->snapshot.motors[model->selected_motor < DEBUG_UI_MOTOR_COUNT ?
                                  model->selected_motor : 0U];
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
    if (!model->snapshot.motion_enabled) {
        model->reason = DEBUG_UI_REASON_DISABLED;
        return;
    }
    if (model->stop_requested) return;
    target = debug_ui_model_active_target(model);
    if (target == 0U && recovery_target > 0U && recovery_target <= DEBUG_UI_MOTOR_COUNT)
        target = recovery_target;
    /* Never introduce an unrelated selected motor into a remote STOP mask. */
    if (target == 0U) { model->reason = DEBUG_UI_REASON_NOT_READY; return; }
    if (model->outgoing_ready) {
        model->outgoing_ready = 0U;
        model->awaiting_result = 0U; /* Unsent motion cannot follow its STOP. */
    }
    memset(&model->outgoing_stop, 0, sizeof(model->outgoing_stop));
    model->outgoing_stop.operation = DEBUG_UI_OPERATION_STOP;
    model->outgoing_stop.target_motor_id = target;
    model->stop_motor_mask = (uint8_t)((model->snapshot.active_motor_mask & 0x7FU) |
        model->snapshot.unconfirmed_disable_mask | (1U << (target - 1U)));
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

/** @brief Reset to a locked overview without inferring any feedback. */
void debug_ui_model_init(DebugUiModel *model)
{
    if (model == NULL) return;
    memset(model, 0, sizeof(*model));
    model->page = DEBUG_UI_PAGE_OVERVIEW;
    model->reason = DEBUG_UI_REASON_NOT_READY;
    model->next_request_id = 1U;
}

/** @brief Check compile-time, ownership, health and commissioned profile gates. */
DebugUiReason debug_ui_model_gate(const DebugUiModel *model, DebugUiOperation operation)
{
#if AETHOR_DEBUG_UI_ALLOW_MOTION
    const DebugUiSnapshot *snapshot;
    const DebugUiMotorView *motor;
    uint8_t deferred_pos_feedback;
#endif
    if (model == NULL) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if ((unsigned)operation > DEBUG_UI_OPERATION_STOP) return DEBUG_UI_REASON_INVALID_ARGUMENT;
    if (operation == DEBUG_UI_OPERATION_STOP)
        return AETHOR_DEBUG_UI_ALLOW_MOTION && model->snapshot.motion_enabled ? DEBUG_UI_REASON_NONE : DEBUG_UI_REASON_DISABLED;
#if !AETHOR_DEBUG_UI_ALLOW_MOTION
    return DEBUG_UI_REASON_DISABLED;
#else
    snapshot = &model->snapshot;
    motor = selected(model);
    if (!model->snapshot_seen || model->now_us < snapshot->timestamp_us ||
        model->now_us - snapshot->timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US)
        return DEBUG_UI_REASON_STALE_FEEDBACK;
    if (!snapshot->motion_enabled || !snapshot->bench_profile) return DEBUG_UI_REASON_DISABLED;
    if (is_mit(operation) && (!AETHOR_DEBUG_UI_ALLOW_MIT || !snapshot->mit_enabled))
        return DEBUG_UI_REASON_DISABLED;
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
    /* Intent only: each motion/mode change obtains a new disabled ACK in the executor. */
    deferred_pos_feedback = (uint8_t)((motor->actual_mode == DEBUG_UI_MODE_POS_VEL ||
        motor->actual_mode == DEBUG_UI_MODE_MIT) &&
        (operation == DEBUG_UI_OPERATION_POS_MOVE || is_mit(operation) ||
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

/** @brief Check edited parameters again against the latest copied feedback. */
static DebugUiReason validate_request(const DebugUiModel *model, const DebugUiRequest *request)
{
    const DebugUiMotorView *motor = selected(model);
    DebugUiMotorProfile bounds = debug_ui_profile_bounds(&motor->profile, is_mit(request->operation));
    const DebugUiMotorProfile *profile = &bounds;
    DebugUiReason gate = debug_ui_model_gate(model, request->operation);
    float target;
    if (gate != DEBUG_UI_REASON_NONE) return gate;
    if (request->operation == DEBUG_UI_OPERATION_SET_MODE) {
        if (request->requested_mode != DEBUG_UI_MODE_MIT && request->requested_mode != DEBUG_UI_MODE_POS_VEL)
            return DEBUG_UI_REASON_INVALID_ARGUMENT;
        if (request->requested_mode == DEBUG_UI_MODE_MIT &&
            (!AETHOR_DEBUG_UI_ALLOW_MIT || !model->snapshot.mit_enabled || !profile_valid(profile, 1U)))
            return DEBUG_UI_REASON_UNCALIBRATED;
    }
    if (request->operation != DEBUG_UI_OPERATION_POS_MOVE && !is_mit(request->operation))
        return DEBUG_UI_REASON_NONE;
    if (!finite_value(request->delta_rad) || !finite_value(request->speed_rad_s) ||
        !finite_value(request->torque_ff_nm) || request->torque_ff_nm != 0.0f)
        return DEBUG_UI_REASON_INVALID_ARGUMENT;
    target = motor->position_rad + request->delta_rad;
    if (request->delta_rad < -profile->max_delta_rad || request->delta_rad > profile->max_delta_rad ||
        request->speed_rad_s <= 0.0f || request->speed_rad_s > profile->max_speed_rad_s)
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
    } else if (model->snapshot_seen && now_us - model->snapshot.timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US) {
        unsigned index;
        for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
            model->snapshot.motors[index].feedback_valid = 0U;
    }
    editing = (uint8_t)(model->page == DEBUG_UI_PAGE_EDIT || model->page == DEBUG_UI_PAGE_NUMBER || model->page == DEBUG_UI_PAGE_REVIEW);
    if (editing) {
        DebugUiReason reason = debug_ui_model_gate(model, model->draft.operation);
        if (model->edit_epoch != model->snapshot.epoch ||
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
        model->page = DEBUG_UI_PAGE_RUNNING;
        if ((!input_valid || !display_valid) &&
            (model->snapshot.active_identity.origin == DEBUG_UI_ORIGIN_LOCAL_UI || model->awaiting_result))
            request_stop(model);
    } else if (model->page == DEBUG_UI_PAGE_RUNNING) {
        model->page = settled_page(model);
        model->stop_requested = 0U;
    }
}

/** @brief Create a bounded draft after gates; never publish from this function. */
uint8_t debug_ui_model_begin(DebugUiModel *model, DebugUiOperation operation,
                             DebugUiMode requested_mode)
{
    DebugUiMotorProfile bounds;
    const DebugUiMotorProfile *profile;
    if (model == NULL || (unsigned)operation > DEBUG_UI_OPERATION_DISABLE) return 0U;
    model->reason = debug_ui_model_gate(model, operation);
    if (model->reason != DEBUG_UI_REASON_NONE) { notice(model); return 0U; }
    bounds = debug_ui_profile_bounds(&selected(model)->profile, is_mit(operation));
    profile = &bounds;
    memset(&model->draft, 0, sizeof(model->draft));
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
        int32_t duration = (int32_t)model->draft.hold_duration_ms + direction * 100;
        if (duration < (int32_t)profile->hold_min_ms) duration = (int32_t)profile->hold_min_ms;
        if (duration > (int32_t)profile->hold_max_ms) duration = (int32_t)profile->hold_max_ms;
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
    navigate(model, DEBUG_UI_PAGE_REVIEW);
    model->review_released = model->review_pressed = 0U;
    model->confirm_elapsed_ms = 0U;
    model->center_consumed = 1U;
}
/** @brief Commit the immutable reviewed action exactly once. */
static void submit_review(DebugUiModel *model)
{
    if (model->edit_epoch != model->snapshot.epoch ||
        model->edit_reference != selected(model)->reference_generation ||
        model->reviewed.target_motor_id != selected(model)->motor_id) {
        model->reason = DEBUG_UI_REASON_OLD_EPOCH;
        model->review_pressed = model->review_released = 0U;
        model->confirm_elapsed_ms = 0U;
        revoke_draft(model);
        return;
    }
    model->reason = validate_request(model, &model->reviewed);
    if (model->reason != DEBUG_UI_REASON_NONE) return;
    model->submitted = model->reviewed;
    identify(model, &model->submitted);
    model->outgoing = model->submitted;
    model->outgoing_ready = model->awaiting_result = 1U;
    model->admission_seen = model->completion_seen = 0U;
    model->center_consumed = 1U;
    model->page = DEBUG_UI_PAGE_RUNNING;
}

/** @brief Dispatch one input event with running STOP ahead of navigation gates. */
void debug_ui_model_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    uint8_t activation;
    int direction;
    if (model == NULL || event == NULL) return;
    if (event->type == DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP) {
        if (event->key == DEBUG_UI_KEY_CENTER &&
            (model->snapshot.active || model->snapshot.pending || model->awaiting_result)) request_stop(model);
        return;
    }
    /* A delivered, debounced CENTER edge is a STOP intent even when the input
     * queue reports overflow/invalid health. It cannot arm or submit motion. */
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
    if (model->page == DEBUG_UI_PAGE_REVIEW) {
        if (event->type == DEBUG_UI_INPUT_EVENT_PRESS && event->key == DEBUG_UI_KEY_LEFT) {
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
        if (direction) model->focus = (uint8_t)((model->focus + 3 + direction) % 3);
        if (activation) {
            if (model->focus == 0U) navigate(model, DEBUG_UI_PAGE_MOTORS);
            else if (model->focus == 1U) {
                model->diagnostic_return_page = DEBUG_UI_PAGE_OVERVIEW;
                navigate(model, DEBUG_UI_PAGE_DIAGNOSTIC_MENU);
            } else {
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
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_ACTIONS);
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
        if (activation) {
            model->diagnostic_page = model->focus;
            navigate(model, DEBUG_UI_PAGE_DIAGNOSTICS);
        }
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, model->diagnostic_return_page);
        break;
    case DEBUG_UI_PAGE_DIAGNOSTICS:
        if (event->key == DEBUG_UI_KEY_LEFT) navigate(model, DEBUG_UI_PAGE_DIAGNOSTIC_MENU);
        break;
    case DEBUG_UI_PAGE_NOTICE:
        if (event->key == DEBUG_UI_KEY_LEFT || event->key == DEBUG_UI_KEY_CENTER)
            navigate(model, model->notice_return_page);
        break;
    case DEBUG_UI_PAGE_RESULT:
    case DEBUG_UI_PAGE_FAULT:
        if (event->key == DEBUG_UI_KEY_RIGHT) {
            model->diagnostic_return_page = model->page;
            navigate(model, DEBUG_UI_PAGE_DIAGNOSTIC_MENU);
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
    model->reason = reason;
    if (request->operation == DEBUG_UI_OPERATION_STOP && reason == DEBUG_UI_REASON_STOP_LATCHED &&
        same_identity(&request->identity, &model->outgoing_stop.identity)) {
        model->stop_waiting = 0U; /* This identity has no admission or terminal. */
        model->stop_latch_state = DEBUG_UI_STOP_LATCH_WAITING;
        model->page = DEBUG_UI_PAGE_RUNNING;
        return;
    }
    if (request->operation != DEBUG_UI_OPERATION_STOP) model->awaiting_result = 0U;
    else { model->stop_waiting = 0U; model->stop_requested = 0U; }
    model->page = DEBUG_UI_PAGE_FAULT;
}

/** @brief Record admission separately from a matching terminal result. */
void debug_ui_model_admission(DebugUiModel *model, const DebugUiAdmission *admission)
{
    if (model == NULL || admission == NULL || !same_identity(&admission->identity, &model->submitted.identity)) return;
    model->admission_seen = admission->accepted;
    model->reason = admission->reason;
}

/** @brief Match complete identity and preserve outstanding STOP cleanup. */
void debug_ui_model_completion(DebugUiModel *model, const DebugUiCompletion *completion)
{
    if (model == NULL || completion == NULL ||
        (!same_identity(&completion->identity, &model->submitted.identity) &&
         !same_identity(&completion->identity, &model->outgoing_stop.identity))) return;
    model->completion = *completion;
    model->completion_seen = 1U;
    if (same_identity(&completion->identity, &model->submitted.identity)) model->awaiting_result = 0U;
    if (same_identity(&completion->identity, &model->outgoing_stop.identity)) {
        model->stop_waiting = 0U;
        model->stop_requested = 0U;
        if (completion->disabled_confirmed)
            model->stop_latch_state = DEBUG_UI_STOP_LATCH_NONE;
    }
    model->reason = completion->reason;
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
