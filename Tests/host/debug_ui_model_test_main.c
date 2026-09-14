/** @file debug_ui_model_test_main.c
 * @brief Behavioral tests of gates, release-before-confirm and immediate STOP.
 * All bench values here are synthetic test fixtures, never commissioning data.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "debug_ui_model.h"

/** @brief Build an explicitly simulated, healthy single-motor bench snapshot. */
static DebugUiSnapshot fixture(void)
{
    DebugUiSnapshot snapshot;
    unsigned index;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.timestamp_us = 1000000ULL;
    snapshot.epoch = 9U;
    snapshot.generation = 1U;
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    snapshot.bench_profile = 1U;
    snapshot.motion_enabled = 1U;
    snapshot.mit_enabled = 1U;
    snapshot.health.input_valid = 1U;
    snapshot.health.display_valid = 1U;
    snapshot.health.ui_valid = 1U;
    snapshot.health.protocol_valid = 1U;
    snapshot.health.ui_timestamp_us = snapshot.timestamp_us;
    snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        DebugUiMotorView *motor = &snapshot.motors[index];
        motor->motor_id = (uint8_t)(index + 1U);
        motor->feedback_seen = motor->feedback_valid = 1U;
        motor->identity_verified = motor->mode_verified = motor->ranges_verified = 1U;
        motor->actual_mode = DEBUG_UI_MODE_POS_VEL;
        motor->reference_generation = 3U;
        motor->position_max_rad = 12.5f;
        motor->velocity_max_rad_s = 30.0f;
        motor->position_rad = 0.25f;
        motor->profile.pos_valid = motor->profile.mit_valid = 1U;
        motor->profile.position_min_rad = -1.0f;
        motor->profile.position_max_rad = 1.0f;
        motor->profile.max_delta_rad = 0.08f;
        motor->profile.mit_position_min_rad = -1.0f;
        motor->profile.mit_position_max_rad = 1.0f;
        motor->profile.mit_max_delta_rad = 0.08f;
        motor->profile.mit_max_speed_rad_s = 0.05f;
        motor->profile.max_speed_rad_s = 0.05f;
        motor->profile.kp_min = 1.0f;
        motor->profile.kp_max = 5.0f;
        motor->profile.kd_min = 0.1f;
        motor->profile.kd_max = 1.0f;
        motor->profile.default_kp = 2.0f;
        motor->profile.default_kd = 0.4f;
        motor->profile.hold_min_ms = 100U;
        motor->profile.hold_max_ms = 1000U;
    }
    return snapshot;
}

/** @brief Deliver a real model event, without involving display focus. */
static void key(DebugUiModel *model, DebugUiInputEventType type, DebugUiKey code,
                uint32_t timestamp, uint32_t held)
{
    DebugUiInputEvent event;
    event.type = type; event.key = code;
    event.timestamp_ms = timestamp; event.held_ms = held;
    debug_ui_model_event(model, &event);
}

/** @brief Start with no inferred feedback, and scroll all seven rows using keys. */
static void test_browse_and_units(void)
{
    DebugUiModel model;
    char text[64];
    unsigned index;
    DebugUiSnapshot snapshot = fixture();
    debug_ui_model_init(&model);
    assert(model.page == DEBUG_UI_PAGE_OVERVIEW && !model.snapshot_seen);
    debug_ui_format_value(text, sizeof(text), 0.0f, 0U, 0U, "Nm");
    assert(strstr(text, "--") != NULL);
    debug_ui_format_value(text, sizeof(text), 12.5f, 1U, 0U, "Nm");
    assert(strstr(text, "12.5") != NULL && strstr(text, "过期") != NULL);
    debug_ui_format_value(text, sizeof(text), NAN, 1U, 1U, "Nm");
    assert(strstr(text, "--") != NULL);
    assert(fabsf(debug_ui_radians_to_degrees(3.14159265358979323846f) - 180.0f) < 0.001f);
    assert(fabsf(debug_ui_degrees_to_radians(-180.0f) + 3.14159265358979323846f) < 0.00001f);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_MOTORS);
    for (index = 0U; index < 8U; ++index)
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_DOWN, 1001U + index, 0U);
    assert(model.selected_motor == 6U && model.list_first == 2U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1010U, 0U);
    assert(model.page == DEBUG_UI_PAGE_DETAIL);
}

/** @brief Enforce two distinct CENTER presses, 800 ms hold and frozen drafts. */
static void test_review_once_and_identity(void)
{
    DebugUiModel model;
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiSnapshot snapshot = fixture();
    float frozen_delta;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    frozen_delta = model.draft.delta_rad;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 4000U, 3000U);
    assert(!debug_ui_model_take_request(&model, &request));
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 4001U, 3001U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 4010U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 4809U, 799U);
    assert(!debug_ui_model_take_request(&model, &request));
    model.draft.delta_rad = 0.07f;
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 4810U, 800U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_MOVE);
    assert(request.delta_rad == frozen_delta && request.identity.epoch == 9U);
    assert(model.submitted.delta_rad == frozen_delta);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 9000U, 4990U);
    assert(!debug_ui_model_take_request(&model, &request));
    memset(&admission, 0, sizeof(admission));
    admission.identity = model.submitted.identity; admission.accepted = 1U;
    debug_ui_model_admission(&model, &admission);
    assert(model.page == DEBUG_UI_PAGE_RUNNING && model.awaiting_result);
    memset(&completion, 0, sizeof(completion));
    completion.identity = model.submitted.identity;
    completion.identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_completion(&model, &completion);
    assert(!model.completion_seen);
    completion.identity = model.submitted.identity;
    completion.disabled_confirmed = 1U;
    debug_ui_model_completion(&model, &completion);
    assert(model.page == DEBUG_UI_PAGE_RESULT && !model.awaiting_result);
}

/** @brief STOP ignores screen focus and uses its own slot during remote activity. */
static void test_running_stop_and_health(void)
{
    DebugUiModel model;
    DebugUiRequest request;
    DebugUiSnapshot snapshot = fixture();
    snapshot.active = 1U; snapshot.target_motor_id = 4U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_DIAGNOSTICS;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 4U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 3000U, 2000U);
    assert(!debug_ui_model_take_request(&model, &request));
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 0U, 1U);
    assert(!debug_ui_model_take_request(&model, &request)); /* remote health is independent */
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_LOCAL_UI;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 0U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP);
}

/** @brief Reject stale, uncalibrated, unsafe or changed-reference edits. */
static void test_gates_and_cancellation(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    debug_ui_model_init(&model);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    snapshot.motors[0].profile.pos_valid = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    assert(model.reason == DEBUG_UI_REASON_UNCALIBRATED);
    snapshot = fixture(); snapshot.motors[0].feedback_valid = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_MODE_MIT));
    assert(!model.outgoing_ready); /* Stale intent alone cannot actuate. */
    snapshot = fixture(); snapshot.motors[0].profile.default_kp = NAN;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_MIT_HOLD, DEBUG_UI_MODE_MIT));
    snapshot = fixture();
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_MIT_MOVE, DEBUG_UI_MODE_MIT));
    snapshot.motors[0].reference_generation++;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_DETAIL && !model.outgoing_ready);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    debug_ui_model_update(&model, NULL, snapshot.timestamp_us + 150001ULL, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_DETAIL);
}

/** @brief Observe remote completion and allow STOP on a subsequent remote action. */
static void test_remote_lifecycle_and_stop_terminal(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    DebugUiCompletion completion;
    debug_ui_model_init(&model);
    snapshot.active = 1U; snapshot.target_motor_id = 2U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_RUNNING);
    snapshot.active = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_DETAIL); /* do not invent remote completion */
    snapshot.active = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    memset(&completion, 0, sizeof(completion));
    completion.identity = request.identity;
    completion.operation = DEBUG_UI_OPERATION_STOP;
    completion.code = DEBUG_UI_STOPPED;
    completion.disabled_confirmed = 1U;
    snapshot.active = 0U;
    debug_ui_model_completion(&model, &completion);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1010U, 10U);
    snapshot.active = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1100U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP);
}

/** @brief A debounced STOP survives invalid input health; HOLD is continuous. */
static void test_invalid_stop_and_interrupted_hold(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    debug_ui_model_init(&model);
    snapshot.active = 1U; snapshot.target_motor_id = 3U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 0U, 1U);
    model.center_down = model.center_consumed = 1U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP);
    debug_ui_model_init(&model);
    key(&model, DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(!debug_ui_model_take_request(&model, &request));
    snapshot.active = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 0U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP, DEBUG_UI_KEY_CENTER, 1001U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP);
    debug_ui_model_init(&model);
    snapshot = fixture();
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1010U, 10U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1020U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1770U, 750U);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1900U, 100U);
    assert(!debug_ui_model_take_request(&model, &request));
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2500U, 700U);
    assert(!debug_ui_model_take_request(&model, &request));
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 2600U, 800U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_MOVE);
}

/** @brief M1 read-only cannot publish STOP even when a remote motor is active. */
static void test_readonly_stop_disabled(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    snapshot.motion_enabled = 0U;
    snapshot.active = 1U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    assert(!debug_ui_model_take_request(&model, &request));
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_STOP) == DEBUG_UI_REASON_DISABLED);
}

/** @brief After cleanup, an explicit local re-acquire matches the application gate. */
static void test_reacquire_after_local_fault(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_MODE_UNKNOWN));
    assert(model.page == DEBUG_UI_PAGE_EDIT && !model.outgoing_ready);
    snapshot.retained_result_count = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(!debug_ui_model_begin(&model, DEBUG_UI_OPERATION_ACQUIRE, DEBUG_UI_MODE_UNKNOWN));
    assert(model.reason == DEBUG_UI_REASON_RESULT_BACKPRESSURE);
}

/** @brief Rechecking cleared gates enters review without immediately acquiring control. */
static void test_prepare_recheck_requires_review(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    debug_ui_model_init(&model);
    snapshot.motors[0].profile.pos_valid = 0U;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.focus = 2U;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_PREPARE);
    assert(!debug_ui_model_take_request(&model, &request));
    snapshot = fixture();
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    assert(!debug_ui_model_take_request(&model, &request));
}

/** @brief Accept the S3519 zero-speed quantization bins while retaining the executor's 0.05 rad/s limit. */
static void test_stationary_feedback_matches_executor_gate(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    const float stationary_samples[] = {-200.0f / 4095.0f, 200.0f / 4095.0f, -0.05f, 0.05f};
    unsigned sample_index;
    debug_ui_model_init(&model);
    model.selected_motor = 6U;
    snapshot.motors[6].velocity_max_rad_s = 200.0f;
    for (sample_index = 0U; sample_index < sizeof(stationary_samples) / sizeof(stationary_samples[0]); ++sample_index) {
        snapshot.motors[6].velocity_rad_s = stationary_samples[sample_index];
        snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_ACQUIRE) == DEBUG_UI_REASON_NONE);
        snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_MOVE) == DEBUG_UI_REASON_NONE);
    }
    snapshot.motors[6].velocity_rad_s = 0.0501f;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_MOVE) == DEBUG_UI_REASON_NOT_READY);
    snapshot.motors[6].velocity_rad_s = -0.0501f;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_MOVE) == DEBUG_UI_REASON_NOT_READY);
    snapshot.motors[6].velocity_rad_s = 200.0f / 4095.0f;
    snapshot.motors[6].enabled = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_POS_MOVE) == DEBUG_UI_REASON_NOT_DISABLED);
    snapshot.motors[6].enabled = 0U;
    snapshot.motors[6].feedback_valid = 0U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_MIT_MOVE) == DEBUG_UI_REASON_NONE);
}

/** @brief POS intent survives an 800 ms review; stale samples cannot unlock other actions. */
static void test_pos_review_defers_feedback_acquisition(DebugUiOperation operation)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned step;
    debug_ui_model_init(&model);
    snapshot.motors[0].feedback_valid = 0U;
    snapshot.motors[0].feedback_age_ms = 1000U;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_ACQUIRE) == DEBUG_UI_REASON_NONE);
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_ACQUIRE) == DEBUG_UI_REASON_NONE);
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED;
    /* A stale absolute value must not be used to reject or describe a relative target. */
    snapshot.motors[0].position_rad = 20.0f;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, operation, DEBUG_UI_MODE_POS_VEL));
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 1U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    for (step = 1U; step <= 9U; ++step) {
        snapshot.timestamp_us = 1000000ULL + step * 100000ULL;
        snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW);
    }
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1910U, 900U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == operation);
    assert(request.created_at_us == snapshot.timestamp_us);
    assert(request.delta_rad <= snapshot.motors[0].profile.max_delta_rad);
}

/** @brief POS angle edits advance by one degree, clamp at bounds, and preserve the speed step. */
static void test_pos_one_degree_edit_step(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    float original_speed;
    snapshot.motors[0].profile.max_delta_rad = debug_ui_degrees_to_radians(2.5f);
    snapshot.motors[0].profile.max_speed_rad_s = debug_ui_degrees_to_radians(20.0f);
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1000U, 0U);
    assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) - 2.0f) < 0.001f);
    key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_UP, 1500U, 500U);
    assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) - 2.5f) < 0.001f);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1600U, 0U);
    assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) - 1.5f) < 0.001f);
    original_speed = model.draft.speed_rad_s;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_RIGHT, 1700U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1800U, 0U);
    assert(fabsf(debug_ui_radians_to_degrees(model.draft.speed_rad_s - original_speed) - 1.0f) < 0.001f);
}

/** @brief Failed cleanup requests STOP for original targets; recovery still requires explicit review. */
static void test_prepare_recovery_and_release_gates(void)
{
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
    snapshot.target_motor_id = 1U;
    snapshot.unconfirmed_disable_mask = 6U;
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_ACQUIRE) == DEBUG_UI_REASON_NOT_DISABLED);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_RELEASE) == DEBUG_UI_REASON_NOT_DISABLED);
    model.page = DEBUG_UI_PAGE_PREPARE;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 2U);
    assert(model.stop_motor_mask == 6U);
    debug_ui_model_init(&model);
    snapshot.unconfirmed_disable_mask = 0U;
    snapshot.motors[0].feedback_valid = 0U;
    snapshot.motors[0].feedback_age_ms = 10000U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    model.page = DEBUG_UI_PAGE_PREPARE;
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW && !debug_ui_model_take_request(&model, &request));
    key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 1001U, 0U);
    key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1010U, 0U);
    snapshot.timestamp_us += 1000000ULL;
    snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    key(&model, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, 1910U, 900U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_ACQUIRE);
    debug_ui_model_init(&model);
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_RELEASE) == DEBUG_UI_REASON_NONE);
    model.selected_motor = 1U;
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_RELEASE) == DEBUG_UI_REASON_NOT_ARMED);
    model.selected_motor = 0U;
    snapshot.motors[0].driver_state = 1U;
    debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
    assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_RELEASE) == DEBUG_UI_REASON_NOT_DISABLED);
}

/** @brief POS/MIT default to ten degrees per second; keys change by one and obey limits. */
static void test_motion_speed_defaults_and_steps(void)
{
    const DebugUiOperation operations[] = {DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_OPERATION_MIT_MOVE};
    unsigned index;
    for (index = 0U; index < 2U; ++index) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        snapshot.motors[0].profile.max_speed_rad_s = debug_ui_degrees_to_radians(20.0f);
        snapshot.motors[0].profile.mit_max_speed_rad_s = debug_ui_degrees_to_radians(20.0f);
        debug_ui_model_init(&model);
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        assert(debug_ui_model_begin(&model, operations[index], DEBUG_UI_MODE_POS_VEL));
        /* Both modes edit output-shaft angles with the same one-degree keys. */
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 800U, 0U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) - 2.0f) < 0.001f);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 900U, 0U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) - 1.0f) < 0.001f);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 910U, 0U);
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_DOWN, 1410U, 500U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.delta_rad) + 1.0f) < 0.001f);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.speed_rad_s) - 10.0f) < 0.001f);
        model.edit_field = 1U;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, 1000U, 0U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.speed_rad_s) - 11.0f) < 0.001f);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1100U, 0U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.speed_rad_s) - 10.0f) < 0.001f);
        model.draft.speed_rad_s = debug_ui_degrees_to_radians(1.0f);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_DOWN, 1200U, 0U);
        assert(fabsf(debug_ui_radians_to_degrees(model.draft.speed_rad_s) - 1.0f) < 0.001f);
    }
}

/** @brief Recheck refreshes a faulted motor's stale moving DISABLE reply, never enabling it. */
static void test_fault_recheck_refreshes_disabled_feedback(void)
{
    unsigned mode_index;
    for (mode_index = 0U; mode_index < 2U; ++mode_index) {
        DebugUiModel model;
        DebugUiSnapshot snapshot = fixture();
        DebugUiRequest request;
        DebugUiCompletion completion;
        snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_FAULT;
        snapshot.target_motor_id = 7U;
        snapshot.motors[6].actual_mode = mode_index ? DEBUG_UI_MODE_MIT : DEBUG_UI_MODE_POS_VEL;
        snapshot.motors[6].feedback_valid = 0U;
        snapshot.motors[6].feedback_age_ms = 322827U;
        snapshot.motors[6].velocity_rad_s = -0.5372467F;
        debug_ui_model_init(&model);
        model.selected_motor = 6U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        model.page = DEBUG_UI_PAGE_PREPARE;
        assert(debug_ui_model_gate(&model, DEBUG_UI_OPERATION_ACQUIRE) == DEBUG_UI_REASON_NOT_READY);
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 1000U, 0U);
        assert(debug_ui_model_take_request(&model, &request));
        assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 7U);
        assert(model.stop_motor_mask == 0x40U);
        key(&model, DEBUG_UI_INPUT_EVENT_REPEAT, DEBUG_UI_KEY_CENTER, 1500U, 500U);
        assert(!debug_ui_model_take_request(&model, &request));
        request = model.outgoing_stop;
        memset(&completion, 0, sizeof(completion));
        completion.identity = request.identity;
        completion.operation = DEBUG_UI_OPERATION_STOP;
        completion.code = DEBUG_UI_STOPPED;
        completion.disabled_confirmed = 1U;
        debug_ui_model_completion(&model, &completion);
        snapshot.timestamp_us += 1000000U;
        snapshot.health.ui_timestamp_us = snapshot.health.protocol_timestamp_us = snapshot.timestamp_us;
        snapshot.motors[6].velocity_rad_s = -200.0F / 4095.0F;
        snapshot.motors[6].feedback_valid = 1U;
        snapshot.motors[6].feedback_age_ms = 0U;
        debug_ui_model_update(&model, &snapshot, snapshot.timestamp_us, 1U, 1U);
        key(&model, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, 2000U, 0U);
        model.page = DEBUG_UI_PAGE_PREPARE;
        key(&model, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, 2010U, 0U);
        assert(model.page == DEBUG_UI_PAGE_REVIEW);
        assert(!debug_ui_model_take_request(&model, &request));
        assert(model.snapshot.authority == DEBUG_UI_AUTHORITY_LOCAL_FAULT);
    }
}

/** @brief Execute behavior groups and expose a simple nonzero failure contract. */
int main(void)
{
    test_fault_recheck_refreshes_disabled_feedback();
    test_motion_speed_defaults_and_steps();
    test_pos_one_degree_edit_step();
    test_prepare_recovery_and_release_gates();
    test_pos_review_defers_feedback_acquisition(DEBUG_UI_OPERATION_POS_MOVE);
    test_pos_review_defers_feedback_acquisition(DEBUG_UI_OPERATION_MIT_MOVE);
    test_pos_review_defers_feedback_acquisition(DEBUG_UI_OPERATION_MIT_HOLD);
    test_pos_review_defers_feedback_acquisition(DEBUG_UI_OPERATION_SET_MODE);
    test_stationary_feedback_matches_executor_gate();
    test_browse_and_units();
    test_review_once_and_identity();
    test_running_stop_and_health();
    test_gates_and_cancellation();
    test_remote_lifecycle_and_stop_terminal();
    test_invalid_stop_and_interrupted_hold();
    test_readonly_stop_disabled();
    test_reacquire_after_local_fault();
    test_prepare_recheck_requires_review();
    puts("DEBUG_UI_MODEL_TESTS_OK (simulation, no hardware)");
    return 0;
}
