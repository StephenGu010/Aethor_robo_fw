/**
 * @file input_model_integration_test.c
 * @brief Replays input edges into the existing model using simulated snapshots.
 * Regression of the independent input/model review with scripted immutable backend receipts;
 * synthetic data only, without application control, hardware TX or motor qualification claims.
 */
#define main original_input_suite_main
#include "debug_ui_input_test_main.c"
#undef main
#define main original_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main
#include "lv_port_indev.h"
static uint8_t bridge_use_landscape; /**< Exercise the real mounting transform when enabled. */

/** @brief Refresh healthy synthetic feedback and optionally deliver queued edges. */
static void bridge_update(InputFixture *input_fixture, DebugUiModel *model,
                          DebugUiSnapshot *snapshot, uint8_t consume_events)
{
    DebugUiInputEvent event;
    uint64_t now_us = (uint64_t)input_fixture->now_ms * 1000U;
    snapshot->timestamp_us = now_us;
    snapshot->health.ui_timestamp_us = now_us;
    snapshot->health.protocol_timestamp_us = now_us;
    snapshot->health.input_valid = input_fixture->input.valid;
    debug_ui_model_update(model, snapshot, now_us, input_fixture->input.valid, 1U);
    if (consume_events != 0U)
    {
        if (debug_ui_input_emergency_event(&input_fixture->input, &event) != 0U) {
            if (bridge_use_landscape) lv_port_indev_event(model, &event);
            else debug_ui_model_event(model, &event);
        }
        while (debug_ui_input_event(&input_fixture->input, &event) != 0U)
        {
            if (bridge_use_landscape) lv_port_indev_event(model, &event);
            else debug_ui_model_event(model, &event);
        }
    }
}

/** @brief Sample each 5 ms and exercise the actual feed/poll/model event sequence. */
static void bridge_level(InputFixture *input_fixture, DebugUiModel *model,
                         DebugUiSnapshot *snapshot, uint16_t raw_adc,
                         uint32_t duration_ms, uint8_t consume_events)
{
    while (duration_ms != 0U)
    {
        uint32_t step_ms = duration_ms < 5U ? duration_ms : 5U;
        fixture_sample(input_fixture, raw_adc, step_ms);
        bridge_update(input_fixture, model, snapshot, consume_events);
        duration_ms -= step_ms;
    }
}

/** @brief Check STOP from a fresh center press after a stable directional transition. */
static void bridge_cross_key_stop(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned int request_seen;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 816U, 30U, 1U);
    snapshot.active = 1U;
    snapshot.target_motor_id = 4U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    request_seen = debug_ui_model_take_request(&model, &request);
    printf("MODEL cross_key_STOP request=%u input_valid=%u\n",
           request_seen, (unsigned int)input_fixture.input.valid);
    assert(request_seen != 0U && request.operation == DEBUG_UI_OPERATION_STOP);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(debug_ui_model_take_request(&model, &request) == 0U);
    puts("MODEL control_none_then_center_STOP=1");
}

/** @brief Check that queue overflow does not silently discard remote STOP intent. */
static void bridge_overflow_stop(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned int request_seen;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    snapshot.active = 1U;
    snapshot.target_motor_id = 4U;
    snapshot.active_identity.origin = DEBUG_UI_ORIGIN_USB;
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 816U, 30U, 0U);
    bridge_level(&input_fixture, &model, &snapshot, 1636U, 30U, 0U);
    bridge_level(&input_fixture, &model, &snapshot, 2457U, 30U, 0U);
    bridge_level(&input_fixture, &model, &snapshot, 3279U, 30U, 0U);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 0U);
    assert(input_fixture.input.event_count == 8U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 0U);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    request_seen = debug_ui_model_take_request(&model, &request);
    printf("MODEL queue_full_remote_STOP request=%u input_valid=%u fault=%u\n",
           request_seen, (unsigned int)input_fixture.input.valid,
           (unsigned int)input_fixture.input.fault);
    assert(request_seen != 0U && request.operation == DEBUG_UI_OPERATION_STOP);
}

/** @brief Check whether interrupted center levels can submit an 800 ms confirmation. */
static void bridge_fragmented_confirm(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned int cycle;
    unsigned int request_seen;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    model.focus = debug_ui_model_parameter_count(&model);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    for (cycle = 0U; cycle < 16U; ++cycle)
    {
        bridge_level(&input_fixture, &model, &snapshot, 500U, 40U, 1U);
        bridge_level(&input_fixture, &model, &snapshot, 0U, 10U, 1U);
    }
    request_seen = debug_ui_model_take_request(&model, &request);
    printf("MODEL fragmented_hold_submits request=%u operation=%u elapsed=%lu input_valid=%u\n",
           request_seen, request_seen != 0U ? (unsigned int)request.operation : 0U,
           (unsigned long)model.confirm_elapsed_ms, (unsigned int)input_fixture.input.valid);
    assert(request_seen == 0U);
}

/** @brief Check the documented accepted sample age boundary independently of model. */
static void bridge_delay_boundary(void)
{
    static const uint32_t delays[] = {0U, 30U, 39U, 40U, 44U, 45U, 49U};
    size_t delay_index;
    for (delay_index = 0U; delay_index < sizeof(delays) / sizeof(delays[0]); ++delay_index)
    {
        DebugUiInput input;
        uint32_t sample_time;
        uint32_t sequence = 0U;
        uint32_t arrival_delay = delays[delay_index];
        debug_ui_input_init(&input, 0U);
        for (sample_time = 5U; sample_time <= 2005U; sample_time += 5U)
        {
            ++sequence;
            assert(debug_ui_input_feed(&input, 4095U, sequence, sample_time,
                                       sample_time + arrival_delay) == 1U);
            debug_ui_input_poll(&input, sample_time + arrival_delay);
        }
        printf("DELAY age=%lu accepted=401 valid=%u armed=%u fault=%u\n",
               (unsigned long)arrival_delay, (unsigned int)input.valid,
               (unsigned int)input.armed, (unsigned int)input.fault);
    }
}

/** @brief 600ms CENTER plus UNKNOWN40ms plus CENTER500ms is not an 800ms hold. */
static void bridge_continuous_confirmation(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    model.focus = debug_ui_model_parameter_count(&model);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 600U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 500U, 40U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 500U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 0U, 400U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_POS_MOVE);
    puts("CONTINUOUS_800MS_CONFIRMATION_OK");
}

/** @brief A minute of directional POS editing stays healthy and never submits motion. */
static void bridge_sustained_pos_edit(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    snapshot.motors[0].profile.max_delta_rad = 25.0f;
    snapshot.motors[0].profile.position_min_rad = -12.5f;
    snapshot.motors[0].profile.position_max_rad = 12.5f;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_POS_MOVE, DEBUG_UI_MODE_POS_VEL));
    model.focus = debug_ui_model_parameter_count(&model);
    model.focus = 0U;
    bridge_level(&input_fixture, &model, &snapshot, 1636U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 2457U, 60000U, 1U);
    assert(input_fixture.input.valid && input_fixture.input.fault_count == 0U);
    assert(model.page == DEBUG_UI_PAGE_NUMBER);
    assert(debug_ui_radians_to_degrees(model.draft.delta_rad) > 350.0f);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 210U, 1U);
    assert(input_fixture.input.valid && !debug_ui_model_take_request(&model, &request));
    puts("SUSTAINED_POS_EDIT_60S_OK no_motion_request");
}

/** @brief Replay real ADC release/800 ms confirmation and STOP for frozen seven-axis HOLD. */
static void bridge_static_hold_confirmation(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    unsigned index;
    snapshot.static_load_enabled = 1U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.focus = debug_ui_model_parameter_count(&model);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 1000U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 600U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 500U, 40U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 500U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 0U, 400U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STATIC_HOLD);
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        assert(request.static_references[index] == snapshot.motors[index].reference_generation);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP);
    assert(model.stop_motor_mask == 0x7FU);
    puts("STATIC_INPUT_CONFIRMATION_OK release_required=1 continuous_800ms=1 stop_all=1");
}

/** @brief Replay selected-only long HOLD confirmation and priority STOP through real ADC decoding. */
static void bridge_single_static_hold_confirmation(void)
{
    InputFixture input_fixture;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    DebugUiMotorView motor = snapshot.motors[5];
    memset(snapshot.motors, 0, sizeof(snapshot.motors));
    motor.actual_mode = DEBUG_UI_MODE_MIT;
    motor.single_static_profile.configured = 1U;
    motor.single_static_profile.minimum_ms = 1000U;
    motor.single_static_profile.maximum_ms = 60000U;
    snapshot.motors[5] = motor;
    snapshot.single_static_load_enabled = 1U;
    fixture_arm(&input_fixture, 1000U);
    debug_ui_model_init(&model);
    model.selected_motor = 5U;
    bridge_update(&input_fixture, &model, &snapshot, 1U);
    assert(debug_ui_model_begin(&model, DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD, DEBUG_UI_MODE_MIT));
    model.draft.hold_duration_ms = 60000U;
    model.focus = debug_ui_model_parameter_count(&model);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 1000U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 600U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 500U, 40U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 500U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input_fixture, &model, &snapshot, 0U, 400U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD && request.target_motor_id == 6U);
    assert(request.hold_duration_ms == 60000U && request.static_references[5] == 3U);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 3279U, 30U, 1U);
    assert(model.selected_motor == 5U && model.page == DEBUG_UI_PAGE_RUNNING);
    bridge_level(&input_fixture, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input_fixture, &model, &snapshot, 0U, 30U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 6U && model.stop_motor_mask == 0x20U);
    puts("SINGLE_STATIC_INPUT_CONFIRMATION_OK id=6 release=1 continuous_800ms=1 duration_ms=60000 stop_mask=32");
}

/** @brief Deliver one spatial key press and its release using production ADC filtering. */
static void bridge_lcd_tap(InputFixture *input, DebugUiModel *model, DebugUiSnapshot *snapshot, uint16_t adc)
{
    bridge_level(input, model, snapshot, adc, 30U, 1U);
    bridge_level(input, model, snapshot, 4095U, 30U, 1U);
}
/** @brief Complete exactly the ADC-produced preflight only after matching receipts and a newer coherent selected feedback snapshot. */
static void bridge_lcd_preflight_reply(InputFixture *input, DebugUiModel *model, DebugUiSnapshot *snapshot,
    const DebugUiRequest *preflight, uint8_t expected_phase)
{
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    DebugUiRequest unexpected;
    uint8_t selected_index;
    uint8_t configured_before = model->single_static_configured;
    uint8_t owned_before;
    uint32_t snapshot_generation = snapshot->generation;
    assert(preflight->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT && preflight->target_motor_id == 7U);
    selected_index = (uint8_t)(preflight->target_motor_id - 1U);
    owned_before = snapshot->motors[selected_index].single_static_lcd_owned;
    assert(model->single_static_preflight == expected_phase && model->awaiting_result && !model->outgoing_ready);
    memset(&admission, 0, sizeof(admission));
    admission.identity = preflight->identity; admission.operation = preflight->operation; admission.accepted = 1U;
    ++admission.identity.request_id;
    debug_ui_model_admission(model, &admission);
    assert(!model->admission_seen);
    admission.identity = preflight->identity;
    debug_ui_model_admission(model, &admission);
    assert(model->admission_seen && model->awaiting_result && !debug_ui_model_take_request(model, &unexpected));
    memset(&completion, 0, sizeof(completion));
    completion.identity = preflight->identity; completion.operation = preflight->operation;
    completion.code = DEBUG_UI_COMPLETED; completion.stage = DEBUG_UI_STAGE_DISABLE;
    completion.disabled_confirmed = 1U; completion.motor_mask = (uint8_t)(1U << selected_index);
    completion.timestamp_us = completion.feedback_timestamp_us = (uint64_t)(input->now_ms + 5U) * 1000ULL;
    assert(completion.feedback_timestamp_us > preflight->created_at_us);
    ++completion.identity.request_id;
    debug_ui_model_completion(model, &completion);
    assert(model->awaiting_result && !model->completion_seen);
    completion.identity = preflight->identity;
    debug_ui_model_completion(model, &completion);
    assert(!model->awaiting_result && model->single_static_preflight == expected_phase && !model->outgoing_ready);
    assert(model->single_static_configured == configured_before && snapshot->motors[selected_index].single_static_lcd_owned == owned_before);
    bridge_level(input, model, snapshot, 4095U, 5U, 1U);
    assert(snapshot->generation == snapshot_generation && model->single_static_preflight == expected_phase);
    assert(!debug_ui_model_take_request(model, &unexpected));
    snapshot->motors[selected_index].feedback_timestamp_us = completion.feedback_timestamp_us;
    snapshot->motors[selected_index].feedback_seen = snapshot->motors[selected_index].feedback_valid = 1U;
    snapshot->motors[selected_index].feedback_age_ms = 0U;
    ++snapshot->generation;
    bridge_update(input, model, snapshot, 1U);
    assert(!model->single_static_preflight && model->single_static_configured == configured_before);
    assert(snapshot->motors[selected_index].single_static_lcd_owned == owned_before);
}
/** @brief Use actual mounting directions and independent configuration/HOLD continuous confirmations. */
static void bridge_static_lcd_launcher(void)
{
    InputFixture input;
    DebugUiModel model;
    DebugUiSnapshot snapshot = fixture();
    DebugUiRequest request;
    DebugUiAdmission admission;
    DebugUiCompletion completion;
    unsigned index;
    snapshot.authority = DEBUG_UI_AUTHORITY_REMOTE;
    snapshot.single_static_load_enabled = 1U;
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index) {
        snapshot.motors[index].actual_mode = DEBUG_UI_MODE_MIT;
        snapshot.motors[index].single_static_evidence_generation = 100U + index;
    }
    fixture_arm(&input, 1000U);
    debug_ui_model_init(&model);
    bridge_use_landscape = 1U;
    bridge_update(&input, &model, &snapshot, 1U);
    bridge_lcd_tap(&input, &model, &snapshot, 2457U); /* Electrical UP is screen LEFT. */
    assert(model.page == DEBUG_UI_PAGE_OVERVIEW && model.focus == 3U);
    bridge_lcd_tap(&input, &model, &snapshot, 0U);
    assert(model.page == DEBUG_UI_PAGE_STATIC_HUB && model.focus == 0U);
    bridge_lcd_tap(&input, &model, &snapshot, 0U);
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_SETUP && model.focus == 0U);
    for (index = 0U; index < 6U; ++index) bridge_lcd_tap(&input, &model, &snapshot, 3279U);
    assert(model.selected_motor == 6U); /* Electrical DOWN is screen RIGHT. */
    bridge_lcd_tap(&input, &model, &snapshot, 816U); /* Electrical LEFT is screen DOWN. */
    assert(model.focus == 1U);
    bridge_level(&input, &model, &snapshot, 3279U, 10000U, 1U);
    bridge_level(&input, &model, &snapshot, 4095U, 30U, 1U);
    assert(model.single_static_seconds == 60U);
    bridge_lcd_tap(&input, &model, &snapshot, 816U);
    assert(model.focus == 2U);
    bridge_level(&input, &model, &snapshot, 0U, 1000U, 1U);
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW && !debug_ui_model_take_request(&model, &request));
    bridge_level(&input, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input, &model, &snapshot, 0U, 820U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input, &model, &snapshot, 0U, 30U, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT && request.target_motor_id == 7U &&
        request.hold_duration_ms == 60000U && request.single_static_evidence_generation == 106U);
    assert(!model.single_static_configured && !snapshot.motors[6].single_static_lcd_owned);
    bridge_lcd_preflight_reply(&input, &model, &snapshot, &request, 1U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_CONFIGURE && request.target_motor_id == 7U &&
        request.hold_duration_ms == 60000U && request.single_static_evidence_generation == 106U);
    memset(&admission, 0, sizeof(admission));
    admission.identity = request.identity; admission.operation = request.operation; admission.accepted = 1U;
    debug_ui_model_admission(&model, &admission);
    memset(&completion, 0, sizeof(completion));
    completion.identity = request.identity; completion.operation = request.operation;
    completion.code = DEBUG_UI_COMPLETED;
    completion.motor_mask = 0x40U; completion.timestamp_us = model.now_us;
    debug_ui_model_completion(&model, &completion);
    assert(!model.single_static_configured);
    bridge_update(&input, &model, &snapshot, 1U);
    assert(!model.single_static_configured && model.single_static_wait_snapshot == 1U &&
        !debug_ui_model_take_request(&model, &request));
    snapshot.motors[6].single_static_profile.configured = snapshot.motors[6].single_static_lcd_owned = 1U;
    snapshot.motors[6].single_static_profile.minimum_ms = 1000U;
    snapshot.motors[6].single_static_profile.maximum_ms = 60000U;
    ++snapshot.motors[6].reference_generation; ++snapshot.motors[6].single_static_evidence_generation; ++snapshot.generation;
    bridge_level(&input, &model, &snapshot, 4095U, 30U, 1U);
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY && model.single_static_configured);
    debug_ui_model_completion(&model, &completion); /* Duplicate terminal must not restart synchronization. */
    assert(model.page == DEBUG_UI_PAGE_SINGLE_STATIC_READY && model.single_static_wait_snapshot == 0U);
    snapshot.authority = DEBUG_UI_AUTHORITY_LOCAL_ARMED; snapshot.target_motor_id = 7U;
    bridge_update(&input, &model, &snapshot, 1U);
    bridge_lcd_tap(&input, &model, &snapshot, 0U);
    assert(debug_ui_model_take_request(&model, &request));
    assert(request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT && !model.static_locked);
    bridge_lcd_preflight_reply(&input, &model, &snapshot, &request, 3U);
    assert(model.static_locked && model.draft.hold_duration_ms == 60000U);
    bridge_level(&input, &model, &snapshot, 0U, 1100U, 1U);
    assert(model.page == DEBUG_UI_PAGE_REVIEW && !debug_ui_model_take_request(&model, &request));
    bridge_level(&input, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input, &model, &snapshot, 0U, 600U, 1U);
    bridge_level(&input, &model, &snapshot, 500U, 40U, 1U);
    bridge_level(&input, &model, &snapshot, 0U, 500U, 1U);
    assert(!debug_ui_model_take_request(&model, &request));
    bridge_level(&input, &model, &snapshot, 0U, 400U, 1U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_PREFLIGHT &&
        request.target_motor_id == 7U && request.hold_duration_ms == 60000U);
    bridge_lcd_preflight_reply(&input, &model, &snapshot, &request, 4U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD &&
        request.target_motor_id == 7U && request.hold_duration_ms == 60000U);
    bridge_level(&input, &model, &snapshot, 4095U, 30U, 1U);
    bridge_level(&input, &model, &snapshot, 0U, 30U, 1U);
    assert(debug_ui_model_take_request(&model, &request) && request.operation == DEBUG_UI_OPERATION_STOP && request.target_motor_id == 7U);
    bridge_use_landscape = 0U;
    puts("STATIC_LCD_ADC_WORKFLOW_OK actual_mounting=1 home4_hub=1 id7_time60=1 config_hold800=1 preflight_phases=1_3_4 exact_receipts=1 coherent_feedback_before_continuation=1 config_requires_ram_snapshot=1 separate_start_hold800=1 interrupted_hold_blocked=1 stop_first=1");
}

/** @brief Exercise integration boundaries without running application or motor control. */
int main(void)
{
    (void)setvbuf(stdout, NULL, _IONBF, 0U);
    bridge_static_lcd_launcher();
    bridge_static_hold_confirmation();
    bridge_single_static_hold_confirmation();
    bridge_sustained_pos_edit();
    bridge_cross_key_stop();
    bridge_overflow_stop();
    bridge_fragmented_confirm();
    bridge_delay_boundary();
    bridge_continuous_confirmation();
    puts("INPUT_MODEL_INTEGRATION_OK");
    return 0;
}
