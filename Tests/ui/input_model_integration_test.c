/**
 * @file input_model_integration_test.c
 * @brief Replays input edges into the existing model using simulated snapshots.
 * Regression of the independent input/model review; synthetic data only.
 */
#define main original_input_suite_main
#include "debug_ui_input_test_main.c"
#undef main
#define main original_model_suite_main
#include "debug_ui_model_test_main.c"
#undef main

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
        if (debug_ui_input_emergency_event(&input_fixture->input, &event) != 0U)
            debug_ui_model_event(model, &event);
        while (debug_ui_input_event(&input_fixture->input, &event) != 0U)
        {
            debug_ui_model_event(model, &event);
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

/** @brief Exercise integration boundaries without running application or motor control. */
int main(void)
{
    (void)setvbuf(stdout, NULL, _IONBF, 0U);
    bridge_sustained_pos_edit();
    bridge_cross_key_stop();
    bridge_overflow_stop();
    bridge_fragmented_confirm();
    bridge_delay_boundary();
    bridge_continuous_confirmation();
    puts("INPUT_MODEL_INTEGRATION_OK");
    return 0;
}
