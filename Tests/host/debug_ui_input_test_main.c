/**
 * @file debug_ui_input_test_main.c
 * @brief Exercises five-way input behavior with deterministic ADC/time traces.
 * Candidate electrical thresholds are tested as software rules, not hardware
 * calibration. No implementation internals are used to drive transitions.
 * TDD evidence (2026-09-08): strict GCC 14.3.0 C99 compilation of the inert
 * implementation succeeded; its first behavior run failed 22/23 cases (exit 1).
 * Review regressions (2026-09-08): before the fixes, 24/31 passed (exit 1),
 * with failures for interrupted holds, cross-key CENTER, overflow STOP and
 * delayed fresh samples. After the fixes, strict C99/C11 both pass 31/31.
 */

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "debug_ui_input.h"

/** @brief Owns a test clock and a monotonic synthetic ADC publisher. */
typedef struct
{
    DebugUiInput input;
    uint32_t now_ms;
    uint32_t sequence;
} InputFixture;

/** @brief Associates a named behavioral contract with its test function. */
typedef struct
{
    const char *name;
    int (*run)(void);
} InputTestCase;

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL %s:%d: %s\n", __func__, __LINE__, #condition); \
        return 0; \
    } \
} while (0)

/** @brief Initializes a trace with one release sample at an arbitrary clock. */
static void fixture_init(InputFixture *fixture, uint32_t start_ms)
{
    fixture->now_ms = start_ms;
    fixture->sequence = 1U;
    debug_ui_input_init(&fixture->input, start_ms);
    assert(debug_ui_input_feed(&fixture->input, 4095U, fixture->sequence,
                                start_ms, start_ms) == 1U);
}

/** @brief Publishes a fresh sample after a caller-selected elapsed interval. */
static void fixture_sample(InputFixture *fixture, uint16_t raw_adc,
                           uint32_t elapsed_ms)
{
    fixture->now_ms += elapsed_ms;
    ++fixture->sequence;
    assert(debug_ui_input_feed(&fixture->input, raw_adc, fixture->sequence,
                                fixture->now_ms, fixture->now_ms) == 1U);
    debug_ui_input_poll(&fixture->input, fixture->now_ms);
}

/** @brief Supplies a constant level every 5 ms, including an exact endpoint. */
static void fixture_level(InputFixture *fixture, uint16_t raw_adc,
                          uint32_t duration_ms)
{
    while (duration_ms != 0U)
    {
        uint32_t step_ms = duration_ms < 5U ? duration_ms : 5U;
        fixture_sample(fixture, raw_adc, step_ms);
        duration_ms -= step_ms;
    }
}

/** @brief Starts a usable trace after the full three-sample and release gate. */
static void fixture_arm(InputFixture *fixture, uint32_t start_ms)
{
    fixture_init(fixture, start_ms);
    fixture_level(fixture, 4095U, 210U);
}

/** @brief Reads exactly one expected edge and its logical key. */
static int expect_event(InputFixture *fixture, DebugUiInputEventType type,
                        DebugUiKey key, DebugUiInputEvent *event)
{
    CHECK(debug_ui_input_event(&fixture->input, event) == 1U);
    CHECK(event->type == type);
    CHECK(event->key == key);
    return 1;
}

/** @brief Verifies startup cannot arm before 200 ms of filtered release. */
static int test_startup_release_gate(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_init(&fixture, 0U);
    CHECK(fixture.input.armed == 0U);
    fixture_level(&fixture, 4095U, 209U);
    CHECK(fixture.input.armed == 0U);
    fixture_sample(&fixture, 4095U, 1U);
    CHECK(fixture.input.armed == 1U && fixture.input.valid == 1U);
    CHECK(fixture.input.key == DEBUG_UI_KEY_NONE);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies a key held at power-on cannot produce a confirmation. */
static int test_power_on_center_requires_release(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_init(&fixture, 0U);
    fixture_level(&fixture, 0U, 3500U);
    CHECK(fixture.input.armed == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.armed == 1U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS,
                       DEBUG_UI_KEY_CENTER, &event));
    return 1;
}

/** @brief Verifies all nominal levels and both inclusive enter boundaries. */
static int test_nominal_and_enter_boundaries(void)
{
    static const uint16_t centers[] = {0U, 816U, 1636U, 2457U, 3279U, 4095U};
    static const DebugUiKey keys[] = {DEBUG_UI_KEY_CENTER, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_NONE};
    size_t key_index;
    int offset;
    for (key_index = 0U; key_index < sizeof(centers) / sizeof(centers[0]); ++key_index)
    {
        for (offset = -151; offset <= 151; ++offset)
        {
            InputFixture fixture;
            int raw_adc = (int)centers[key_index] + offset;
            DebugUiKey expected_key;
            if ((raw_adc < 0) || (raw_adc > 4095))
            {
                continue;
            }
            fixture_arm(&fixture, 0U);
            /* Leave NONE hysteresis first so its enter window is tested too. */
            if (keys[key_index] == DEBUG_UI_KEY_NONE)
            {
                fixture_level(&fixture, 2457U, 30U);
            }
            fixture_level(&fixture, (uint16_t)raw_adc, 30U);
            expected_key = ((offset < -150) || (offset > 150)) ?
                           DEBUG_UI_KEY_UNKNOWN : keys[key_index];
            CHECK(fixture.input.filtered_key == expected_key);
            if (expected_key != DEBUG_UI_KEY_UNKNOWN)
            {
                CHECK(fixture.input.key == expected_key);
            }
        }
    }
    return 1;
}

/** @brief Verifies exit windows include +/-200 and reject the next code. */
static int test_exit_hysteresis_boundaries(void)
{
    static const uint16_t centers[] = {0U, 816U, 1636U, 2457U, 3279U, 4095U};
    static const DebugUiKey keys[] = {DEBUG_UI_KEY_CENTER, DEBUG_UI_KEY_LEFT,
        DEBUG_UI_KEY_RIGHT, DEBUG_UI_KEY_UP, DEBUG_UI_KEY_DOWN, DEBUG_UI_KEY_NONE};
    size_t key_index;
    int direction;
    for (key_index = 0U; key_index < sizeof(centers) / sizeof(centers[0]); ++key_index)
    {
        for (direction = -1; direction <= 1; direction += 2)
        {
            InputFixture fixture;
            int exit_adc = (int)centers[key_index] + direction * 200;
            if ((exit_adc < 0) || (exit_adc > 4095))
            {
                continue;
            }
            fixture_arm(&fixture, 0U);
            fixture_level(&fixture, centers[key_index], 30U);
            fixture_level(&fixture, (uint16_t)exit_adc, 30U);
            CHECK(fixture.input.filtered_key == keys[key_index]);
            CHECK(fixture.input.key == keys[key_index]);
            fixture_level(&fixture, (uint16_t)(exit_adc + direction), 10U);
            CHECK(fixture.input.filtered_key == DEBUG_UI_KEY_UNKNOWN);
        }
    }
    return 1;
}

/** @brief Verifies one isolated spike is rejected by the three-point median. */
static int test_median_rejects_spikes_and_bounce(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_sample(&fixture, 0U, 5U);
    fixture_level(&fixture, 4095U, 30U);
    CHECK(fixture.input.key == DEBUG_UI_KEY_NONE);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 816U, 15U);
    fixture_level(&fixture, 4095U, 30U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 816U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, &event));
    fixture_sample(&fixture, 4095U, 5U);
    fixture_level(&fixture, 816U, 30U);
    CHECK(fixture.input.key == DEBUG_UI_KEY_LEFT);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies press and release wait the entire 20 ms debounce window. */
static int test_press_and_release_debounce_edges(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 29U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 0U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.timestamp_ms == 240U && event.held_ms == 0U);
    fixture_level(&fixture, 4095U, 29U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 4095U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 30U);
    return 1;
}

/** @brief Verifies CENTER after a stable direction remains visible to STOP logic. */
static int test_cross_key_reports_every_new_center_press(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 816U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, &event));
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, &event));
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_LEFT, &event));
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 499U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    return 1;
}

/** @brief Verifies CENTER hold duration reaches 3 s without another press/repeat. */
static int test_center_hold_is_coalesced_and_never_repeats(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 499U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 0U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 500U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 2500U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 3000U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies repeats at 500/650 ms for each direction, without bursts. */
static int test_direction_repeat_schedule(void)
{
    static const uint16_t centers[] = {816U, 1636U, 2457U, 3279U};
    size_t key_index;
    for (key_index = 0U; key_index < sizeof(centers) / sizeof(centers[0]); ++key_index)
    {
        InputFixture fixture;
        DebugUiInputEvent event;
        unsigned int repeats = 0U;
        fixture_arm(&fixture, 0U);
        fixture_level(&fixture, centers[key_index], 30U);
        CHECK(debug_ui_input_event(&fixture.input, &event) == 1U);
        fixture_level(&fixture, centers[key_index], 499U);
        CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
        fixture_sample(&fixture, centers[key_index], 1U);
        while (debug_ui_input_event(&fixture.input, &event) != 0U)
        {
            if (event.type == DEBUG_UI_INPUT_EVENT_REPEAT)
            {
                CHECK(event.held_ms == 500U);
                ++repeats;
            }
        }
        CHECK(repeats == 1U);
        fixture_level(&fixture, centers[key_index], 149U);
        while (debug_ui_input_event(&fixture.input, &event) != 0U)
        {
            CHECK(event.type == DEBUG_UI_INPUT_EVENT_HOLD);
        }
        fixture_sample(&fixture, centers[key_index], 1U);
        while (debug_ui_input_event(&fixture.input, &event) != 0U)
        {
            if (event.type == DEBUG_UI_INPUT_EVENT_REPEAT)
            {
                CHECK(event.held_ms == 650U);
                ++repeats;
            }
        }
        CHECK(repeats == 2U);
        fixture_level(&fixture, centers[key_index], 3000U);
        repeats = 0U;
        while (debug_ui_input_event(&fixture.input, &event) != 0U)
        {
            repeats += event.type == DEBUG_UI_INPUT_EVENT_REPEAT ? 1U : 0U;
        }
        CHECK(repeats == 1U && fixture.input.valid == 1U);
    }
    return 1;
}

/** @brief Verifies UNKNOWN at 49/50 ms invalidates and discards pending PRESS. */
static int test_unknown_timeout_and_rearm(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 816U, 30U);
    fixture_level(&fixture, 500U, 59U);
    CHECK(fixture.input.valid == 1U);
    fixture_sample(&fixture, 500U, 1U);
    CHECK(fixture.input.valid == 0U && fixture.input.armed == 0U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_UNKNOWN);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 816U, 800U);
    CHECK(fixture.input.armed == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.valid == 1U && fixture.input.fault == DEBUG_UI_INPUT_FAULT_NONE);
    CHECK(fixture.input.fault_count != 0U);
    return 1;
}

/** @brief Verifies a stopped ADC clears stale events exactly at 50 ms. */
static int test_sample_timeout_and_old_key_cannot_restart(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms + 49U);
    CHECK(fixture.input.valid == 1U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms + 50U);
    CHECK(fixture.input.valid == 0U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture.now_ms += 50U;
    fixture_level(&fixture, 0U, 1000U);
    CHECK(fixture.input.armed == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.armed == 1U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    return 1;
}

/** @brief Verifies feed itself catches a 50 ms gap even without a prior poll. */
static int test_feed_detects_gap_before_accepting_new_sample(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 816U, 30U);
    CHECK(fixture.input.armed == 1U && fixture.input.key == DEBUG_UI_KEY_LEFT);
    fixture_sample(&fixture, 816U, 50U);
    CHECK(fixture.input.valid == 0U && fixture.input.armed == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 816U, 600U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies replay, old sequence/time and future/stale samples cannot act. */
static int test_sample_order_and_age_validation(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    uint32_t now_ms;
    fixture_arm(&fixture, 0U);
    now_ms = fixture.now_ms;
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence, now_ms, now_ms) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence - 1U, now_ms + 1U, now_ms + 1U) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence + 1U, now_ms, now_ms + 2U) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence + 1U, now_ms - 1U, now_ms + 3U) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence + 1U, now_ms + 5U, now_ms + 4U) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 0U, fixture.sequence + 1U, now_ms - 49U, now_ms + 5U) == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    CHECK(debug_ui_input_feed(&fixture.input, 4095U, fixture.sequence, now_ms, now_ms + 50U) == 0U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    return 1;
}

/** @brief Verifies accepted delayed samples cannot complete debounce by polling. */
static int test_poll_cannot_invent_samples_or_finish_debounce(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 10U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms + 20U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    CHECK(fixture.input.key == DEBUG_UI_KEY_NONE);
    debug_ui_input_poll(&fixture.input, fixture.now_ms - 1U);
    CHECK(fixture.input.valid == 1U);
    fixture.now_ms += 20U;
    fixture_sample(&fixture, 0U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    return 1;
}

/** @brief Verifies all out-of-range raw values start invalidity, without clipping. */
static int test_invalid_conversion_and_filter_recovery(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    fixture_sample(&fixture, 65535U, 5U);
    CHECK(fixture.input.filtered_key == DEBUG_UI_KEY_UNKNOWN);
    fixture_level(&fixture, 4096U, 49U);
    CHECK(fixture.input.valid == 1U);
    fixture_sample(&fixture, 4096U, 1U);
    CHECK(fixture.input.valid == 0U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 215U);
    CHECK(fixture.input.armed == 1U);
    return 1;
}

/** @brief Verifies 10 s pressed time faults and drops queued events. */
static int test_stuck_key_exact_boundary(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    fixture_level(&fixture, 0U, 9999U);
    CHECK(fixture.input.valid == 1U);
    fixture_sample(&fixture, 0U, 1U);
    CHECK(fixture.input.valid == 0U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_STUCK_KEY);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.armed == 1U);
    return 1;
}

/** @brief Armed direction keys support sustained repeat; release removes deferred repeat intent. */
static int test_direction_repeat_past_ten_seconds(void)
{
    static const uint16_t directions[] = {816U, 1636U, 2457U, 3279U};
    unsigned direction;
    for (direction = 0U; direction < 4U; ++direction)
    {
        InputFixture fixture;
        DebugUiInputEvent event;
        unsigned repeats = 0U;
        fixture_arm(&fixture, 0U);
        fixture_level(&fixture, directions[direction], 30000U);
        CHECK(fixture.input.valid && fixture.input.armed);
        CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_NONE);
        while (debug_ui_input_event(&fixture.input, &event))
        {
            repeats += event.type == DEBUG_UI_INPUT_EVENT_REPEAT ? 1U : 0U;
        }
        CHECK(repeats == 1U);
        fixture_level(&fixture, 4095U, 40U);
        CHECK(fixture.input.valid);
        while (debug_ui_input_event(&fixture.input, &event))
        {
            CHECK(event.type != DEBUG_UI_INPUT_EVENT_REPEAT);
        }
    }
    return 1;
}

/** @brief Verifies even a never-armed continuous low level records a stuck fault. */
static int test_unarmed_stuck_key_is_diagnosed(void)
{
    InputFixture fixture;
    fixture_init(&fixture, 0U);
    fixture_level(&fixture, 0U, 10030U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_STUCK_KEY);
    CHECK(fixture.input.armed == 0U);
    return 1;
}

/** @brief Verifies event backlog failure is bounded and does not replay edges. */
static int test_event_overflow_requires_release(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    unsigned int transition;
    fixture_arm(&fixture, 0U);
    for (transition = 0U; transition < 12U; ++transition)
    {
        fixture_level(&fixture, (transition % 2U) != 0U ? 816U : 1636U, 30U);
    }
    CHECK(fixture.input.armed == 0U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.armed == 1U);
    return 1;
}

/** @brief Verifies arming, press/hold/repeat/release work across uint32 wrap. */
static int test_wraparound_for_all_normal_timers(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    unsigned int repeat_count = 0U;
    fixture_arm(&fixture, UINT32_MAX - 100U);
    CHECK(fixture.input.armed == 1U);
    fixture_arm(&fixture, UINT32_MAX - 500U);
    fixture_level(&fixture, 816U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_LEFT, &event));
    fixture_level(&fixture, 816U, 650U);
    while (debug_ui_input_event(&fixture.input, &event) != 0U)
    {
        CHECK(event.held_ms >= 500U && event.held_ms <= 650U);
        repeat_count += event.type == DEBUG_UI_INPUT_EVENT_REPEAT ? 1U : 0U;
    }
    CHECK(repeat_count == 1U);
    fixture_level(&fixture, 4095U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_RELEASE, DEBUG_UI_KEY_LEFT, &event));
    fixture_arm(&fixture, UINT32_MAX - 225U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    return 1;
}

/** @brief Verifies timeout, UNKNOWN and stuck boundaries survive uint32 wrap. */
static int test_wraparound_fault_timers(void)
{
    InputFixture fixture;
    fixture_arm(&fixture, UINT32_MAX - 250U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms + 50U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    fixture_arm(&fixture, UINT32_MAX - 250U);
    fixture_level(&fixture, 500U, 60U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_UNKNOWN);
    fixture_arm(&fixture, UINT32_MAX - 5000U);
    fixture_level(&fixture, 0U, 10030U);
    CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_STUCK_KEY);
    return 1;
}

/** @brief Verifies sequence zero is a valid wrapped successor, not a replay. */
static int test_sequence_wraparound(void)
{
    InputFixture fixture;
    fixture.now_ms = 0U;
    fixture.sequence = UINT32_MAX - 10U;
    debug_ui_input_init(&fixture.input, 0U);
    fixture_sample(&fixture, 4095U, 5U);
    fixture_level(&fixture, 4095U, 210U);
    CHECK(fixture.input.armed == 1U);
    CHECK(fixture.sequence < 100U);
    return 1;
}

/** @brief Verifies missing startup data and invalid pointers fail harmlessly. */
static int test_empty_startup_and_null_arguments(void)
{
    DebugUiInput input;
    DebugUiInputEvent event;
    debug_ui_input_init(NULL, 0U);
    debug_ui_input_poll(NULL, 0U);
    CHECK(debug_ui_input_feed(NULL, 0U, 0U, 0U, 0U) == 0U);
    CHECK(debug_ui_input_event(NULL, &event) == 0U);
    debug_ui_input_init(&input, 0U);
    CHECK(debug_ui_input_event(&input, NULL) == 0U);
    debug_ui_input_poll(&input, 50U);
    CHECK(input.fault == DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    CHECK(debug_ui_input_event(&input, &event) == 0U);
    return 1;
}

/** @brief Verifies noisy in-window samples cannot create duplicate edges. */
static int test_deterministic_noise_trace(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    uint32_t random_state = 123456789U;
    unsigned int sample_index;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 2457U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_UP, &event));
    for (sample_index = 0U; sample_index < 800U; ++sample_index)
    {
        random_state = random_state * 1664525U + 1013904223U;
        fixture_sample(&fixture, (uint16_t)(2257U + random_state % 401U), 5U);
        CHECK(fixture.input.key == DEBUG_UI_KEY_UP);
        while (debug_ui_input_event(&fixture.input, &event) != 0U)
        {
            CHECK(event.type == DEBUG_UI_INPUT_EVENT_HOLD ||
                  event.type == DEBUG_UI_INPUT_EVENT_REPEAT);
        }
    }
    return 1;
}

/** @brief Replays the reviewed fragmented hold without accumulating confirmation. */
static int test_fragmented_unknown_never_accumulates_hold(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    unsigned int cycle;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    for (cycle = 0U; cycle < 16U; ++cycle)
    {
        fixture_level(&fixture, 500U, 40U);
        fixture_level(&fixture, 0U, 10U);
        CHECK(fixture.input.valid == 1U);
        CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    }
    return 1;
}

/** @brief Verifies uncertainty drops old holds and recovery waits 20 ms anew. */
static int test_unknown_clears_hold_and_restarts_continuous_time(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, UINT32_MAX - 800U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 700U);
    fixture_level(&fixture, 500U, 10U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 29U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 0U, 1U);
    fixture_level(&fixture, 0U, 499U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 0U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 500U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies a short cross-key excursion also interrupts ongoing intent. */
static int test_unstable_direction_interrupts_center_hold(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 700U);
    fixture_level(&fixture, 816U, 10U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 500U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 500U);
    return 1;
}

/** @brief Verifies explicit ADC invalidity interrupts, even below the fault timer. */
static int test_invalid_sample_interrupts_center_hold(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    fixture_level(&fixture, 0U, 700U);
    fixture_sample(&fixture, UINT16_MAX, 5U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 35U);
    fixture_level(&fixture, 0U, 499U);
    CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
    fixture_sample(&fixture, 0U, 1U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_HOLD, DEBUG_UI_KEY_CENTER, &event));
    CHECK(event.held_ms == 500U);
    return 1;
}

/** @brief Fills the ordinary queue with four distinct directions and release. */
static void fixture_fill_event_queue(InputFixture *fixture, uint8_t release_key)
{
    fixture_level(fixture, 816U, 30U);
    fixture_level(fixture, 1636U, 30U);
    fixture_level(fixture, 2457U, 30U);
    fixture_level(fixture, 3279U, 30U);
    if (release_key != 0U)
    {
        fixture_level(fixture, 4095U, 30U);
    }
}

/** @brief Verifies only a lost debounced CENTER gets one emergency-only event. */
static int test_overflow_preserves_center_stop_separately(void)
{
    unsigned int release_key;
    for (release_key = 0U; release_key <= 1U; ++release_key)
    {
        InputFixture fixture;
        DebugUiInputEvent event;
        fixture_arm(&fixture, UINT32_MAX - 300U);
        fixture_fill_event_queue(&fixture, (uint8_t)release_key);
        fixture_level(&fixture, 0U, 30U);
        CHECK(fixture.input.valid == 0U && fixture.input.armed == 0U);
        CHECK(fixture.input.fault == DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW);
        CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
        CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 1U);
        CHECK(event.type == DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP);
        CHECK(event.key == DEBUG_UI_KEY_CENTER && event.held_ms == 0U);
        CHECK(event.timestamp_ms == fixture.now_ms);
        CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
        fixture_level(&fixture, 0U, 1000U);
        CHECK(debug_ui_input_event(&fixture.input, &event) == 0U);
        CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
        fixture_level(&fixture, 816U, 30U);
        fixture_level(&fixture, 0U, 30U);
        CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 1U);
        CHECK(fixture.input.valid == 0U);
    }
    return 1;
}

/** @brief Verifies ordinary confirmation never produces a duplicate STOP event. */
static int test_normal_center_has_no_emergency_duplicate(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_level(&fixture, 0U, 30U);
    CHECK(expect_event(&fixture, DEBUG_UI_INPUT_EVENT_PRESS, DEBUG_UI_KEY_CENTER, &event));
    CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
    fixture_level(&fixture, 0U, 3000U);
    CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
    return 1;
}

/** @brief Verifies timeout and rearming both discard a stale emergency intent. */
static int test_emergency_lifetime_is_bounded_by_fault_and_rearm(void)
{
    InputFixture fixture;
    DebugUiInputEvent event;
    fixture_arm(&fixture, 0U);
    fixture_fill_event_queue(&fixture, 1U);
    fixture_level(&fixture, 0U, 30U);
    debug_ui_input_poll(&fixture.input, fixture.now_ms + 50U);
    CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
    fixture_arm(&fixture, 0U);
    fixture_fill_event_queue(&fixture, 1U);
    fixture_level(&fixture, 0U, 30U);
    fixture_level(&fixture, 4095U, 215U);
    CHECK(fixture.input.armed == 1U);
    CHECK(debug_ui_input_emergency_event(&fixture.input, &event) == 0U);
    CHECK(debug_ui_input_emergency_event(NULL, &event) == 0U);
    CHECK(debug_ui_input_emergency_event(&fixture.input, NULL) == 0U);
    return 1;
}

/** @brief Verifies every accepted age below 50 ms can form a usable release trace. */
static int test_delayed_fresh_samples_can_arm(void)
{
    uint32_t delay_ms;
    for (delay_ms = 0U; delay_ms < 50U; ++delay_ms)
    {
        DebugUiInput input;
        uint32_t elapsed_ms;
        uint32_t start_ms = UINT32_MAX - 100U;
        debug_ui_input_init(&input, start_ms);
        for (elapsed_ms = 5U; elapsed_ms <= 2005U; elapsed_ms += 5U)
        {
            uint32_t sampled_at_ms = start_ms + elapsed_ms;
            CHECK(debug_ui_input_feed(&input, 4095U, elapsed_ms / 5U,
                  sampled_at_ms, sampled_at_ms + delay_ms) == 1U);
            debug_ui_input_poll(&input, sampled_at_ms + delay_ms);
        }
        CHECK(input.armed == 1U && input.valid == 1U);
        CHECK(input.fault == DEBUG_UI_INPUT_FAULT_NONE && input.fault_count == 0U);
    }
    return 1;
}

/** @brief Runs named behavior cases and returns nonzero for any failed case. */
int main(void)
{
    static const InputTestCase test_cases[] = {
        {"startup_release_gate", test_startup_release_gate},
        {"power_on_center_requires_release", test_power_on_center_requires_release},
        {"nominal_and_enter_boundaries", test_nominal_and_enter_boundaries},
        {"exit_hysteresis_boundaries", test_exit_hysteresis_boundaries},
        {"median_rejects_spikes_and_bounce", test_median_rejects_spikes_and_bounce},
        {"press_and_release_debounce_edges", test_press_and_release_debounce_edges},
        {"cross_key_reports_every_new_center_press", test_cross_key_reports_every_new_center_press},
        {"center_hold_is_coalesced_and_never_repeats", test_center_hold_is_coalesced_and_never_repeats},
        {"direction_repeat_schedule", test_direction_repeat_schedule},
        {"unknown_timeout_and_rearm", test_unknown_timeout_and_rearm},
        {"sample_timeout_and_old_key_cannot_restart", test_sample_timeout_and_old_key_cannot_restart},
        {"feed_detects_gap_before_accepting_new_sample", test_feed_detects_gap_before_accepting_new_sample},
        {"sample_order_and_age_validation", test_sample_order_and_age_validation},
        {"poll_cannot_invent_samples_or_finish_debounce", test_poll_cannot_invent_samples_or_finish_debounce},
        {"invalid_conversion_and_filter_recovery", test_invalid_conversion_and_filter_recovery},
        {"stuck_key_exact_boundary", test_stuck_key_exact_boundary},
        {"direction_repeat_past_ten_seconds", test_direction_repeat_past_ten_seconds},
        {"unarmed_stuck_key_is_diagnosed", test_unarmed_stuck_key_is_diagnosed},
        {"event_overflow_requires_release", test_event_overflow_requires_release},
        {"wraparound_for_all_normal_timers", test_wraparound_for_all_normal_timers},
        {"wraparound_fault_timers", test_wraparound_fault_timers},
        {"sequence_wraparound", test_sequence_wraparound},
        {"empty_startup_and_null_arguments", test_empty_startup_and_null_arguments},
        {"deterministic_noise_trace", test_deterministic_noise_trace},
        {"fragmented_unknown_never_accumulates_hold", test_fragmented_unknown_never_accumulates_hold},
        {"unknown_clears_hold_and_restarts_continuous_time", test_unknown_clears_hold_and_restarts_continuous_time},
        {"unstable_direction_interrupts_center_hold", test_unstable_direction_interrupts_center_hold},
        {"invalid_sample_interrupts_center_hold", test_invalid_sample_interrupts_center_hold},
        {"overflow_preserves_center_stop_separately", test_overflow_preserves_center_stop_separately},
        {"normal_center_has_no_emergency_duplicate", test_normal_center_has_no_emergency_duplicate},
        {"emergency_lifetime_is_bounded_by_fault_and_rearm", test_emergency_lifetime_is_bounded_by_fault_and_rearm},
        {"delayed_fresh_samples_can_arm", test_delayed_fresh_samples_can_arm}
    };
    size_t test_index;
    unsigned int passed = 0U;
    for (test_index = 0U; test_index < sizeof(test_cases) / sizeof(test_cases[0]); ++test_index)
    {
        int success = test_cases[test_index].run();
        printf("%s %s\n", success ? "PASS" : "FAIL", test_cases[test_index].name);
        passed += success ? 1U : 0U;
    }
    printf("debug_ui_input: %u/%u passed\n", passed,
           (unsigned int)(sizeof(test_cases) / sizeof(test_cases[0])));
    return passed == sizeof(test_cases) / sizeof(test_cases[0]) ? 0 : 1;
}
