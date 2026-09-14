/**
 * @file debug_ui_input.c
 * @brief Implements pure C ADC filtering, release arming and bounded key events.
 * Electrical thresholds and left/right mapping remain unmeasured candidates.
 */

#include "debug_ui_input.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

/** @brief Associates each logical key with its nominal 12-bit ADC center. */
static const uint16_t input_adc_centers[DEBUG_UI_KEY_UNKNOWN] = {
    4095U, 0U, 816U, 1636U, 2457U, 3279U
};

/** @brief Orders wrapping timestamps/sequences without signed conversions. */
static uint8_t input_is_after(uint32_t value, uint32_t previous)
{
    uint32_t elapsed = value - previous;
    return ((elapsed != 0U) && (elapsed < UINT32_C(0x80000000))) ? 1U : 0U;
}

/** @brief Recognizes physical keys, excluding release and unknown levels. */
static uint8_t input_is_pressed_key(DebugUiKey key)
{
    return ((key >= DEBUG_UI_KEY_CENTER) && (key <= DEBUG_UI_KEY_DOWN)) ? 1U : 0U;
}

/** @brief Invalidates input and discards all history that could replay a key. */
static void input_invalidate(DebugUiInput *input, DebugUiInputFault fault)
{
    if (input->fault != fault)
    {
        if (input->fault_count < UINT32_MAX)
        {
            ++input->fault_count;
        }
    }
    input->fault = fault;
    input->armed = 0U;
    input->valid = 0U;
    input->key = DEBUG_UI_KEY_UNKNOWN;
    input->filtered_key = DEBUG_UI_KEY_UNKNOWN;
    input->candidate_key = DEBUG_UI_KEY_UNKNOWN;
    input->filtered_adc = UINT16_MAX;
    input->sample_count = 0U;
    input->sample_index = 0U;
    input->unknown_pending = 0U;
    input->release_pending = 0U;
    input->key_pending = 0U;
    input->press_active = 0U;
    input->continuity_pending = 0U;
    input->repeat_started = 0U;
    input->event_count = 0U;
    if (fault != DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW)
    {
        input->center_down = 0U;
        input->emergency_pending = 0U;
    }
    /* Preserve publisher sequence/time so old samples remain rejected. */
}

/** @brief Retains one overflow-only CENTER intent, without authorizing actions. */
static void input_latch_emergency_stop(DebugUiInput *input)
{
    input->emergency_event.type = DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP;
    input->emergency_event.key = DEBUG_UI_KEY_CENTER;
    input->emergency_event.timestamp_ms = input->now_ms;
    input->emergency_event.held_ms = 0U;
    input->emergency_pending = 1U;
}

/** @brief Removes an event while preserving the generation order of the rest. */
static void input_remove_event(DebugUiInput *input, uint8_t event_index)
{
    assert(event_index < input->event_count);
    --input->event_count;
    while (event_index < input->event_count)
    {
        input->events[event_index] = input->events[event_index + 1U];
        ++event_index;
    }
}

/** @brief Enqueues an edge or coalesces an ongoing event, failing closed if full. */
static uint8_t input_queue_event(DebugUiInput *input, DebugUiInputEventType type,
                                 DebugUiKey key, uint32_t held_ms)
{
    uint8_t event_index;
    DebugUiInputEvent *event;
    assert(input->event_count <= DEBUG_UI_INPUT_EVENT_CAPACITY);
    if ((type == DEBUG_UI_INPUT_EVENT_HOLD) ||
        (type == DEBUG_UI_INPUT_EVENT_REPEAT))
    {
        for (event_index = 0U; event_index < input->event_count; ++event_index)
        {
            if ((input->events[event_index].type == type) &&
                (input->events[event_index].key == key))
            {
                input_remove_event(input, event_index);
                break;
            }
        }
    }
    if (input->event_count == DEBUG_UI_INPUT_EVENT_CAPACITY)
    {
        input_invalidate(input, DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW);
        return 0U;
    }
    event = &input->events[input->event_count];
    event->type = type;
    event->key = key;
    event->timestamp_ms = input->now_ms;
    event->held_ms = held_ms;
    ++input->event_count;
    return 1U;
}

/** @brief Drops deferred HOLD/REPEAT at release before a consumer can act on them. */
static void input_remove_ongoing_events(DebugUiInput *input, DebugUiKey key)
{
    uint8_t event_index = 0U;
    while (event_index < input->event_count)
    {
        const DebugUiInputEvent *event = &input->events[event_index];
        if ((event->key == key) &&
            ((event->type == DEBUG_UI_INPUT_EVENT_HOLD) ||
             (event->type == DEBUG_UI_INPUT_EVENT_REPEAT)))
        {
            input_remove_event(input, event_index);
        }
        else
        {
            ++event_index;
        }
    }
}

/** @brief Checks exact inclusive entry/exit widths in a nonwrapping ADC domain. */
static uint8_t input_within_window(uint16_t value, uint16_t center, uint16_t width)
{
    uint32_t distance = value >= center ? (uint32_t)value - center :
                                         (uint32_t)center - value;
    return distance <= width ? 1U : 0U;
}

/** @brief Retains the prior classification within its exit window, else enters. */
static DebugUiKey input_classify(uint16_t value, DebugUiKey previous)
{
    unsigned int key_index;
    if (((unsigned int)previous < (unsigned int)DEBUG_UI_KEY_UNKNOWN) &&
        (input_within_window(value, input_adc_centers[previous],
                             DEBUG_UI_INPUT_EXIT_WINDOW) != 0U))
    {
        return previous;
    }
    for (key_index = 0U; key_index < (unsigned int)DEBUG_UI_KEY_UNKNOWN; ++key_index)
    {
        if (input_within_window(value, input_adc_centers[key_index],
                                DEBUG_UI_INPUT_ENTER_WINDOW) != 0U)
        {
            return (DebugUiKey)key_index;
        }
    }
    return DEBUG_UI_KEY_UNKNOWN;
}

/** @brief Returns the middle of exactly three independent conversion values. */
static uint16_t input_median(const uint16_t samples[3])
{
    uint16_t lower = samples[0] < samples[1] ? samples[0] : samples[1];
    uint16_t upper = samples[0] > samples[1] ? samples[0] : samples[1];
    if (samples[2] < lower)
    {
        return lower;
    }
    return samples[2] > upper ? upper : samples[2];
}

/** @brief Advances a forward or tied clock without consulting stale sample data. */
static uint8_t input_set_time(DebugUiInput *input, uint32_t now_ms)
{
    if ((now_ms != input->now_ms) &&
        (input_is_after(now_ms, input->now_ms) == 0U))
    {
        return 0U;
    }
    input->now_ms = now_ms;
    return 1U;
}

/** @brief Checks current published liveness and independent fault timers. */
static uint8_t input_advance_health(DebugUiInput *input, uint32_t now_ms)
{
    uint32_t last_sample_ms;
    if (input_set_time(input, now_ms) == 0U)
    {
        return 0U;
    }
    last_sample_ms = input->have_sample != 0U ? input->sampled_at_ms :
                                              input->initialized_at_ms;
    if ((uint32_t)(now_ms - last_sample_ms) >= DEBUG_UI_INPUT_INVALID_MS)
    {
        input_invalidate(input, DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    }
    else if ((input->unknown_pending != 0U) &&
             ((uint32_t)(now_ms - input->unknown_since_ms) >= DEBUG_UI_INPUT_INVALID_MS))
    {
        input_invalidate(input, DEBUG_UI_INPUT_FAULT_UNKNOWN);
    }
    else if ((input->key_pending != 0U) &&
             /* Armed direction holds are intentional repeats, not stuck confirmation keys. */
             ((input->armed == 0U) || (input->key == DEBUG_UI_KEY_CENTER)) &&
             ((uint32_t)(now_ms - input->key_since_ms) >= DEBUG_UI_INPUT_STUCK_MS))
    {
        input_invalidate(input, DEBUG_UI_INPUT_FAULT_STUCK_KEY);
    }
    return 1U;
}

/** @brief Publishes a debounced transition, with release-first ordering. */
static void input_commit_key(DebugUiInput *input, DebugUiKey key)
{
    uint8_t new_center_press =
        ((key == DEBUG_UI_KEY_CENTER) && (input->center_down == 0U)) ? 1U : 0U;
    input->center_down = key == DEBUG_UI_KEY_CENTER ? 1U : 0U;
    if (input->press_active != 0U)
    {
        input_remove_ongoing_events(input, input->key);
        if (input_queue_event(input, DEBUG_UI_INPUT_EVENT_RELEASE, input->key,
                               input->now_ms - input->pressed_at_ms) == 0U)
        {
            if (new_center_press != 0U)
            {
                input_latch_emergency_stop(input);
            }
            return;
        }
    }
    input->press_active = 0U;
    input->key = key;
    input->key_pending = input_is_pressed_key(key);
    input->key_since_ms = input->now_ms;
    input->continuity_pending = 0U;
    if ((input_is_pressed_key(key) != 0U) && (input->armed != 0U))
    {
        input->press_active = 1U;
        input->pressed_at_ms = input->now_ms;
        input->continuous_since_ms = input->now_ms;
        input->held_at_ms = input->now_ms;
        input->repeat_started = 0U;
        if ((input_queue_event(input, DEBUG_UI_INPUT_EVENT_PRESS, key, 0U) == 0U) &&
            (new_center_press != 0U))
        {
            input_latch_emergency_stop(input);
        }
    }
    else if ((new_center_press != 0U) &&
             (input->fault == DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW))
    {
        input_latch_emergency_stop(input);
    }
}

/** @brief Debounces classified samples and measures actual sampled release. */
static void input_accept_classification(DebugUiInput *input, DebugUiKey key,
                                         uint32_t sampled_at_ms)
{
    input->filtered_key = key;
    if ((key != input->key) && (input->press_active != 0U))
    {
        input->continuity_pending = 1U;
        input_remove_ongoing_events(input, input->key);
    }
    if (input->candidate_key != key)
    {
        input->candidate_key = key;
        input->candidate_since_ms = sampled_at_ms;
    }
    if (key == DEBUG_UI_KEY_UNKNOWN)
    {
        input->release_pending = 0U;
        if (input->unknown_pending == 0U)
        {
            input->unknown_pending = 1U;
            input->unknown_since_ms = input->now_ms;
        }
        return;
    }
    input->unknown_pending = 0U;
    if (key == DEBUG_UI_KEY_NONE)
    {
        if (input->release_pending == 0U)
        {
            input->release_pending = 1U;
            input->release_since_ms = sampled_at_ms;
        }
    }
    else
    {
        input->release_pending = 0U;
    }
    if ((input->key != key) &&
        ((uint32_t)(sampled_at_ms - input->candidate_since_ms) >= DEBUG_UI_INPUT_DEBOUNCE_MS))
    {
        input_commit_key(input, key);
    }
    else if ((input->continuity_pending != 0U) && (input->key == key) &&
             ((uint32_t)(sampled_at_ms - input->candidate_since_ms) >= DEBUG_UI_INPUT_DEBOUNCE_MS))
    {
        input->continuity_pending = 0U;
        input->continuous_since_ms = input->now_ms;
        input->held_at_ms = input->now_ms;
        input->repeat_started = 0U;
    }
    if ((key == DEBUG_UI_KEY_NONE) && (input->key == DEBUG_UI_KEY_NONE) &&
        (input->release_pending != 0U) &&
        ((uint32_t)(sampled_at_ms - input->release_since_ms) >= DEBUG_UI_INPUT_ARM_MS))
    {
        input->armed = 1U;
        input->valid = 1U;
        input->fault = DEBUG_UI_INPUT_FAULT_NONE;
        input->emergency_pending = 0U;
    }
}

/** @brief Emits at most one hold/repeat update per time, without catching up. */
static void input_update_ongoing_events(DebugUiInput *input)
{
    uint32_t held_ms;
    if ((input->valid == 0U) || (input->armed == 0U) ||
        (input->press_active == 0U) || (input->filtered_key != input->key) ||
        (input->candidate_key != input->key) || (input->continuity_pending != 0U))
    {
        return;
    }
    held_ms = input->now_ms - input->continuous_since_ms;
    if ((held_ms < DEBUG_UI_INPUT_HOLD_MS) || (input->held_at_ms == input->now_ms))
    {
        return;
    }
    input->held_at_ms = input->now_ms;
    if (input_queue_event(input, DEBUG_UI_INPUT_EVENT_HOLD, input->key, held_ms) == 0U)
    {
        return;
    }
    if ((input->key != DEBUG_UI_KEY_CENTER) &&
        ((input->repeat_started == 0U) ||
         ((uint32_t)(input->now_ms - input->repeated_at_ms) >= DEBUG_UI_INPUT_REPEAT_MS)))
    {
        input->repeat_started = 1U;
        /* Retain the continuous-intent cadence while skipping missed repeats. */
        input->repeated_at_ms = input->now_ms -
            ((held_ms - DEBUG_UI_INPUT_HOLD_MS) % DEBUG_UI_INPUT_REPEAT_MS);
        (void)input_queue_event(input, DEBUG_UI_INPUT_EVENT_REPEAT, input->key, held_ms);
    }
}

/** @brief Starts unarmed with no median history or reusable pending events. */
void debug_ui_input_init(DebugUiInput *input, uint32_t now_ms)
{
    if (input != NULL)
    {
        memset(input, 0, sizeof(*input));
        input->initialized_at_ms = now_ms;
        input->now_ms = now_ms;
        input->key = DEBUG_UI_KEY_UNKNOWN;
        input->filtered_key = DEBUG_UI_KEY_UNKNOWN;
        input->candidate_key = DEBUG_UI_KEY_UNKNOWN;
        input->raw_adc = UINT16_MAX;
        input->filtered_adc = UINT16_MAX;
    }
}

/** @brief Accepts distinct fresh samples and advances median/debounce state. */
uint8_t debug_ui_input_feed(DebugUiInput *input, uint16_t adc_raw,
                            uint32_t sample_sequence, uint32_t sampled_at_ms,
                            uint32_t now_ms)
{
    DebugUiKey classified_key = DEBUG_UI_KEY_UNKNOWN;
    if ((input == NULL) || (input_set_time(input, now_ms) == 0U))
    {
        return 0U;
    }
    if (((uint32_t)(now_ms - sampled_at_ms) >= DEBUG_UI_INPUT_INVALID_MS) ||
        ((input->have_sample != 0U) &&
         ((input_is_after(sample_sequence, input->sample_sequence) == 0U) ||
          (input_is_after(sampled_at_ms, input->sampled_at_ms) == 0U))) ||
        ((input->have_sample == 0U) && (sampled_at_ms != input->initialized_at_ms) &&
         (input_is_after(sampled_at_ms, input->initialized_at_ms) == 0U)))
    {
        (void)input_advance_health(input, now_ms);
        return 0U;
    }
    if ((input->have_sample != 0U) &&
        ((uint32_t)(sampled_at_ms - input->sampled_at_ms) >= DEBUG_UI_INPUT_INVALID_MS))
    {
        input_invalidate(input, DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT);
    }
    input->have_sample = 1U;
    input->sample_sequence = sample_sequence;
    input->sampled_at_ms = sampled_at_ms;
    input->raw_adc = adc_raw;
    /* New liveness is visible before checking UNKNOWN/stuck timers. */
    (void)input_advance_health(input, now_ms);
    if (adc_raw > DEBUG_UI_INPUT_ADC_MAX)
    {
        input->sample_count = 0U;
        input->sample_index = 0U;
        input->filtered_adc = UINT16_MAX;
    }
    else
    {
        assert(input->sample_index < 3U);
        assert(input->sample_count <= 3U);
        input->samples[input->sample_index] = adc_raw;
        input->sample_index = (uint8_t)((input->sample_index + 1U) % 3U);
        if (input->sample_count < 3U)
        {
            ++input->sample_count;
        }
        if (input->sample_count == 3U)
        {
            input->filtered_adc = input_median(input->samples);
            classified_key = input_classify(input->filtered_adc, input->filtered_key);
        }
    }
    input_accept_classification(input, classified_key, sampled_at_ms);
    (void)input_advance_health(input, now_ms);
    input_update_ongoing_events(input);
    return 1U;
}

/** @brief Advances safety and held-key timers without altering sample history. */
void debug_ui_input_poll(DebugUiInput *input, uint32_t now_ms)
{
    if ((input != NULL) && (input_advance_health(input, now_ms) != 0U))
    {
        input_update_ongoing_events(input);
    }
}

/** @brief Copies and removes the oldest pending event without advancing time. */
uint8_t debug_ui_input_event(DebugUiInput *input, DebugUiInputEvent *event)
{
    if ((input == NULL) || (event == NULL) || (input->event_count == 0U))
    {
        return 0U;
    }
    *event = input->events[0];
    input_remove_event(input, 0U);
    return 1U;
}

/** @brief Pops a distinct overflow-only STOP intent; normal events stay separate. */
uint8_t debug_ui_input_emergency_event(DebugUiInput *input, DebugUiInputEvent *event)
{
    if ((input == NULL) || (event == NULL) || (input->emergency_pending == 0U))
    {
        return 0U;
    }
    *event = input->emergency_event;
    input->emergency_pending = 0U;
    return 1U;
}
