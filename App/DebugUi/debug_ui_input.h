/**
 * @file debug_ui_input.h
 * @brief Defines the pure C, single-owner ADC five-way input state machine.
 *
 * ADC centers and windows are unmeasured hardware candidates. The board must
 * verify the physical left/right mapping and distributions before motion use.
 * No HAL, RTOS, LVGL, allocation, or interrupt-side calls are required here.
 */

#ifndef APP_DEBUG_UI_DEBUG_UI_INPUT_H
#define APP_DEBUG_UI_DEBUG_UI_INPUT_H

#include <stdint.h>

#define DEBUG_UI_INPUT_ADC_MAX (4095U)
#define DEBUG_UI_INPUT_ENTER_WINDOW (150U)
#define DEBUG_UI_INPUT_EXIT_WINDOW (200U)
#define DEBUG_UI_INPUT_DEBOUNCE_MS (20U)
#define DEBUG_UI_INPUT_ARM_MS (200U)
#define DEBUG_UI_INPUT_INVALID_MS (50U)
#define DEBUG_UI_INPUT_HOLD_MS (500U)
#define DEBUG_UI_INPUT_REPEAT_MS (150U)
#define DEBUG_UI_INPUT_STUCK_MS (10000U)
#define DEBUG_UI_INPUT_EVENT_CAPACITY (8U)

/** @brief Names logical keys; UNKNOWN is never a nearest-key approximation. */
typedef enum
{
    DEBUG_UI_KEY_NONE = 0,
    DEBUG_UI_KEY_CENTER,
    DEBUG_UI_KEY_LEFT,
    DEBUG_UI_KEY_RIGHT,
    DEBUG_UI_KEY_UP,
    DEBUG_UI_KEY_DOWN,
    DEBUG_UI_KEY_UNKNOWN
} DebugUiKey;

/** @brief Describes an ordinary edge/update or a separate overflow-only STOP. */
typedef enum
{
    DEBUG_UI_INPUT_EVENT_NONE = 0,
    DEBUG_UI_INPUT_EVENT_PRESS,
    DEBUG_UI_INPUT_EVENT_RELEASE,
    DEBUG_UI_INPUT_EVENT_HOLD,
    DEBUG_UI_INPUT_EVENT_REPEAT,
    DEBUG_UI_INPUT_EVENT_EMERGENCY_STOP
} DebugUiInputEventType;

/** @brief Records the latest fault until 200 ms of filtered release rearms. */
typedef enum
{
    DEBUG_UI_INPUT_FAULT_NONE = 0,
    DEBUG_UI_INPUT_FAULT_UNKNOWN,
    DEBUG_UI_INPUT_FAULT_SAMPLE_TIMEOUT,
    DEBUG_UI_INPUT_FAULT_STUCK_KEY,
    DEBUG_UI_INPUT_FAULT_EVENT_OVERFLOW
} DebugUiInputFault;

/**
 * @brief Copies an input event into consumer-owned storage.
 * timestamp_ms is generation time. RELEASE held_ms measures time since PRESS;
 * HOLD/REPEAT held_ms measures uninterrupted intent since PRESS or recovery.
 * Any classified uncertainty discards pending HOLD/REPEAT and interrupts that
 * clock. Same-key recovery requires 20 ms of new stable samples before its
 * clock restarts; recovery alone does not synthesize a new PRESS.
 * HOLD starts at 500 ms and updates while held, allowing a UI-owned 3 s gate.
 * REPEAT starts at 500 ms, then every 150 ms, for direction keys only.
 * Armed direction repeats have no hold timeout. CENTER and never-armed keys
 * retain the 10 s stuck-key fault; fresh samples and release arming remain mandatory.
 * Pending HOLD/REPEAT of the same key/type coalesce; missed repeats never burst.
 * RELEASE removes pending HOLD/REPEAT for that press, so late holds cannot
 * confirm an action after the operator has already released the key.
 */
typedef struct
{
    DebugUiInputEventType type;
    DebugUiKey key;
    uint32_t timestamp_ms;
    uint32_t held_ms;
} DebugUiInputEvent;

/**
 * @brief Owns all filtering, timing and bounded events in one UI task.
 * key/filtered_key/raw_adc/filtered_adc/valid/armed/fault/fault_count are
 * read-only diagnostics for consumers. All other fields are private state.
 * valid and armed become true together after a filtered NONE lasting 200 ms;
 * an uncertainty shorter than 50 ms retains validity but emits no held events.
 * Any fault clears ordinary events and requires release before new actions.
 * Queue overflow alone retains a separate emergency-only CENTER event; neither
 * that event nor subsequent overflow-state CENTER edges authorize new actions.
 * All times are uint32 milliseconds with forward intervals below 2^31 ms;
 * sample sequence numbers use the same modular ordering rule. Poll at least
 * every 50 ms, before consuming events, even when ADC conversions have stopped.
 * Never share this object directly with an ISR; copy the ISR sample to feed.
 */
typedef struct
{
    DebugUiKey key;
    DebugUiKey filtered_key;
    uint16_t raw_adc;
    uint16_t filtered_adc;
    uint8_t valid;
    uint8_t armed;
    DebugUiInputFault fault;
    uint32_t fault_count;
    DebugUiKey candidate_key;
    uint16_t samples[3];
    uint8_t sample_count;
    uint8_t sample_index;
    uint8_t have_sample;
    uint8_t unknown_pending;
    uint8_t release_pending;
    uint8_t key_pending;
    uint8_t press_active;
    uint8_t center_down;
    uint8_t continuity_pending;
    uint8_t emergency_pending;
    uint8_t repeat_started;
    uint8_t event_count;
    uint32_t initialized_at_ms;
    uint32_t now_ms;
    uint32_t sample_sequence;
    uint32_t sampled_at_ms;
    uint32_t candidate_since_ms;
    uint32_t unknown_since_ms;
    uint32_t release_since_ms;
    uint32_t key_since_ms;
    uint32_t pressed_at_ms;
    uint32_t continuous_since_ms;
    uint32_t held_at_ms;
    uint32_t repeated_at_ms;
    DebugUiInputEvent events[DEBUG_UI_INPUT_EVENT_CAPACITY];
    DebugUiInputEvent emergency_event;
} DebugUiInput;

/** @brief Resets a caller-owned object to unarmed UNKNOWN; NULL is ignored. */
void debug_ui_input_init(DebugUiInput *input, uint32_t now_ms);

/**
 * @brief Accepts one distinct, ordered ADC sample; returns 1 only if accepted.
 * Both sequence and acquisition time must advance after the first sample.
 * now_ms is arrival time; future/stale (age >= 50 ms) samples are rejected.
 * adc_raw > 4095 is an explicit invalid conversion, never clipped or filtered
 * into a key. It starts UNKNOWN timing and discards the median history.
 * Rejected samples cannot refresh liveness or debounce. Health time still
 * advances to now_ms, so replaying samples cannot prevent a timeout.
 * Fresh accepted samples replace liveness before health checks; an actual
 * acquisition-time gap >= 50 ms still invalidates and resets the median.
 * UNKNOWN duration is measured from arrival of the uncertain classification,
 * so a valid transport delay is not charged as time spent observing UNKNOWN.
 */
uint8_t debug_ui_input_feed(DebugUiInput *input,
                            uint16_t adc_raw,
                            uint32_t sample_sequence,
                            uint32_t sampled_at_ms,
                            uint32_t now_ms);

/**
 * @brief Advances fault/hold/repeat timers; old times are ignored, ties allowed.
 * Debounce and arming require fresh samples and cannot complete by polling.
 */
void debug_ui_input_poll(DebugUiInput *input, uint32_t now_ms);

/**
 * @brief Pops one event (1) or reports none/invalid pointers (0).
 * Call poll with current time first. Every new debounced CENTER is reported,
 * including after a direction. Confirmation/release gates belong to the UI
 * model; direct cross-key transitions release the old active key first.
 */
uint8_t debug_ui_input_event(DebugUiInput *input, DebugUiInputEvent *event);

/**
 * @brief Pops an overflow-only CENTER STOP intent, separate from normal events.
 * This event may only request STOP for an already-running action. Consumers
 * must ignore it for all idle/unlock/edit/confirm behavior, even after recovery.
 * A normally queued CENTER PRESS never creates this extra event. Other faults
 * and a complete release rearm discard it. NULL arguments return 0.
 */
uint8_t debug_ui_input_emergency_event(DebugUiInput *input, DebugUiInputEvent *event);

#endif
