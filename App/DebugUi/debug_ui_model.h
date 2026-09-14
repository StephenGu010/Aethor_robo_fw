/** @file debug_ui_model.h
 * @brief Single-owner, allocation-free five-key page and confirmation model.
 * The application remains the final authority; all request parameters use rad,
 * rad/s and Nm at the motor output shaft / current power-on reference.
 */
#ifndef APP_DEBUG_UI_DEBUG_UI_MODEL_H
#define APP_DEBUG_UI_DEBUG_UI_MODEL_H
#include <stddef.h>
#include "debug_ui_contract.h"
#include "debug_ui_input.h"

#define DEBUG_UI_REVIEW_HOLD_MS (800U)
#define DEBUG_UI_VISIBLE_MOTORS (5U)

/** @brief Pages are persistent model states, independent of any LVGL focus. */
typedef enum { DEBUG_UI_PAGE_OVERVIEW = 0, DEBUG_UI_PAGE_MOTORS,
    DEBUG_UI_PAGE_DETAIL, DEBUG_UI_PAGE_DIAGNOSTICS, DEBUG_UI_PAGE_EDIT,
    DEBUG_UI_PAGE_REVIEW, DEBUG_UI_PAGE_RUNNING, DEBUG_UI_PAGE_RESULT,
    DEBUG_UI_PAGE_FAULT, DEBUG_UI_PAGE_PREPARE, DEBUG_UI_PAGE_REGISTERS,
    DEBUG_UI_PAGE_ACTIONS, /**< Appended to preserve all existing page identifiers. */
    DEBUG_UI_PAGE_COUNT } DebugUiPage;

/** @brief Snapshot evidence for STOP_LATCHED; these are not application terminals. */
typedef enum { DEBUG_UI_STOP_LATCH_NONE = 0, DEBUG_UI_STOP_LATCH_WAITING,
    DEBUG_UI_STOP_LATCH_CONFIRMED, DEBUG_UI_STOP_LATCH_UNCONFIRMED } DebugUiStopLatchState;

/** @brief Owned only by UiTask. Draft, frozen review and submitted action differ.
 * Outgoing STOP has a separate slot so ordinary mailbox pressure cannot hide it.
 * Snapshot is a value copy; no control-task storage is referenced by the view.
 */
typedef struct
{
    DebugUiSnapshot snapshot;
    DebugUiRequest draft;
    DebugUiRequest reviewed;
    DebugUiRequest submitted;
    DebugUiRequest outgoing;
    DebugUiRequest outgoing_stop;
    DebugUiCompletion completion;
    DebugUiPage page;
    DebugUiReason reason;
    DebugUiStopLatchState stop_latch_state;
    uint64_t now_us;
    uint32_t next_request_id;
    uint32_t review_pressed_ms;
    uint32_t confirm_elapsed_ms;
    uint32_t edit_reference;
    uint32_t edit_epoch;
    uint8_t selected_motor;
    uint8_t list_first;
    uint8_t focus;
    uint8_t edit_field;
    uint8_t diagnostic_page;
    uint8_t snapshot_seen;
    uint8_t input_valid;
    uint8_t display_valid;
    uint8_t center_down;
    uint8_t center_consumed;
    uint8_t review_released;
    uint8_t review_pressed;
    uint8_t outgoing_ready;
    uint8_t stop_ready;
    uint8_t awaiting_result;
    uint8_t admission_seen;
    uint8_t stop_requested;
    uint8_t stop_waiting;
    uint8_t stop_motor_mask; /**< Captured/observed cleanup targets, independent of focus. */
    uint8_t completion_seen;
} DebugUiModel;

/** @brief Initialize locked overview; no motor status is invented. */
void debug_ui_model_init(DebugUiModel *model);
/** @brief Replace snapshot and cancel drafts on stale data/reference/epoch loss. */
void debug_ui_model_update(DebugUiModel *model, const DebugUiSnapshot *snapshot,
                           uint64_t now_us, uint8_t input_valid, uint8_t display_valid);
/** @brief Consume a debounced edge/hold; running CENTER always requests STOP. */
void debug_ui_model_event(DebugUiModel *model, const DebugUiInputEvent *event);
/** @brief Validate and start editing an operation on the selected motor. */
uint8_t debug_ui_model_begin(DebugUiModel *model, DebugUiOperation operation,
                             DebugUiMode requested_mode);
/** @brief Return the current gate reason for the selected motor/operation. */
DebugUiReason debug_ui_model_gate(const DebugUiModel *model, DebugUiOperation operation);
/** @brief Resolve actual activity/cleanup before pending local work; zero means unknown. */
uint8_t debug_ui_model_active_target(const DebugUiModel *model);
/** @brief Pop STOP first, otherwise one ordinary request; never auto-retry motion. */
uint8_t debug_ui_model_take_request(DebugUiModel *model, DebugUiRequest *request);
/** @brief Record refusal or result-less STOP latch without re-queuing an action. */
void debug_ui_model_submit_failed(DebugUiModel *model, const DebugUiRequest *request,
                                  DebugUiReason reason);
/** @brief Distinguish admission from terminal completion using full identity. */
void debug_ui_model_admission(DebugUiModel *model, const DebugUiAdmission *admission);
/** @brief Accept only this UI's submitted/STOP terminal identities. */
void debug_ui_model_completion(DebugUiModel *model, const DebugUiCompletion *completion);
/** @brief Convert output-shaft coordinates at the UI boundary only. */
float debug_ui_radians_to_degrees(float radians);
/** @brief Convert edited display degrees into internal radians. */
float debug_ui_degrees_to_radians(float degrees);
/** @brief Format unknown as -- and retain a visibly stale finite last value. */
void debug_ui_format_value(char *buffer, size_t capacity, float value,
                           uint8_t seen, uint8_t valid, const char *unit);
/** @brief Return a static, Chinese rejection explanation. */
const char *debug_ui_reason_text(DebugUiReason reason);
#endif
