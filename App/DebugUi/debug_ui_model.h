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
#include "debug_ui_load_trials.h"

#define DEBUG_UI_REVIEW_HOLD_MS (800U)
#define DEBUG_UI_VISIBLE_MOTORS (5U)

/** @brief Pages are persistent model states, independent of any LVGL focus. */
typedef enum { DEBUG_UI_PAGE_OVERVIEW = 0, DEBUG_UI_PAGE_MOTORS,
    DEBUG_UI_PAGE_DETAIL, DEBUG_UI_PAGE_DIAGNOSTICS, DEBUG_UI_PAGE_EDIT,
    DEBUG_UI_PAGE_REVIEW, DEBUG_UI_PAGE_RUNNING, DEBUG_UI_PAGE_RESULT,
    DEBUG_UI_PAGE_FAULT, DEBUG_UI_PAGE_PREPARE, DEBUG_UI_PAGE_REGISTERS,
    DEBUG_UI_PAGE_ACTIONS, /**< Appended to preserve all existing page identifiers. */
    DEBUG_UI_PAGE_NUMBER, DEBUG_UI_PAGE_MODES, DEBUG_UI_PAGE_RECOVERY,
    DEBUG_UI_PAGE_DIAGNOSTIC_MENU, DEBUG_UI_PAGE_NOTICE,
    DEBUG_UI_PAGE_STATIC_LOAD, DEBUG_UI_PAGE_STATIC_ACTIONS, DEBUG_UI_PAGE_STATIC_RESULT,
    DEBUG_UI_PAGE_SINGLE_STATIC_LOAD, DEBUG_UI_PAGE_SINGLE_STATIC_ACTIONS,
    DEBUG_UI_PAGE_SINGLE_STATIC_RESULT, DEBUG_UI_PAGE_SINGLE_STATIC_IDS,
    DEBUG_UI_PAGE_STATIC_HUB, /**< Home launcher for single/seven static workflows. */
    DEBUG_UI_PAGE_SINGLE_STATIC_SETUP, DEBUG_UI_PAGE_SINGLE_STATIC_CONFIG_REVIEW,
    DEBUG_UI_PAGE_SINGLE_STATIC_READY, DEBUG_UI_PAGE_SINGLE_STATIC_RELOCK,
    DEBUG_UI_PAGE_LOAD_METADATA, DEBUG_UI_PAGE_LOAD_NUMBER,
    DEBUG_UI_PAGE_LOAD_HISTORY, DEBUG_UI_PAGE_LOAD_DETAIL, /**< Display-only pages append stable identifiers. */
    DEBUG_UI_PAGE_SINGLE_STATIC_PREFLIGHT, /**< Explicit bounded DISABLE, distinct from RAM configuration. */
    DEBUG_UI_PAGE_POS_LOAD_SETUP, DEBUG_UI_PAGE_POS_LOAD_NUMBER,
    DEBUG_UI_PAGE_POS_LOAD_RESULT, /**< Independent all-POS workflow; existing identifiers remain unchanged. */
    DEBUG_UI_PAGE_SINGLE_STATIC_GAINS, /**< Idle-only gain editor. */
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
    DebugUiRequest single_static_continuation; /**< Frozen operation; preflight never replaces a captured HOLD pose. */
    DebugUiCompletion completion;
    DebugUiPage page;
    DebugUiRequest number_backup; /**< Value before opening a numeric widget. */
    DebugUiPage review_return_page;
    DebugUiPage notice_return_page;
    DebugUiPage prepare_return_page;
    DebugUiPage diagnostic_return_page;
    DebugUiPage diagnostic_parent_page; /**< Tabs return to their direct source or the legacy diagnostic menu. */
    uint8_t page_focus[DEBUG_UI_PAGE_COUNT];
    uint8_t page_first[DEBUG_UI_PAGE_COUNT];
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
    uint8_t pos_digit_cursor; /**< 0 sign, 1 hundreds, 2 tens, 3 ones, 4 tenths. */
    uint8_t pos_digit_negative; /**< Preserve an explicitly chosen minus sign at zero. */
    uint8_t pos_digit_left_down; /**< 0 idle, 1 pressed, 2 cancelled awaiting release. */
    uint16_t pos_digit_value; /**< Unsigned display tenths, or integer seconds for duration. */
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
    uint8_t static_locked; /**< Selected static poses/references in draft remain coherent. */
    uint8_t static_result_detail; /**< Retained result: 0 peaks, 1 temperatures, 2 samples. */
    uint8_t static_result_retained; /**< Explicit history view, separate from this submission's result. */
    DebugUiPage single_static_relock_destination; /**< Navigate only after matching LOCK and snapshot. */
    uint32_t single_static_snapshot_generation; /**< Snapshot observed before lifecycle submission. */
    uint32_t single_static_owned_reference; /**< Latest confirmed LCD configuration reference. */
    uint32_t single_static_owned_evidence; /**< Evidence frozen for this LCD installation. */
    uint8_t single_static_flow; /**< New guided flow is distinct from legacy static drafts. */
    uint16_t single_static_ff_centi; /**< Magnitude in 0.01 Nm; no mass-to-command conversion. */
    uint8_t single_static_ff_negative;
    uint8_t single_static_ff_digit; /**< 0 sign, 1 ones, 2 tenths, 3 hundredths. */
    uint8_t single_static_ff_editing;
    uint16_t single_static_ff_saved_centi;
    uint8_t single_static_ff_saved_negative;
    uint8_t single_static_ff_left_down;
    uint16_t single_static_kp; /**< ID7 output-shaft stiffness draft. */
    uint16_t single_static_kd_centi; /**< ID7 damping draft in hundredths. */
    uint16_t gain_edit_value; /**< Scratch only; cancelled edits never change the draft. */
    uint8_t gain_editing;
    uint8_t gain_digit;
    uint8_t gain_left_down;
    uint8_t single_static_seconds; /**< One selected 1..60s value drives configuration and HOLD. */
    uint8_t single_static_configured; /**< Both CONFIGURE completion and snapshot were observed. */
    uint8_t single_static_wait_snapshot; /**< 1 CONFIGURE, 2 LOCK, 3 ACQUIRE synchronization. */
    uint8_t single_static_relock_motor_id; /**< Actual prior local target being handed back; selection stays unchanged. */
    uint8_t single_static_lock_owned; /**< LOCK only expects removal of the LCD-owned profile it observed. */
    uint8_t single_static_lock_then_configure; /**< Release local authority before config review. */
    uint8_t single_static_lifecycle_cancelled; /**< STOP cancels the submitted lifecycle identity until a new submission. */
    uint8_t single_static_config_cancelled; /**< STOP permanently cancels a pending config identity. */
    uint8_t single_static_pending_motor; /**< Deferred edit applies after LOCK acknowledgement. */
    uint8_t single_static_pending_seconds; /**< Deferred duration applies after LOCK acknowledgement. */
    uint8_t single_static_preflight; /**< 1 CONFIGURE, 2 LOCK, 3 capture, 4 start. */
    uint8_t load_edit_field; /**< 0 lever, 1 mass; independent of every control-request field. */
    uint8_t load_history_offset; /**< Newest-first history selection; never changes motor selection. */
    uint8_t pos_load_flow; /**< Independent all-POS setup and report routing. */
    uint8_t pos_load_result_detail; /**< 0 outcome and recovery, 1 peaks and samples. */
    DebugUiRequest pos_load_saved_draft; /**< POS values retained across the independent ownership review. */
    uint32_t pos_load_check_boot_id; /**< Original boot bound to this ownership review's one permitted epoch rotation. */
    uint8_t pos_load_check_pending; /**< 0 idle, 1 ownership review/request, 2 cancelled by STOP. */
    DebugUiPage load_return_page; /**< Display-only metadata returns to the workflow that opened it. */
    DebugUiLoadTrials load_trials; /**< Display-only per-ID metadata and bounded RAM history. */
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
/** @brief Number of editable parameters, excluding the preview row. */
uint8_t debug_ui_model_parameter_count(const DebugUiModel *model);
/** @brief Map a parameter row to delta/speed/Kp/Kd/time (0..4), or 255. */
uint8_t debug_ui_model_parameter_field(const DebugUiModel *model, uint8_t index);
/** @brief Return whether the result acknowledgement must open control checks. */
uint8_t debug_ui_model_result_requires_check(const DebugUiModel *model);
/** @brief Allow only an idle, unconfigured relock page to cancel its uncommitted preparation intent. */
/** @brief Return the explicitly edited signed output-shaft protocol request. */
static inline float debug_ui_model_single_static_feedforward(const DebugUiModel *model)
{
    float magnitude = (float)model->single_static_ff_centi * 0.01f;
    return model->single_static_ff_negative ? -magnitude : magnitude;
}
uint8_t debug_ui_model_single_static_can_cancel_handoff(const DebugUiModel *model);
/** @brief Return the current gate reason for the selected motor/operation. */
DebugUiReason debug_ui_model_gate(const DebugUiModel *model, DebugUiOperation operation);
/** @brief Intersect all seven published HOLD limits without extending the existing cap. */
uint8_t debug_ui_model_static_duration_bounds(const DebugUiModel *model,
                                             uint32_t *minimum_ms, uint32_t *maximum_ms);
/** @brief Read the selected axis's explicitly configured independent 1..60s bounds. */
uint8_t debug_ui_model_single_static_duration_bounds(const DebugUiModel *model,
                                                    uint32_t *minimum_ms, uint32_t *maximum_ms);
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
