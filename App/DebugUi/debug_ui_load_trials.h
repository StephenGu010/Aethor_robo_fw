/** @file debug_ui_load_trials.h
 * @brief Bounded, display-only single-axis load metadata and recent RAM history.
 * These values never enter control requests, permissions, gains or protection limits.
 */
#ifndef APP_DEBUG_UI_DEBUG_UI_LOAD_TRIALS_H
#define APP_DEBUG_UI_DEBUG_UI_LOAD_TRIALS_H

#include <stddef.h>
#include "debug_ui_contract.h"

#define DEBUG_UI_LOAD_TRIAL_CAPACITY (16U)
#define DEBUG_UI_LOAD_TRIALS_MAX_BYTES (2048U)
#define DEBUG_UI_LOAD_LEVER_MIN_MM (1U)
#define DEBUG_UI_LOAD_LEVER_MAX_MM (2000U)
#define DEBUG_UI_LOAD_MASS_MAX_G (10000U)

/** @brief User-entered perpendicular horizontal lever and extra hanging mass, without gearing. */
typedef struct
{
    uint16_t lever_mm;
    uint16_t mass_g;
    uint8_t confirmed; /**< Defaults are placeholders until explicitly confirmed. */
} DebugUiLoadMetadata;

/** @brief Compact copy of an actual terminal; enum values retain their contract meaning. */
typedef struct
{
    uint16_t detail;
    uint8_t code;
    uint8_t stage;
    uint8_t reason;
    uint8_t error;
    uint8_t failed_motor_id;
    uint8_t disabled_confirmed;
    uint8_t seen;
} DebugUiLoadTrialOutcome;

/** @brief One immutable submission scope with separately evidenced report and terminal data. */
typedef struct
{
    DebugUiIdentity identity;
    DebugUiIdentity stop_identity;
    uint64_t submitted_at_us;
    uint64_t report_started_at_us;
    uint32_t duration_ms;
    uint32_t elapsed_ms; /**< Application HOLD clock; does not prove time under physical load. */
    uint32_t samples;
    float peak_error_rad;
    float peak_torque_nm;
    float peak_mos_c;
    float peak_rotor_c;
    DebugUiLoadMetadata metadata;
    DebugUiLoadTrialOutcome hold;
    DebugUiLoadTrialOutcome stop;
    uint8_t target_motor_id;
    uint8_t operation; /**< Frozen contract operation, currently SINGLE_STATIC_HOLD only. */
    uint8_t report_seen;
    uint8_t report_phase; /**< Only an evidenced TAKEOVER-to-HOLD transition may change the report clock origin. */
    uint8_t report_active;
    uint8_t report_finalized; /**< A terminal plus inactive matched report seals timing and peaks. */
    uint8_t measurements_valid;
    uint8_t report_complete;
    uint8_t report_stopped;
    uint8_t disabled_known;
    uint8_t disabled_confirmed;
    uint8_t submit_refused;
} DebugUiLoadTrialRecord;

/** @brief UI-owned fixed storage: per-ID candidates, one review and sixteen newest-first records. */
typedef struct
{
    DebugUiLoadTrialRecord records[DEBUG_UI_LOAD_TRIAL_CAPACITY];
    DebugUiLoadMetadata candidates[DEBUG_UI_MOTOR_COUNT];
    DebugUiLoadMetadata frozen;
    DebugUiCompletion pending_stop; /**< One accepted STOP awaiting explicit report provenance. */
    uint64_t last_snapshot_at_us;
    uint32_t epoch;
    uint32_t boot_id;
    uint32_t frozen_epoch;
    uint32_t last_submitted_request_id;
    uint8_t count;
    uint8_t next;
    uint8_t epoch_seen;
    uint8_t boot_seen;
    uint8_t frozen_valid;
    uint8_t frozen_motor_id;
    uint8_t pending_stop_seen;
    uint8_t pending_stop_record;
} DebugUiLoadTrials;

/** @brief C99/ARMCC5 compile-time bound includes every byte of additional model storage. */
typedef char DebugUiLoadTrialsStorageBound[(sizeof(DebugUiLoadTrials) <= DEBUG_UI_LOAD_TRIALS_MAX_BYTES) ? 1 : -1];

/** @brief Reset RAM history and create unconfirmed 250mm/0g candidates for IDs 1..7. */
void debug_ui_load_trials_init(DebugUiLoadTrials *ledger);
/** @brief Return a read-only one-based candidate, or NULL for an invalid ID. */
const DebugUiLoadMetadata *debug_ui_load_trials_candidate(const DebugUiLoadTrials *ledger, uint8_t motor_id);
/** @brief Validate candidate bounds; an actual value change clears its confirmation. */
uint8_t debug_ui_load_trials_set(DebugUiLoadTrials *ledger, uint8_t motor_id, uint16_t lever_mm, uint16_t mass_g);
/** @brief Explicitly confirm the selected candidate without submitting any action. */
uint8_t debug_ui_load_trials_confirm(DebugUiLoadTrials *ledger, uint8_t motor_id);
/** @brief Calculate extra-mass gravity torque only: kg * 9.81 * perpendicular lever metres. */
float debug_ui_load_trials_gravity_nm(const DebugUiLoadMetadata *metadata);
/** @brief Freeze the current candidate on entry to a single-axis HOLD review. */
uint8_t debug_ui_load_trials_freeze(DebugUiLoadTrials *ledger, uint8_t motor_id);
/** @brief Bind a frozen review to one actual identified SINGLE_STATIC_HOLD, once per identity. */
uint8_t debug_ui_load_trials_submit(DebugUiLoadTrials *ledger, const DebugUiRequest *request);
/** @brief Record a matching submit refusal with its real reason and no invented disable receipt. */
void debug_ui_load_trials_refused(DebugUiLoadTrials *ledger, const DebugUiRequest *request, DebugUiReason reason);
/** @brief Observe fresh, monotonic snapshots; retained scopes can finish across authority epoch changes. */
void debug_ui_load_trials_observe(DebugUiLoadTrials *ledger, const DebugUiSnapshot *snapshot, uint64_t now_us);
/** @brief Observe an accepted terminal; STOP additionally requires the actual outgoing STOP request.
 * STOP is applied only after the matched report explicitly names its complete stop_identity.
 */
void debug_ui_load_trials_completion(DebugUiLoadTrials *ledger, const DebugUiCompletion *completion,
                                     const DebugUiRequest *stop_request);
/** @brief Return read-only history by newest-first offset; no application state is referenced. */
const DebugUiLoadTrialRecord *debug_ui_load_trials_newest(const DebugUiLoadTrials *ledger, uint8_t offset);

#endif
