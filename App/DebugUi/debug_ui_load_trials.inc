/** @file debug_ui_load_trials.inc
 * @brief Pure C99 load ledger implementation, included once by debug_ui_model.c.
 * No HAL, RTOS, mailbox, controller, heap allocation or command submission is used.
 */
#include "debug_ui_load_trials.h"
#include <float.h>
#include <string.h>

/** @brief Compare the complete provenance key and reject a missing request identity. */
static uint8_t load_trial_same_identity(const DebugUiIdentity *left, const DebugUiIdentity *right)
{
    return (uint8_t)(left->request_id != 0U && left->origin == right->origin &&
        left->epoch == right->epoch && left->request_id == right->request_id);
}

/** @brief Check floats portably on both ARMCC5 and strict host C99. */
static uint8_t load_trial_finite(float value)
{
    return (uint8_t)(value == value && value <= FLT_MAX && value >= -FLT_MAX);
}

/** @brief Convert a validated one-based record ID to its single-axis mask. */
static uint8_t load_trial_mask(const DebugUiLoadTrialRecord *record)
{
    return (uint8_t)(1U << (record->target_motor_id - 1U));
}

/** @brief Find only an already retained full identity; authority epoch changes cannot erase its result. */
static DebugUiLoadTrialRecord *load_trial_find(DebugUiLoadTrials *ledger, const DebugUiIdentity *identity)
{
    uint8_t index;
    if (!ledger->epoch_seen) return NULL;
    for (index = 0U; index < ledger->count; ++index) {
        DebugUiLoadTrialRecord *record = &ledger->records[index];
        if (load_trial_same_identity(&record->identity, identity)) return record;
    }
    return NULL;
}

/** @brief Retain exact terminal codes, causes and diagnostic fields in bounded compact storage. */
static void load_trial_copy_outcome(DebugUiLoadTrialOutcome *outcome, const DebugUiCompletion *completion)
{
    outcome->code = (uint8_t)completion->code;
    outcome->stage = (uint8_t)completion->stage;
    outcome->reason = (uint8_t)completion->reason;
    outcome->error = completion->error;
    outcome->detail = completion->detail;
    outcome->failed_motor_id = completion->failed_motor_id;
    outcome->disabled_confirmed = completion->disabled_confirmed;
    outcome->seen = 1U;
}

/** @brief Keep STOP cleanup evidence authoritative without replacing the original HOLD cause. */
static void load_trial_apply_stop(DebugUiLoadTrialRecord *record, const DebugUiCompletion *completion)
{
    if (record->stop.seen) return;
    load_trial_copy_outcome(&record->stop, completion);
    record->disabled_known = 1U;
    record->disabled_confirmed = completion->disabled_confirmed;
    if (record->report_seen && !record->report_active) record->report_finalized = 1U;
}

/** @brief Reset volatile history and all explicit candidate confirmations. */
void debug_ui_load_trials_init(DebugUiLoadTrials *ledger)
{
    uint8_t index;
    if (ledger == NULL) return;
    memset(ledger, 0, sizeof(*ledger));
    for (index = 0U; index < DEBUG_UI_MOTOR_COUNT; ++index)
        ledger->candidates[index].lever_mm = 250U;
}

/** @brief Expose candidate metadata without making it writable through the read API. */
const DebugUiLoadMetadata *debug_ui_load_trials_candidate(const DebugUiLoadTrials *ledger, uint8_t motor_id)
{
    if (ledger == NULL || motor_id == 0U || motor_id > DEBUG_UI_MOTOR_COUNT) return NULL;
    return &ledger->candidates[motor_id - 1U];
}

/** @brief Validate a candidate edit and clear confirmation only when a number changes. */
uint8_t debug_ui_load_trials_set(DebugUiLoadTrials *ledger, uint8_t motor_id, uint16_t lever_mm, uint16_t mass_g)
{
    DebugUiLoadMetadata *metadata;
    if (debug_ui_load_trials_candidate(ledger, motor_id) == NULL ||
        lever_mm < DEBUG_UI_LOAD_LEVER_MIN_MM || lever_mm > DEBUG_UI_LOAD_LEVER_MAX_MM ||
        mass_g > DEBUG_UI_LOAD_MASS_MAX_G) return 0U;
    metadata = &ledger->candidates[motor_id - 1U];
    if (metadata->lever_mm != lever_mm || metadata->mass_g != mass_g) {
        metadata->lever_mm = lever_mm;
        metadata->mass_g = mass_g;
        metadata->confirmed = 0U;
    }
    return 1U;
}

/** @brief Confirm a selected RAM candidate without touching any control contract. */
uint8_t debug_ui_load_trials_confirm(DebugUiLoadTrials *ledger, uint8_t motor_id)
{
    if (debug_ui_load_trials_candidate(ledger, motor_id) == NULL) return 0U;
    ledger->candidates[motor_id - 1U].confirmed = 1U;
    return 1U;
}

/** @brief Display the gravity moment of extra hanging mass with no internal/external reduction. */
float debug_ui_load_trials_gravity_nm(const DebugUiLoadMetadata *metadata)
{
    if (metadata == NULL) return 0.0F;
    return ((float)metadata->mass_g * 0.001F) * 9.81F * ((float)metadata->lever_mm * 0.001F);
}

/** @brief Freeze one candidate and its known epoch before a reviewed request receives an identity. */
uint8_t debug_ui_load_trials_freeze(DebugUiLoadTrials *ledger, uint8_t motor_id)
{
    const DebugUiLoadMetadata *metadata = debug_ui_load_trials_candidate(ledger, motor_id);
    if (metadata == NULL) return 0U;
    ledger->frozen = *metadata;
    ledger->frozen_motor_id = motor_id;
    ledger->frozen_epoch = ledger->epoch;
    ledger->frozen_valid = 1U;
    return 1U;
}

/** @brief Create exactly one record at actual single-axis HOLD submission, including later failures. */
uint8_t debug_ui_load_trials_submit(DebugUiLoadTrials *ledger, const DebugUiRequest *request)
{
    DebugUiLoadTrialRecord *record;
    if (ledger == NULL || request == NULL || !ledger->frozen_valid ||
        request->operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
        request->identity.origin != DEBUG_UI_ORIGIN_LOCAL_UI || request->identity.request_id == 0U ||
        request->target_motor_id != ledger->frozen_motor_id || request->hold_duration_ms == 0U) return 0U;
    if (ledger->epoch_seen && (request->identity.epoch != ledger->epoch ||
        ledger->frozen_epoch != ledger->epoch || request->identity.request_id <= ledger->last_submitted_request_id)) return 0U;
    if (load_trial_find(ledger, &request->identity) != NULL) return 0U;
    ledger->epoch = request->identity.epoch;
    ledger->epoch_seen = 1U;
    ledger->last_submitted_request_id = request->identity.request_id;
    record = &ledger->records[ledger->next];
    memset(record, 0, sizeof(*record));
    record->identity = request->identity;
    record->submitted_at_us = request->created_at_us;
    record->duration_ms = request->hold_duration_ms;
    record->target_motor_id = request->target_motor_id;
    record->operation = (uint8_t)request->operation;
    record->metadata = ledger->frozen;
    ledger->next = (uint8_t)((ledger->next + 1U) % DEBUG_UI_LOAD_TRIAL_CAPACITY);
    if (ledger->count < DEBUG_UI_LOAD_TRIAL_CAPACITY) ++ledger->count;
    ledger->frozen_valid = ledger->pending_stop_seen = 0U;
    return 1U;
}

/** @brief Preserve a matching mailbox refusal without manufacturing an asynchronous terminal. */
void debug_ui_load_trials_refused(DebugUiLoadTrials *ledger, const DebugUiRequest *request, DebugUiReason reason)
{
    DebugUiLoadTrialRecord *record;
    if (ledger == NULL || request == NULL || reason == DEBUG_UI_REASON_NONE) return;
    record = load_trial_find(ledger, &request->identity);
    if (record == NULL || request->operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
        request->target_motor_id != record->target_motor_id || request->hold_duration_ms != record->duration_ms ||
        record->hold.seen) return;
    record->hold.seen = record->submit_refused = 1U;
    record->hold.code = (uint8_t)DEBUG_UI_FAILED;
    record->hold.reason = (uint8_t)reason;
}

/** @brief Copy fresh matched report data; identity, mask, duration and sample validity remain separate. */
void debug_ui_load_trials_observe(DebugUiLoadTrials *ledger, const DebugUiSnapshot *snapshot, uint64_t now_us)
{
    DebugUiLoadTrialRecord *record;
    const DebugUiStaticLoadReport *report;
    uint8_t motor_index;
    uint8_t hold_transition;
    if (ledger == NULL || snapshot == NULL || snapshot->timestamp_us > now_us ||
        now_us - snapshot->timestamp_us >= DEBUG_UI_HEALTH_TIMEOUT_US) return;
    if (ledger->boot_seen && snapshot->boot_id != ledger->boot_id) debug_ui_load_trials_init(ledger);
    if (ledger->boot_seen && snapshot->timestamp_us <= ledger->last_snapshot_at_us) return;
    if (ledger->epoch_seen && snapshot->epoch < ledger->epoch) return;
    ledger->boot_seen = 1U;
    ledger->boot_id = snapshot->boot_id;
    ledger->last_snapshot_at_us = snapshot->timestamp_us;
    if (!ledger->epoch_seen || snapshot->epoch != ledger->epoch) {
        ledger->epoch = snapshot->epoch;
        ledger->epoch_seen = 1U;
        ledger->last_submitted_request_id = 0U;
        ledger->frozen_valid = 0U;
    }
    report = &snapshot->static_load;
    record = load_trial_find(ledger, &report->identity);
    /* One forward transition replaces the takeover observation origin with the HOLD
     * origin. All identity/scope/finality gates below still apply; an established
     * HOLD clock never accepts another origin or a phase regression. */
    hold_transition = (uint8_t)(record != NULL && record->report_seen && record->report_active &&
        record->report_phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER &&
        report->phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD && report->takeover_complete && !report->takeover_active &&
        record->elapsed_ms == 0U && report->started_at_us > record->report_started_at_us);
    if (record == NULL || record->submit_refused || report->operation != DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD ||
        report->motor_mask != load_trial_mask(record) || report->duration_ms != record->duration_ms ||
        snapshot->timestamp_us <= record->submitted_at_us || report->started_at_us < record->submitted_at_us ||
        report->started_at_us > snapshot->timestamp_us ||
        (record->report_seen && !hold_transition && (report->started_at_us != record->report_started_at_us ||
         report->elapsed_ms < record->elapsed_ms ||
         (record->report_phase == DEBUG_UI_STATIC_LOAD_PHASE_HOLD &&
          report->phase == DEBUG_UI_STATIC_LOAD_PHASE_TAKEOVER)))) return;
    if (!record->report_finalized) {
        motor_index = (uint8_t)(record->target_motor_id - 1U);
        record->report_seen = 1U;
        record->report_phase = (uint8_t)report->phase;
        record->report_started_at_us = report->started_at_us;
        record->elapsed_ms = report->elapsed_ms;
        record->report_active = report->active;
        record->report_complete = report->complete;
        record->report_stopped = report->stopped;
        record->measurements_valid = (uint8_t)((report->valid_mask & load_trial_mask(record)) != 0U &&
            report->samples[motor_index] > 0U && load_trial_finite(report->peak_error_rad[motor_index]) &&
            load_trial_finite(report->peak_torque_nm[motor_index]) && load_trial_finite(report->peak_mos_c[motor_index]) &&
            load_trial_finite(report->peak_rotor_c[motor_index]));
        if (record->measurements_valid) {
            record->samples = report->samples[motor_index];
            record->peak_error_rad = report->peak_error_rad[motor_index];
            record->peak_torque_nm = report->peak_torque_nm[motor_index];
            record->peak_mos_c = report->peak_mos_c[motor_index];
            record->peak_rotor_c = report->peak_rotor_c[motor_index];
        }
        if (!report->active && (record->hold.seen || record->stop.seen || report->complete || report->stopped))
            record->report_finalized = 1U;
    }
    if (report->complete && !record->stop.seen && !record->hold.seen) {
        record->disabled_known = 1U;
        record->disabled_confirmed = report->disabled_confirmed;
    }
    if (!record->stop.seen && report->stop_identity.request_id != 0U &&
        report->stop_identity.origin == DEBUG_UI_ORIGIN_LOCAL_UI &&
        !load_trial_same_identity(&report->stop_identity, &record->identity))
        record->stop_identity = report->stop_identity;
    if (ledger->pending_stop_seen && record == &ledger->records[ledger->pending_stop_record] &&
        load_trial_same_identity(&record->stop_identity, &ledger->pending_stop.identity)) {
        load_trial_apply_stop(record, &ledger->pending_stop);
        ledger->pending_stop_seen = 0U;
    }
}

/** @brief Accept matching HOLD terminals or an actual STOP whose report association is explicit. */
void debug_ui_load_trials_completion(DebugUiLoadTrials *ledger, const DebugUiCompletion *completion,
                                     const DebugUiRequest *stop_request)
{
    DebugUiLoadTrialRecord *record;
    if (ledger == NULL || completion == NULL) return;
    if (completion->operation == DEBUG_UI_OPERATION_SINGLE_STATIC_HOLD) {
        record = load_trial_find(ledger, &completion->identity);
        if (record == NULL || record->hold.seen || completion->motor_mask != load_trial_mask(record) ||
            completion->timestamp_us < record->submitted_at_us) return;
        load_trial_copy_outcome(&record->hold, completion);
        if (record->report_seen && !record->report_active) record->report_finalized = 1U;
        if (!record->stop.seen) {
            record->disabled_known = 1U;
            record->disabled_confirmed = completion->disabled_confirmed;
        }
        return;
    }
    if (completion->operation != DEBUG_UI_OPERATION_STOP || stop_request == NULL ||
        stop_request->operation != DEBUG_UI_OPERATION_STOP || stop_request->identity.origin != DEBUG_UI_ORIGIN_LOCAL_UI ||
        !load_trial_same_identity(&completion->identity, &stop_request->identity) ||
        !ledger->epoch_seen || ledger->count == 0U) return;
    record = &ledger->records[(ledger->next + DEBUG_UI_LOAD_TRIAL_CAPACITY - 1U) % DEBUG_UI_LOAD_TRIAL_CAPACITY];
    if (record->target_motor_id != stop_request->target_motor_id ||
        completion->motor_mask != load_trial_mask(record) || record->submit_refused || record->stop.seen ||
        stop_request->created_at_us < record->submitted_at_us || completion->timestamp_us < stop_request->created_at_us ||
        load_trial_same_identity(&record->identity, &completion->identity)) return;
    if (record->stop_identity.request_id != 0U) {
        if (load_trial_same_identity(&record->stop_identity, &completion->identity))
            load_trial_apply_stop(record, completion);
    } else if (!ledger->pending_stop_seen) {
        ledger->pending_stop = *completion;
        ledger->pending_stop_record = (uint8_t)(record - ledger->records);
        ledger->pending_stop_seen = 1U;
    }
}

/** @brief Map newest-first history to fixed ring slots without exposing mutable records. */
const DebugUiLoadTrialRecord *debug_ui_load_trials_newest(const DebugUiLoadTrials *ledger, uint8_t offset)
{
    if (ledger == NULL || offset >= ledger->count) return NULL;
    return &ledger->records[(ledger->next + DEBUG_UI_LOAD_TRIAL_CAPACITY - 1U - offset) % DEBUG_UI_LOAD_TRIAL_CAPACITY];
}
