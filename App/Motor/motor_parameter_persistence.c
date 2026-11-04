/** @file motor_parameter_persistence.c
 * @brief Fixed ID7 transactional persistence; no HAL, arbitrary writes, retry, restore or enable. */
#include "motor_parameter_persistence.h"
#include <string.h>
#include <math.h>
/** @brief Compares complete owner provenance rather than relying on identical CAN bytes. */
uint8_t motor_persistence_identity_equal(const MotorPersistenceIdentity *left, const MotorPersistenceIdentity *right)
{
    return (uint8_t)(left && right && left->boot_epoch == right->boot_epoch &&
        left->request_id == right->request_id && left->owner_generation == right->owner_generation &&
        left->sequence == right->sequence && left->phase == right->phase);
}
/** @brief Keeps the slot pinned even when the initial pending evidence is insufficient. */
uint8_t motor_persistence_tracker_arm(MotorPersistenceTracker *tracker, const MotorPersistenceIdentity *identity,
    uint32_t buffer_mask, uint32_t pending_mask, uint32_t transmitted_mask)
{
    if (!tracker || !identity || tracker->active || !identity->boot_epoch || !identity->request_id ||
        !identity->owner_generation || !identity->sequence || !identity->phase ||
        !buffer_mask || (buffer_mask & (buffer_mask - 1U))) { return 0U; }
    tracker->identity = *identity;
    tracker->buffer_mask = buffer_mask;
    tracker->active = 1U;
    tracker->cancelled = 0U;
    tracker->pending_seen = (uint8_t)((pending_mask & buffer_mask) != 0U);
    tracker->pending_proven = (uint8_t)((pending_mask & buffer_mask) && !(transmitted_mask & buffer_mask));
    return 1U;
}
/** @brief Makes any later completion unsuccessful without prematurely freeing hardware ownership. */
void motor_persistence_tracker_cancel(MotorPersistenceTracker *tracker)
{ if (tracker && tracker->active) { tracker->cancelled = 1U; } }
/** @brief Retains accepted but not-yet-visible requests until observed pending actually clears. */
uint8_t motor_persistence_tracker_collect(MotorPersistenceTracker *tracker,
    uint32_t pending_mask, uint32_t transmitted_mask, uint32_t cancelled_mask,
    MotorPersistenceIdentity *identity, uint8_t *succeeded)
{
    if (!tracker || !identity || !succeeded || !tracker->active) { return 0U; }
    if (pending_mask & tracker->buffer_mask) {
        tracker->pending_seen = 1U;
        if (!tracker->cancelled && !((transmitted_mask | cancelled_mask) & tracker->buffer_mask)) {
            tracker->pending_proven = 1U;
        }
        return 0U;
    }
    /* TXBAR can precede TXBRP publication. Even cancellation or an old completion
     * bit cannot make that invisible accepted request safe to reuse. */
    if (!tracker->pending_seen) { return 0U; }
    *identity = tracker->identity;
    *succeeded = (uint8_t)(tracker->pending_proven && !tracker->cancelled &&
        (transmitted_mask & tracker->buffer_mask) && !(cancelled_mask & tracker->buffer_mask));
    tracker->active = 0U;
    return 1U;
}
/** @brief Reads native float bits without pointer aliasing. */
static float persistence_float(uint32_t raw)
{ float value; memcpy(&value, &raw, sizeof(value)); return value; }
/** @brief Requires complete finite observations and exact known motor identity/ranges. */
uint8_t motor_persistence_validate(const MotorPersistenceSnapshot *snapshot, uint8_t require_target)
{
    unsigned index;
    float position_range;
    if (!snapshot) { return 0U; }
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        if (!snapshot->items[index].valid || (!motor_persistence_is_unsigned(index) &&
            !isfinite(persistence_float(snapshot->items[index].raw)))) { return 0U; }
    }
    position_range = persistence_float(snapshot->items[21].raw);
    return (uint8_t)(snapshot->items[7].raw == 23U && snapshot->items[8].raw == 7U &&
        snapshot->items[14].raw == 808597046U && snapshot->items[36].raw == 49U &&
        persistence_float(snapshot->items[22].raw) == 200.0F &&
        persistence_float(snapshot->items[23].raw) == 10.0F &&
        (require_target ? snapshot->items[10].raw == 2U && position_range == 512.0F :
        (snapshot->items[10].raw == 1U || snapshot->items[10].raw == 2U) &&
        (position_range == 12.5F || position_range == 512.0F)));
}
/** @brief Allows precisely PMAX512 and mode2 deltas while preserving the other24 raw words. */
uint8_t motor_persistence_configuration_matches(const MotorPersistenceSnapshot *baseline,
    const MotorPersistenceSnapshot *current, uint8_t allow_targets)
{
    unsigned index;
    if (!motor_persistence_validate(baseline, 0U) || !motor_persistence_validate(current, allow_targets)) {
        return 0U;
    }
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        if (!motor_persistence_is_configuration(index) || (allow_targets && (index == 10U || index == 21U))) {
            continue;
        }
        if (baseline->items[index].raw != current->items[index].raw) { return 0U; }
    }
    return 1U;
}
/** @brief Maps dense export indices to the exact published register whitelist. */
uint8_t motor_persistence_rid(unsigned index)
{
    if (index < 37U) { return (uint8_t)index; }
    if (index < 43U) { return (uint8_t)(index + 13U); }
    if (index < 45U) { return (uint8_t)(index + 37U); }
    return 255U;
}
/** @brief Preserves native unsigned representation for vendor integer registers. */
uint8_t motor_persistence_is_unsigned(unsigned index)
{
    uint8_t register_address = motor_persistence_rid(index);
    return (uint8_t)((register_address >= 7U && register_address <= 10U) ||
        (register_address >= 13U && register_address <= 16U) ||
        register_address == 35U || register_address == 36U);
}
/** @brief Marks the 26 public RW values for comparison, never for generic writes. */
uint8_t motor_persistence_is_configuration(unsigned index)
{
    uint8_t register_address = motor_persistence_rid(index);
    return (uint8_t)(register_address <= 10U ||
        (register_address >= 21U && register_address <= 35U));
}

/** @brief Terminates irreversibly while preserving Save and uncertain-value latches. */
static void persistence_abort(MotorPersistenceTransaction *transaction, MotorPersistenceError error)
{
    transaction->state = MOTOR_PERSISTENCE_ABORTED;
    transaction->error = error;
    transaction->external_disable_required = 1U;
    transaction->pending = 0U;
}
/** @brief Checks monotonic clocks and fixed budgets without renewing them on activity. */
static uint8_t persistence_alive(MotorPersistenceTransaction *transaction, uint64_t now_us)
{
    if (!transaction || transaction->state == MOTOR_PERSISTENCE_IDLE ||
        transaction->state == MOTOR_PERSISTENCE_COMPLETE || transaction->state == MOTOR_PERSISTENCE_ABORTED) {
        return 0U;
    }
    if (now_us < transaction->last_us) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_CLOCK); return 0U;
    }
    transaction->last_us = now_us;
    if (now_us >= transaction->total_deadline_us || now_us >= transaction->phase_deadline_us ||
        (transaction->pending && now_us >= transaction->frame_deadline_us)) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_TIMEOUT); return 0U;
    }
    return 1U;
}
/** @brief Starts one bounded read set, using only the single persistent observation buffer. */
static void persistence_read_start(MotorPersistenceTransaction *transaction, MotorPersistenceState state)
{
    memset(&transaction->current, 0, sizeof(transaction->current));
    transaction->read_index = 0U;
    transaction->state = state;
}
/** @brief Compares compact baseline configuration, permitting precisely the required target words. */
static uint8_t persistence_matches(const MotorPersistenceTransaction *transaction)
{
    unsigned index;
    if (!motor_persistence_validate(&transaction->current, 1U)) { return 0U; }
    for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
        if (motor_persistence_is_configuration(index) && index != 10U && index != 21U &&
            transaction->baseline_raw[index] != transaction->current.items[index].raw) { return 0U; }
    }
    return 1U;
}
/** @brief Advances only fully confirmed write steps, skipping unchanged fixed targets. */
static void persistence_write_advance(MotorPersistenceTransaction *transaction)
{
    if (transaction->write_step == 0U && transaction->baseline_raw[21] == 0x44000000U) {
        ++transaction->write_step;
    }
    if (transaction->write_step == 1U && transaction->baseline_raw[10] == 2U) {
        ++transaction->write_step;
    }
    if (transaction->write_step == 2U) {
        persistence_read_start(transaction, MOTOR_PERSISTENCE_PREPARE_VERIFY);
    }
}
/** @brief Resolves one correlated reply only after actual TX evidence and time ordering. */
static void persistence_finish_reply(MotorPersistenceTransaction *transaction)
{
    MotorPersistenceItem *item;
    unsigned index;
    uint64_t now_us = transaction->last_us;
    if (!transaction->actual_tx || !transaction->deferred) { return; }
    /* TXBTO polling may occur after RX delivery. No physical transmit timestamp is invented. */
    transaction->pending = 0U;
    if (transaction->state == MOTOR_PERSISTENCE_SAVE_WAIT) {
        transaction->state = MOTOR_PERSISTENCE_SETTLE;
        /* Wait 30ms after ACK observation, conservatively later than actual TX. */
        transaction->settle_until_us = now_us + 30000U;
        return;
    }
    if (transaction->state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        uint8_t position_write = (uint8_t)(transaction->write_step == 0U);
        uint32_t expected = position_write ? 0x44000000U : 2U;
        if (transaction->deferred_raw != expected) {
            persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_SNAPSHOT); return;
        }
        if (position_write) {
            transaction->confirmed_pmax_raw = expected; transaction->pmax_known = 1U;
        } else { transaction->confirmed_mode = expected; transaction->mode_known = 1U; }
        ++transaction->write_step;
        persistence_write_advance(transaction);
        return;
    }
    item = &transaction->current.items[transaction->read_index];
    item->identity = transaction->identity;
    item->created_us = transaction->created_us; item->submitted_us = transaction->submitted_us;
    item->boundary_us = transaction->boundary_us;
    item->send_completed_observed_us = transaction->send_completed_observed_us;
    item->rx_lower_us = transaction->deferred_lower_us; item->received_us = transaction->deferred_received_us;
    item->raw = transaction->deferred_raw; item->valid = 1U;
    if (++transaction->read_index < MOTOR_PERSISTENCE_REGISTER_COUNT) { return; }
    if (transaction->state == MOTOR_PERSISTENCE_BASELINE) {
        if (!motor_persistence_validate(&transaction->current, 0U)) {
            persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_SNAPSHOT); return;
        }
        for (index = 0U; index < MOTOR_PERSISTENCE_REGISTER_COUNT; ++index) {
            transaction->baseline_raw[index] = transaction->current.items[index].raw;
        }
        transaction->confirmed_pmax_raw = transaction->baseline_raw[21];
        transaction->confirmed_mode = transaction->baseline_raw[10];
        transaction->pmax_known = transaction->mode_known = 1U;
        transaction->skip_save = motor_persistence_validate(&transaction->current, 1U);
        transaction->state = transaction->skip_save ? MOTOR_PERSISTENCE_COMPLETE : MOTOR_PERSISTENCE_BASELINE_READY;
        transaction->phase_deadline_us = now_us + MOTOR_PERSISTENCE_READY_BUDGET_US;
        return;
    }
    if (!persistence_matches(transaction)) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_SNAPSHOT); return;
    }
    if (transaction->state == MOTOR_PERSISTENCE_PREPARE_VERIFY) {
        transaction->state = MOTOR_PERSISTENCE_PREPARED;
        transaction->phase_deadline_us = now_us + MOTOR_PERSISTENCE_READY_BUDGET_US;
    } else if (transaction->state == MOTOR_PERSISTENCE_SAVE_VERIFY) {
        transaction->state = MOTOR_PERSISTENCE_SAVE_HEALTH;
        transaction->verified_us = now_us;
    } else if (transaction->state == MOTOR_PERSISTENCE_READBACK) {
        transaction->state = MOTOR_PERSISTENCE_COMPLETE;
        transaction->save_uncertain = 0U;
    }
}
/** @brief Resets storage only on explicit boot initialization; never called by abort or begin. */
void motor_persistence_init(MotorPersistenceTransaction *transaction)
{ if (transaction) { memset(transaction, 0, sizeof(*transaction)); } }
/** @brief Begins one bounded all-register baseline under fixed nonzero ownership. */
uint8_t motor_persistence_begin(MotorPersistenceTransaction *transaction, uint32_t boot_epoch,
    uint32_t request_id, uint32_t owner_generation, uint64_t now_us)
{
    if (!transaction || transaction->state != MOTOR_PERSISTENCE_IDLE || transaction->save_used ||
        !boot_epoch || !request_id || !owner_generation || now_us > UINT64_MAX - MOTOR_PERSISTENCE_TOTAL_BUDGET_US) {
        return 0U;
    }
    transaction->identity.boot_epoch = boot_epoch; transaction->identity.request_id = request_id;
    transaction->identity.owner_generation = owner_generation;
    transaction->last_us = now_us;
    transaction->total_deadline_us = now_us + MOTOR_PERSISTENCE_TOTAL_BUDGET_US;
    transaction->phase_deadline_us = now_us + MOTOR_PERSISTENCE_READ_BUDGET_US;
    persistence_read_start(transaction, MOTOR_PERSISTENCE_BASELINE);
    return 1U;
}
/** @brief Invalidates STOP/ownership immediately and moves settled Save into readback. */
void motor_persistence_tick(MotorPersistenceTransaction *transaction, uint64_t now_us,
    uint32_t boot_epoch, uint32_t owner_generation, uint8_t stop)
{
    if (!persistence_alive(transaction, now_us)) { return; }
    if (stop) { persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_STOP); return; }
    if (boot_epoch != transaction->identity.boot_epoch || owner_generation != transaction->identity.owner_generation) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_CONTEXT); return;
    }
    if (transaction->state == MOTOR_PERSISTENCE_SETTLE && now_us >= transaction->settle_until_us) {
        persistence_read_start(transaction, MOTOR_PERSISTENCE_READBACK);
    }
}
/** @brief Explicit preparation is available only after a complete validated baseline. */
uint8_t motor_persistence_prepare(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us)
{
    if (!persistence_alive(transaction, now_us) || transaction->state != MOTOR_PERSISTENCE_BASELINE_READY ||
        !motor_persistence_health_valid(transaction, health, now_us, 0U)) { return 0U; }
    transaction->health = *health;
    transaction->state = MOTOR_PERSISTENCE_PREPARE_WRITE;
    transaction->phase_deadline_us = now_us + MOTOR_PERSISTENCE_PREPARE_BUDGET_US;
    transaction->write_step = 0U;
    persistence_write_advance(transaction);
    return 1U;
}
/** @brief Replaces health only while preparing; invalid refresh irreversibly revokes admission. */
uint8_t motor_persistence_refresh_health(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us)
{
    if (!persistence_alive(transaction, now_us) || transaction->state != MOTOR_PERSISTENCE_PREPARE_WRITE) { return 0U; }
    if (!motor_persistence_health_valid(transaction, health, now_us, 0U)) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_HEALTH); return 0U;
    }
    transaction->health = *health;
    return 1U;
}
/** @brief Applies the exact disabled safety envelope with no guessed register OV range. */
uint8_t motor_persistence_health_valid(const MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us, uint64_t earliest_us)
{
    return (uint8_t)(transaction && health && health->valid && health->disabled && health->idle &&
        !health->fault && health->boot_epoch == transaction->identity.boot_epoch &&
        health->observed_us >= earliest_us && health->observed_us <= now_us &&
        now_us - health->observed_us <= 100000U && isfinite(health->velocity) &&
        isfinite(health->torque) && isfinite(health->motor_temperature) && isfinite(health->driver_temperature) &&
        fabsf(health->velocity) <= 0.05F && fabsf(health->torque) <= 0.15F &&
        health->motor_temperature < 50.0F && health->driver_temperature < 50.0F);
}
/** @brief Starts a fresh full read before any irreversible command is made available. */
uint8_t motor_persistence_request_save(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us)
{
    if (!persistence_alive(transaction, now_us) || transaction->state != MOTOR_PERSISTENCE_PREPARED ||
        transaction->save_used || !motor_persistence_health_valid(transaction, health, now_us, 0U)) { return 0U; }
    transaction->phase_deadline_us = now_us + MOTOR_PERSISTENCE_SAVE_BUDGET_US;
    persistence_read_start(transaction, MOTOR_PERSISTENCE_SAVE_VERIFY);
    return 1U;
}
/** @brief Requires a strictly post-verification feedback refresh before permitting AA preparation. */
uint8_t motor_persistence_authorize_save(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us)
{
    if (!persistence_alive(transaction, now_us) || transaction->state != MOTOR_PERSISTENCE_SAVE_HEALTH ||
        transaction->save_used || !motor_persistence_health_valid(transaction, health, now_us, transaction->verified_us + 1U)) {
        return 0U;
    }
    transaction->health = *health;
    transaction->state = MOTOR_PERSISTENCE_SAVE_WAIT;
    return 1U;
}
/** @brief Constructs only the fixed read whitelist, two target writes or single AA command. */
uint8_t motor_persistence_next(MotorPersistenceTransaction *transaction, uint64_t now_us,
    CanFrame *frame, MotorPersistenceIdentity *identity)
{
    uint32_t raw;
    unsigned index;
    if (!frame || !identity || !persistence_alive(transaction, now_us) || transaction->pending) { return 0U; }
    if (!transaction->pending) {
        if (transaction->state != MOTOR_PERSISTENCE_BASELINE && transaction->state != MOTOR_PERSISTENCE_PREPARE_WRITE &&
            transaction->state != MOTOR_PERSISTENCE_PREPARE_VERIFY && transaction->state != MOTOR_PERSISTENCE_SAVE_VERIFY &&
            transaction->state != MOTOR_PERSISTENCE_SAVE_WAIT && transaction->state != MOTOR_PERSISTENCE_READBACK) { return 0U; }
        memset(&transaction->frame, 0, sizeof(transaction->frame));
        transaction->frame.identifier = 0x7ffU; transaction->frame.length = 4U;
        transaction->frame.data[0] = 7U;
        if (transaction->state == MOTOR_PERSISTENCE_SAVE_WAIT) {
            if (transaction->save_used) { return 0U; }
            transaction->frame.data[2] = 0xaaU;
            transaction->frame.data[3] = 1U;
        } else if (transaction->state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
            transaction->frame.length = 8U; transaction->frame.data[2] = 0x55U;
            transaction->frame.data[3] = transaction->write_step == 0U ? 21U : 10U;
            raw = transaction->write_step == 0U ? 0x44000000U : 2U;
            for (index = 0U; index < 4U; ++index) { transaction->frame.data[index + 4U] = (uint8_t)(raw >> (8U * index)); }
        } else {
            transaction->frame.data[2] = 0x33U;
            transaction->frame.data[3] = motor_persistence_rid(transaction->read_index);
        }
        transaction->identity.phase = (uint8_t)transaction->state;
        ++transaction->identity.sequence;
        transaction->created_us = now_us;
        transaction->frame_deadline_us = now_us + MOTOR_PERSISTENCE_FRAME_BUDGET_US;
        transaction->submitted = transaction->actual_tx = transaction->deferred = 0U;
        transaction->pending = 1U;
    }
    if (transaction->submitted) { return 0U; }
    *frame = transaction->frame; *identity = transaction->identity;
    return 1U;
}
/** @brief Latches submission once only with an empty boundary no older than frame preparation. */
uint8_t motor_persistence_submitted(MotorPersistenceTransaction *transaction,
    const MotorPersistenceIdentity *identity, uint64_t now_us, uint64_t empty_boundary_us, uint8_t rx_empty)
{
    if (!persistence_alive(transaction, now_us) || !transaction->pending || transaction->submitted ||
        !motor_persistence_identity_equal(identity, &transaction->identity) || !rx_empty ||
        empty_boundary_us < transaction->created_us || empty_boundary_us > now_us) { return 0U; }
    if (transaction->state == MOTOR_PERSISTENCE_SAVE_WAIT) {
        if (transaction->save_used || !motor_persistence_health_valid(transaction, &transaction->health,
            now_us, transaction->verified_us + 1U)) {
            persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_HEALTH); return 0U;
        }
        transaction->save_used = transaction->save_uncertain = 1U;
    }
    if (transaction->state == MOTOR_PERSISTENCE_PREPARE_WRITE) {
        if (!motor_persistence_health_valid(transaction, &transaction->health, now_us, 0U)) {
            persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_HEALTH); return 0U;
        }
        if (transaction->write_step == 0U) { transaction->pmax_known = 0U; }
        else { transaction->mode_known = 0U; }
    }
    transaction->submitted = 1U; transaction->submitted_us = now_us;
    transaction->boundary_us = empty_boundary_us;
    return 1U;
}
/** @brief Matches trusted hardware completion against every immutable identity field. */
uint8_t motor_persistence_report_tx(MotorPersistenceTransaction *transaction,
    const MotorPersistenceIdentity *identity, uint64_t completed_observed_us, uint64_t now_us, uint8_t succeeded)
{
    if (!persistence_alive(transaction, now_us) || !transaction->pending || !transaction->submitted ||
        transaction->actual_tx || !motor_persistence_identity_equal(identity, &transaction->identity)) { return 0U; }
    if (!succeeded || completed_observed_us < transaction->submitted_us || completed_observed_us > now_us) {
        persistence_abort(transaction, MOTOR_PERSISTENCE_ERROR_TX); return 0U;
    }
    transaction->send_completed_observed_us = completed_observed_us; transaction->actual_tx = 1U;
    persistence_finish_reply(transaction);
    return 1U;
}
/** @brief Accepts only the pending exact wire reply; identical post-TX stale replies remain unresolvable. */
uint8_t motor_persistence_accept(MotorPersistenceTransaction *transaction, const CanFrame *frame,
    uint8_t is_standard_data, uint64_t rx_lower_us, uint64_t received_us)
{
    uint32_t raw = 0U;
    unsigned index;
    if (!frame || !persistence_alive(transaction, received_us) || !transaction->pending ||
        !transaction->submitted || transaction->deferred || !is_standard_data || frame->identifier != 23U ||
        frame->data[0] != 7U || frame->data[1] != 0U || rx_lower_us > received_us ||
        rx_lower_us < transaction->boundary_us || rx_lower_us < transaction->submitted_us) { return 0U; }
    if (transaction->state == MOTOR_PERSISTENCE_SAVE_WAIT) {
        if (frame->length != 4U || frame->data[2] != 0xaaU || frame->data[3] != 1U) { return 0U; }
    } else {
        /* The driver echoes 0x55 for a write and 0x33 for a read. A read-shaped
         * response cannot confirm the outstanding immutable write request. */
        if (frame->length != 8U || frame->data[2] != transaction->frame.data[2] ||
            frame->data[3] != transaction->frame.data[3]) { return 0U; }
        for (index = 0U; index < 4U; ++index) { raw |= (uint32_t)frame->data[4U + index] << (8U * index); }
    }
    transaction->deferred_raw = raw; transaction->deferred_lower_us = rx_lower_us;
    transaction->deferred_received_us = received_us; transaction->deferred = 1U;
    persistence_finish_reply(transaction);
    return 1U;
}
