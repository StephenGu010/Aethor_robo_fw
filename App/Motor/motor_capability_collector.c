/** @file motor_capability_collector.c
 * @brief Bounded motor-7 capability collection with exact TX/reply matching and no retry. */
#include "motor_capability_collector.h"
#include <float.h>
#include <string.h>
/** @brief Returns a fixed vendor RID without exposing arbitrary register selection. */
uint8_t motor_capability_rid(uint8_t index)
{
    static const uint8_t registers[MOTOR_CAPABILITY_COUNT] = {1U, 2U, 3U, 11U, 12U, 20U};
    return index < MOTOR_CAPABILITY_COUNT ? registers[index] : 0U;
}
/** @brief Initializes static diagnostic storage before this boot's first request. */
void motor_capability_init(MotorCapabilityCollector *collector)
{ if (collector != NULL) { memset(collector, 0, sizeof(*collector)); } }
/** @brief Keeps terminal data and the boot latch when an active sequence fails. */
void motor_capability_abort(MotorCapabilityCollector *collector, MotorCapabilityReason reason)
{
    if (collector != NULL && (collector->state == MOTOR_CAPABILITY_COLLECTING ||
        (collector->used && collector->state == MOTOR_CAPABILITY_IDLE)))
    { collector->state = MOTOR_CAPABILITY_ABORTED; collector->reason = reason;
      collector->outstanding = 0U; collector->deferred = 0U; }
}
/** @brief Records the first abnormal reason once without erasing or relabelling completed observations. */
void motor_capability_revoke_completed(MotorCapabilityCollector *collector, MotorCapabilityReason reason)
{
    if (collector != NULL && collector->state == MOTOR_CAPABILITY_COMPLETE && !collector->completed_revoked)
    {
        collector->completed_revoked = 1U;
        if (collector->reason == MOTOR_CAPABILITY_REASON_NONE) { collector->reason = reason; }
    }
}
/** @brief Starts exactly once so no later same-RID request can collide with old traffic. */
uint8_t motor_capability_start(MotorCapabilityCollector *collector, uint32_t request_id,
    uint32_t epoch, uint64_t timestamp_us)
{
    if (!timestamp_us || !motor_capability_reserve(collector, request_id, epoch)) { return 0U; }
    return motor_capability_start_reserved(collector, timestamp_us);
}
/** @brief Latches identity and epoch once per boot without starting or extending a read deadline. */
uint8_t motor_capability_reserve(MotorCapabilityCollector *collector, uint32_t request_id, uint32_t epoch)
{
    if (collector == NULL || collector->used || !request_id || !epoch) { return 0U; }
    collector->used = 1U; collector->request_id = request_id; collector->epoch = epoch;
    return 1U;
}
/** @brief Starts the original 20/144ms budgets once; preparation elapsed time is tracked separately. */
uint8_t motor_capability_start_reserved(MotorCapabilityCollector *collector, uint64_t timestamp_us)
{
    if (collector == NULL || !collector->used || collector->state != MOTOR_CAPABILITY_IDLE ||
        collector->started_us || !timestamp_us) { return 0U; }
    collector->started_us = timestamp_us; collector->state = MOTOR_CAPABILITY_COLLECTING;
    return 1U;
}
/** @brief Checks nonrenewable per-item and total bounds before any state advancement. */
static uint8_t within_budget(MotorCapabilityCollector *collector, uint64_t timestamp_us)
{
    if (!timestamp_us || timestamp_us < collector->started_us ||
        timestamp_us - collector->started_us > MOTOR_CAPABILITY_TOTAL_TIMEOUT_US ||
        (collector->outstanding && (timestamp_us < collector->pending_us ||
         timestamp_us - collector->pending_us > MOTOR_CAPABILITY_ITEM_TIMEOUT_US)))
    { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_TIMEOUT); return 0U; }
    return 1U;
}
/** @brief Produces one four-byte read and never renews an outstanding transaction. */
uint8_t motor_capability_next(MotorCapabilityCollector *collector, uint32_t epoch,
    uint64_t timestamp_us, CanFrame *frame)
{
    if (collector == NULL || frame == NULL || collector->state != MOTOR_CAPABILITY_COLLECTING) { return 0U; }
    if (epoch != collector->epoch)
    { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_BINDING); return 0U; }
    if (!within_budget(collector, timestamp_us)) { return 0U; }
    if (collector->outstanding)
    {
        MotorCapabilityItem *item = &collector->items[collector->count];
        if (collector->emitted || collector->rx_boundary_us <= collector->pending_us ||
            collector->rx_boundary_us > timestamp_us) { return 0U; }
        item->pre_submit_boundary_us = collector->rx_boundary_us;
        item->queue_submitted_us = timestamp_us;
        collector->emitted = 1U; *frame = collector->outstanding_frame;
        return 1U;
    }
    if (s3519_pack_capability_read(7U, collector->count, frame) != S3519_CODEC_STATUS_OK)
    { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_TRANSPORT); return 0U; }
    collector->outstanding_frame = *frame; collector->pending_us = timestamp_us;
    collector->items[collector->count].request_created_us = timestamp_us;
    collector->outstanding = 1U; collector->transmitted = 0U; collector->deferred = 0U;
    collector->emitted = 0U;
    return 0U;
}
/** @brief Remembers the actual empty-FIFO observation boundary, preserving millisecond quantization. */
void motor_capability_observe_rx_boundary(MotorCapabilityCollector *collector, uint64_t boundary_us)
{ if (collector != NULL && boundary_us >= collector->rx_boundary_us) { collector->rx_boundary_us = boundary_us; } }
/** @brief Validates float values by vendor register constraints, never by a guessed model. */
static uint8_t valid_value(uint8_t index, float value)
{
    if (!(value == value && value <= FLT_MAX && value >= -FLT_MAX)) { return 0U; }
    if (index == 0U) { return (uint8_t)(value >= 0.0F); }
    if (index == 1U) { return (uint8_t)(value >= 80.0F && value < 200.0F); }
    if (index == 2U) { return (uint8_t)(value > 0.0F && value < 1.0F); }
    if (index == 5U) { return (uint8_t)(value > 0.0F); }
    return 1U;
}
/** @brief Commits only a complete reply after the exact request has a successful completion receipt. */
static void commit_reply(MotorCapabilityCollector *collector)
{
    const CanFrame *frame = &collector->deferred_reply;
    MotorCapabilityItem *item = &collector->items[collector->count];
    uint32_t raw_bits;
    float value;
    if (!collector->deferred || !collector->transmitted) { return; }
    raw_bits = (uint32_t)frame->data[4] | ((uint32_t)frame->data[5] << 8U) |
        ((uint32_t)frame->data[6] << 16U) | ((uint32_t)frame->data[7] << 24U);
    memcpy(&value, &raw_bits, sizeof(value));
    if (!valid_value(collector->count, value))
    { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_VALUE); return; }
    item->raw_bits = raw_bits; item->value = value; item->valid = 1U;
    item->positive = (uint8_t)(value > 0.0F);
    item->rx_lower_us = collector->deferred_lower_us;
    item->received_observed_us = collector->deferred_observed_us;
    collector->outstanding = 0U; collector->deferred = 0U; ++collector->count;
    if (collector->count == MOTOR_CAPABILITY_COUNT) { collector->state = MOTOR_CAPABILITY_COMPLETE; }
}
/** @brief Matches full request bytes and epoch; records a polling upper bound, never physical TX time. */
uint8_t motor_capability_report_transmit(MotorCapabilityCollector *collector, const CanFrame *frame,
    uint8_t succeeded, uint64_t completed_observed_us, uint32_t epoch)
{
    if (collector == NULL || frame == NULL || collector->state != MOTOR_CAPABILITY_COLLECTING ||
        !collector->outstanding || !collector->emitted || collector->transmitted ||
        completed_observed_us < collector->items[collector->count].queue_submitted_us ||
        frame->length != collector->outstanding_frame.length ||
        frame->identifier != collector->outstanding_frame.identifier ||
        memcmp(frame->data, collector->outstanding_frame.data, frame->length) != 0) { return 0U; }
    if (epoch != collector->epoch)
    { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_BINDING); return 1U; }
    if (!within_budget(collector, completed_observed_us)) { return 1U; }
    if (!succeeded) { motor_capability_abort(collector, MOTOR_CAPABILITY_REASON_TRANSPORT); return 1U; }
    collector->transmitted = 1U;
    collector->items[collector->count].send_completed_observed_us = completed_observed_us;
    commit_reply(collector);
    return 1U;
}
/** @brief Quarantines fixed-RID traffic forever; only matching in-window replies can commit once. */
uint8_t motor_capability_accept(MotorCapabilityCollector *collector, const CanFrame *frame,
    uint64_t rx_lower_us, uint64_t received_observed_us)
{
    uint8_t item_index;
    uint8_t candidate = 0U;
    if (collector == NULL || frame == NULL || frame->length < 4U) { return 0U; }
    for (item_index = 0U; item_index < MOTOR_CAPABILITY_COUNT; ++item_index)
    { if (frame->data[3] == motor_capability_rid(item_index)) { candidate = 1U; } }
    if (!candidate || (frame->data[2] != 0x33U && frame->data[2] != 0x55U &&
        !(frame->identifier == 23U && frame->data[0] == 7U && frame->data[1] == 0U))) { return 0U; }
    if (collector->state != MOTOR_CAPABILITY_COLLECTING || !collector->outstanding ||
        !collector->emitted || collector->deferred || frame->identifier != 23U || frame->length != 8U ||
        frame->data[0] != 7U || frame->data[1] != 0U || frame->data[2] != 0x33U ||
        frame->data[3] != motor_capability_rid(collector->count) ||
        !rx_lower_us || rx_lower_us < collector->items[collector->count].pre_submit_boundary_us ||
        received_observed_us < rx_lower_us || !within_budget(collector, received_observed_us))
    { if (collector->rejected_count != UINT32_MAX) { ++collector->rejected_count; } return 1U; }
    collector->deferred_reply = *frame; collector->deferred_lower_us = rx_lower_us;
    collector->deferred_observed_us = received_observed_us; collector->deferred = 1U;
    commit_reply(collector);
    return 1U;
}

/** @brief Recognizes the narrow capability transport contract without broadening generic codec access. */
uint8_t motor_capability_is_request(const CanFrame *frame)
{
    uint8_t index;
    if (frame == NULL || frame->identifier != 0x7FFU || frame->length != 4U ||
        frame->data[0] != 7U || frame->data[1] != 0U || frame->data[2] != 0x33U) { return 0U; }
    for (index = 0U; index < MOTOR_CAPABILITY_COUNT; ++index)
    { if (frame->data[3] == motor_capability_rid(index)) { return 1U; } }
    return 0U;
}
/** @brief Pins a single hardware FIFO mask after HAL submission and proves a new pending request. */
uint8_t motor_capability_fifo_arm(MotorCapabilityFifoTracker *tracker, const CanFrame *frame,
    uint32_t buffer_mask, uint32_t pending_mask, uint32_t transmitted_mask)
{
    if (tracker == NULL || tracker->active || !motor_capability_is_request(frame) ||
        !buffer_mask || (buffer_mask & (buffer_mask - 1U))) { return 0U; }
    tracker->frame = *frame; tracker->buffer_mask = buffer_mask; tracker->active = 1U;
    tracker->pending_proven = (uint8_t)((pending_mask & buffer_mask) != 0U &&
        (transmitted_mask & buffer_mask) == 0U);
    return 1U;
}
/** @brief Requires pending-to-transmitted transition; old completion bits and cancellations fail closed. */
uint8_t motor_capability_fifo_collect(MotorCapabilityFifoTracker *tracker,
    uint32_t pending_mask, uint32_t transmitted_mask, uint32_t cancelled_mask,
    CanFrame *frame, uint8_t *succeeded)
{
    if (tracker == NULL || frame == NULL || succeeded == NULL || !tracker->active ||
        (pending_mask & tracker->buffer_mask)) { return 0U; }
    *frame = tracker->frame;
    *succeeded = (uint8_t)(tracker->pending_proven && (transmitted_mask & tracker->buffer_mask) &&
        !(cancelled_mask & tracker->buffer_mask));
    tracker->active = 0U;
    return 1U;
}
