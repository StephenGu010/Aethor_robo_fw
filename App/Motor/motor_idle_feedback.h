/** @file motor_idle_feedback.h
 * @brief Bounded read-only idle feedback acquisition; never changes a motor state or cache timestamp.
 * Header-local implementation keeps existing build targets independent of a new translation unit.
 * Requests are scheduling attempts, not physical TX receipts or proof of query compatibility.
 */
#ifndef APP_MOTOR_MOTOR_IDLE_FEEDBACK_H
#define APP_MOTOR_MOTOR_IDLE_FEEDBACK_H

#include <string.h>
#include "motor_runtime.h"

#ifndef AETHOR_IDLE_FEEDBACK_ENABLE
#define AETHOR_IDLE_FEEDBACK_ENABLE (0U)
#endif
#if AETHOR_IDLE_FEEDBACK_ENABLE && (!AETHOR_DEBUG_UI_ENABLE || !AETHOR_DEBUG_UI_ALLOW_MOTION || AETHOR_ADRC_BENCH)
#error "Idle feedback requires the motion-capable LCD facade, never the dedicated ADRC bench."
#endif

/* Two actual 4 ms control-task ticks; seven axes take 56 ms before reply latency. */
#define MOTOR_IDLE_FEEDBACK_SLOT_US (8000ULL)
/* Leave ProtocolTask a visible quiet window after each round, rather than only
 * a transient true value inside the higher-priority ControlTask. Seven axes: 64 ms. */
#define MOTOR_IDLE_FEEDBACK_ROUND_GAP_US (16000ULL)
#define MOTOR_IDLE_FEEDBACK_PERIOD_US (50000ULL)
#define MOTOR_IDLE_FEEDBACK_REPLY_US (20000ULL)
#define MOTOR_IDLE_FEEDBACK_BACKOFF_US (1000000ULL)
#define MOTOR_IDLE_FEEDBACK_MISS_LIMIT (3U)

/** @brief Per-axis evidence and deadlines; unrelated axes cannot acknowledge a missing reply. */
typedef struct
{
    uint64_t requested_us;
    uint32_t generation;
    uint32_t requests_created;
    uint32_t new_feedback_observed;
    uint32_t timeouts;
    uint8_t issued;
    uint8_t pending;
    uint8_t misses;
} MotorIdleFeedbackAxis;

/** @brief One fair seven-axis scheduler with a global traffic ceiling of 125 requests/second. */
typedef struct
{
    MotorIdleFeedbackAxis axes[ARM_JOINT_COUNT];
    uint64_t last_slot_us;
    uint8_t slot_seen;
    uint8_t next_joint;
} MotorIdleFeedback;

/** @brief Cancel only monitor expectations on takeover; retain failure backoff and every cached sample. */
static inline void motor_idle_feedback_pause(MotorIdleFeedback *monitor, uint64_t now_us)
{
    uint8_t joint_index;
    for (joint_index = 0U; joint_index < ARM_JOINT_COUNT; ++joint_index)
    { monitor->axes[joint_index].pending = 0U; }
    monitor->last_slot_us = now_us;
    monitor->slot_seen = 1U;
}

/** @brief Create at most one vendor 8-byte 0xCC read; only production RX can make feedback fresh.
 * Three unanswered requests reduce that axis to one request/second. A new accepted sample
 * restores the normal cadence but does not prove it was caused by this query.
 */
static inline MotorRuntimeStatus motor_idle_feedback_next(MotorIdleFeedback *monitor,
    const MotorRuntime *runtime, uint64_t now_us, CanFrame *frame)
{
    uint8_t offset;
    if (!monitor || !runtime || !runtime->initialized || !runtime->configuration || !frame)
    { return MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT; }
    if (monitor->slot_seen && now_us < monitor->last_slot_us)
    { motor_idle_feedback_pause(monitor, now_us); return MOTOR_RUNTIME_STATUS_WAITING; }
    for (offset = 0U; offset < ARM_JOINT_COUNT; ++offset)
    {
        MotorIdleFeedbackAxis *axis = &monitor->axes[offset];
        const MotorObject *motor = &runtime->bank.motors[offset];
        if (motor->feedback_valid && motor->feedback_generation != axis->generation &&
            motor->feedback.timestamp_us <= now_us &&
            now_us - motor->feedback.timestamp_us <= MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US)
        {
            axis->generation = motor->feedback_generation;
            axis->pending = axis->misses = 0U;
            ++axis->new_feedback_observed;
        }
        if (axis->pending && now_us >= axis->requested_us &&
            now_us - axis->requested_us >= MOTOR_IDLE_FEEDBACK_REPLY_US)
        {
            axis->pending = 0U;
            if (axis->misses < MOTOR_IDLE_FEEDBACK_MISS_LIMIT) { ++axis->misses; }
            ++axis->timeouts;
        }
    }
    if (monitor->slot_seen && now_us - monitor->last_slot_us <
        (monitor->next_joint == 0U ? MOTOR_IDLE_FEEDBACK_ROUND_GAP_US : MOTOR_IDLE_FEEDBACK_SLOT_US))
    { return MOTOR_RUNTIME_STATUS_WAITING; }
    for (offset = 0U; offset < ARM_JOINT_COUNT; ++offset)
    {
        uint8_t joint_index = (uint8_t)((monitor->next_joint + offset) % ARM_JOINT_COUNT);
        MotorIdleFeedbackAxis *axis = &monitor->axes[joint_index];
        const MotorDiscoveryResult *discovery = &runtime->discovery.results[joint_index];
        const JointConfig *joint = &runtime->configuration->joints[joint_index];
        uint64_t period_us = axis->misses >= MOTOR_IDLE_FEEDBACK_MISS_LIMIT ?
            MOTOR_IDLE_FEEDBACK_BACKOFF_US : MOTOR_IDLE_FEEDBACK_PERIOD_US;
        /* This board's ID7 mode-2 driver has no CC reply; its independent raw
         * position reader handles display without fabricating full feedback. */
        if ((joint_index == 6U && discovery->observed_control_mode == 2U) ||
            !(runtime->discovery.verified_joint_mask & (1U << joint_index)) ||
            (discovery->verified_fields_mask & MOTOR_DISCOVERY_ALL_FIELDS_MASK) != MOTOR_DISCOVERY_ALL_FIELDS_MASK ||
            discovery->observed_esc_id != joint->esc_id || discovery->observed_master_id != joint->master_id ||
            axis->pending || (axis->issued && (now_us < axis->requested_us || now_us - axis->requested_us < period_us)))
        { continue; }
        if (s3519_pack_feedback_query_with_length((uint8_t)joint->esc_id, 8U, frame) != S3519_CODEC_STATUS_OK)
        { return MOTOR_RUNTIME_STATUS_CODEC_ERROR; }
        axis->generation = runtime->bank.motors[joint_index].feedback_generation;
        axis->requested_us = now_us;
        axis->issued = axis->pending = 1U;
        ++axis->requests_created;
        monitor->last_slot_us = now_us;
        monitor->slot_seen = 1U;
        monitor->next_joint = (uint8_t)((joint_index + 1U) % ARM_JOINT_COUNT);
        return MOTOR_RUNTIME_STATUS_FRAME_READY;
    }
    return MOTOR_RUNTIME_STATUS_WAITING;
}

#endif
