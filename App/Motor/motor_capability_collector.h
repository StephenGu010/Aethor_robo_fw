/** @file motor_capability_collector.h
 * @brief Motor-7 fixed diagnostic reads; no write, model or motion qualification interface.
 * @note One collection per initialized boot prevents an arbitrarily late same-RID reply
 *       from being mistaken for a later collection, including across STOP and release.
 *       TX completion and RX timestamps are observation bounds, not physical sample times. */
#ifndef APP_MOTOR_CAPABILITY_COLLECTOR_H
#define APP_MOTOR_CAPABILITY_COLLECTOR_H
#include "s3519_codec.h"
#define MOTOR_CAPABILITY_COUNT (6U)
#define MOTOR_CAPABILITY_ITEM_TIMEOUT_US (20000ULL)
#define MOTOR_CAPABILITY_TOTAL_TIMEOUT_US (144000ULL)
/** @brief Diagnostic lifecycle independent of control qualification. */
typedef enum { MOTOR_CAPABILITY_IDLE = 0, MOTOR_CAPABILITY_COLLECTING,
    MOTOR_CAPABILITY_COMPLETE, MOTOR_CAPABILITY_ABORTED } MotorCapabilityState;
/** @brief Immutable first failure reason; rejected packets are separately counted. */
typedef enum { MOTOR_CAPABILITY_REASON_NONE = 0, MOTOR_CAPABILITY_REASON_TRANSPORT,
    MOTOR_CAPABILITY_REASON_TIMEOUT, MOTOR_CAPABILITY_REASON_VALUE,
    MOTOR_CAPABILITY_REASON_BINDING, MOTOR_CAPABILITY_REASON_STOP } MotorCapabilityReason;
/** @brief Raw register observation with explicit transport provenance and unknown model semantics. */
typedef struct
{
    uint64_t queue_submitted_us;
    uint64_t request_created_us;
    uint64_t pre_submit_boundary_us;
    uint64_t send_completed_observed_us;
    uint64_t rx_lower_us;
    uint64_t received_observed_us;
    uint32_t raw_bits;
    float value;
    uint8_t valid;
    uint8_t positive; /* Diagnostic only for Damp/Inertia; never an axis or b0 claim. */
} MotorCapabilityItem;
/** @brief Single outstanding fixed read plus one deferred reply awaiting matched TX completion. */
typedef struct
{
    MotorCapabilityItem items[MOTOR_CAPABILITY_COUNT];
    CanFrame outstanding_frame;
    CanFrame deferred_reply;
    uint64_t started_us;
    uint64_t pending_us;
    uint64_t deferred_lower_us;
    uint64_t deferred_observed_us;
    uint64_t rx_boundary_us;
    uint32_t request_id;
    uint32_t epoch;
    uint32_t rejected_count;
    MotorCapabilityState state;
    MotorCapabilityReason reason;
    uint8_t count;
    uint8_t outstanding;
    uint8_t transmitted;
    uint8_t deferred;
    uint8_t used;
    uint8_t emitted;
    uint8_t completed_revoked; /* Permanent until init; completed raw observations remain immutable. */
} MotorCapabilityCollector;
/** @brief Clears collector only during application initialization before any bus activity. */
void motor_capability_init(MotorCapabilityCollector *collector);
/** @brief Returns one fixed RID, or zero for an out-of-range index. */
uint8_t motor_capability_rid(uint8_t index);
/** @brief Starts one boot-bound collection; caller must establish owner and fresh disabled health. */
uint8_t motor_capability_start(MotorCapabilityCollector *collector, uint32_t request_id,
    uint32_t epoch, uint64_t timestamp_us);
/** @brief Reserves the sole boot-local collection before finite DISABLE preparation starts. */
uint8_t motor_capability_reserve(MotorCapabilityCollector *collector, uint32_t request_id, uint32_t epoch);
/** @brief Starts reserved read budgets only after trusted fresh disabled preparation succeeds. */
uint8_t motor_capability_start_reserved(MotorCapabilityCollector *collector, uint64_t timestamp_us);
/** @brief Produces at most one fixed read; epoch changes and deadlines abort without retries. */
uint8_t motor_capability_next(MotorCapabilityCollector *collector, uint32_t epoch,
    uint64_t timestamp_us, CanFrame *frame);
/** @brief Supplies only a trusted platform-published empty-FIFO boundary, never a USB timestamp. */
void motor_capability_observe_rx_boundary(MotorCapabilityCollector *collector, uint64_t boundary_us);
/** @brief Matches the exact sent request and completion observation; submission alone is insufficient. */
uint8_t motor_capability_report_transmit(MotorCapabilityCollector *collector, const CanFrame *frame,
    uint8_t succeeded, uint64_t completed_observed_us, uint32_t epoch);
/** @brief Consumes candidate capability replies even after terminal state, protecting MotorBank. */
uint8_t motor_capability_accept(MotorCapabilityCollector *collector, const CanFrame *frame,
    uint64_t rx_lower_us, uint64_t received_observed_us);
/** @brief Aborts active work while retaining used latch, results and permanent late-reply quarantine. */
void motor_capability_abort(MotorCapabilityCollector *collector, MotorCapabilityReason reason);
/** @brief Permanently revokes a completed diagnostic after an abnormal observation, retaining raw results. */
void motor_capability_revoke_completed(MotorCapabilityCollector *collector, MotorCapabilityReason reason);
/** @brief Fixed-read FIFO slot receipt; only pending proof followed by TXBTO grants success. */
typedef struct
{
    CanFrame frame;
    uint32_t buffer_mask;
    uint8_t active;
    uint8_t pending_proven;
} MotorCapabilityFifoTracker;
/** @brief Recognizes only fixed motor-7 four-byte capability reads, never generic writes. */
uint8_t motor_capability_is_request(const CanFrame *frame);
/** @brief Pins one FIFO slot; missing pending proof or old TXBTO bits become failed receipts. */
uint8_t motor_capability_fifo_arm(MotorCapabilityFifoTracker *tracker, const CanFrame *frame,
    uint32_t buffer_mask, uint32_t pending_mask, uint32_t transmitted_mask);
/** @brief Collects exactly one completion after the pinned slot leaves pending; caller prevents slot reuse. */
uint8_t motor_capability_fifo_collect(MotorCapabilityFifoTracker *tracker,
    uint32_t pending_mask, uint32_t transmitted_mask, uint32_t cancelled_mask,
    CanFrame *frame, uint8_t *succeeded);
#endif
