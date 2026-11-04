/** @file motor_parameter_persistence.h
 * @brief Fixed ID7 offline-testable persistence transaction; caller owns bus and disabled health. */
#ifndef APP_MOTOR_PARAMETER_PERSISTENCE_H
#define APP_MOTOR_PARAMETER_PERSISTENCE_H
#include <stdint.h>
#include "can_frame.h"
#define MOTOR_PERSISTENCE_TRANSACTION_VERSION (1U)
#define MOTOR_PERSISTENCE_FRAME_BUDGET_US (100000ULL)
#define MOTOR_PERSISTENCE_READ_BUDGET_US (5000000ULL)
#define MOTOR_PERSISTENCE_PREPARE_BUDGET_US (6000000ULL)
#define MOTOR_PERSISTENCE_READY_BUDGET_US (10000000ULL)
#define MOTOR_PERSISTENCE_SAVE_BUDGET_US (12000000ULL)
#define MOTOR_PERSISTENCE_TOTAL_BUDGET_US (45000000ULL)
#define MOTOR_PERSISTENCE_REGISTER_COUNT (45U)
#define MOTOR_PERSISTENCE_CONFIGURATION_COUNT (26U)
/** @brief Immutable transaction binding carried independently from CAN bytes. */
typedef struct {
    uint32_t boot_epoch, request_id, owner_generation, sequence;
    uint8_t phase;
} MotorPersistenceIdentity;
/** @brief Raw observation with host-visible TX and RX observation bounds.
 * send_completed_observed_us is TX-completion observation time, never a physical SOF timestamp. */
typedef struct {
    MotorPersistenceIdentity identity;
    uint64_t created_us, submitted_us, boundary_us, send_completed_observed_us, rx_lower_us, received_us;
    uint32_t raw;
    uint8_t valid;
} MotorPersistenceItem;
/** @brief Dense fixed-whitelist observation set; no writable configuration alias. */
typedef struct { MotorPersistenceItem items[MOTOR_PERSISTENCE_REGISTER_COUNT]; } MotorPersistenceSnapshot;
/** @brief Pins one hardware slot and its immutable owner until pending clears. */
typedef struct {
    MotorPersistenceIdentity identity;
    uint32_t buffer_mask;
    uint8_t active, pending_proven, cancelled;
    uint8_t pending_seen; /**< Absence of TXBRP before the first Tx scan cannot release an accepted request. */
} MotorPersistenceTracker;
/** @brief Compares all transaction identity fields without padding dependence. */
uint8_t motor_persistence_identity_equal(const MotorPersistenceIdentity *left, const MotorPersistenceIdentity *right);
/** @brief Pins a single submitted slot; queue or HAL acceptance never grants success. */
uint8_t motor_persistence_tracker_arm(MotorPersistenceTracker *tracker, const MotorPersistenceIdentity *identity,
    uint32_t buffer_mask, uint32_t pending_mask, uint32_t transmitted_mask);
/** @brief Irreversibly marks cancellation until the pending slot is collected. */
void motor_persistence_tracker_cancel(MotorPersistenceTracker *tracker);
/** @brief Collects only after an observed pending request clears, rejecting cancellation and stale TXBTO.
 * A caller that has accepted a hardware request but never observed pending must keep ownership;
 * a zero-status poll is not evidence that a delayed Tx scan cannot still publish the request. */
uint8_t motor_persistence_tracker_collect(MotorPersistenceTracker *tracker,
    uint32_t pending_mask, uint32_t transmitted_mask, uint32_t cancelled_mask,
    MotorPersistenceIdentity *identity, uint8_t *succeeded);
/** @brief Validates completeness, native finite values and fixed target identity. */
uint8_t motor_persistence_validate(const MotorPersistenceSnapshot *snapshot, uint8_t require_target);
/** @brief Compares all26 RW raw values, optionally allowing only the two fixed target values. */
uint8_t motor_persistence_configuration_matches(const MotorPersistenceSnapshot *baseline,
    const MotorPersistenceSnapshot *current, uint8_t allow_targets);
/** @brief Returns a published register address, or 255 for an invalid index. */
uint8_t motor_persistence_rid(unsigned index);
/** @brief Identifies registers whose four raw bytes represent uint32, not float. */
uint8_t motor_persistence_is_unsigned(unsigned index);
/** @brief Identifies baseline configuration membership; does not authorize writing. */
uint8_t motor_persistence_is_configuration(unsigned index);
/** @brief Explicit bounded phases; terminal states cannot restart without initialization. */
typedef enum {
    MOTOR_PERSISTENCE_IDLE = 0, MOTOR_PERSISTENCE_BASELINE,
    MOTOR_PERSISTENCE_BASELINE_READY, MOTOR_PERSISTENCE_PREPARE_WRITE,
    MOTOR_PERSISTENCE_PREPARE_VERIFY, MOTOR_PERSISTENCE_PREPARED,
    MOTOR_PERSISTENCE_SAVE_VERIFY, MOTOR_PERSISTENCE_SAVE_HEALTH,
    MOTOR_PERSISTENCE_SAVE_WAIT, MOTOR_PERSISTENCE_SETTLE,
    MOTOR_PERSISTENCE_READBACK, MOTOR_PERSISTENCE_COMPLETE, MOTOR_PERSISTENCE_ABORTED
} MotorPersistenceState;
/** @brief Latched failure category; abort never restores values or issues enable/disable. */
typedef enum {
    MOTOR_PERSISTENCE_ERROR_NONE = 0, MOTOR_PERSISTENCE_ERROR_TIMEOUT,
    MOTOR_PERSISTENCE_ERROR_CONTEXT, MOTOR_PERSISTENCE_ERROR_STOP,
    MOTOR_PERSISTENCE_ERROR_SNAPSHOT, MOTOR_PERSISTENCE_ERROR_TX,
    MOTOR_PERSISTENCE_ERROR_HEALTH, MOTOR_PERSISTENCE_ERROR_CLOCK
} MotorPersistenceError;
/** @brief Caller-supplied actual disabled feedback and idle admission evidence, bound to boot epoch.
 * observed_us is the trusted feedback lower time bound, not the time this struct is filled.
 * The App must refresh FD using the confirmed current mode and must never fabricate this data. */
typedef struct {
    uint32_t boot_epoch;
    uint64_t observed_us;
    float velocity, torque, motor_temperature, driver_temperature;
    uint8_t valid, disabled, idle, fault;
} MotorPersistenceHealth;
/** @brief Single-owner transaction storage; allocate persistently, never as repeated stack snapshots.
 * baseline_raw is compact; current alone contains full per-register observation evidence.
 * Public fields are read-only to callers after init. save_used survives every abort.
 * CanFrame has no IDE/RTR fields: accept requires the adapter's standard-data validation.
 * Vendor replies contain no nonce: an identical stale reply after a new TX is indistinguishable. */
typedef struct {
    MotorPersistenceSnapshot current;
    uint32_t baseline_raw[MOTOR_PERSISTENCE_REGISTER_COUNT];
    MotorPersistenceIdentity identity;
    MotorPersistenceHealth health;
    CanFrame frame;
    uint64_t last_us, total_deadline_us, phase_deadline_us, frame_deadline_us;
    uint64_t created_us, submitted_us, boundary_us, send_completed_observed_us;
    uint64_t deferred_lower_us, deferred_received_us, settle_until_us, verified_us;
    uint32_t deferred_raw, confirmed_pmax_raw, confirmed_mode;
    MotorPersistenceState state;
    MotorPersistenceError error;
    uint8_t read_index, write_step, pending, submitted, actual_tx, deferred;
    uint8_t save_used, save_uncertain, skip_save, external_disable_required;
    uint8_t pmax_known, mode_known;
} MotorPersistenceTransaction;
/** @brief Initializes once per genuine firmware boot; re-init is never a retry or power-cycle proof. */
void motor_persistence_init(MotorPersistenceTransaction *transaction);
/** @brief Starts the one permitted transaction using nonzero immutable ownership identity. */
uint8_t motor_persistence_begin(MotorPersistenceTransaction *transaction, uint32_t boot_epoch,
    uint32_t request_id, uint32_t owner_generation, uint64_t now_us);
/** @brief Must be called on every owner/epoch/STOP event and periodically, even while awaiting input. */
void motor_persistence_tick(MotorPersistenceTransaction *transaction, uint64_t now_us,
    uint32_t boot_epoch, uint32_t owner_generation, uint8_t stop);
/** @brief Explicitly permits only changed PMAX512/mode2 volatile writes after valid baseline. */
uint8_t motor_persistence_prepare(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us);
/** @brief Refreshes actual disabled/idle health between writes without extending the budget.
 * Invalid health in PREPARE_WRITE irreversibly aborts, revoking even an already prepared frame. */
uint8_t motor_persistence_refresh_health(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us);
/** @brief Requires fresh healthy disabled idle evidence, then rereads all45 before any Save frame. */
uint8_t motor_persistence_request_save(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us);
/** @brief Checks bounded finite actual feedback; optional lower bound requires a post-verify refresh. */
uint8_t motor_persistence_health_valid(const MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us, uint64_t earliest_us);
/** @brief Final explicit Save gate with feedback strictly newer than completed Save verification. */
uint8_t motor_persistence_authorize_save(MotorPersistenceTransaction *transaction,
    const MotorPersistenceHealth *health, uint64_t now_us);
/** @brief Emits one immutable frame/identity exactly once; subsequent calls await completion.
 * Caller retains the returned pair until final submit; queue rejection is not permission to retry. */
uint8_t motor_persistence_next(MotorPersistenceTransaction *transaction, uint64_t now_us,
    CanFrame *frame, MotorPersistenceIdentity *identity);
/** @brief Commits one admission after empty RX boundary; AA latches used/uncertain before enqueue. */
uint8_t motor_persistence_submitted(MotorPersistenceTransaction *transaction,
    const MotorPersistenceIdentity *identity, uint64_t now_us, uint64_t empty_boundary_us,
    uint8_t rx_empty);
/** @brief Supplies exact receipt and hardware completion observation time; failed TX aborts.
 * TXBTO is actual-transmission evidence, but its polling time is NOT physical transmit time. */
uint8_t motor_persistence_report_tx(MotorPersistenceTransaction *transaction,
    const MotorPersistenceIdentity *identity, uint64_t completed_observed_us, uint64_t now_us,
    uint8_t succeeded);
/** @brief Accepts exact STD23 DATA reply only; RX lower bound must follow empty boundary/submission.
 * One early-observed reply may defer until TX evidence arrives, never extending any deadline. */
uint8_t motor_persistence_accept(MotorPersistenceTransaction *transaction, const CanFrame *frame,
    uint8_t is_standard_data, uint64_t rx_lower_us, uint64_t received_us);
#endif
