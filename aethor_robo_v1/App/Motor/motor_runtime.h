/**
 * @file motor_runtime.h
 * @brief Defines task-facing seven-motor discovery and feedback routing.
 */

#ifndef APP_MOTOR_MOTOR_RUNTIME_H
#define APP_MOTOR_MOTOR_RUNTIME_H

#include <stdint.h>

#include "motor_bank.h"
#include "motor_discovery.h"

#define MOTOR_RUNTIME_FEEDBACK_STALE_AFTER_US (100000ULL)
#define MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES (14U)

/** @brief Owns the fail-safe disable frames for all seven configured motors. */
typedef struct
{
    CanFrame frames[MOTOR_RUNTIME_EMERGENCY_DISABLE_MAX_FRAMES];
    uint8_t count;
} MotorEmergencyFrameBatch;

/**
 * @brief Stores discovered dynamic limits for S3519 POS_VEL commands.
 */
typedef struct
{
    float position_max_rad;
    float velocity_mapping_max_rad_s;
    float maximum_speed_rad_s;
    float move_speed_limit_rad_s;
} MotorPositionVelocityLimits;

/**
 * @brief Reports deterministic discovery and receive-routing outcomes.
 */
typedef enum
{
    MOTOR_RUNTIME_STATUS_OK = 0,
    MOTOR_RUNTIME_STATUS_FRAME_READY,
    MOTOR_RUNTIME_STATUS_WAITING,
    MOTOR_RUNTIME_STATUS_DISCOVERY_COMPLETE,
    MOTOR_RUNTIME_STATUS_INVALID_ARGUMENT,
    MOTOR_RUNTIME_STATUS_NOT_INITIALIZED,
    MOTOR_RUNTIME_STATUS_DISCOVERY_ERROR,
    MOTOR_RUNTIME_STATUS_CODEC_ERROR,
    MOTOR_RUNTIME_STATUS_RANGE_UNAVAILABLE,
    MOTOR_RUNTIME_STATUS_ID_MISMATCH,
    MOTOR_RUNTIME_STATUS_STALE_FEEDBACK,
    MOTOR_RUNTIME_STATUS_ACTION_COMPLETE,
    MOTOR_RUNTIME_STATUS_ACTION_FAILED,
    MOTOR_RUNTIME_STATUS_POSITION_OUT_OF_RANGE,
    MOTOR_RUNTIME_STATUS_SPEED_OUT_OF_RANGE,
    MOTOR_RUNTIME_STATUS_FAULT_PRESENT
} MotorRuntimeStatus;

/** @brief Describes the bounded seven-motor control-mode write/readback cycle. */
typedef enum
{
    MOTOR_MODE_SWITCH_IDLE = 0,
    MOTOR_MODE_SWITCH_WRITING,
    MOTOR_MODE_SWITCH_READ_READY,
    MOTOR_MODE_SWITCH_READ_WAITING,
    MOTOR_MODE_SWITCH_COMPLETE,
    MOTOR_MODE_SWITCH_FAILED
} MotorModeSwitchState;

/**
 * @brief Owns all static receive-side state for the first seven-axis arm.
 */
typedef struct
{
    const ArmConfig *configuration;
    MotorBank bank;
    MotorDiscovery discovery;
    uint32_t accepted_parameter_response_count;
    uint32_t rejected_parameter_response_count;
    uint32_t accepted_feedback_count;
    uint32_t rejected_feedback_count;
    MotorModeSwitchState mode_switch_state;
    uint64_t mode_request_sent_at_us;
    S3519ControlMode requested_control_mode;
    uint8_t mode_switch_joint_mask;
    uint8_t mode_switch_joint_index;
    uint8_t mode_switch_attempt_count;
    uint8_t discovery_active;
    uint8_t initialized;
} MotorRuntime;

/**
 * @brief Initializes seven motor identities and read-only discovery state.
 * @param runtime Destination runtime.
 * @param configuration Frozen seven-axis mapping.
 * @return OK or a validated initialization error.
 */
MotorRuntimeStatus motor_runtime_init(MotorRuntime *runtime,
                                      const ArmConfig *configuration);

/**
 * @brief Starts a fresh discovery pass for an explicit bench motor subset.
 * @param runtime Initialized motor runtime.
 * @param target_joint_mask Nonzero J1-J7 bit mask.
 * @return OK or an argument/discovery error.
 */
MotorRuntimeStatus motor_runtime_begin_discovery(MotorRuntime *runtime,
                                                 uint8_t target_joint_mask);

/**
 * @brief Produces at most one bounded read-only discovery frame.
 * @param runtime Initialized runtime.
 * @param timestamp_us Current monotonic timestamp.
 * @param frame Destination request frame.
 * @return FRAME_READY, WAITING, DISCOVERY_COMPLETE, or an error.
 */
MotorRuntimeStatus motor_runtime_next_discovery_frame(MotorRuntime *runtime,
                                                      uint64_t timestamp_us,
                                                      CanFrame *frame);

/**
 * @brief Aborts only active discovery and mode-switch parameter sequences.
 * @param runtime Initialized runtime whose feedback and completed discovery
 *        data remain intact; selected mode fields invalidated by an active
 *        switch stay unverified.
 * @return OK, INVALID_ARGUMENT, or NOT_INITIALIZED; repeated calls are idempotent.
 */
MotorRuntimeStatus motor_runtime_abort_active_parameter_sequences(
    MotorRuntime *runtime);

/**
 * @brief Routes one validated Classic CAN frame to discovery or feedback decode.
 * @param runtime Initialized runtime.
 * @param frame Frame copied from the bounded RX inbox.
 * @param timestamp_us Receive timestamp assigned to control feedback.
 * @return Detailed decode, identity, or freshness status.
 */
MotorRuntimeStatus motor_runtime_accept_frame(MotorRuntime *runtime,
                                              const CanFrame *frame,
                                              uint64_t timestamp_us);

/**
 * @brief Copies one coherent seven-axis feedback snapshot.
 * @param runtime Initialized runtime.
 * @param timestamp_us Snapshot publication timestamp.
 * @param stale_after_us Maximum accepted feedback age.
 * @param snapshot Destination snapshot.
 * @return OK or an argument/initialization error.
 */
MotorRuntimeStatus motor_runtime_get_snapshot(const MotorRuntime *runtime,
                                              uint64_t timestamp_us,
                                              uint64_t stale_after_us,
                                              MotorFeedbackSnapshot *snapshot);

/**
 * @brief Builds fail-safe disable frames, covering both identifiers if mode is unknown.
 * @param runtime Initialized seven-motor runtime.
 * @param batch Destination bounded emergency batch.
 * @return OK or an argument/codec error.
 */
MotorRuntimeStatus motor_runtime_build_emergency_disable(
    const MotorRuntime *runtime,
    MotorEmergencyFrameBatch *batch);

/**
 * @brief Builds fail-safe disable frames for only the selected motors.
 * @param runtime Initialized seven-motor runtime owning discovered modes.
 * @param motor_mask Nonzero J1-J7 selection mask.
 * @param batch Destination bounded emergency batch, cleared on failure.
 * @return OK or an argument, initialization, or codec error.
 */
MotorRuntimeStatus motor_runtime_build_emergency_disable_subset(
    const MotorRuntime *runtime,
    uint8_t motor_mask,
    MotorEmergencyFrameBatch *batch);

/**
 * @brief Encodes one ordered all-or-nothing J1-J7 motor control group.
 * @param runtime Initialized runtime owning identities, gains, and ranges.
 * @param control_mode Confirmed S3519 POS_VEL or MIT mode.
 * @param motor_position_rad Seven already-mapped motor positions.
 * @param motor_velocity_rad_s Seven already-mapped signed motor velocities.
 * @param frames Destination ordered group, unchanged on failure.
 * @return OK or an argument, range, or codec error.
 */
MotorRuntimeStatus motor_runtime_build_control_group(
    const MotorRuntime *runtime,
    S3519ControlMode control_mode,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_velocity_rad_s[ARM_JOINT_COUNT],
    CanFrame frames[ARM_JOINT_COUNT]);

/**
 * @brief Gets complete discovered POS_VEL limits for one motor.
 * @param runtime Initialized runtime owning the current discovery results.
 * @param joint_index Zero-based target joint index.
 * @param limits Destination dynamic limit contract.
 * @return OK or an argument, initialization, or unavailable-range error.
 */
MotorRuntimeStatus motor_runtime_get_position_velocity_limits(
    const MotorRuntime *runtime,
    uint8_t joint_index,
    MotorPositionVelocityLimits *limits);

/**
 * @brief Validates a selected fault-free positive-speed POS_VEL move.
 * @param runtime Initialized runtime owning current discovered limits.
 * @param feedback_snapshot Freshness-filtered feedback snapshot.
 * @param motor_mask Nonzero J1-J7 selection mask.
 * @param motor_position_rad Joint-indexed motor targets.
 * @param motor_speed_rad_s Joint-indexed positive motor speed limits.
 * @param failed_joint_index Destination first failing zero-based joint index.
 * @return OK or a stable argument, discovery, feedback, fault, or range error.
 */
MotorRuntimeStatus motor_runtime_validate_position_velocity_move_subset(
    const MotorRuntime *runtime,
    const MotorFeedbackSnapshot *feedback_snapshot,
    uint8_t motor_mask,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_speed_rad_s[ARM_JOINT_COUNT],
    uint8_t *failed_joint_index);

/**
 * @brief Encodes selected POS_VEL targets without altering unselected motors.
 * @param runtime Initialized runtime owning identities.
 * @param motor_mask Nonzero J1-J7 selection mask.
 * @param motor_position_rad Joint-indexed motor targets.
 * @param motor_velocity_rad_s Joint-indexed nonnegative speed limits.
 * @param batch Destination bounded selected-motor frame batch.
 * @return OK or an argument/range/codec error.
 */
MotorRuntimeStatus motor_runtime_build_position_velocity_subset(
    const MotorRuntime *runtime,
    uint8_t motor_mask,
    const float motor_position_rad[ARM_JOINT_COUNT],
    const float motor_velocity_rad_s[ARM_JOINT_COUNT],
    MotorEmergencyFrameBatch *batch);

/**
 * @brief Starts a seven-motor volatile control-mode write/readback operation.
 * @param runtime Initialized runtime with completed discovery.
 * @param control_mode Requested MIT or POS_VEL mode.
 * @return OK or an initialization/state error.
 */
MotorRuntimeStatus motor_runtime_begin_control_mode_switch(
    MotorRuntime *runtime,
    S3519ControlMode control_mode);

/**
 * @brief Starts a volatile control-mode write/readback for a selected subset.
 * @param runtime Initialized runtime with selected discovery complete.
 * @param control_mode Requested MIT or POS_VEL mode.
 * @param motor_mask Nonzero J1-J7 mask.
 * @return OK or an argument/discovery/state error.
 */
MotorRuntimeStatus motor_runtime_begin_control_mode_switch_mask(
    MotorRuntime *runtime,
    S3519ControlMode control_mode,
    uint8_t motor_mask);

/**
 * @brief Produces the next mode write or readback request frame.
 * @param runtime Runtime owning the transition.
 * @param timestamp_us Current monotonic timestamp.
 * @param frame Destination frame.
 * @return FRAME_READY, WAITING, ACTION_COMPLETE, or an error.
 */
MotorRuntimeStatus motor_runtime_next_control_mode_frame(
    MotorRuntime *runtime,
    uint64_t timestamp_us,
    CanFrame *frame);

/**
 * @brief Builds one enable/disable/clear command for each selected motor.
 * @param runtime Initialized runtime.
 * @param control_mode Confirmed motor control mode.
 * @param command Vendor special command.
 * @param motor_mask Selected zero-based joint mask.
 * @param batch Destination bounded batch.
 * @return OK or an argument/codec error.
 */
MotorRuntimeStatus motor_runtime_build_mode_command_batch(
    const MotorRuntime *runtime,
    S3519ControlMode control_mode,
    S3519ModeCommand command,
    uint8_t motor_mask,
    MotorEmergencyFrameBatch *batch);

#endif
