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
    MOTOR_RUNTIME_STATUS_STALE_FEEDBACK
} MotorRuntimeStatus;

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

#endif
