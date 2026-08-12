/**
 * @file protocol_engine.h
 * @brief Defines bounded sessions and replay-safe request lifecycle handling.
 */

#ifndef APP_PROTOCOL_PROTOCOL_ENGINE_H
#define APP_PROTOCOL_PROTOCOL_ENGINE_H

#include <stddef.h>
#include <stdint.h>

#include "ascii_protocol.h"
#include "arm_controller.h"
#include "diagnostics.h"
#include "joint_reference.h"
#include "motor_types.h"

#define PROTOCOL_ENGINE_MESSAGE_CAPACITY (520U)
#define PROTOCOL_ENGINE_MAX_OUTPUT_COUNT (2U)
#define PROTOCOL_ENGINE_RECENT_RESULT_CAPACITY (32U)
#define PROTOCOL_ENGINE_RECENT_RESULT_RETENTION_US (60000000ULL)
#define PROTOCOL_ENGINE_COMMAND_CAPACITY (8U)
#define PROTOCOL_ENGINE_RESULT_CAPACITY (8U)
#define PROTOCOL_ENGINE_HEARTBEAT_PERIOD_US (250000ULL)
#define PROTOCOL_ENGINE_WATCHDOG_TIMEOUT_US (1000000ULL)

/** @brief Selects the physical USB transmit queue for an encoded output. */
typedef enum
{
    PROTOCOL_OUTPUT_HIGH_PRIORITY = 0,
    PROTOCOL_OUTPUT_QUERY,
    PROTOCOL_OUTPUT_TELEMETRY
} ProtocolOutputPriority;

/** @brief Reports request parsing, replay, session, and dispatch outcomes. */
typedef enum
{
    PROTOCOL_ENGINE_STATUS_OK = 0,
    PROTOCOL_ENGINE_STATUS_REPLAYED,
    PROTOCOL_ENGINE_STATUS_REQUEST_ID_CONFLICT,
    PROTOCOL_ENGINE_STATUS_SESSION_MISMATCH,
    PROTOCOL_ENGINE_STATUS_BAD_REQUEST,
    PROTOCOL_ENGINE_STATUS_OUTPUT_TOO_SMALL,
    PROTOCOL_ENGINE_STATUS_INVALID_ARGUMENT
} ProtocolEngineStatus;

/** @brief Owns one immutable encoded response selected for USB transmission. */
typedef struct
{
    char data[PROTOCOL_ENGINE_MESSAGE_CAPACITY];
    uint16_t length;
    ProtocolOutputPriority priority;
} ProtocolOutputMessage;

/** @brief Owns all immediate outputs produced by one request. */
typedef struct
{
    ProtocolOutputMessage messages[PROTOCOL_ENGINE_MAX_OUTPUT_COUNT];
    uint8_t count;
} ProtocolOutputBatch;

/** @brief Stores one replay-safe result for up to 60 seconds. */
typedef struct
{
    char response[PROTOCOL_ENGINE_MESSAGE_CAPACITY];
    uint64_t completed_at_us;
    uint32_t request_id;
    uint32_t body_hash;
    uint16_t response_length;
    ProtocolOutputPriority priority;
    uint8_t valid;
} ProtocolRecentResult;

/** @brief Supplies one coherent application snapshot for synchronous queries. */
typedef struct
{
    ArmSnapshot arm;
    JointStateSnapshot joints;
    MotorFeedbackSnapshot motors;
    DiagnosticCounters diagnostics;
    uint64_t timestamp_us;
} ProtocolQueryContext;

/** @brief Identifies one state-changing command accepted by the protocol task. */
typedef enum
{
    PROTOCOL_COMMAND_ALIGN_REFERENCE = 0,
    PROTOCOL_COMMAND_SET_MODE,
    PROTOCOL_COMMAND_ENABLE,
    PROTOCOL_COMMAND_STOP,
    PROTOCOL_COMMAND_DISABLE,
    PROTOCOL_COMMAND_CLEAR_FAULT,
    PROTOCOL_COMMAND_MOVE_JOINTS,
    PROTOCOL_COMMAND_INIT_MOTORS,
    PROTOCOL_COMMAND_MOVE_RELATIVE,
    PROTOCOL_COMMAND_LINK_TIMEOUT
} ProtocolCommandType;

/** @brief Owns one validated fixed-size command transferred to ArmControlTask. */
typedef struct
{
    float values[ARM_JOINT_COUNT];
    float speeds[ARM_JOINT_COUNT];
    uint64_t accepted_at_us;
    uint32_t request_id;
    uint32_t session_id;
    ProtocolCommandType type;
    uint8_t motor_mask;
    ArmControlMode control_mode;
    uint8_t scope_joint;
} ProtocolCommand;

/** @brief Identifies one terminal action result produced by ArmControlTask. */
typedef enum
{
    PROTOCOL_COMMAND_RESULT_COMPLETED = 0,
    PROTOCOL_COMMAND_RESULT_STOPPED,
    PROTOCOL_COMMAND_RESULT_FAILED,
    PROTOCOL_COMMAND_RESULT_CANCELLED
} ProtocolCommandResultCode;

/** @brief Owns one bounded terminal command result transferred to ProtocolTask. */
typedef struct
{
    float values[ARM_JOINT_COUNT];
    float auxiliary_values[ARM_JOINT_COUNT];
    uint64_t completed_at_us;
    uint32_t request_id;
    uint32_t session_id;
    ProtocolCommandType type;
    ProtocolCommandResultCode code;
    uint16_t detail;
} ProtocolCommandResult;

/** @brief Owns the fixed current session and bounded recent-result cache. */
typedef struct
{
    ProtocolRecentResult recent_results[PROTOCOL_ENGINE_RECENT_RESULT_CAPACITY];
    ProtocolCommand commands[PROTOCOL_ENGINE_COMMAND_CAPACITY];
    ProtocolCommand stop_command;
    ProtocolCommandResult results[PROTOCOL_ENGINE_RESULT_CAPACITY];
    ProtocolQueryContext query_context;
    char stream_fields[64];
    uint64_t last_valid_request_at_us;
    uint64_t next_telemetry_due_us;
    uint32_t boot_id;
    uint32_t session_id;
    uint32_t next_session_nonce;
    uint32_t telemetry_sequence;
    uint32_t event_sequence;
    uint8_t recent_write_index;
    volatile uint8_t command_write_sequence;
    volatile uint8_t command_read_sequence;
    volatile uint8_t stop_write_sequence;
    volatile uint8_t stop_read_sequence;
    volatile uint8_t result_write_sequence;
    volatile uint8_t result_read_sequence;
    uint8_t stream_rate_hz;
    uint8_t query_context_valid;
    uint8_t session_active;
    uint8_t watchdog_timeout_reported;
    ArmState last_published_state;
    uint8_t last_published_state_valid;
} ProtocolEngine;

/**
 * @brief Initializes one protocol engine for the current firmware boot.
 * @param engine Destination engine.
 * @param boot_id Nonzero boot identity returned to the host.
 */
void protocol_engine_init(ProtocolEngine *engine, uint32_t boot_id);

/**
 * @brief Copies the latest coherent application values used by query commands.
 * @param engine Initialized engine.
 * @param query_context Snapshot assembled by the application facade.
 */
void protocol_engine_update_query_context(
    ProtocolEngine *engine,
    const ProtocolQueryContext *query_context);

/**
 * @brief Pops the oldest accepted business command for ArmControlTask.
 * @param engine Initialized engine.
 * @param command Destination command.
 * @return One when a command was copied, otherwise zero.
 */
uint8_t protocol_engine_pop_command(ProtocolEngine *engine,
                                    ProtocolCommand *command);

/**
 * @brief Pops only the independent highest-priority STOP slot.
 * @param engine Initialized engine.
 * @param command Destination STOP command.
 * @return One when copied, otherwise zero.
 */
uint8_t protocol_engine_pop_stop_command(ProtocolEngine *engine,
                                         ProtocolCommand *command);

/**
 * @brief Cancels all accepted commands not yet taken by ArmControlTask.
 * @param engine Initialized engine.
 */
void protocol_engine_cancel_pending_commands(ProtocolEngine *engine);

/**
 * @brief Submits one terminal result from ArmControlTask without formatting.
 * @param engine Initialized engine.
 * @param result Immutable terminal result.
 * @return One when queued, otherwise zero.
 */
uint8_t protocol_engine_submit_command_result(
    ProtocolEngine *engine,
    const ProtocolCommandResult *result);

/**
 * @brief Formats the oldest terminal result and replaces its replay cache entry.
 * @param engine Initialized engine.
 * @param output_batch Destination DONE output.
 * @return One when a result was formatted, otherwise zero.
 */
uint8_t protocol_engine_pop_result_output(
    ProtocolEngine *engine,
    ProtocolOutputBatch *output_batch);

/**
 * @brief Generates due telemetry and immediate state-change events.
 * @param engine Initialized engine with a current query context.
 * @param timestamp_us Current monotonic timestamp.
 * @param output_batch Destination containing zero to two outputs.
 * @return Number of generated outputs.
 */
uint8_t protocol_engine_generate_stream_output(
    ProtocolEngine *engine,
    uint64_t timestamp_us,
    ProtocolOutputBatch *output_batch);

/**
 * @brief Formats one transport-layer line overflow error without parsing.
 * @param output_batch Destination high-priority error output.
 * @return OK or INVALID_ARGUMENT/OUTPUT_TOO_SMALL.
 */
ProtocolEngineStatus protocol_engine_format_line_too_long(
    ProtocolOutputBatch *output_batch);

/**
 * @brief Processes one complete CRC-protected request line.
 * @param engine Initialized engine.
 * @param line Complete request line.
 * @param length Exact line length.
 * @param timestamp_us Current monotonic timestamp.
 * @param output_batch Destination immediate outputs.
 * @return Detailed parse, session, replay, or dispatch status.
 */
ProtocolEngineStatus protocol_engine_process_line(ProtocolEngine *engine,
                                                  const char *line,
                                                  size_t length,
                                                  uint64_t timestamp_us,
                                                  ProtocolOutputBatch *output_batch);

/**
 * @brief Detects the first 1,000 ms communication watchdog expiry per session.
 * @param engine Initialized engine.
 * @param timestamp_us Current monotonic timestamp.
 * @return One exactly once after each session timeout, otherwise zero.
 */
uint8_t protocol_engine_watchdog_expired(ProtocolEngine *engine,
                                         uint64_t timestamp_us);

#endif
