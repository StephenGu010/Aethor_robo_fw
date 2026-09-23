/** @file adrc_protocol.h
 * @brief Bounded ADRC USB adapter; the application serializes every API with its task critical hooks.
 * @note All asynchronous mutations require nonzero, increasing boot-local request IDs. Replayed
 *       retained IDs return their cached response; evicted IDs below the admission watermark
 *       are rejected. HELLO does not reset this watermark. Restart IDs only after firmware boot.
 * @note Control task consumes commands and publishes snapshots; ProtocolTask alone formats
 *       replies and replaces engine cache entries. No API calls the experiment supervisor.
 * @note Control fields: b0,wc,wo,target,mode,duration_ms. Mapping fields: pos_scale,vel_scale,
 *       torque_scale. Limits: torque_limit,torque_slew,reference_accel,near_zero. Identification:
 *       torque,pulse_ms. values[] follows that ordering; field_mask bit N marks values[N].
 *       Applying any configuration must invalidate prior trusted qualification until the
 *       control owner revalidates it. USB never supplies evidence flags.
 */
#ifndef APP_ADRC_PROTOCOL_H
#define APP_ADRC_PROTOCOL_H
#include "adrc_experiment.h"
#include "../Protocol/protocol_engine.h"

/** @brief Selects a structured operation executed exclusively by the control task. */
typedef enum { ADRC_PROTOCOL_PREPARE = 0, ADRC_PROTOCOL_CONFIG, ADRC_PROTOCOL_RUN,
    ADRC_PROTOCOL_STOP, ADRC_PROTOCOL_CLEAR, ADRC_PROTOCOL_HEARTBEAT } AdrcProtocolCommandType;
/** @brief Selects a configuration patch; no USB group carries trusted evidence. */
typedef enum { ADRC_CONFIG_CONTROL = 0, ADRC_CONFIG_MAPPING, ADRC_CONFIG_LIMITS, ADRC_CONFIG_IDENTIFY } AdrcProtocolConfigGroup;
/** @brief Owns one immutable parsed request; field_mask identifies present values in group order. */
typedef struct
{
    AdrcProtocolCommandType type;
    AdrcProtocolConfigGroup group;
    uint32_t request_id;
    uint64_t timestamp_us;
    uint32_t field_mask;
    float values[6];
    uint8_t axis_index;
} AdrcProtocolCommand;
/** @brief Published immutable view; qualified is supplied only by trusted local commissioning logic. */
typedef struct
{
    AdrcExperimentStatus status;
    uint8_t qualified;
    uint8_t trace_frozen;
} AdrcProtocolSnapshot;
/** @brief Reads an already frozen trace under the application's serialization, never mutating supervision. */
typedef uint8_t (*AdrcProtocolTraceReader)(void *context, uint32_t index, AdrcExperimentTrace *record);
/** @brief Fixed gateway with a normal mailbox and independent STOP and heartbeat mailboxes. */
typedef struct
{
    AdrcProtocolCommand command;
    AdrcProtocolCommand stop_command;
    AdrcProtocolCommand heartbeat_command;
    AdrcProtocolSnapshot snapshot;
    AdrcProtocolTraceReader trace_reader;
    void *trace_context;
    uint32_t active_run_request_id;
    /** @brief Monotonic boot-local mutation IDs prevent reexecution after bounded cache eviction. */
    uint32_t highest_admitted_request_id;
    uint64_t completed_at_us[3];
    AdrcExperimentResult completed_result[3];
    /** @brief Per-channel states: zero free, one queued, two executing, three terminal retained. */
    uint8_t channel_state[3];
    uint8_t cancelled_by_stop[3];
    uint8_t command_pending;
    uint8_t stop_pending;
    uint8_t heartbeat_pending;
} AdrcProtocolGateway;
/** @brief Initializes an unqualified gateway; it cannot authorize a hardware run. */
void adrc_protocol_init(AdrcProtocolGateway *gateway);
/** @brief Installs the optional frozen trace reader before concurrent tasks start. */
void adrc_protocol_set_trace_reader(AdrcProtocolGateway *gateway, AdrcProtocolTraceReader reader, void *context);
/** @brief Publishes a coherent control-task snapshot under the application's critical hooks. */
void adrc_protocol_publish_snapshot(AdrcProtocolGateway *gateway, const AdrcProtocolSnapshot *snapshot);
/** @brief Parses only after shared canonical replay checks; never calls the supervisor. */
ProtocolEngineStatus adrc_protocol_handle_request(void *context, const TextProtocolRequest *request,
    uint64_t timestamp_us, ProtocolOutputBatch *output, uint8_t *retain_request);
/** @brief Control task consumes STOP, heartbeat, then normal work under critical hooks. */
uint8_t adrc_protocol_pop_command(AdrcProtocolGateway *gateway, AdrcProtocolCommand *command);
/** @brief Control task retains the matching executing channel terminal; zero means it was not retained. */
uint8_t adrc_protocol_complete_command(AdrcProtocolGateway *gateway, uint32_t request_id,
    AdrcExperimentResult result, uint64_t timestamp_us);
/** @brief Protocol task replaces cached accepted with DONE and consumes the retained terminal. */
uint8_t adrc_protocol_pop_result_output(AdrcProtocolGateway *gateway, ProtocolEngine *engine,
    ProtocolOutputBatch *output);
#endif
