/** @file adrc_shaft_pilot_adapter.h
 * @brief Trusted one-boot finite pilot binding, timestamp and independent guard adapter. */
#ifndef APP_ADRC_SHAFT_PILOT_ADAPTER_H
#define APP_ADRC_SHAFT_PILOT_ADAPTER_H
#include "adrc_shaft_pilot.h"
#include "motor_discovery.h"
#include "text_protocol.h"
#include "protocol_engine.h"
struct AdrcAppBridge;
/** @brief Small bridge-owned lifetime; full raw trace lives in a separate static object. */
typedef struct
{
    ShaftPilot *core;
    MotorDiscoveryResult discovery;
    ShaftPilotReceipt receipt;
    uint64_t boot_id, session_id, binding_id;
    uint64_t observed_us, observed_rx_us;
    uint64_t prepare_started_us, prepare_submit_us, prepare_completed_us;
    uint64_t lease_feedback_us, cleanup_dwell_us, cleanup_rx_us;
    uint64_t cleanup_completed_us;
    uint64_t transmit_command_id, transmit_issued_us, external_command_sequence, service_us;
    ShaftPilotSample cleanup_history[SHAFT_PILOT_HISTORY_CAPACITY];
    float cleanup_min_rad, cleanup_max_rad;
    uint32_t owner_epoch, request_id, capture_id, guard_id, collector_id;
    uint32_t capability_raw_bits[6];
    uint8_t used, prepare, guard_armed, guard_ack, bridge_fault, stop_latched;
    /* Accepted handback freezes historical pilot evidence, not live release authority. */
    uint8_t prepare_receipt, cleanup_receipt, cleanup_safe_release, released, handoff_started;
    uint8_t cleanup_history_count;
} AdrcShaftPilotAdapter;
/** @brief Initializes separate fixed storage only as part of serialized application initialization. */
void adrc_shaft_adapter_init(struct AdrcAppBridge *bridge);
/** @brief Saves receive upper bound only after the runtime accepted a new motor-7 normal feedback. */
void adrc_shaft_adapter_note_feedback(struct AdrcAppBridge *bridge, uint32_t accepted_before,
    uint64_t rx_lower_us, uint64_t observed_us);
/** @brief Saves the first real preparation submission while its single disable slot is outstanding. */
void adrc_shaft_adapter_note_submit(struct AdrcAppBridge *bridge, uint64_t timestamp_us);
/** @brief Retains matching real completion before any stop or slot clearing can erase it. */
void adrc_shaft_adapter_receipt(struct AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t success, uint64_t observed_us);
/** @brief Executes exclusive preparation, finite core, guard and persistent disable recovery. */
uint8_t adrc_shaft_adapter_service(struct AdrcAppBridge *bridge, uint64_t now_us);
/** @brief Handles fixed entry/read-only shards and revokes conflicting legacy mutations. */
uint8_t adrc_shaft_adapter_process(struct AdrcAppBridge *bridge, const TextProtocolRequest *request,
    uint64_t now_us, ProtocolOutputBatch *output, ProtocolEngineStatus *status);
/** @brief Returns true only after current safe release and actual independent guard acknowledgement. */
uint8_t adrc_shaft_adapter_release_ready(const struct AdrcAppBridge *bridge);
#endif
