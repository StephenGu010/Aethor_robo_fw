/** @file adrc_app_bridge.h
 * @brief Single-owner ADRC application adapter; callers serialize every API using task critical hooks.
 * @note Qualification is rejected when decoded torque steps cannot satisfy one 4 ms slew increment,
 *       when even half a torque LSB exceeds the amplitude cap, or when supplied velocity resolution
 *       understates the protocol grid. An unencodable command disables; limits are never expanded.
 *       The grid generally has no exact zero. CAN receipts are nominal commands, not measured torque.
 */
#ifndef APP_ADRC_APP_BRIDGE_H
#define APP_ADRC_APP_BRIDGE_H
#include "adrc_bench.h"
#include "motor_runtime.h"

/** @brief Fixed bridge storage with a single expiring output and matched transmit receipt. */
typedef struct
{
    AdrcBench bench;
    MotorRuntime *runtime;
    ProtocolEngine *engine;
    CanFrame frame;
    float decoded_torque_nm;
    float receipt_torque_nm;
    uint64_t next_feedback_query_us;
    uint64_t last_feedback_us;
    uint32_t feedback_query_count;
    uint32_t feedback_sample_count;
    uint32_t feedback_interval_min_us;
    uint32_t feedback_interval_max_us;
    uint8_t frame_kind;
    uint8_t frame_pending;
    uint8_t awaiting_receipt;
    uint8_t receipt_valid;
    uint8_t send_failed;
    uint8_t stop_pending;
    uint8_t discovery_axis;
    uint8_t initialized;
} AdrcAppBridge;
/** @brief Initializes hardware provenance with no qualification and registers the protocol gateway. */
AdrcExperimentResult adrc_app_bridge_init(AdrcAppBridge *bridge, MotorRuntime *runtime,
    ProtocolEngine *engine, AdrcControllerCallback controller, void *controller_context);
/** @brief Runs only the selected-axis 4 ms owner and replaces the expiring output slot. */
uint8_t adrc_app_bridge_service(AdrcAppBridge *bridge, uint64_t timestamp_us);
/** @brief Rejects all legacy mutation paths before they can reach their command queues. */
ProtocolEngineStatus adrc_app_bridge_process_line(AdrcAppBridge *bridge, const char *line,
    size_t length, uint64_t timestamp_us, ProtocolOutputBatch *output);
/** @brief Pops the sole current frame; kind is enable=0, nominal torque=1, disable=2. */
uint8_t adrc_app_bridge_pop_frame(AdrcAppBridge *bridge, CanFrame *frame,
    uint8_t *kind, float *decoded_torque_nm);
/** @brief Matches a bus transmission receipt; nominal decoded torque is not measured motor torque. */
void adrc_app_bridge_report_transmit(AdrcAppBridge *bridge, uint8_t kind,
    float decoded_torque_nm, uint8_t succeeded);
/** @brief Latches a local safety stop for the control owner and revokes unsent actuation. */
void adrc_app_bridge_request_stop(AdrcAppBridge *bridge);
/** @brief Latches a transport failure and selected-axis disable request without touching other motors. */
void adrc_app_bridge_report_fault(AdrcAppBridge *bridge);
/** @brief Returns only read-only discovery while no experiment or frame is active. */
MotorRuntimeStatus adrc_app_bridge_next_discovery(AdrcAppBridge *bridge, uint64_t timestamp_us,
    CanFrame *frame);
/** @brief Installs local measured evidence only after matching discovered MIT identity/ranges and feedback. */
AdrcExperimentResult adrc_app_bridge_set_evidence(AdrcAppBridge *bridge,
    const AdrcExperimentConfig *config, const AdrcExperimentQualification *qualification,
    uint64_t timestamp_us);
#endif
