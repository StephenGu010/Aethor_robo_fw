/** @file adrc_interface_analysis_bridge.c
 * @brief Host-only complete production interface replay with synthetic motor-7 evidence.
 * No device transports are linked. Hardware-provenance fixtures exist only in this
 * process to exercise the local evidence gate; they never qualify a real motor.
 */
#include <stdio.h>
#include <string.h>
#include "adrc_app_bridge.h"
#include "adrc_generated_adapter.h"
#include "adrc_controller.h"

static AdrcAppBridge interface_bridge;
static MotorRuntime interface_runtime;
static ProtocolEngine interface_engine;
static uint32_t interface_request_id;

/** @brief Initialize real runtime/bridge code around explicitly synthetic discovered ranges. */
int interface_analysis_initialize(float position_range, float velocity_range, float torque_range)
{
    MotorDiscoveryResult *discovery;
    if (motor_runtime_init(&interface_runtime, arm_config_get_production()) != MOTOR_RUNTIME_STATUS_OK)
    { return 0; }
    protocol_engine_init(&interface_engine, 1U);
    adrc_controller_initialize();
    if (adrc_app_bridge_init_deferred(&interface_bridge, &interface_runtime, &interface_engine,
        adrc_generated_controller_step, NULL) != ADRC_RESULT_OK) { return 0; }
    interface_bridge.bench.draft_config.axis_index = 6U;
    interface_bridge.discovery_axis = 6U;
    discovery = &interface_runtime.discovery.results[6];
    discovery->verified_fields_mask = MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    discovery->observed_esc_id = 7U;
    discovery->observed_master_id = 0x17U;
    discovery->observed_control_mode = 1U;
    discovery->ranges.position_max_rad = position_range;
    discovery->ranges.velocity_max_rad_s = velocity_range;
    discovery->ranges.torque_max_nm = torque_range;
    interface_runtime.discovery.verified_joint_mask = 0x40U;
    interface_runtime.discovery.state = MOTOR_DISCOVERY_STATE_COMPLETE;
    interface_runtime.discovery_active = 0U;
    interface_request_id = 0U;
    return 1;
}

/** @brief Feed a simulated CAN feedback frame through production runtime decode and identity routing. */
int interface_analysis_feedback(uint64_t feedback_us, uint32_t position_code,
    uint32_t velocity_code, uint8_t enabled)
{
    CanFrame frame;
    if (position_code > 65535U || velocity_code > 4095U || enabled > 1U) { return -1; }
    memset(&frame, 0, sizeof(frame));
    frame.identifier = 0x17U;
    frame.length = 8U;
    frame.data[0] = (uint8_t)((enabled << 4U) | 7U);
    frame.data[1] = (uint8_t)(position_code >> 8U);
    frame.data[2] = (uint8_t)position_code;
    frame.data[3] = (uint8_t)(velocity_code >> 4U);
    frame.data[4] = (uint8_t)(((velocity_code & 15U) << 4U) | 7U);
    frame.data[5] = 0xffU;
    frame.data[6] = 25U;
    frame.data[7] = 25U;
    return (int)motor_runtime_accept_frame(&interface_runtime, &frame, feedback_us);
}

/** @brief Install synthetic local evidence only after passive production estimator warmup. */
int interface_analysis_configure_tuned(uint64_t now_us, uint8_t mode, float reference,
    float bandwidth, float observer_bandwidth)
{
    AdrcExperimentConfig config = interface_bridge.bench.draft_config;
    AdrcExperimentQualification evidence;
    memset(&evidence, 0, sizeof(evidence));
    config.mode = (AdrcExperimentMode)mode;
    config.interface_profile = ADRC_INTERFACE_POSITION32_QUANTIZED;
    config.duration_us = 1200000U;
    config.reference_rad_s = reference;
    config.reference_accel_rad_s2 = 0.2F;
    config.stop_accel_rad_s2 = 0.5F;
    config.torque_slew_nm_s = 1.25F;
    config.torque_limit_nm = 0.1F;
    config.velocity_quantum_rad_s = 0.012F;
    config.near_zero_rad_s = 0.025F;
    config.standstill_noise_rad_s = 0.0005F;
    config.b0 = 10.0F;
    config.wc_rad_s = bandwidth;
    config.wo_rad_s = observer_bandwidth;
    evidence.axis_index = 6U;
    evidence.provenance = ADRC_ENV_HARDWARE;
    evidence.verified_flags = ADRC_QUAL_CLOSED_LOOP_REQUIRED | ADRC_QUAL_POSITION32_REQUIRED;
    evidence.position_max_rad = 12.5F;
    evidence.velocity_max_rad_s = 0.3F;
    evidence.torque_max_nm = 10.0F;
    return (int)adrc_app_bridge_set_evidence(&interface_bridge, &config, &evidence, now_us);
}

/** @brief Preserve the original five-to-one tuning entry for exact baseline reproduction. */
int interface_analysis_configure(uint64_t now_us, uint8_t mode, float reference, float bandwidth)
{
    return interface_analysis_configure_tuned(now_us, mode, reference, bandwidth, 5.0F * bandwidth);
}

/** @brief Exercise real text admission and completion channels; command choices grant no evidence. */
int interface_analysis_command(uint64_t now_us, uint8_t command)
{
    static const char *const commands[] = { "prepare motor=7", "run", "heartbeat", "stop" };
    char line[80];
    int length;
    ProtocolOutputBatch output;
    if (command >= 4U) { return -1; }
    length = snprintf(line, sizeof(line), "%lu adrc %s", (unsigned long)++interface_request_id,
        commands[command]);
    if (length <= 0 || (size_t)length >= sizeof(line)) { return -1; }
    return (int)adrc_app_bridge_process_line(&interface_bridge, line, (size_t)length, now_us, &output);
}

/** @brief Service production supervisor/generated control, return frame and match a synthetic TX receipt.
 * Values: frame kind (-1=no frame), nominal decoded torque, state, fault,
 * configured, disabled-confirmed, estimated velocity, raw velocity, trace flags,
 * trace count, actual 12-bit transmitted torque code (-1 for no torque frame).
 */
int interface_analysis_service(uint64_t now_us, float *values)
{
    CanFrame frame;
    ProtocolOutputBatch output;
    AdrcExperiment *experiment = &interface_bridge.bench.experiment;
    uint8_t kind = 0U;
    float decoded = 0.0F;
    unsigned int index;
    if (values == NULL) { return 0; }
    for (index = 0U; index < 11U; ++index) { values[index] = 0.0F; }
    values[0] = -1.0F;
    values[10] = -1.0F;
    (void)adrc_app_bridge_service(&interface_bridge, now_us);
    if (adrc_app_bridge_pop_frame(&interface_bridge, &frame, &kind, &decoded) != 0U)
    {
        values[0] = (float)kind;
        values[1] = decoded;
        if (kind == 1U)
        {
            if ((frame.data[3] & 15U) != 0U || frame.data[4] != 0U ||
                frame.data[5] != 0U || (frame.data[6] & 0xf0U) != 0U) { return 0; }
            values[10] = (float)(((uint16_t)(frame.data[6] & 15U) << 8U) | frame.data[7]);
        }
        adrc_app_bridge_report_transmit(&interface_bridge, kind, decoded, 1U);
    }
    /* Consume retained terminals exactly as a polling client would do. */
    while (adrc_protocol_pop_result_output(&interface_bridge.bench.gateway,
        &interface_engine, &output) != 0U) { }
    values[2] = (float)experiment->status.state;
    values[3] = (float)experiment->status.fault;
    values[4] = (float)experiment->configured;
    values[5] = (float)experiment->status.disabled_confirmed;
    values[9] = (float)experiment->status.trace_count;
    if (experiment->status.trace_count != 0U)
    {
        const AdrcExperimentTrace *record = &experiment->trace[experiment->status.trace_count - 1U];
        values[6] = record->velocity_rad_s;
        values[7] = record->raw_velocity_rad_s;
        values[8] = (float)record->flags;
    }
    return 1;
}

/** @brief Observe host supervision timing and controller internals without changing control state. */
int interface_analysis_diagnostics(double *values)
{
    const AdrcExperiment *experiment = &interface_bridge.bench.experiment;
    const AdrcExperimentTrace *record;
    unsigned int index;
    if (values == NULL) { return 0; }
    for (index = 0U; index < 8U; ++index) { values[index] = 0.0; }
    values[0] = experiment->reference_rad_s;
    values[4] = experiment->low_speed_tracking;
    values[5] = (double)experiment->low_speed_started_us;
    values[6] = (double)experiment->disable_requested_us;
    values[7] = (double)experiment->stop_started_us;
    if (experiment->status.trace_count != 0U)
    {
        record = &experiment->trace[experiment->status.trace_count - 1U];
        values[1] = record->torque_raw_nm;
        values[2] = record->z1;
        values[3] = record->z2;
    }
    return 1;
}
