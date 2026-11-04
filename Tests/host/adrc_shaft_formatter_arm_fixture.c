/** @file adrc_shaft_formatter_arm_fixture.c
 * @brief Offline-only ARM formatter stack witness; fabricated values cannot authorize any motor. */
#include "adrc_app_bridge.h"
#include <float.h>
#include <stdint.h>
#include <string.h>
/** @brief Synthetic static fixture storage keeps fixture buffers out of measured production call-stack depth. */
static AdrcAppBridge formatter_bridge;
static ShaftPilot formatter_core;
static ProtocolOutputBatch formatter_output;
unsigned formatter_length;
/** @brief Prepares finite numeric ABI extremes without calling initialization, HAL, ownership or motion APIs. */
void shaft_formatter_arm_init(void)
{
    ShaftPilotTrace *record;
    unsigned index;
    memset(&formatter_bridge,0,sizeof(formatter_bridge)); memset(&formatter_core,0,sizeof(formatter_core));
    formatter_bridge.initialized=1U; formatter_bridge.shaft.core=&formatter_core;
    formatter_bridge.shaft.used=1U; formatter_bridge.shaft.prepare=2U;
    formatter_bridge.shaft.request_id=UINT32_MAX; formatter_bridge.shaft.boot_id=UINT32_MAX;
    formatter_bridge.shaft.binding_id=UINT64_MAX; formatter_bridge.shaft.session_id=UINT64_MAX;
    formatter_bridge.shaft.owner_epoch=UINT32_MAX;
    formatter_bridge.shaft.capture_id=formatter_bridge.shaft.guard_id=formatter_bridge.shaft.collector_id=UINT32_MAX;
    formatter_bridge.shaft.guard_ack=1U;
    for(index=0U;index<6U;++index) { formatter_bridge.shaft.capability_raw_bits[index]=UINT32_MAX; }
    formatter_bridge.shaft.discovery.observed_esc_id=7U; formatter_bridge.shaft.discovery.observed_master_id=23U;
    formatter_bridge.shaft.discovery.observed_control_mode=1U;
    formatter_bridge.shaft.discovery.verified_fields_mask=MOTOR_DISCOVERY_ALL_FIELDS_MASK;
    formatter_bridge.shaft.discovery.hardware_version=formatter_bridge.shaft.discovery.software_version=UINT32_MAX;
    formatter_bridge.shaft.discovery.sub_version=UINT32_MAX;
    formatter_core.initialized=1U; formatter_core.trace_frozen=1U; formatter_core.status.trace_count=1U;
    formatter_core.status.phase=SHAFT_PHASE_DONE; formatter_core.status.outcome=SHAFT_END_TIME;
    formatter_core.status.safe_release=1U; formatter_core.status.last_successful_decoded_torque_nm=-FLT_MAX;
    formatter_core.status.peak_successful_decoded_torque_nm=FLT_MAX;
    formatter_core.plan.direction=-1; formatter_core.plan.cap_nm=1.75F; formatter_core.plan.slew_nm_s=4.0F; formatter_core.plan.step_nm=.016F;
    formatter_core.config.gear_ratio=19.2031994F; formatter_core.config.position_max_native_rad=12.5F;
    formatter_core.config.velocity_max_rad_s=200.0F; formatter_core.config.torque_max_nm=10.0F;
    formatter_core.config.baseline.point_count=64U; formatter_core.config.baseline.span64_shaft_rad=1.98649049e-5F;
    formatter_core.config.baseline.velocity32_abs_peak_rad_s=.000620778279F;
    formatter_core.config.baseline.raw_zero_center_rad_s=-.0488433838F;
    formatter_core.config.baseline.torque_abs_peak_nm=.0122098923F;
    record=&formatter_core.trace[0]; record->input.now_us=record->input.binding_id=record->input.session_id=UINT64_MAX;
    record->input.motor_id=7U; record->input.feedback.rx_us=record->input.feedback.observed_us=UINT64_MAX;
    record->input.feedback.native_position_rad=record->input.feedback.raw_velocity_rad_s=record->input.feedback.torque_nm=-FLT_MAX;
    record->input.feedback.mos_temperature_c=record->input.feedback.rotor_temperature_c=-FLT_MAX;
    record->input.feedback.driver_fault=UINT32_MAX;
    record->input.receipt.command_id=record->input.receipt.completed_observed_us=UINT64_MAX;
    record->input.receipt.decoded_torque_nm=-FLT_MAX;
    record->output.binding_id=record->output.session_id=record->output.command_id=record->output.issued_us=UINT64_MAX;
    record->output.torque_nm=record->output.previous_sent_nm=record->output.maximum_step_nm=record->output.cap_nm=-FLT_MAX;
    record->window_rx_old_us=record->window_observed_old_us=record->window_rx_new_us=record->window_observed_new_us=UINT64_MAX;
    record->window_tmin_us=record->window_tmax_us=UINT64_MAX;
}
/** @brief Executes one real bridge/parser/adapter/CLIB formatter path; it never enters any motion branch. */
unsigned shaft_formatter_arm_case(unsigned index)
{
    static const char *queries[15]={"4294967295 adrc shaft_status",
        "4294967295 adrc shaft_binding index=0","4294967295 adrc shaft_binding index=1",
        "4294967295 adrc shaft_binding index=2","4294967295 adrc shaft_binding index=3",
        "4294967295 adrc shaft_trace index=0 part=0","4294967295 adrc shaft_trace index=0 part=1",
        "4294967295 adrc shaft_trace index=0 part=2","4294967295 adrc shaft_trace index=0 part=3",
        "4294967295 adrc shaft_binding index=4","4294967295 adrc shaft_binding index=4",
        "4294967295 adrc shaft_binding index=4","4294967295 adrc shaft_binding index=1",
        "4294967295 adrc shaft_binding index=4","4294967295 adrc shaft_binding index=1"};
    if(index>=15U) { return 1U; }
    formatter_core.observe_profile=(uint8_t)(index>=9U);
    formatter_core.platform_profile=(uint8_t)(index>=10U);
    formatter_core.platform_wide_profile=(uint8_t)(index==11U || index==12U);
    formatter_core.platform70_profile=(uint8_t)(index>=13U);
    /* 新索引分别冻结第五分片和PMAX/新量化，仍执行实际生产CLIB格式化。 */
    formatter_core.config.position_max_native_rad=index>=13U?1024.0F:(index>=11U?512.0F:12.5F);
    formatter_core.status.quantum_native_rad=2.0F*formatter_core.config.position_max_native_rad/65535.0F;
    formatter_core.status.quantum_shaft_rad=formatter_core.status.quantum_native_rad/formatter_core.config.gear_ratio;
    if(adrc_app_bridge_process_line(&formatter_bridge,queries[index],strlen(queries[index]),UINT64_MAX,
        &formatter_output)!=PROTOCOL_ENGINE_STATUS_OK) { return 2U; }
    formatter_length=formatter_output.messages[0].length;
    if(index==9U && strcmp(formatter_output.messages[0].data,
        "ok 4294967295 adrc shaft_binding id=4294967295 index=4 profile=4 speed=20.6821518 angle=0.52359879 reverse=0.0872664601 active=1.57079637 stopped=3.14159274 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"))
    { return 4U; }
    if(index==10U && strcmp(formatter_output.messages[0].data,
        "ok 4294967295 adrc shaft_binding id=4294967295 index=4 profile=5 speed=20.6821518 reverse=0.0872664601 reserve=3.14159274 lookahead_us=25000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"))
    { return 5U; }
    if(index==11U && strcmp(formatter_output.messages[0].data,
        "ok 4294967295 adrc shaft_binding id=4294967295 index=4 profile=6 speed=20.6821518 reverse=0.0872664601 reserve=12.566371 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"))
    { return 6U; }
    if(index==12U && (!strstr(formatter_output.messages[0].data,"pmax=512 vmax=200 tmax=10") ||
        !strstr(formatter_output.messages[0].data,"qnative=0.0156252384")))
    { return 7U; }
    if(index==13U && strcmp(formatter_output.messages[0].data,
        "ok 4294967295 adrc shaft_binding id=4294967295 index=4 profile=7 speed=28.9550114 reverse=0.0872664601 reserve=25.1327419 lookahead_us=25000 entry_us=1058000 ramp_us=900000 plateau_us=200000 rated_rpm=395\r\n"))
    { return 8U; }
    if(index==14U && (!strstr(formatter_output.messages[0].data,"pmax=1024 vmax=200 tmax=10") ||
        !strstr(formatter_output.messages[0].data,"qnative=0.0312504768")))
    { return 9U; }
    return formatter_output.count!=1U || formatter_length!=strlen(formatter_output.messages[0].data) ||
        formatter_length>=PROTOCOL_ENGINE_MESSAGE_CAPACITY ? 3U : 0U;
}
