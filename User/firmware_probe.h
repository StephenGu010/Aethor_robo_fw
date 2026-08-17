/**
 * @file firmware_probe.h
 * @brief Transition-only structured diagnostics for the USB CDC debug channel.
 */

#ifndef FIRMWARE_PROBE_H
#define FIRMWARE_PROBE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Stable firmware stages exposed to COM-port tooling. */
typedef enum
{
    FIRMWARE_PROBE_STAGE_BOOT = 0,
    FIRMWARE_PROBE_STAGE_CONFIG_VALIDATE,
    FIRMWARE_PROBE_STAGE_MODE_SETUP,
    FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY,
    FIRMWARE_PROBE_STAGE_READY,
    FIRMWARE_PROBE_STAGE_ENABLING,
    FIRMWARE_PROBE_STAGE_MOVING,
    FIRMWARE_PROBE_STAGE_HOLDING,
    FIRMWARE_PROBE_STAGE_FAULT
} FirmwareProbeStage;

/** @brief Bounded state projected by the controller into diagnostic events. */
typedef struct
{
    FirmwareProbeStage stage;
    uint8_t active_mask;
    uint8_t ready_mask;
    uint8_t enabled_mask;
    uint8_t arrived_mask;
    uint8_t fault_code;
    uint8_t fault_joint;
    uint8_t bus_off;
} FirmwareProbeSnapshot;

/** @brief Probe delivery counters used to detect a saturated USB transmit queue. */
typedef struct
{
    uint32_t emitted_line_count;
    uint32_t dropped_line_count;
    uint32_t command_sequence;
} FirmwareProbeStatistics;

typedef int (*FirmwareProbeEmitFunction)(const char *line);

void firmware_probe_init(FirmwareProbeEmitFunction emit_function,
                         const FirmwareProbeSnapshot *initial_snapshot,
                         uint32_t current_time_ms);
void firmware_probe_observe(const FirmwareProbeSnapshot *snapshot,
                            uint32_t current_time_ms);
void firmware_probe_record_command(const char *command_name,
                                   uint32_t command_sequence,
                                   const char *result,
                                   uint32_t current_time_ms);
const FirmwareProbeStatistics *firmware_probe_get_statistics(void);
const char *firmware_probe_stage_name(FirmwareProbeStage stage);

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_PROBE_H */
