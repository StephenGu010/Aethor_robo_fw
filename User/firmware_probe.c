/**
 * @file firmware_probe.c
 * @brief Formats bounded key-value probe events without dynamic allocation.
 */

#include "firmware_probe.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define FIRMWARE_PROBE_LINE_CAPACITY 192U

static FirmwareProbeEmitFunction firmware_probe_emit_function;
static FirmwareProbeSnapshot firmware_probe_previous_snapshot;
static FirmwareProbeStatistics firmware_probe_statistics;
static uint8_t firmware_probe_initialized;

/**
 * @brief Emits one preformatted bounded probe line and updates delivery counters.
 * @param line Null-terminated probe line.
 */
static void firmware_probe_emit_line(const char *line)
{
    assert(line != NULL);
    if ((firmware_probe_emit_function != NULL) &&
        (firmware_probe_emit_function(line) == 0))
    {
        firmware_probe_statistics.emitted_line_count++;
    }
    else
    {
        firmware_probe_statistics.dropped_line_count++;
    }
}

/**
 * @brief Returns a stable machine-readable name for one firmware stage.
 * @param stage Firmware stage.
 * @return Static uppercase stage name.
 */
const char *firmware_probe_stage_name(FirmwareProbeStage stage)
{
    switch (stage)
    {
        case FIRMWARE_PROBE_STAGE_BOOT:
            return "BOOT";
        case FIRMWARE_PROBE_STAGE_CONFIG_VALIDATE:
            return "CONFIG_VALIDATE";
        case FIRMWARE_PROBE_STAGE_MODE_SETUP:
            return "MODE_SETUP";
        case FIRMWARE_PROBE_STAGE_RANGE_DISCOVERY:
            return "RANGE_DISCOVERY";
        case FIRMWARE_PROBE_STAGE_READY:
            return "READY";
        case FIRMWARE_PROBE_STAGE_ENABLING:
            return "ENABLING";
        case FIRMWARE_PROBE_STAGE_MOVING:
            return "MOVING";
        case FIRMWARE_PROBE_STAGE_HOLDING:
            return "HOLDING";
        case FIRMWARE_PROBE_STAGE_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Initializes transition tracking and announces the current boot profile.
 * @param emit_function Task-context line delivery callback.
 * @param initial_snapshot Initial controller snapshot.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
void firmware_probe_init(FirmwareProbeEmitFunction emit_function,
                         const FirmwareProbeSnapshot *initial_snapshot,
                         uint32_t current_time_ms)
{
    char line[FIRMWARE_PROBE_LINE_CAPACITY];

    assert(emit_function != NULL);
    assert(initial_snapshot != NULL);
    firmware_probe_emit_function = emit_function;
    firmware_probe_previous_snapshot = *initial_snapshot;
    memset(&firmware_probe_statistics, 0, sizeof(firmware_probe_statistics));
    firmware_probe_initialized = 1U;
    (void)snprintf(line,
                   sizeof(line),
                   "probe seq=1 t=%lu level=INFO event=boot stage=%s active=0x%02X",
                   (unsigned long)current_time_ms,
                   firmware_probe_stage_name(initial_snapshot->stage),
                   (unsigned int)initial_snapshot->active_mask);
    firmware_probe_emit_line(line);
}

/**
 * @brief Emits only stage, mask, and fault transitions from one controller snapshot.
 * @param snapshot Current controller snapshot.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
void firmware_probe_observe(const FirmwareProbeSnapshot *snapshot,
                            uint32_t current_time_ms)
{
    char line[FIRMWARE_PROBE_LINE_CAPACITY];

    if ((snapshot == NULL) || (firmware_probe_initialized == 0U))
    {
        return;
    }

    if (snapshot->stage != firmware_probe_previous_snapshot.stage)
    {
        (void)snprintf(line,
                       sizeof(line),
                       "probe seq=%lu t=%lu level=INFO event=stage old=%s new=%s",
                       (unsigned long)(firmware_probe_statistics.emitted_line_count + 1U),
                       (unsigned long)current_time_ms,
                       firmware_probe_stage_name(firmware_probe_previous_snapshot.stage),
                       firmware_probe_stage_name(snapshot->stage));
        firmware_probe_emit_line(line);
    }
    if ((snapshot->active_mask != firmware_probe_previous_snapshot.active_mask) ||
        (snapshot->ready_mask != firmware_probe_previous_snapshot.ready_mask) ||
        (snapshot->enabled_mask != firmware_probe_previous_snapshot.enabled_mask) ||
        (snapshot->arrived_mask != firmware_probe_previous_snapshot.arrived_mask))
    {
        (void)snprintf(line,
                       sizeof(line),
                       "probe seq=%lu t=%lu level=INFO event=masks active=0x%02X ready=0x%02X enabled=0x%02X arrived=0x%02X",
                       (unsigned long)(firmware_probe_statistics.emitted_line_count + 1U),
                       (unsigned long)current_time_ms,
                       (unsigned int)snapshot->active_mask,
                       (unsigned int)snapshot->ready_mask,
                       (unsigned int)snapshot->enabled_mask,
                       (unsigned int)snapshot->arrived_mask);
        firmware_probe_emit_line(line);
    }
    if ((snapshot->fault_code != 0U) &&
        ((snapshot->fault_code != firmware_probe_previous_snapshot.fault_code) ||
         (snapshot->fault_joint != firmware_probe_previous_snapshot.fault_joint) ||
         (snapshot->bus_off != firmware_probe_previous_snapshot.bus_off)))
    {
        (void)snprintf(line,
                       sizeof(line),
                       "probe seq=%lu t=%lu level=ERROR event=fault code=%u joint=%u busoff=%u",
                       (unsigned long)(firmware_probe_statistics.emitted_line_count + 1U),
                       (unsigned long)current_time_ms,
                       (unsigned int)snapshot->fault_code,
                       (unsigned int)snapshot->fault_joint,
                       (unsigned int)snapshot->bus_off);
        firmware_probe_emit_line(line);
    }
    firmware_probe_previous_snapshot = *snapshot;
}

/**
 * @brief Emits one correlated command acceptance or rejection event.
 * @param command_name Stable command name without user payload data.
 * @param command_sequence Sequence allocated at the USB command boundary.
 * @param result Stable result name.
 * @param current_time_ms Current monotonic time in milliseconds.
 */
void firmware_probe_record_command(const char *command_name,
                                   uint32_t command_sequence,
                                   const char *result,
                                   uint32_t current_time_ms)
{
    char line[FIRMWARE_PROBE_LINE_CAPACITY];

    if ((command_name == NULL) || (result == NULL) ||
        (firmware_probe_initialized == 0U))
    {
        return;
    }
    firmware_probe_statistics.command_sequence = command_sequence;
    (void)snprintf(line,
                   sizeof(line),
                   "probe seq=%lu t=%lu level=INFO event=command cmd_seq=%lu name=%.24s result=%.24s",
                   (unsigned long)(firmware_probe_statistics.emitted_line_count + 1U),
                   (unsigned long)current_time_ms,
                   (unsigned long)command_sequence,
                   command_name,
                   result);
    firmware_probe_emit_line(line);
}

/**
 * @brief Returns probe delivery and command-correlation counters.
 * @return Address of static read-only statistics.
 */
const FirmwareProbeStatistics *firmware_probe_get_statistics(void)
{
    return &firmware_probe_statistics;
}
