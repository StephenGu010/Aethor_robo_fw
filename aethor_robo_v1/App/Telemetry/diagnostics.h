/**
 * @file diagnostics.h
 * @brief Defines fixed-capacity Phase 0 diagnostic events and counters.
 */

#ifndef APP_TELEMETRY_DIAGNOSTICS_H
#define APP_TELEMETRY_DIAGNOSTICS_H

#include <stdbool.h>
#include <stdint.h>

#define DIAGNOSTICS_CAPACITY (256U)
#define DIAGNOSTIC_WATERMARK_NOT_SAMPLED (UINT32_MAX)

/**
 * @brief Defines diagnostic event severity without text formatting.
 */
typedef enum
{
    DIAGNOSTIC_SEVERITY_INFO = 0,
    DIAGNOSTIC_SEVERITY_WARNING,
    DIAGNOSTIC_SEVERITY_ERROR
} DiagnosticSeverity;

/**
 * @brief Defines stable event identifiers exposed to future telemetry.
 */
typedef enum
{
    DIAGNOSTIC_CODE_BOOT = 1,
    DIAGNOSTIC_CODE_SELF_TEST_STARTED,
    DIAGNOSTIC_CODE_CONFIG_INVALID,
    DIAGNOSTIC_CODE_CONFIG_INCOMPLETE
} DiagnosticCode;

/**
 * @brief Stores one fixed-size structured diagnostic event.
 */
typedef struct
{
    uint64_t timestamp_us;
    uint32_t sequence;
    DiagnosticCode code;
    DiagnosticSeverity severity;
    uint32_t detail;
} DiagnosticEvent;

/**
 * @brief Stores monotonic software counters and sampled resource watermarks.
 */
typedef struct
{
    uint32_t service_cycles;
    uint32_t config_validation_failures;
    uint32_t can_rx_frames;
    uint32_t can_tx_frames;
    uint32_t uart_rx_bytes;
    uint32_t uart_tx_bytes;
    uint32_t queue_high_watermark;
    uint32_t minimum_stack_words;
    uint32_t minimum_heap_bytes;
} DiagnosticCounters;

/**
 * @brief Owns a fixed event ring and counters without dynamic allocation.
 */
typedef struct
{
    DiagnosticEvent events[DIAGNOSTICS_CAPACITY];
    DiagnosticCounters counters;
    uint32_t next_sequence;
    uint32_t dropped_count;
    uint16_t head;
    uint16_t count;
} Diagnostics;

/**
 * @brief Initializes a diagnostic store to a deterministic empty state.
 * @param diagnostics Store to initialize; null is ignored.
 */
void diagnostics_init(Diagnostics *diagnostics);

/**
 * @brief Appends an event and overwrites the oldest event when full.
 * @param diagnostics Store receiving the event.
 * @param timestamp_us Monotonic event timestamp in microseconds.
 * @param code Stable event identifier.
 * @param severity Event severity.
 * @param detail Event-specific numeric detail.
 * @return true when the event was stored.
 */
bool diagnostics_push(Diagnostics *diagnostics,
                      uint64_t timestamp_us,
                      DiagnosticCode code,
                      DiagnosticSeverity severity,
                      uint32_t detail);

/**
 * @brief Copies one event addressed from oldest to newest.
 * @param diagnostics Store to inspect.
 * @param logical_index Zero-based index from the oldest retained event.
 * @param event Output event copy.
 * @return true when the index and pointers are valid.
 */
bool diagnostics_get(const Diagnostics *diagnostics,
                     uint16_t logical_index,
                     DiagnosticEvent *event);

/**
 * @brief Copies current diagnostic counters.
 * @param diagnostics Store to inspect.
 * @param counters Output counter snapshot.
 * @return true when both pointers are valid.
 */
bool diagnostics_get_counters(const Diagnostics *diagnostics,
                              DiagnosticCounters *counters);

/**
 * @brief Records one application service cycle with saturating arithmetic.
 * @param diagnostics Store to update; null is ignored.
 */
void diagnostics_record_service_cycle(Diagnostics *diagnostics);

/**
 * @brief Records one failed configuration validation with saturation.
 * @param diagnostics Store to update; null is ignored.
 */
void diagnostics_record_config_validation_failure(Diagnostics *diagnostics);

#endif
