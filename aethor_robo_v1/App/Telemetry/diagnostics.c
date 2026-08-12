/**
 * @file diagnostics.c
 * @brief Implements fixed-capacity diagnostic storage without heap allocation.
 */

#include "diagnostics.h"

#include <assert.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Increments a counter without allowing unsigned wraparound.
 * @param value Counter to update.
 */
static void diagnostics_increment_saturating(uint32_t *value)
{
    assert(value != NULL);
    if (*value < UINT32_MAX)
    {
        (*value)++;
    }
}

/**
 * @brief Initializes a diagnostic store to a deterministic empty state.
 * @param diagnostics Store to initialize; null is ignored.
 */
void diagnostics_init(Diagnostics *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    (void)memset(diagnostics, 0, sizeof(*diagnostics));
    diagnostics->next_sequence = 1U;
    diagnostics->counters.minimum_stack_words = DIAGNOSTIC_WATERMARK_NOT_SAMPLED;
    diagnostics->counters.minimum_heap_bytes = DIAGNOSTIC_WATERMARK_NOT_SAMPLED;
}

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
                      uint32_t detail)
{
    uint16_t write_index;
    DiagnosticEvent *event;

    if (diagnostics == NULL)
    {
        return false;
    }

    assert(diagnostics->count <= DIAGNOSTICS_CAPACITY);
    assert(diagnostics->head < DIAGNOSTICS_CAPACITY);

    if (diagnostics->count < DIAGNOSTICS_CAPACITY)
    {
        write_index = (uint16_t)((diagnostics->head + diagnostics->count) %
                                 DIAGNOSTICS_CAPACITY);
        diagnostics->count++;
    }
    else
    {
        write_index = diagnostics->head;
        diagnostics->head = (uint16_t)((diagnostics->head + 1U) %
                                       DIAGNOSTICS_CAPACITY);
        diagnostics_increment_saturating(&diagnostics->dropped_count);
    }

    event = &diagnostics->events[write_index];
    event->timestamp_us = timestamp_us;
    event->sequence = diagnostics->next_sequence;
    event->code = code;
    event->severity = severity;
    event->detail = detail;

    diagnostics->next_sequence++;
    if (diagnostics->next_sequence == 0U)
    {
        diagnostics->next_sequence = 1U;
    }

    if (diagnostics->counters.queue_high_watermark < diagnostics->count)
    {
        diagnostics->counters.queue_high_watermark = diagnostics->count;
    }

    return true;
}

/**
 * @brief Copies one event addressed from oldest to newest.
 * @param diagnostics Store to inspect.
 * @param logical_index Zero-based index from the oldest retained event.
 * @param event Output event copy.
 * @return true when the index and pointers are valid.
 */
bool diagnostics_get(const Diagnostics *diagnostics,
                     uint16_t logical_index,
                     DiagnosticEvent *event)
{
    uint16_t physical_index;

    if ((diagnostics == NULL) || (event == NULL) ||
        (logical_index >= diagnostics->count))
    {
        return false;
    }

    assert(diagnostics->count <= DIAGNOSTICS_CAPACITY);
    assert(diagnostics->head < DIAGNOSTICS_CAPACITY);
    physical_index = (uint16_t)((diagnostics->head + logical_index) %
                                DIAGNOSTICS_CAPACITY);
    *event = diagnostics->events[physical_index];
    return true;
}

/**
 * @brief Copies current diagnostic counters.
 * @param diagnostics Store to inspect.
 * @param counters Output counter snapshot.
 * @return true when both pointers are valid.
 */
bool diagnostics_get_counters(const Diagnostics *diagnostics,
                              DiagnosticCounters *counters)
{
    if ((diagnostics == NULL) || (counters == NULL))
    {
        return false;
    }

    *counters = diagnostics->counters;
    return true;
}

/**
 * @brief Records one application service cycle with saturating arithmetic.
 * @param diagnostics Store to update; null is ignored.
 */
void diagnostics_record_service_cycle(Diagnostics *diagnostics)
{
    if (diagnostics != NULL)
    {
        diagnostics_increment_saturating(&diagnostics->counters.service_cycles);
    }
}

/**
 * @brief Records one failed configuration validation with saturation.
 * @param diagnostics Store to update; null is ignored.
 */
void diagnostics_record_config_validation_failure(Diagnostics *diagnostics)
{
    if (diagnostics != NULL)
    {
        diagnostics_increment_saturating(
            &diagnostics->counters.config_validation_failures);
    }
}
