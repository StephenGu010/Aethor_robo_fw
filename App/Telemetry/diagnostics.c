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
    diagnostics->counters.control_period_min_us =
        DIAGNOSTIC_WATERMARK_NOT_SAMPLED;
    diagnostics->counters.control_group_skew_max_us =
        DIAGNOSTIC_WATERMARK_NOT_SAMPLED;
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
        diagnostics->counters.event_overwrite_count = diagnostics->dropped_count;
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

/**
 * @brief Records one observed ArmControlTask start-to-start period.
 */
void diagnostics_record_control_period(Diagnostics *diagnostics,
                                       uint32_t period_us)
{
    DiagnosticCounters *counters;

    if ((diagnostics == NULL) || (period_us == 0U))
    {
        return;
    }
    counters = &diagnostics->counters;
    counters->control_period_last_us = period_us;
    if ((counters->control_period_min_us == DIAGNOSTIC_WATERMARK_NOT_SAMPLED) ||
        (period_us < counters->control_period_min_us))
    {
        counters->control_period_min_us = period_us;
    }
    if (period_us > counters->control_period_max_us)
    {
        counters->control_period_max_us = period_us;
    }
    if (period_us > 4000U)
    {
        diagnostics_increment_saturating(
            &counters->control_deadline_miss_count);
        diagnostics_increment_saturating(
            &counters->control_consecutive_miss_count);
        if (counters->control_consecutive_miss_max <
            counters->control_consecutive_miss_count)
        {
            counters->control_consecutive_miss_max =
                counters->control_consecutive_miss_count;
        }
    }
    else
    {
        counters->control_consecutive_miss_count = 0U;
    }
}

/**
 * @brief Replaces sampled transport/resource values with a coherent snapshot.
 */
void diagnostics_update_runtime_sample(
    Diagnostics *diagnostics,
    const RuntimeDiagnosticSample *sample)
{
    DiagnosticCounters *counters;

    if ((diagnostics == NULL) || (sample == NULL))
    {
        return;
    }
    counters = &diagnostics->counters;
    counters->can_rx_frames = sample->can_rx_frames;
    counters->can_tx_frames = sample->can_tx_frames;
    counters->can_rx_overflow_count = sample->can_rx_overflow_count;
    counters->can_tx_error_count = sample->can_tx_error_count;
    counters->can_bus_off_count = sample->can_bus_off_count;
    counters->can_tx_queue_high_watermark =
        sample->can_tx_queue_high_watermark;
    counters->control_group_reject_count =
        sample->control_group_reject_count;
    counters->usb_rx_bytes = sample->usb_rx_bytes;
    counters->usb_rx_overflow_count = sample->usb_rx_overflow_count;
    counters->usb_overlong_line_count = sample->usb_overlong_line_count;
    counters->usb_high_queue_high_watermark =
        sample->usb_high_queue_high_watermark;
    counters->usb_query_queue_high_watermark =
        sample->usb_query_queue_high_watermark;
    counters->usb_telemetry_queue_high_watermark =
        sample->usb_telemetry_queue_high_watermark;
    counters->usb_telemetry_drop_count = sample->usb_telemetry_drop_count;
    counters->usb_high_queue_full_count = sample->usb_high_queue_full_count;
    counters->usb_transmit_busy_count = sample->usb_transmit_busy_count;
    counters->usb_transmit_error_count = sample->usb_transmit_error_count;
    counters->control_group_skew_max_us = sample->control_group_skew_max_us;
    counters->minimum_stack_words = sample->minimum_stack_words;
    counters->minimum_heap_bytes = sample->minimum_heap_bytes;
}
