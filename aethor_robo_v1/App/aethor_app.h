/**
 * @file aethor_app.h
 * @brief Defines the Phase 0 application lifecycle and read-only snapshots.
 */

#ifndef APP_AETHOR_APP_H
#define APP_AETHOR_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "arm_controller.h"
#include "diagnostics.h"

/**
 * @brief Initializes all static Phase 0 application state.
 * @param timestamp_us Initialization timestamp in microseconds.
 */
void aethor_app_init(uint64_t timestamp_us);

/**
 * @brief Executes one non-blocking Phase 0 application service cycle.
 * @param timestamp_us Current monotonic time in microseconds.
 */
void aethor_app_service(uint64_t timestamp_us);

/**
 * @brief Copies the current arm state snapshot.
 * @param snapshot Output snapshot.
 * @return true after initialization when snapshot is non-null.
 */
bool aethor_app_get_snapshot(ArmSnapshot *snapshot);

/**
 * @brief Copies one retained diagnostic event.
 * @param logical_index Zero-based index from the oldest retained event.
 * @param event Output event copy.
 * @return true when initialized and the requested event exists.
 */
bool aethor_app_get_diagnostic(uint16_t logical_index,
                               DiagnosticEvent *event);

/**
 * @brief Copies the current diagnostic counters.
 * @param counters Output counter snapshot.
 * @return true after initialization when counters is non-null.
 */
bool aethor_app_get_diagnostic_counters(DiagnosticCounters *counters);

#endif
