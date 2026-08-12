/**
 * @file aethor_app.c
 * @brief Implements the allocation-free Phase 0 application facade.
 */

#include "aethor_app.h"

#include "arm_config.h"

static Diagnostics application_diagnostics;
static ArmController application_controller;
static uint8_t application_initialized;

/**
 * @brief Initializes all static Phase 0 application state.
 * @param timestamp_us Initialization timestamp in microseconds.
 */
void aethor_app_init(uint64_t timestamp_us)
{
    diagnostics_init(&application_diagnostics);
    arm_controller_init(&application_controller,
                        arm_config_get_production(),
                        &application_diagnostics,
                        timestamp_us);
    application_initialized = 1U;
}

/**
 * @brief Executes one non-blocking Phase 0 application service cycle.
 * @param timestamp_us Current monotonic time in microseconds.
 */
void aethor_app_service(uint64_t timestamp_us)
{
    if (application_initialized != 0U)
    {
        arm_controller_step(&application_controller, timestamp_us);
    }
}

/**
 * @brief Copies the current arm state snapshot.
 * @param snapshot Output snapshot.
 * @return true after initialization when snapshot is non-null.
 */
bool aethor_app_get_snapshot(ArmSnapshot *snapshot)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return arm_controller_get_snapshot(&application_controller, snapshot);
}

/**
 * @brief Copies one retained diagnostic event.
 * @param logical_index Zero-based index from the oldest retained event.
 * @param event Output event copy.
 * @return true when initialized and the requested event exists.
 */
bool aethor_app_get_diagnostic(uint16_t logical_index,
                               DiagnosticEvent *event)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return diagnostics_get(&application_diagnostics, logical_index, event);
}

/**
 * @brief Copies the current diagnostic counters.
 * @param counters Output counter snapshot.
 * @return true after initialization when counters is non-null.
 */
bool aethor_app_get_diagnostic_counters(DiagnosticCounters *counters)
{
    if (application_initialized == 0U)
    {
        return false;
    }

    return diagnostics_get_counters(&application_diagnostics, counters);
}
