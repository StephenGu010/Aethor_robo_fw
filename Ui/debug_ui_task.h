/** @file debug_ui_task.h
 * @brief Static UiTask entrypoint and narrow Core-owned scheduling/timing hooks.
 */
#ifndef UI_DEBUG_UI_TASK_H
#define UI_DEBUG_UI_TASK_H
#include <stdint.h>
/** @brief CMSIS-RTOS v1 entrypoint; register 2048 words, Low priority, UI enabled only. */
void StartDebugUiTask(void const *argument);
/** @brief Core hook: notify ProtocolTask of STOP/request/independent health progress. */
void AethorNotifyProtocolTask(void);
/** @brief Core hook: application monotonic microseconds, copied in task context. */
uint64_t AethorUiTimestampUs(void);
/** @brief Core hook: measured maximum complete control execution and period, us. */
void AethorGetControlTiming(uint32_t *execution_max_us, uint32_t *period_max_us);
#endif
