/** @file debug_ui_graphics_guard.c
 * @brief Bounded local failure handling; never disables global interrupts.
 * Guard calls must originate from the single UiTask owner, never an ISR.
 */
#include "debug_ui_graphics_guard.h"
#include <setjmp.h>
#include <stddef.h>
static jmp_buf graphics_boundary;
static uint8_t boundary_active;

/** @brief Execute one UiTask operation with a local, non-reentrant fault boundary. */
uint8_t debug_ui_graphics_run(void (*operation)(void *), void *context)
{
    if (operation == NULL || boundary_active) return 0U;
    if (setjmp(graphics_boundary) != 0) {
        boundary_active = 0U;
        return 0U;
    }
    boundary_active = 1U;
    operation(context);
    boundary_active = 0U;
    return 1U;
}

/** @brief Leave a failing LVGL call before any caller dereferences failed allocation. */
void debug_ui_graphics_abort(void)
{
    if (boundary_active) longjmp(graphics_boundary, 1);
    /* All production entrypoints are guarded. ISR flush_ready has no allocator. */
}
