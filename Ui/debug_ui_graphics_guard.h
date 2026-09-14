/** @file debug_ui_graphics_guard.h
 * @brief UiTask-local escape boundary for LVGL assertions and pool exhaustion.
 */
#ifndef UI_DEBUG_UI_GRAPHICS_GUARD_H
#define UI_DEBUG_UI_GRAPHICS_GUARD_H
#include <stdint.h>
/** @brief Execute one LVGL-only operation; return zero on a local assertion.
 * No owned hardware buffer is released here. Platform completion/abort retains
 * exclusive responsibility for DMA quiescence and flush_ready notification.
 */
uint8_t debug_ui_graphics_run(void (*operation)(void *), void *context);
/** @brief Escape to the active UI-only boundary after recording graphics failure. */
void debug_ui_graphics_abort(void);
#endif
