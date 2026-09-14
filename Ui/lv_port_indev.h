/** @file lv_port_indev.h
 * @brief Debounced five-key model dispatch plus read-only LVGL keypad bridge.
 */
#ifndef UI_LV_PORT_INDEV_H
#define UI_LV_PORT_INDEV_H
#include "debug_ui_view.h"
/** @brief Register a keypad with no activation callbacks; model owns all commands. */
uint8_t lv_port_indev_init(lv_group_t *group);
/** @brief Map electrical key labels to directions relative to landscape text. */
DebugUiKey lv_port_indev_screen_key(DebugUiKey key);
/** @brief Process model events immediately, including STOP during display wait. */
void lv_port_indev_event(DebugUiModel *model, const DebugUiInputEvent *event);
#endif
