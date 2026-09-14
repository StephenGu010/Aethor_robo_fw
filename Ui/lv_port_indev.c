/** @file lv_port_indev.c
 * @brief LVGL keypad state is informational; confirmation belongs to the model.
 * dispatch does not call LVGL, so it is safe during the display wait callback.
 */
#include "lv_port_indev.h"
static lv_indev_drv_t keypad_driver;
static uint32_t keypad_code;
static lv_indev_state_t keypad_state = LV_INDEV_STATE_RELEASED;

/** @brief Expose the latest edge without repeating model actions through LVGL. */
static void keypad_read(lv_indev_drv_t *driver, lv_indev_data_t *data)
{
    (void)driver;
    data->key = keypad_code;
    data->state = keypad_state;
    data->continue_reading = false;
}

/** @brief Register one persistent LVGL keypad; no callbacks publish motor commands. */
uint8_t lv_port_indev_init(lv_group_t *group)
{
    lv_indev_t *device;
    lv_indev_drv_init(&keypad_driver);
    keypad_driver.type = LV_INDEV_TYPE_KEYPAD;
    keypad_driver.read_cb = keypad_read;
    device = lv_indev_drv_register(&keypad_driver);
    if (device == NULL) return 0U;
    lv_indev_set_group(device, group);
    return 1U;
}

/** @brief Rotate electrical directions to match the observed landscape mounting. */
DebugUiKey lv_port_indev_screen_key(DebugUiKey key)
{
    switch (key) {
    case DEBUG_UI_KEY_LEFT: return DEBUG_UI_KEY_DOWN;
    case DEBUG_UI_KEY_RIGHT: return DEBUG_UI_KEY_UP;
    case DEBUG_UI_KEY_UP: return DEBUG_UI_KEY_LEFT;
    case DEBUG_UI_KEY_DOWN: return DEBUG_UI_KEY_RIGHT;
    default: return key;
    }
}

/** @brief Deliver screen-relative STOP/model semantics before LVGL key state. */
void lv_port_indev_event(DebugUiModel *model, const DebugUiInputEvent *event)
{
    DebugUiInputEvent screen_event;
    if (model == NULL || event == NULL) return;
    screen_event = *event;
    screen_event.key = lv_port_indev_screen_key(event->key);
    debug_ui_model_event(model, &screen_event); /* STOP precedes display focus. */
    switch (screen_event.key) {
    case DEBUG_UI_KEY_CENTER: keypad_code = LV_KEY_ENTER; break;
    case DEBUG_UI_KEY_LEFT: keypad_code = LV_KEY_LEFT; break;
    case DEBUG_UI_KEY_RIGHT: keypad_code = LV_KEY_RIGHT; break;
    case DEBUG_UI_KEY_UP: keypad_code = LV_KEY_UP; break;
    case DEBUG_UI_KEY_DOWN: keypad_code = LV_KEY_DOWN; break;
    default: keypad_state = LV_INDEV_STATE_RELEASED; return;
    }
    if (event->type == DEBUG_UI_INPUT_EVENT_RELEASE) keypad_state = LV_INDEV_STATE_RELEASED;
    else if (event->type == DEBUG_UI_INPUT_EVENT_PRESS) keypad_state = LV_INDEV_STATE_PRESSED;
}
