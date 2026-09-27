/** @file lv_port_disp.h
 * @brief LVGL v8 double-buffer bridge; platform alone owns transfer completion.
 */
#ifndef UI_LV_PORT_DISP_H
#define UI_LV_PORT_DISP_H
#include "lvgl.h"
#include "../App/Platform/lcd_st7789.h"

/** @brief Coherent diagnostics for one complete LVGL invalid-area refresh. */
typedef struct {
    uint32_t last_bytes;
    uint32_t last_duration_ms;
    uint32_t fps_tenths;
    uint32_t completed;
    uint32_t failed;
} LvPortDispRefreshStatus;

/** @brief Initialize platform and LVGL display; called once inside UiTask guard. */
uint8_t lv_port_disp_init(void);
/** @brief Service hardware deadlines from normal loops and bounded wait_cb. */
void lv_port_disp_service(uint32_t now_ms);
/** @brief Copy health without accessing LVGL in an ISR. */
uint8_t lv_port_disp_healthy(void);
/** @brief Return last platform diagnostic copy. */
void lv_port_disp_status(LcdSt7789Status *status);
/** @brief Copy a coherent complete-refresh diagnostic snapshot. */
void lv_port_disp_refresh_status(LvPortDispRefreshStatus *status);
/** @brief Attempt one idle recovery per boot, only after ownership is released. */
uint8_t lv_port_disp_try_recover(uint32_t now_ms);
/** @brief UiTask hook: service ADC/health/STOP then yield 1ms; never mutate LVGL. */
void DebugUiWaitService(void);
/** @brief Task-owned monotonic ms shared with HAL ADC samples and LCD deadlines. */
uint32_t DebugUiNowMs(void);
#endif
