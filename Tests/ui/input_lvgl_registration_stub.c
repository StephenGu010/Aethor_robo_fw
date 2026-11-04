/** @file input_lvgl_registration_stub.c
 * @brief Unused LVGL registration endpoints for the ADC/model-only host executable.
 * Production lv_port_indev_event and its landscape transform are linked unchanged.
 */
#include "lv_port_indev.h"
/** @brief Registration is outside this input trace; leave the value inert. */
void lv_indev_drv_init(lv_indev_drv_t *driver) { (void)driver; }
/** @brief No device is registered by this host trace. */
lv_indev_t *lv_indev_drv_register(lv_indev_drv_t *driver) { (void)driver; return NULL; }
/** @brief No LVGL focus group participates in production model dispatch. */
void lv_indev_set_group(lv_indev_t *device, lv_group_t *group) { (void)device; (void)group; }
