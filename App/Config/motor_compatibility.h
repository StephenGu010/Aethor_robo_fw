/** @file motor_compatibility.h
 * @brief Explicit per-motor special-command convention for the commissioned bench.
 * Motor7: DM SDK __control_cmd sends FC/FD/FE/FB to SlaveID. USB2CAN confirmed
 * base-ID disable; the main controller received no reply to POS-offset disable.
 * POS data still uses 0x100 + ESC. LCD builds can explicitly extend this preset
 * to same-model IDs confirmed by the user; this is not per-motor bench evidence.
 */
#ifndef APP_CONFIG_MOTOR_COMPATIBILITY_H
#define APP_CONFIG_MOTOR_COMPATIBILITY_H
#include "app_profile.h"

/* Motor7, 2026-09-14: live Gr readback 19.2032; p_m and packed position
 * follow the photographed black rotor. Position is rotor-side, velocity is
 * output-side on this controller. Only LCD bench commands opt into conversion.
 * Recommission this constant when replacing the motor/driver or changing Gr. */
#define AETHOR_S3519_MOTOR7_POSITION_RATIO (19.2032F)

#ifndef AETHOR_S3519_SAME_MODEL_MASK
#if AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
#define AETHOR_S3519_SAME_MODEL_MASK (0x40U)
#else
#define AETHOR_S3519_SAME_MODEL_MASK (0U)
#endif
#endif
#if (AETHOR_S3519_SAME_MODEL_MASK & ~0x7FU)
#error "Same-model S3519 preset supports ESC IDs 1..7 only."
#endif
#if AETHOR_S3519_SAME_MODEL_MASK && (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE)
#error "Same-model bench compatibility must not override production calibration."
#endif

#ifndef AETHOR_S3519_BASE_SPECIAL_COMMAND_MASK
#if AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
#define AETHOR_S3519_BASE_SPECIAL_COMMAND_MASK (AETHOR_S3519_SAME_MODEL_MASK)
#else
#define AETHOR_S3519_BASE_SPECIAL_COMMAND_MASK (0U)
#endif
#endif

#if (AETHOR_S3519_BASE_SPECIAL_COMMAND_MASK & ~0x7FU)
#error "Special-command compatibility mask supports commissioned ESC IDs 1..7 only."
#endif
#endif
