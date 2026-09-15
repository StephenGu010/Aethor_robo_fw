/** @file debug_ui_config.h
 * @brief Compile-time local debug gates; no motor is commissioned by default.
 */
#ifndef APP_CONFIG_DEBUG_UI_CONFIG_H
#define APP_CONFIG_DEBUG_UI_CONFIG_H

#include "app_profile.h"
#include "motor_compatibility.h"

#ifndef AETHOR_DEBUG_UI_ENABLE
#define AETHOR_DEBUG_UI_ENABLE (0U)
#endif
#ifndef AETHOR_DEBUG_UI_ALLOW_MOTION
#define AETHOR_DEBUG_UI_ALLOW_MOTION (0U)
#endif
#ifndef AETHOR_DEBUG_UI_ALLOW_MIT
#define AETHOR_DEBUG_UI_ALLOW_MIT (0U)
#endif
/* Opt-in motor7 POS protocol envelope; requires discovery and does not authorize MIT. */
#ifndef AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE
#define AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE (0U)
#endif
/* Opt-in bounded motor7 MIT trial; independent of the POS protocol envelope. */
#ifndef AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE
#define AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE (0U)
#endif
/* Legacy opt-in flags retain ID7; an explicit mask extends the same discovery
 * and gain preset to the user-confirmed same-model motors, never absent IDs. */
#ifndef AETHOR_DEBUG_UI_S3519_PROFILE_MASK
#define AETHOR_DEBUG_UI_S3519_PROFILE_MASK (AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE ? 0x40U : 0U)
#endif
#if (AETHOR_DEBUG_UI_S3519_PROFILE_MASK & ~0x7FU) || \
    (AETHOR_DEBUG_UI_S3519_PROFILE_MASK & ~(AETHOR_S3519_SAME_MODEL_MASK | 0U))
#error "Auto profiles require a matching same-model preset for every selected ID."
#endif
#if AETHOR_DEBUG_UI_S3519_PROFILE_MASK && !AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE
#error "Auto profiles require the explicit discovered POS bench opt-in."
#endif
#if (AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE != 0) && (AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE != 1)
#error "Motor7 MIT profile flag must be zero or one."
#endif
#if AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE && (!AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE || !AETHOR_DEBUG_UI_ALLOW_MIT)
#error "Motor7 MIT trial requires the motor7 POS profile and MIT build gate."
#endif
#if (AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE != 0) && (AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE != 1)
#error "Motor7 POS profile flag must be zero or one."
#endif
#if AETHOR_DEBUG_UI_MOTOR7_POS_PROFILE && (!AETHOR_DEBUG_UI_ENABLE || !AETHOR_DEBUG_UI_ALLOW_MOTION || \
    (AETHOR_DEBUG_UI_ALLOW_MIT && !AETHOR_DEBUG_UI_MOTOR7_MIT_PROFILE) || (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE))
#error "Recorded motor7 profile requires the POS bench or explicit bounded MIT trial build."
#endif
#if ((AETHOR_DEBUG_UI_ENABLE != 0) && (AETHOR_DEBUG_UI_ENABLE != 1)) || \
    ((AETHOR_DEBUG_UI_ALLOW_MOTION != 0) && (AETHOR_DEBUG_UI_ALLOW_MOTION != 1)) || \
    ((AETHOR_DEBUG_UI_ALLOW_MIT != 0) && (AETHOR_DEBUG_UI_ALLOW_MIT != 1))
#error "Debug UI feature flags must be zero or one."
#endif
#if (AETHOR_DEBUG_UI_ALLOW_MOTION && !AETHOR_DEBUG_UI_ENABLE) || \
    (AETHOR_DEBUG_UI_ALLOW_MIT && !AETHOR_DEBUG_UI_ALLOW_MOTION)
#error "Debug UI motion requires UI; MIT requires motion."
#endif
#if AETHOR_DEBUG_UI_ALLOW_MOTION && \
    (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE)
#error "Production profiles must not enable local debug motion."
#endif

#define DEBUG_UI_MOTOR_COUNT (7U)
/* Select the connected bench motor without changing discovery or motion gates. */
#ifndef DEBUG_UI_INITIAL_MOTOR_ID
#if AETHOR_ACTIVE_PROFILE == AETHOR_PROFILE_USB_BENCH_RELATIVE
#define DEBUG_UI_INITIAL_MOTOR_ID (7U)
#else
#define DEBUG_UI_INITIAL_MOTOR_ID (1U)
#endif
#endif
#if DEBUG_UI_INITIAL_MOTOR_ID < 1 || DEBUG_UI_INITIAL_MOTOR_ID > DEBUG_UI_MOTOR_COUNT
#error "Initial LCD motor ID must be in 1..7."
#endif
#define DEBUG_UI_RESULT_CAPACITY (4U)
#define DEBUG_UI_REQUEST_MAX_AGE_US (250000ULL)
#define DEBUG_UI_HEALTH_TIMEOUT_US (150000ULL)
#define DEBUG_UI_HEALTH_REPORT_PERIOD_US (50000ULL)
#define DEBUG_UI_DISPLAY_WIDTH (280U)
#define DEBUG_UI_DISPLAY_HEIGHT (240U)
#define DEBUG_UI_DISPLAY_BUFFER_LINES (24U)

#endif
