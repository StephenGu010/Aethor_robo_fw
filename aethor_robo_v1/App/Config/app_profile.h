/**
 * @file app_profile.h
 * @brief Selects the bounded bench or fully commissioned production behavior.
 */

#ifndef APP_CONFIG_APP_PROFILE_H
#define APP_CONFIG_APP_PROFILE_H

#include <stdint.h>

#define AETHOR_PROFILE_USB_BENCH_RELATIVE (1U)
#define AETHOR_PROFILE_ARM_PRODUCTION (2U)

/**
 * @brief Identifies the only supported application behavior profiles.
 */
typedef enum
{
    AETHOR_APPLICATION_PROFILE_USB_BENCH_RELATIVE = AETHOR_PROFILE_USB_BENCH_RELATIVE,
    AETHOR_APPLICATION_PROFILE_ARM_PRODUCTION = AETHOR_PROFILE_ARM_PRODUCTION
} AethorApplicationProfile;

#ifndef AETHOR_ACTIVE_PROFILE
#define AETHOR_ACTIVE_PROFILE AETHOR_PROFILE_USB_BENCH_RELATIVE
#endif

#define AETHOR_BENCH_MAX_RELATIVE_DEGREES (3.0F)
#define AETHOR_BENCH_MAX_SPEED_DEGREES_S (3.0F)
#define AETHOR_PRODUCTION_REQUIRES_COMPLETE_CONFIGURATION (1U)

#if (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_USB_BENCH_RELATIVE) && \
    (AETHOR_ACTIVE_PROFILE != AETHOR_PROFILE_ARM_PRODUCTION)
#error "AETHOR_ACTIVE_PROFILE selects an unsupported application profile."
#endif

#endif
