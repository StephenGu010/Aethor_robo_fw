/**
 * @file build_info.c
 * @brief Provides immutable firmware identity and build metadata.
 */

#include "build_info.h"

#include "board_config.h"

#ifndef AETHOR_GIT_DESCRIPTION
#define AETHOR_GIT_DESCRIPTION "not-injected"
#endif

static const BuildInfo build_information = {
    "Aethor_robo",
    "0.1.0-phase0",
    AETHOR_GIT_DESCRIPTION,
    BOARD_CONTROLLER_ID,
    BOARD_ARM_ID,
    "aethor-arm-ascii-v1"
};

/**
 * @brief Returns immutable firmware identity and build metadata.
 * @return Pointer to process-lifetime static build information.
 */
const BuildInfo *build_info_get(void)
{
    return &build_information;
}
