/**
 * @file build_info.h
 * @brief Defines immutable firmware identity and build metadata.
 */

#ifndef APP_CONFIG_BUILD_INFO_H
#define APP_CONFIG_BUILD_INFO_H

#define BUILD_INFO_PRODUCT_NAME_CAPACITY (32U)
#define BUILD_INFO_VERSION_CAPACITY (24U)
#define BUILD_INFO_GIT_DESCRIPTION_CAPACITY (48U)
#define BUILD_INFO_CONTROLLER_ID_CAPACITY (32U)
#define BUILD_INFO_ARM_ID_CAPACITY (16U)
#define BUILD_INFO_PROTOCOL_VERSION_CAPACITY (32U)

/**
 * @brief Stores fixed-capacity identity strings for firmware diagnostics.
 */
typedef struct
{
    char product_name[BUILD_INFO_PRODUCT_NAME_CAPACITY];
    char firmware_version[BUILD_INFO_VERSION_CAPACITY];
    char git_description[BUILD_INFO_GIT_DESCRIPTION_CAPACITY];
    char controller_id[BUILD_INFO_CONTROLLER_ID_CAPACITY];
    char arm_id[BUILD_INFO_ARM_ID_CAPACITY];
    char protocol_version[BUILD_INFO_PROTOCOL_VERSION_CAPACITY];
} BuildInfo;

/**
 * @brief Returns immutable firmware identity and build metadata.
 * @return Pointer to process-lifetime static build information.
 */
const BuildInfo *build_info_get(void);

#endif
