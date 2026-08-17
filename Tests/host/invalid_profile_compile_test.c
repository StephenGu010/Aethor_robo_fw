/**
 * @file invalid_profile_compile_test.c
 * @brief Provides a minimal translation unit for application-profile rejection tests.
 */

#include "app_profile.h"

/**
 * @brief Returns a fixed value when the selected profile is accepted by the header.
 * @return Always returns zero.
 */
int invalid_profile_compile_probe(void)
{
    return 0;
}
