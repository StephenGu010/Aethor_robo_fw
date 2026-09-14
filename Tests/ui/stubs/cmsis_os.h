/** @file cmsis_os.h @brief Host-only RTOS timing declarations for the UI review. */
#ifndef REVIEW_CMSIS_OS_H
#define REVIEW_CMSIS_OS_H
#include <stdint.h>
/** @brief Advance simulated scheduler time only. */
int osDelay(uint32_t milliseconds);
#endif

