/** @file task.h @brief Host-only stack watermark declaration. */
#include <stdint.h>
/** @brief Return a synthetic watermark, never hardware stack evidence. */
uint32_t uxTaskGetStackHighWaterMark(void *task);

