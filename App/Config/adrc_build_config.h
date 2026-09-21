/** @file adrc_build_config.h
 * @brief Explicit opt-in for the isolated ADRC firmware target; legacy targets keep their behavior.
 */
#ifndef APP_CONFIG_ADRC_BUILD_CONFIG_H
#define APP_CONFIG_ADRC_BUILD_CONFIG_H
#ifndef AETHOR_ADRC_BENCH
#define AETHOR_ADRC_BENCH 0
#endif
#if AETHOR_ADRC_BENCH != 0 && AETHOR_ADRC_BENCH != 1
#error "AETHOR_ADRC_BENCH must be zero or one."
#endif
#endif
