/** @file aethor_task_stacks.h
 * @brief Static task capacities for static-load receive and integrated LCD paths.
 *
 * Static-load opt-ins need extra CAN receive capacity. Integrated LCD and static
 * builds need extra parser and diagnostics capacity for their known call chains.
 * Other builds retain the CubeMX stack depths;
 * actual stack high-water measurements remain a separate hardware requirement.
 */
#ifndef AETHOR_TASK_STACKS_H
#define AETHOR_TASK_STACKS_H

#include "debug_ui_config.h"

#if AETHOR_DEBUG_UI_ALLOW_MOTION && (AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_POS_LOAD_TEST)
#define AETHOR_CAN_RX_TASK_STACK_WORDS (640U)
#else
#define AETHOR_CAN_RX_TASK_STACK_WORDS (384U)
#endif

#if AETHOR_ADRC_LCD_INTEGRATED || AETHOR_DEBUG_UI_ALLOW_SINGLE_STATIC_LOAD || AETHOR_DEBUG_UI_ALLOW_STATIC_LOAD
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (2048U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (512U)
#else
#define AETHOR_PROTOCOL_TASK_STACK_WORDS (1280U)
#define AETHOR_DIAGNOSTICS_TASK_STACK_WORDS (384U)
#endif

#endif
