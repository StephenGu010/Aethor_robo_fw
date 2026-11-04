/** @file debug_ui_static_trace.h
 * @brief Version-one passive static startup trace ABI. All floats are native rad, rad/s, N m or native gains.
 * Integers and IEEE754 binary32 values are little endian. SOF is a 16-bit hardware bus ordering counter,
 * never motor sample/execution time. Target acceptance remains unknown. Explicit padding is always zero.
 */
#ifndef DEBUG_UI_STATIC_TRACE_H
#define DEBUG_UI_STATIC_TRACE_H
#include <stddef.h>
#include <stdint.h>
#include "debug_ui_single_static_tx.h"
#define DEBUG_UI_STATIC_TRACE_CAPACITY 128U
#define DEBUG_UI_STATIC_TRACE_HEADER_INDEX 0xFFFFFFFFUL
/** @brief Stable event values distinguish application dequeue, HAL submission and proven actual TX. */
typedef enum {
    STATIC_TRACE_BEGIN = 1, STATIC_TRACE_DEQUEUED = 2, STATIC_TRACE_HAL_SUBMIT = 3,
    STATIC_TRACE_ACTUAL_TX = 4, STATIC_TRACE_TX_UNKNOWN = 5, STATIC_TRACE_RAW_RX = 6,
    STATIC_TRACE_ACCEPTED_RX = 7, STATIC_TRACE_FIRST_FAULT = 8, STATIC_TRACE_TERMINAL = 9,
    STATIC_TRACE_PHASE = 10, STATIC_TRACE_CANCELLED = 11, STATIC_TRACE_QUEUED = 12
} DebugUiStaticTraceEvent;
/** @brief Exactly 96 bytes; fixed offsets are checked for both production compiler and host builds. */
typedef struct {
    uint64_t timestamp_us;             /**< 0: application lower-bound clock, not motor sample time. */
    uint64_t observed_us;              /**< 8: application observation clock. */
    uint32_t serial, feedback_generation, epoch, request_id, sequence, fault; /**< 16..39. */
    float target_rad, position_rad, velocity_rad_s, torque_nm, kp, kd, torque_ff_nm; /**< 40..67. */
    uint16_t can_id, sof;              /**< 68..71. */
    uint8_t event, phase, motor_id, driver_state, length, sof_valid; /**< 72..77. */
    uint8_t raw[8];                    /**< 78..85: exact original data bytes. */
    uint8_t reserved[10];
} DebugUiStaticTraceRecord;
/** @brief Exactly 80 bytes per axis; frozen discovered ranges and profile bounds are never synthesized. */
typedef struct {
    uint32_t reference, evidence, feedback_generation, esc_id, master_id, mode;
    float target_rad, position_ratio, kp, kd, torque_ff_nm, range_position_rad,
        range_velocity_rad_s, range_torque_nm, minimum_rad, maximum_rad, maximum_speed_rad_s;
    uint32_t reserved[3];
} DebugUiStaticTraceAxis;
/** @brief Exactly 832 bytes: 80-byte header, seven 80-byte axes, permanent first fault and terminal. */
typedef struct {
    uint32_t schema, generation, count, total, overflow, gaps, active, frozen,
        first_fault_valid, terminal_valid, record_size, header_size;
    uint64_t started_us;
    uint32_t epoch, request_id, motor_mask, target_acceptance, sof_clock, cleanup_outcome;
    /* cleanup_outcome at offset 76: 0 unverified/not applicable; 1 newer safe disabled observed;
     * 2 trace-only timeout unknown. Outcome 1 never proves causality or actual emergency TX. */
    DebugUiStaticTraceAxis axes[7];
    DebugUiStaticTraceRecord first_fault, terminal;
} DebugUiStaticTraceHeader;
typedef char StaticTraceRecordSizeCheck[(sizeof(DebugUiStaticTraceRecord) == 96U) ? 1 : -1];
typedef char StaticTraceRecordOffsetCheck[(offsetof(DebugUiStaticTraceRecord, raw) == 78U) ? 1 : -1];
typedef char StaticTraceAxisSizeCheck[(sizeof(DebugUiStaticTraceAxis) == 80U) ? 1 : -1];
typedef char StaticTraceHeaderSizeCheck[(sizeof(DebugUiStaticTraceHeader) == 832U) ? 1 : -1];
typedef char StaticTraceFloatSizeCheck[(sizeof(float) == 4U) ? 1 : -1];
typedef char StaticTraceHeaderAxesCheck[(offsetof(DebugUiStaticTraceHeader, axes) == 80U) ? 1 : -1];
typedef char StaticTraceHeaderFaultCheck[(offsetof(DebugUiStaticTraceHeader, first_fault) == 640U) ? 1 : -1];
typedef char StaticTraceHeaderTerminalCheck[(offsetof(DebugUiStaticTraceHeader, terminal) == 736U) ? 1 : -1];
#endif
