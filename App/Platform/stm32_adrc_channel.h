/** @file stm32_adrc_channel.h
 * @brief Single-owner dedicated FDCAN buffer for expiring ADRC commands and transmit receipts.
 */
#ifndef APP_PLATFORM_STM32_ADRC_CHANNEL_H
#define APP_PLATFORM_STM32_ADRC_CHANNEL_H
#include "can_frame.h"

/** @brief Identifies the one outstanding operation; torque receipts alone feed the observer. */
typedef enum { ADRC_CAN_ENABLE = 0, ADRC_CAN_TORQUE, ADRC_CAN_DISABLE,
    ADRC_CAN_PROBE, ADRC_CAN_WATCHDOG_PARAMETER } AdrcCanKind;
/** @brief Reports bus transmission, never mere queue acceptance or motor execution. */
typedef struct
{
    float decoded_torque_nm;
    AdrcCanKind kind;
    uint8_t available;
    uint8_t transmitted;
} AdrcCanReceipt;

/** @brief Clears software bookkeeping before FDCAN starts; does not issue an actuator command. */
void stm32_adrc_channel_init(void);
/** @brief Poll once at the next control tick; an unfinished request is cancelled and reported failed. */
AdrcCanReceipt stm32_adrc_channel_collect(void);
/** @brief Submits at most one frame in dedicated buffer 0; never appends to the legacy FIFO. */
uint8_t stm32_adrc_channel_submit(const CanFrame *frame, AdrcCanKind kind, float decoded_torque_nm);
/** @brief Preloads only motor-7 DISABLE and arms one nonrenewable 1..20 ms TIM23 deadline. */
uint8_t stm32_adrc_guard_arm(const CanFrame *disable_frame, uint32_t now_ms, uint32_t duration_ms);
/** @brief Arms a 12 ms trial lease with an immutable 2000 ms cap; now must equal HAL_GetTick(). */
uint8_t stm32_adrc_guard_arm_trial(const CanFrame *disable_frame, uint32_t now_ms);
/** @brief Trusted bridge renews only after fresh feedback and matched actual TX; never revives expiry. */
uint8_t stm32_adrc_guard_renew_trial(uint32_t now_ms);
/** @brief Called by TIM23 only; requests DISABLE without RTOS, task progress or blocking HAL. */
void stm32_adrc_guard_tick_isr(uint32_t now_ms);
/** @brief Reports 0 idle, 1 armed, 2 requested/unconfirmed, 3 actual TX observed by later tick. */
uint8_t stm32_adrc_guard_state(void);
/** @brief Copies timer request times coherently; these are not motor execution timestamps. */
uint8_t stm32_adrc_guard_observe(uint32_t *deadline_ms, uint32_t *fired_ms);
/** @brief Trusted local caller must first verify fresh disabled feedback; never exposed as USB assertion. */
uint8_t stm32_adrc_guard_ack_disabled(void);
#endif
