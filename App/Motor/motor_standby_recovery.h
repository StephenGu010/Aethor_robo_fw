/** @file motor_standby_recovery.h
 * @brief Scheduling state for bounded read-only ID7 discovery after late or lost motor power.
 */
#ifndef APP_MOTOR_STANDBY_RECOVERY_H
#define APP_MOTOR_STANDBY_RECOVERY_H

#define MOTOR_STANDBY_RECOVERY_COOLDOWN_US (2000000ULL)
#define MOTOR_STANDBY_RECOVERY_STALE_US (3000000ULL)
#define MOTOR_STANDBY_RECOVERY_BUDGET_US (2000000ULL)

/** @brief Own only a background discovery round and monotonic scheduling evidence; no control authority. */
typedef struct
{
    uint64_t last_service_us;
    uint64_t next_attempt_us;
    uint64_t started_us;
    uint64_t healthy_since_us;
    uint8_t owned;
    uint8_t lost_reference;
} MotorStandbyRecovery;

#endif

