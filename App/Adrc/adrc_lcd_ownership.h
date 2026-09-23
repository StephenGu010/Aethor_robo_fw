/** @file adrc_lcd_ownership.h
 * @brief HAL-independent exclusive ownership contract for LCD-MIT and single-axis ADRC.
 */
#ifndef APP_ADRC_LCD_OWNERSHIP_H
#define APP_ADRC_LCD_OWNERSHIP_H

#include <stdint.h>

#define ADRC_LCD_OWNER_RELEASE_TIMEOUT_US (500000ULL)
#define ADRC_LCD_OWNER_ACQUIRE_TIMEOUT_US (100000ULL)
#define ADRC_LCD_OWNER_FEEDBACK_MAX_AGE_US (8000ULL)

/** @brief Names the only control path allowed to produce ordinary motor frames. */
typedef enum
{
    ADRC_LCD_OWNER_LCD = 0,
    ADRC_LCD_OWNER_ACQUIRING,
    ADRC_LCD_OWNER_ADRC,
    ADRC_LCD_OWNER_RELEASING
} AdrcLcdOwnerState;

/** @brief Reports request admission and control-task transition outcomes. */
typedef enum
{
    ADRC_LCD_OWNERSHIP_OK = 0,
    ADRC_LCD_OWNERSHIP_BAD_ARGUMENT,
    ADRC_LCD_OWNERSHIP_BUSY,
    ADRC_LCD_OWNERSHIP_STALE_ID,
    ADRC_LCD_OWNERSHIP_UNSAFE,
    ADRC_LCD_OWNERSHIP_WAITING,
    ADRC_LCD_OWNERSHIP_TRANSFERRED,
    ADRC_LCD_OWNERSHIP_RELEASED,
    ADRC_LCD_OWNERSHIP_TIMEOUT
} AdrcLcdOwnershipStatus;

/** @brief Copies all measured handoff gates into one control-task decision. */
typedef struct
{
    uint64_t now_us;
    uint64_t feedback_us;
    uint64_t probe_submitted_us;
    uint8_t lcd_idle;
    uint8_t can_idle;
    uint8_t probe_transmitted;
    uint8_t probe_failed;
    uint8_t feedback_fresh;
    uint8_t disabled;
    uint8_t no_fault;
    uint8_t stationary;
    uint8_t mit_discovered;
} AdrcLcdOwnershipEvidence;

/** @brief Retains owner state and monotonic IDs without a USB-controlled qualification flag. */
typedef struct
{
    AdrcLcdOwnerState state;
    uint64_t acquire_started_us;
    uint64_t release_started_us;
    uint64_t disable_transmitted_us;
    uint32_t active_request_id;
    uint32_t highest_request_id;
    uint8_t axis_index;
} AdrcLcdOwnership;

/** @brief Initializes the power-up owner to LCD with no pending handoff. */
void adrc_lcd_ownership_init(AdrcLcdOwnership *ownership);

/** @brief Queues an ADRC acquire request without performing a transport or motor action. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_submit_acquire(
    AdrcLcdOwnership *ownership, uint8_t axis_index, uint32_t request_id);

/** @brief Queues a release that must be followed by a real disable receipt and feedback. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_submit_release(
    AdrcLcdOwnership *ownership, uint32_t request_id, uint64_t now_us);

/** @brief Records a matched successful disable transmission without claiming drive execution. */
void adrc_lcd_ownership_report_disable_transmit(
    AdrcLcdOwnership *ownership, uint64_t transmitted_us, uint8_t succeeded);

/** @brief Advances ownership only from fresh measured evidence in the control task. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_service(
    AdrcLcdOwnership *ownership, const AdrcLcdOwnershipEvidence *evidence);

#endif
