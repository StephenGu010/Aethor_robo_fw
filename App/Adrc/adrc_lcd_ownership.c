/** @file adrc_lcd_ownership.c
 * @brief Rejects unsafe LCD/ADRC handoffs and waits for measured disable before release.
 */
#include "adrc_lcd_ownership.h"

#include <stddef.h>
#include <string.h>

/** @brief Rejects stale, future, or missing feedback before checking its state flags. */
static uint8_t feedback_is_current(const AdrcLcdOwnershipEvidence *evidence)
{
    return (uint8_t)(evidence->feedback_fresh != 0U && evidence->feedback_us != 0ULL &&
        evidence->now_us >= evidence->feedback_us &&
        evidence->now_us - evidence->feedback_us <= ADRC_LCD_OWNER_FEEDBACK_MAX_AGE_US);
}

/** @brief Initializes the power-up owner to LCD with no pending handoff. */
void adrc_lcd_ownership_init(AdrcLcdOwnership *ownership)
{
    if (ownership == NULL) { return; }
    memset(ownership, 0, sizeof(*ownership));
    ownership->state = ADRC_LCD_OWNER_LCD;
}

/** @brief Queues an ADRC acquire request without performing a transport or motor action. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_submit_acquire(
    AdrcLcdOwnership *ownership, uint8_t axis_index, uint32_t request_id)
{
    if (ownership == NULL || axis_index >= 7U || request_id == 0U)
    { return ADRC_LCD_OWNERSHIP_BAD_ARGUMENT; }
    if (ownership->state != ADRC_LCD_OWNER_LCD) { return ADRC_LCD_OWNERSHIP_BUSY; }
    if (request_id <= ownership->highest_request_id) { return ADRC_LCD_OWNERSHIP_STALE_ID; }
    ownership->axis_index = axis_index;
    ownership->active_request_id = request_id;
    ownership->highest_request_id = request_id;
    ownership->state = ADRC_LCD_OWNER_ACQUIRING;
    return ADRC_LCD_OWNERSHIP_OK;
}

/** @brief Queues a release that must be followed by a real disable receipt and feedback. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_submit_release(
    AdrcLcdOwnership *ownership, uint32_t request_id, uint64_t now_us)
{
    if (ownership == NULL || request_id == 0U || now_us == 0ULL)
    { return ADRC_LCD_OWNERSHIP_BAD_ARGUMENT; }
    if (request_id <= ownership->highest_request_id) { return ADRC_LCD_OWNERSHIP_STALE_ID; }
    if (ownership->state != ADRC_LCD_OWNER_ADRC) { return ADRC_LCD_OWNERSHIP_BUSY; }
    ownership->active_request_id = request_id;
    ownership->highest_request_id = request_id;
    ownership->release_started_us = now_us;
    ownership->disable_transmitted_us = 0ULL;
    ownership->state = ADRC_LCD_OWNER_RELEASING;
    return ADRC_LCD_OWNERSHIP_OK;
}

/** @brief Records a matched successful disable transmission without claiming drive execution. */
void adrc_lcd_ownership_report_disable_transmit(
    AdrcLcdOwnership *ownership, uint64_t transmitted_us, uint8_t succeeded)
{
    if (ownership == NULL || ownership->state != ADRC_LCD_OWNER_RELEASING ||
        succeeded == 0U || transmitted_us < ownership->release_started_us)
    { return; }
    ownership->disable_transmitted_us = transmitted_us;
}

/** @brief Advances ownership only from fresh measured evidence in the control task. */
AdrcLcdOwnershipStatus adrc_lcd_ownership_service(
    AdrcLcdOwnership *ownership, const AdrcLcdOwnershipEvidence *evidence)
{
    if (ownership == NULL || evidence == NULL) { return ADRC_LCD_OWNERSHIP_BAD_ARGUMENT; }
    if (ownership->state == ADRC_LCD_OWNER_ACQUIRING)
    {
        if (evidence->lcd_idle == 0U || evidence->can_idle == 0U ||
            feedback_is_current(evidence) == 0U || evidence->disabled == 0U ||
            evidence->no_fault == 0U || evidence->stationary == 0U ||
            evidence->mit_discovered == 0U)
        {
            ownership->state = ADRC_LCD_OWNER_LCD;
            return ADRC_LCD_OWNERSHIP_UNSAFE;
        }
        ownership->state = ADRC_LCD_OWNER_ADRC;
        return ADRC_LCD_OWNERSHIP_TRANSFERRED;
    }
    if (ownership->state == ADRC_LCD_OWNER_RELEASING)
    {
        if (ownership->disable_transmitted_us != 0ULL &&
            feedback_is_current(evidence) != 0U &&
            evidence->feedback_us > ownership->disable_transmitted_us &&
            evidence->disabled != 0U && evidence->no_fault != 0U &&
            evidence->stationary != 0U && evidence->can_idle != 0U)
        {
            ownership->state = ADRC_LCD_OWNER_LCD;
            return ADRC_LCD_OWNERSHIP_RELEASED;
        }
        if (evidence->now_us >= ownership->release_started_us &&
            evidence->now_us - ownership->release_started_us >
                ADRC_LCD_OWNER_RELEASE_TIMEOUT_US)
        { return ADRC_LCD_OWNERSHIP_TIMEOUT; }
    }
    return ADRC_LCD_OWNERSHIP_WAITING;
}
