/** @file adrc_lcd_ownership_test_main.c
 * @brief Offline ownership tests; synthetic evidence never qualifies hardware.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "adrc_lcd_ownership.h"

/** @brief Builds one fully safe synthetic handoff snapshot at a known time. */
static AdrcLcdOwnershipEvidence safe_evidence(uint64_t now_us)
{
    AdrcLcdOwnershipEvidence evidence;
    memset(&evidence, 0, sizeof(evidence));
    evidence.now_us = now_us;
    evidence.feedback_us = now_us - 1000ULL;
    evidence.lcd_idle = 1U;
    evidence.can_idle = 1U;
    evidence.feedback_fresh = 1U;
    evidence.disabled = 1U;
    evidence.no_fault = 1U;
    evidence.stationary = 1U;
    evidence.mit_discovered = 1U;
    return evidence;
}

/** @brief Requires boot to remain LCD-owned until a control-task service accepts evidence. */
static void test_acquire_requires_every_gate(void)
{
    AdrcLcdOwnership owner;
    AdrcLcdOwnershipEvidence evidence;
    uint8_t gate_index;
    adrc_lcd_ownership_init(&owner);
    assert(owner.state == ADRC_LCD_OWNER_LCD);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_OK);
    assert(owner.state == ADRC_LCD_OWNER_ACQUIRING);
    evidence = safe_evidence(10000ULL);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_TRANSFERRED);
    assert(owner.state == ADRC_LCD_OWNER_ADRC && owner.axis_index == 6U);

    for (gate_index = 0U; gate_index < 7U; ++gate_index)
    {
        uint8_t *gates[7];
        adrc_lcd_ownership_init(&owner);
        assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_OK);
        evidence = safe_evidence(10000ULL);
        gates[0] = &evidence.lcd_idle;
        gates[1] = &evidence.can_idle;
        gates[2] = &evidence.feedback_fresh;
        gates[3] = &evidence.disabled;
        gates[4] = &evidence.no_fault;
        gates[5] = &evidence.stationary;
        gates[6] = &evidence.mit_discovered;
        *gates[gate_index] = 0U;
        assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_UNSAFE);
        assert(owner.state == ADRC_LCD_OWNER_LCD);
    }
}

/** @brief Rejects invalid axes, stale identifiers, future feedback and reentrant handoff. */
static void test_acquire_rejects_replay_and_bad_time(void)
{
    AdrcLcdOwnership owner;
    AdrcLcdOwnershipEvidence evidence;
    adrc_lcd_ownership_init(&owner);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 7U, 1U) == ADRC_LCD_OWNERSHIP_BAD_ARGUMENT);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 0U) == ADRC_LCD_OWNERSHIP_BAD_ARGUMENT);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_OK);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 2U) == ADRC_LCD_OWNERSHIP_BUSY);
    evidence = safe_evidence(10000ULL);
    evidence.feedback_us = 10001ULL;
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_UNSAFE);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_STALE_ID);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 3U) == ADRC_LCD_OWNERSHIP_OK);
}

/** @brief Never returns control before a successful disable receipt and newer safe feedback. */
static void test_release_waits_for_physical_disable(void)
{
    AdrcLcdOwnership owner;
    AdrcLcdOwnershipEvidence evidence = safe_evidence(10000ULL);
    adrc_lcd_ownership_init(&owner);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_OK);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_TRANSFERRED);
    assert(adrc_lcd_ownership_submit_release(&owner, 2U, 12000ULL) == ADRC_LCD_OWNERSHIP_OK);
    assert(owner.state == ADRC_LCD_OWNER_RELEASING);
    evidence = safe_evidence(14000ULL);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_WAITING);
    adrc_lcd_ownership_report_disable_transmit(&owner, 15000ULL, 0U);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_WAITING);
    adrc_lcd_ownership_report_disable_transmit(&owner, 15000ULL, 1U);
    evidence = safe_evidence(16000ULL);
    evidence.feedback_us = 15000ULL;
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_WAITING);
    evidence = safe_evidence(17000ULL);
    evidence.disabled = 0U;
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_WAITING);
    evidence = safe_evidence(18000ULL);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_RELEASED);
    assert(owner.state == ADRC_LCD_OWNER_LCD);
    assert(adrc_lcd_ownership_submit_release(&owner, 2U, 19000ULL) == ADRC_LCD_OWNERSHIP_STALE_ID);
}

/** @brief A timed-out release remains locked until later measured disable. */
static void test_release_timeout_stays_locked(void)
{
    AdrcLcdOwnership owner;
    AdrcLcdOwnershipEvidence evidence = safe_evidence(10000ULL);
    adrc_lcd_ownership_init(&owner);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 1U) == ADRC_LCD_OWNERSHIP_OK);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_TRANSFERRED);
    assert(adrc_lcd_ownership_submit_release(&owner, 2U, 12000ULL) == ADRC_LCD_OWNERSHIP_OK);
    evidence = safe_evidence(512001ULL);
    assert(adrc_lcd_ownership_service(&owner, &evidence) == ADRC_LCD_OWNERSHIP_TIMEOUT);
    assert(owner.state == ADRC_LCD_OWNER_RELEASING);
    assert(adrc_lcd_ownership_submit_acquire(&owner, 6U, 3U) == ADRC_LCD_OWNERSHIP_BUSY);
}

/** @brief Runs the deterministic ownership contract on a host without CAN hardware. */
int main(void)
{
    test_acquire_requires_every_gate();
    test_acquire_rejects_replay_and_bad_time();
    test_release_waits_for_physical_disable();
    test_release_timeout_stays_locked();
    puts("ADRC_LCD_OWNERSHIP_TESTS_PASSED");
    return 0;
}
