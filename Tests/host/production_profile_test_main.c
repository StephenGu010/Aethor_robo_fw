/**
 * @file production_profile_test_main.c
 * @brief Verifies production builds reject uncommissioned physical parameters.
 */

#include <assert.h>
#include <stdio.h>

#include "arm_config.h"
#include "arm_controller.h"
#include "diagnostics.h"

/**
 * @brief Runs the production-profile fail-closed startup contract.
 * @return Zero when the unverified configuration is latched as a fault.
 */
int main(void)
{
    ArmController controller;
    ArmSnapshot snapshot;
    Diagnostics diagnostics;

    diagnostics_init(&diagnostics);
    arm_controller_init(&controller,
                        arm_config_get_production(),
                        &diagnostics,
                        1000U);
    arm_controller_step(&controller, 2000U);
    arm_controller_step(&controller, 3000U);

    assert(arm_controller_get_snapshot(&controller, &snapshot));
    assert(snapshot.state == ARM_STATE_FAULT);
    assert(snapshot.fault == ARM_FAULT_CONFIG_INCOMPLETE);
    assert(snapshot.enabled == 0U);
    assert(snapshot.moving == 0U);
    puts("PRODUCTION_PROFILE_TEST_PASSED");
    return 0;
}
