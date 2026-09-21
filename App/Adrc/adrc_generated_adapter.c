/** @file adrc_generated_adapter.c
 * @brief Adapts the generated adrc_controller model to the pure supervisor callback contract.
 * This file is handwritten integration code; adrc_controller.c must come from validated Simulink output.
 */
#include "adrc_generated_adapter.h"
#include "adrc_controller.h"
#include <float.h>
#include <stddef.h>
#include <string.h>

/** @brief Finite check compatible with the project ARMCC C99 runtime. */
static int adapter_finite(float value)
{
    return value == value && value >= -FLT_MAX && value <= FLT_MAX;
}

/** @brief Resets on each experiment and returns raw torque; supervision performs actuator limits. */
int adrc_generated_controller_step(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output)
{
    (void)context;
    if (output != NULL) { memset(output, 0, sizeof(*output)); }
    if (input == NULL || output == NULL ||
        (input->mode != ADRC_MODE_PI && input->mode != ADRC_MODE_LADRC) ||
        input->reset > 1U || input->new_sample > 1U ||
        !adapter_finite(input->reference_rad_s) || !adapter_finite(input->velocity_rad_s) ||
        !adapter_finite(input->prev_sent_nm) || !adapter_finite(input->b0) ||
        !adapter_finite(input->wc_rad_s) || !adapter_finite(input->wo_rad_s) ||
        input->b0 <= 0.0F || input->wc_rad_s <= 0.0F ||
        input->wo_rad_s < input->wc_rad_s || input->wo_rad_s > 250.0F)
    { return 0; }
    if (input->reset) { adrc_controller_initialize(); }
    adrc_controller_U.mode = (uint8_T)input->mode;
    adrc_controller_U.reference_rad_s = input->reference_rad_s;
    adrc_controller_U.velocity_rad_s = input->velocity_rad_s;
    adrc_controller_U.prev_sent_nm = input->prev_sent_nm;
    adrc_controller_U.b0 = input->b0;
    adrc_controller_U.wc_rad_s = input->wc_rad_s;
    adrc_controller_U.wo_rad_s = input->wo_rad_s;
    adrc_controller_U.reset = input->reset;
    adrc_controller_U.new_sample = input->new_sample;
    adrc_controller_step();
    if (!adapter_finite(adrc_controller_Y.torque_raw_nm) ||
        !adapter_finite(adrc_controller_Y.z1) || !adapter_finite(adrc_controller_Y.z2))
    { return 0; }
    output->torque_raw_nm = adrc_controller_Y.torque_raw_nm;
    output->z1 = adrc_controller_Y.z1;
    output->z2 = adrc_controller_Y.z2;
    return 1;
}
