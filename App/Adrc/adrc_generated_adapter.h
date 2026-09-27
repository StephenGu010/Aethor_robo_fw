/** @file adrc_generated_adapter.h
 * @brief Checked single-instance boundary around the real Simulink-generated controller.
 */
#ifndef APP_ADRC_GENERATED_ADAPTER_H
#define APP_ADRC_GENERATED_ADAPTER_H
#include "adrc_experiment.h"
/** @brief Runs generated code only for finite valid inputs; owned solely by ArmControlTask. */
int adrc_generated_controller_step(void *context, const AdrcControllerInput *input,
    AdrcControllerOutput *output);
#endif
