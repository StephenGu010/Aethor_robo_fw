/*
 * File: adrc_controller.c
 *
 * Code generated for Simulink model 'adrc_controller'.
 *
 * Model version                  : 1.2
 * Simulink Coder version         : 26.1 (R2026a) 20-Nov-2025
 * C/C++ source code generated on : Mon Sep 21 16:10:28 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "adrc_controller.h"
#include <math.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_adrc_controller_T adrc_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_adrc_controller_T adrc_controller_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_adrc_controller_T adrc_controller_Y;

/* Real-time model */
static RT_MODEL_adrc_controller_T adrc_controller_M_;
RT_MODEL_adrc_controller_T *const adrc_controller_M = &adrc_controller_M_;

/* Model step function */
void adrc_controller_step(void)
{
  real32_T rtb_Innovation;
  real32_T rtb_IntegralSampleGate;
  real32_T rtb_Kc;
  real32_T rtb_PiError;

  /* Product: '<Root>/Kc' incorporates:
   *  Constant: '<Root>/One'
   *  Constant: '<Root>/Ts'
   *  Gain: '<Root>/NegativeControllerTime'
   *  Inport: '<Root>/wc_rad_s'
   *  Math: '<Root>/ControllerPole'
   *  Product: '<Root>/ControllerTime'
   *  Sum: '<Root>/OneMinusControllerPole'
   *
   * About '<Root>/ControllerPole':
   *  Operator: exp
   */
  rtb_Kc = (1.0F - expf(-(adrc_controller_U.wc_rad_s * 0.004F))) / 0.004F;

  /* Sum: '<Root>/PiError' incorporates:
   *  Inport: '<Root>/reference_rad_s'
   *  Inport: '<Root>/velocity_rad_s'
   */
  rtb_PiError = adrc_controller_U.reference_rad_s -
    adrc_controller_U.velocity_rad_s;

  /* Switch: '<Root>/IntegralReset' incorporates:
   *  Constant: '<Root>/Ts'
   *  Constant: '<Root>/Zero'
   *  Inport: '<Root>/new_sample'
   *  Inport: '<Root>/prev_sent_nm'
   *  Inport: '<Root>/reset'
   *  Product: '<Root>/AntiWindupAdvance'
   *  Sum: '<Root>/ActuatorTrackingError'
   *  Sum: '<Root>/IntegralAdvance'
   *  Switch: '<Root>/IntegralSampleGate'
   *  UnitDelay: '<Root>/IntegralMemory'
   *  UnitDelay: '<Root>/PreviousRaw'
   */
  if (adrc_controller_U.reset != 0) {
    adrc_controller_DW.IntegralMemory_DSTATE = 0.0F;
  } else {
    if (adrc_controller_U.new_sample != 0) {
      /* Switch: '<Root>/IntegralSampleGate' incorporates:
       *  Constant: '<Root>/Ts'
       *  Inport: '<Root>/b0'
       *  Product: '<Root>/IntegralErrorAdvance'
       *  Product: '<Root>/PiKi'
       */
      rtb_IntegralSampleGate = rtb_Kc * rtb_Kc / adrc_controller_U.b0 * 0.004F *
        rtb_PiError;
    } else {
      /* Switch: '<Root>/IntegralSampleGate' incorporates:
       *  Constant: '<Root>/Zero'
       */
      rtb_IntegralSampleGate = 0.0F;
    }

    adrc_controller_DW.IntegralMemory_DSTATE = 0.004F * rtb_Kc *
      (adrc_controller_U.prev_sent_nm - adrc_controller_DW.PreviousRaw_DSTATE) +
      (adrc_controller_DW.IntegralMemory_DSTATE + rtb_IntegralSampleGate);
  }

  /* End of Switch: '<Root>/IntegralReset' */

  /* Sum: '<Root>/Z1Prediction' incorporates:
   *  Constant: '<Root>/Ts'
   *  Inport: '<Root>/b0'
   *  Inport: '<Root>/prev_sent_nm'
   *  Product: '<Root>/InputAdvance'
   *  Product: '<Root>/StateAdvance'
   *  UnitDelay: '<Root>/Z1Memory'
   *  UnitDelay: '<Root>/Z2Memory'
   */
  adrc_controller_DW.Z1Memory_DSTATE = (0.004F *
    adrc_controller_DW.Z2Memory_DSTATE + adrc_controller_DW.Z1Memory_DSTATE) +
    adrc_controller_U.b0 * 0.004F * adrc_controller_U.prev_sent_nm;

  /* Math: '<Root>/ObserverPole' incorporates:
   *  Constant: '<Root>/Ts'
   *  Gain: '<Root>/NegativeObserverTime'
   *  Inport: '<Root>/wo_rad_s'
   *  Product: '<Root>/ObserverTime'
   *
   * About '<Root>/ObserverPole':
   *  Operator: exp
   */
  rtb_IntegralSampleGate = expf(-(adrc_controller_U.wo_rad_s * 0.004F));

  /* Sum: '<Root>/Innovation' incorporates:
   *  Inport: '<Root>/velocity_rad_s'
   */
  rtb_Innovation = adrc_controller_U.velocity_rad_s -
    adrc_controller_DW.Z1Memory_DSTATE;

  /* Switch: '<Root>/Z1Reset' incorporates:
   *  Constant: '<Root>/Zero'
   *  Inport: '<Root>/new_sample'
   *  Inport: '<Root>/reset'
   *  Switch: '<Root>/Z1SampleGate'
   *  Switch: '<Root>/Z2Reset'
   *  Switch: '<Root>/Z2SampleGate'
   *  UnitDelay: '<Root>/Z2Memory'
   */
  if (adrc_controller_U.reset != 0) {
    /* Sum: '<Root>/Z1Prediction' incorporates:
     *  Inport: '<Root>/velocity_rad_s'
     */
    adrc_controller_DW.Z1Memory_DSTATE = adrc_controller_U.velocity_rad_s;
    adrc_controller_DW.Z2Memory_DSTATE = 0.0F;
  } else if (adrc_controller_U.new_sample != 0) {
    /* Sum: '<Root>/Z1Prediction' incorporates:
     *  Constant: '<Root>/One'
     *  Product: '<Root>/PoleSquared'
     *  Product: '<Root>/Z1Correction'
     *  Sum: '<Root>/L1'
     *  Sum: '<Root>/Z1Corrected'
     *  Switch: '<Root>/Z1SampleGate'
     *  UnitDelay: '<Root>/Z1Memory'
     */
    adrc_controller_DW.Z1Memory_DSTATE += (1.0F - rtb_IntegralSampleGate *
      rtb_IntegralSampleGate) * rtb_Innovation;

    /* UnitDelay: '<Root>/Z2Memory' incorporates:
     *  Constant: '<Root>/One'
     *  Constant: '<Root>/Ts'
     *  Product: '<Root>/L2'
     *  Product: '<Root>/Z2Correction'
     *  Sum: '<Root>/OneMinusPole'
     *  Sum: '<Root>/Z2Corrected'
     *  Switch: '<Root>/Z2SampleGate'
     */
    adrc_controller_DW.Z2Memory_DSTATE += (1.0F - rtb_IntegralSampleGate) *
      (1.0F - rtb_IntegralSampleGate) / 0.004F * rtb_Innovation;
  }

  /* End of Switch: '<Root>/Z1Reset' */

  /* Switch: '<Root>/RawSelection' incorporates:
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/mode'
   *  RelationalOperator: '<S1>/Compare'
   */
  if (adrc_controller_U.mode == 1) {
    /* Switch: '<Root>/RawSelection' incorporates:
     *  Constant: '<Root>/Two'
     *  Inport: '<Root>/b0'
     *  Product: '<Root>/PiKp'
     *  Product: '<Root>/PiProportional'
     *  Sum: '<Root>/PiRaw'
     *  UnitDelay: '<Root>/IntegralMemory'
     */
    adrc_controller_Y.torque_raw_nm = 2.0F * rtb_Kc / adrc_controller_U.b0 *
      rtb_PiError + adrc_controller_DW.IntegralMemory_DSTATE;
  } else {
    /* Switch: '<Root>/RawSelection' incorporates:
     *  Inport: '<Root>/b0'
     *  Inport: '<Root>/reference_rad_s'
     *  Product: '<Root>/LadrcRaw'
     *  Product: '<Root>/ObserverFeedback'
     *  Sum: '<Root>/DisturbanceCompensation'
     *  Sum: '<Root>/ObserverError'
     *  UnitDelay: '<Root>/Z1Memory'
     *  UnitDelay: '<Root>/Z2Memory'
     */
    adrc_controller_Y.torque_raw_nm = ((adrc_controller_U.reference_rad_s -
      adrc_controller_DW.Z1Memory_DSTATE) * rtb_Kc -
      adrc_controller_DW.Z2Memory_DSTATE) / adrc_controller_U.b0;
  }

  /* End of Switch: '<Root>/RawSelection' */

  /* Switch: '<Root>/RawMemoryReset' incorporates:
   *  Constant: '<Root>/Zero'
   *  Inport: '<Root>/reset'
   *  UnitDelay: '<Root>/PreviousRaw'
   */
  if (adrc_controller_U.reset != 0) {
    adrc_controller_DW.PreviousRaw_DSTATE = 0.0F;
  } else {
    adrc_controller_DW.PreviousRaw_DSTATE = adrc_controller_Y.torque_raw_nm;
  }

  /* End of Switch: '<Root>/RawMemoryReset' */

  /* Outport: '<Root>/z2' incorporates:
   *  UnitDelay: '<Root>/Z2Memory'
   */
  adrc_controller_Y.z2 = adrc_controller_DW.Z2Memory_DSTATE;

  /* Outport: '<Root>/z1' incorporates:
   *  UnitDelay: '<Root>/Z1Memory'
   */
  adrc_controller_Y.z1 = adrc_controller_DW.Z1Memory_DSTATE;
}

/* Model initialize function */
void adrc_controller_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void adrc_controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
