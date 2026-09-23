/*
 * File: adrc_controller.h
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

#ifndef adrc_controller_h_
#define adrc_controller_h_
#ifndef adrc_controller_COMMON_INCLUDES_
#define adrc_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* adrc_controller_COMMON_INCLUDES_ */

#include "adrc_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T IntegralMemory_DSTATE;      /* '<Root>/IntegralMemory' */
  real32_T PreviousRaw_DSTATE;         /* '<Root>/PreviousRaw' */
  real32_T Z1Memory_DSTATE;            /* '<Root>/Z1Memory' */
  real32_T Z2Memory_DSTATE;            /* '<Root>/Z2Memory' */
} DW_adrc_controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  uint8_T mode;                        /* '<Root>/mode' */
  real32_T reference_rad_s;            /* '<Root>/reference_rad_s' */
  real32_T velocity_rad_s;             /* '<Root>/velocity_rad_s' */
  real32_T prev_sent_nm;               /* '<Root>/prev_sent_nm' */
  real32_T b0;                         /* '<Root>/b0' */
  real32_T wc_rad_s;                   /* '<Root>/wc_rad_s' */
  real32_T wo_rad_s;                   /* '<Root>/wo_rad_s' */
  uint8_T reset;                       /* '<Root>/reset' */
  uint8_T new_sample;                  /* '<Root>/new_sample' */
} ExtU_adrc_controller_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T torque_raw_nm;              /* '<Root>/torque_raw_nm' */
  real32_T z1;                         /* '<Root>/z1' */
  real32_T z2;                         /* '<Root>/z2' */
} ExtY_adrc_controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_adrc_controller_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_adrc_controller_T adrc_controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_adrc_controller_T adrc_controller_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_adrc_controller_T adrc_controller_Y;

/* Model entry point functions */
extern void adrc_controller_initialize(void);
extern void adrc_controller_step(void);
extern void adrc_controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_adrc_controller_T *const adrc_controller_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'adrc_controller'
 * '<S1>'   : 'adrc_controller/IsPi'
 */
#endif                                 /* adrc_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
