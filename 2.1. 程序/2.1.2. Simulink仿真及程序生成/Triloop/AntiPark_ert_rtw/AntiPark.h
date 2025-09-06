/*
 * File: AntiPark.h
 *
 * Code generated for Simulink model 'AntiPark'.
 *
 * Model version                  : 6.273
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Tue Mar 25 15:57:38 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef AntiPark_h_
#define AntiPark_h_
#ifndef AntiPark_COMMON_INCLUDES_
#define AntiPark_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* AntiPark_COMMON_INCLUDES_ */

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ud;                         /* '<Root>/ud' */
  real32_T uq;                         /* '<Root>/uq' */
  real32_T theta_sin;                  /* '<Root>/theta_sin' */
  real32_T theta_cos;                  /* '<Root>/theta_cos' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T ualpha;                     /* '<Root>/ualpha' */
  real32_T ubeta;                      /* '<Root>/ubeta' */
} ExtY;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void AntiPark_initialize(void);
extern void AntiPark_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('positionloop/currloop/AntiPark')    - opens subsystem positionloop/currloop/AntiPark
 * hilite_system('positionloop/currloop/AntiPark/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'positionloop/currloop'
 * '<S1>'   : 'positionloop/currloop/AntiPark'
 */
#endif                                 /* AntiPark_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
