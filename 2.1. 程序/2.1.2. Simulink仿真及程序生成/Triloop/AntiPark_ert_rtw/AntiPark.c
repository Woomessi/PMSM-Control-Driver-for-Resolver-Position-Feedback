/*
 * File: AntiPark.c
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

#include "AntiPark.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void AntiPark_step(void)
{
  /* Outport: '<Root>/ualpha' incorporates:
   *  Inport: '<Root>/theta_cos'
   *  Inport: '<Root>/theta_sin'
   *  Inport: '<Root>/ud'
   *  Inport: '<Root>/uq'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Sum: '<S1>/Add'
   */
  rtY.ualpha = rtU.ud * rtU.theta_cos - rtU.uq * rtU.theta_sin;

  /* Outport: '<Root>/ubeta' incorporates:
   *  Inport: '<Root>/theta_cos'
   *  Inport: '<Root>/theta_sin'
   *  Inport: '<Root>/ud'
   *  Inport: '<Root>/uq'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Add1'
   */
  rtY.ubeta = rtU.ud * rtU.theta_sin + rtU.uq * rtU.theta_cos;
}

/* Model initialize function */
void AntiPark_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
