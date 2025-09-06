/*
 * File: svpwm.c
 *
 * Code generated for Simulink model 'svpwm'.
 *
 * Model version                  : 6.39
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Tue Jan  7 21:30:08 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "svpwm.h"
#include "rtwtypes.h"
#include <math.h>
#include "mw_cmsis.h"

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3]);

/* Output and update for atomic system: '<Root>/SVPWM1' */
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3])
{
  real32_T tmp[3];
  real32_T tmp_0[3];
  real32_T rtb_Min;
  real32_T rtb_Sum1_j;
  real32_T rtb_Sum_m;

  /* Gain: '<S3>/Gain' */
  rtb_Min = -0.5F * rtu_Valpha;

  /* Gain: '<S3>/Gain1' */
  rtb_Sum1_j = 0.866025388F * rtu_Vbeta;

  /* Sum: '<S3>/Sum' */
  rtb_Sum_m = rtb_Min + rtb_Sum1_j;

  /* Sum: '<S3>/Sum1' */
  rtb_Sum1_j = rtb_Min - rtb_Sum1_j;

  /* Gain: '<S4>/Gain' incorporates:
   *  MinMax: '<S4>/Min'
   *  MinMax: '<S4>/Min1'
   *  Sum: '<S4>/Sum'
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_m), rtb_Sum1_j) + fmaxf(fmaxf
              (rtu_Valpha, rtb_Sum_m), rtb_Sum1_j)) * -0.5F;

  /* Sum: '<S2>/Sum' */
  rty_tABC[0] = rtb_Min + rtu_Valpha;
  rty_tABC[1] = rtb_Min + rtb_Sum_m;
  rty_tABC[2] = rtb_Min + rtb_Sum1_j;

  /* Gain: '<S2>/Gain' */
  mw_arm_scale_1_f32(&rtConstP.Gain_Gain, &rty_tABC[0], &tmp[0], 3U);

  /* Sum: '<S2>/Sum1' incorporates:
   *  Constant: '<S2>/Constant'
   *  Gain: '<S2>/Gain'
   *  Product: '<S2>/Divide'
   */
  tmp[0] /= rtu_v_bus;
  tmp[1] /= rtu_v_bus;
  tmp[2] /= rtu_v_bus;
  mw_arm_bias_2_f32(&tmp[0], &rtConstP.Constant_Value, &tmp_0[0], 3U);

  /* Gain: '<S2>/PWM_HalfPeriod' incorporates:
   *  Sum: '<S2>/Sum1'
   */
  mw_arm_scale_1_f32(&rtConstP.PWM_HalfPeriod_Gain, &tmp_0[0], &rty_tABC[0], 3U);
}

/* Model step function */
void svpwm_step(void)
{
  real32_T rtb_SinCos;
  real32_T rtb_SinCos1;

  /* Trigonometry: '<S1>/SinCos1' incorporates:
   *  Inport: '<Root>/position_e'
   */
  rtb_SinCos1 = cosf(rtU.position_e);

  /* Trigonometry: '<S1>/SinCos' incorporates:
   *  Inport: '<Root>/position_e'
   */
  rtb_SinCos = sinf(rtU.position_e);

  /* Outputs for Atomic SubSystem: '<Root>/SVPWM1' */

  /* Sum: '<S1>/Add' incorporates:
   *  Inport: '<Root>/ud'
   *  Inport: '<Root>/uq'
   *  Inport: '<Root>/v_bus'
   *  Outport: '<Root>/tABC'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Product: '<S1>/Product2'
   *  Product: '<S1>/Product3'
   *  Sum: '<S1>/Add1'
   */
  SVPWM1(rtU.ud * rtb_SinCos1 - rtU.uq * rtb_SinCos, rtU.ud * rtb_SinCos +
         rtU.uq * rtb_SinCos1, rtU.v_bus, rtY.tABC);

  /* End of Outputs for SubSystem: '<Root>/SVPWM1' */
}

/* Model initialize function */
void svpwm_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
