/*
 * File: Current.c
 *
 * Code generated for Simulink model 'Current'.
 *
 * Model version                  : 6.242
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Mon Nov 18 22:45:42 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Current.h"
#include "rtwtypes.h"
#include <math.h>
#include "mw_cmsis.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                  real32_T rty_tABC[3]);

/* Output and update for atomic system: '<S3>/SVPWM' */
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                  real32_T rty_tABC[3])
{
  real32_T tmp[3];
  real32_T tmp_0[3];
  real32_T rtb_Min;
  real32_T rtb_Sum1_l;
  real32_T rtb_Sum_o;

  /* Gain: '<S9>/Gain' */
  rtb_Min = -0.5F * rtu_Valpha;

  /* Gain: '<S9>/Gain1' */
  rtb_Sum1_l = 0.866025388F * rtu_Vbeta;

  /* Sum: '<S9>/Sum' */
  rtb_Sum_o = rtb_Min + rtb_Sum1_l;

  /* Sum: '<S9>/Sum1' */
  rtb_Sum1_l = rtb_Min - rtb_Sum1_l;

  /* Gain: '<S10>/Gain' incorporates:
   *  MinMax: '<S10>/Min'
   *  MinMax: '<S10>/Min1'
   *  Sum: '<S10>/Sum'
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_o), rtb_Sum1_l) + fmaxf(fmaxf
              (rtu_Valpha, rtb_Sum_o), rtb_Sum1_l)) * -0.5F;

  /* Sum: '<S7>/Sum' */
  rty_tABC[0] = rtb_Min + rtu_Valpha;
  rty_tABC[1] = rtb_Min + rtb_Sum_o;
  rty_tABC[2] = rtb_Min + rtb_Sum1_l;

  /* Gain: '<S7>/Gain' */
  mw_arm_scale_1_f32(&rtConstP.Gain_Gain, &rty_tABC[0], &tmp[0], 3U);

  /* Sum: '<S7>/Sum1' incorporates:
   *  Constant: '<S7>/Constant'
   *  Gain: '<S7>/Gain'
   *  Product: '<S7>/Divide'
   */
  tmp[0] /= rtu_v_bus;
  tmp[1] /= rtu_v_bus;
  tmp[2] /= rtu_v_bus;
  mw_arm_bias_2_f32(&tmp[0], &rtConstP.Constant_Value, &tmp_0[0], 3U);

  /* Gain: '<S7>/PWM_HalfPeriod' incorporates:
   *  Sum: '<S7>/Sum1'
   */
  mw_arm_scale_1_f32(&rtConstP.PWM_HalfPeriod_Gain, &tmp_0[0], &rty_tABC[0], 3U);
}

/* Model step function */
void Current_step(void)
{
  real32_T rtb_Add_h;
  real32_T rtb_DeadZone;
  real32_T rtb_DeadZone_i;
  real32_T rtb_IProdOut_b;
  real32_T rtb_Integrator_c;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_l;
  real32_T rtb_SinCos;
  real32_T rtb_SinCos1;
  int8_T tmp;
  int8_T tmp_0;

  /* Outputs for Atomic SubSystem: '<Root>/Current' */
  /* Outputs for Atomic SubSystem: '<S1>/Speed and Current Loop5' */
  /* Outputs for Atomic SubSystem: '<S2>/currloop' */
  /* Trigonometry: '<S3>/SinCos1' incorporates:
   *  Inport: '<Root>/PositionFd_e'
   */
  rtb_SinCos1 = cosf(rtU.PositionFd_e);

  /* Sum: '<S5>/Add1' incorporates:
   *  Gain: '<S5>/Gain'
   *  Gain: '<S5>/Gain1'
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S5>/Add'
   */
  rtb_Integrator_c = 0.666666687F * rtU.ia - (rtU.ib + rtU.ic) * 0.333333343F;

  /* Gain: '<S5>/Gain2' incorporates:
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S5>/Add2'
   */
  rtb_IProdOut_b = (rtU.ib - rtU.ic) * 0.577350259F;

  /* Trigonometry: '<S3>/SinCos' incorporates:
   *  Inport: '<Root>/PositionFd_e'
   */
  rtb_SinCos = sinf(rtU.PositionFd_e);

  /* Sum: '<S6>/Add' incorporates:
   *  Product: '<S6>/Product'
   *  Product: '<S6>/Product1'
   */
  rtb_Add_h = rtb_Integrator_c * rtb_SinCos1 + rtb_IProdOut_b * rtb_SinCos;

  /* Sum: '<S57>/Sum' incorporates:
   *  Constant: '<S8>/Constant'
   *  DiscreteIntegrator: '<S48>/Integrator'
   *  Inport: '<Root>/P_id'
   *  Product: '<S53>/PProd Out'
   *  Sum: '<S8>/Sum1'
   */
  rtb_DeadZone = (0.0F - rtb_Add_h) * rtU.P_id + rtDW.Integrator_DSTATE;

  /* Saturate: '<S55>/Saturation' */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_Saturation = 24.9415321F;
  } else if (rtb_DeadZone < -24.9415321F) {
    rtb_Saturation = -24.9415321F;
  } else {
    rtb_Saturation = rtb_DeadZone;
  }

  /* End of Saturate: '<S55>/Saturation' */

  /* Sum: '<S6>/Add1' incorporates:
   *  Product: '<S6>/Product2'
   *  Product: '<S6>/Product3'
   */
  rtb_Integrator_c = rtb_IProdOut_b * rtb_SinCos1 - rtb_Integrator_c *
    rtb_SinCos;

  /* Sum: '<S8>/Sum7' incorporates:
   *  Inport: '<Root>/iq_Ref'
   */
  rtb_IProdOut_b = rtU.iq_Ref - rtb_Integrator_c;

  /* Sum: '<S109>/Sum' incorporates:
   *  DiscreteIntegrator: '<S100>/Integrator'
   *  Inport: '<Root>/P_iq'
   *  Product: '<S105>/PProd Out'
   */
  rtb_DeadZone_i = rtb_IProdOut_b * rtU.P_iq + rtDW.Integrator_DSTATE_j;

  /* Saturate: '<S107>/Saturation' */
  if (rtb_DeadZone_i > 24.9415321F) {
    rtb_Saturation_l = 24.9415321F;
  } else if (rtb_DeadZone_i < -24.9415321F) {
    rtb_Saturation_l = -24.9415321F;
  } else {
    rtb_Saturation_l = rtb_DeadZone_i;
  }

  /* End of Saturate: '<S107>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S3>/SVPWM' */
  /* Sum: '<S4>/Add' incorporates:
   *  Inport: '<Root>/v_bus'
   *  Outport: '<Root>/tABC'
   *  Product: '<S4>/Product'
   *  Product: '<S4>/Product1'
   *  Product: '<S4>/Product2'
   *  Product: '<S4>/Product3'
   *  Sum: '<S4>/Add1'
   */
  SVPWM(rtb_Saturation * rtb_SinCos1 - rtb_Saturation_l * rtb_SinCos,
        rtb_Saturation * rtb_SinCos + rtb_Saturation_l * rtb_SinCos1, rtU.v_bus,
        rtY.tABC);

  /* End of Outputs for SubSystem: '<S3>/SVPWM' */

  /* DeadZone: '<S92>/DeadZone' */
  if (rtb_DeadZone_i > 24.9415321F) {
    rtb_DeadZone_i -= 24.9415321F;
  } else if (rtb_DeadZone_i >= -24.9415321F) {
    rtb_DeadZone_i = 0.0F;
  } else {
    rtb_DeadZone_i -= -24.9415321F;
  }

  /* End of DeadZone: '<S92>/DeadZone' */

  /* Product: '<S97>/IProd Out' incorporates:
   *  Inport: '<Root>/I_iq'
   */
  rtb_IProdOut_b *= rtU.I_iq;

  /* DeadZone: '<S40>/DeadZone' */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_DeadZone -= 24.9415321F;
  } else if (rtb_DeadZone >= -24.9415321F) {
    rtb_DeadZone = 0.0F;
  } else {
    rtb_DeadZone -= -24.9415321F;
  }

  /* End of DeadZone: '<S40>/DeadZone' */

  /* Product: '<S45>/IProd Out' incorporates:
   *  Constant: '<S8>/Constant'
   *  Inport: '<Root>/I_id'
   *  Sum: '<S8>/Sum1'
   */
  rtb_SinCos1 = (0.0F - rtb_Add_h) * rtU.I_id;

  /* Switch: '<S38>/Switch1' incorporates:
   *  Constant: '<S38>/Clamping_zero'
   *  Constant: '<S38>/Constant'
   *  Constant: '<S38>/Constant2'
   *  RelationalOperator: '<S38>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S38>/Switch2' incorporates:
   *  Constant: '<S38>/Clamping_zero'
   *  Constant: '<S38>/Constant3'
   *  Constant: '<S38>/Constant4'
   *  RelationalOperator: '<S38>/fix for DT propagation issue1'
   */
  if (rtb_SinCos1 > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S38>/Switch' incorporates:
   *  Constant: '<S38>/Clamping_zero'
   *  Constant: '<S38>/Constant1'
   *  Logic: '<S38>/AND3'
   *  RelationalOperator: '<S38>/Equal1'
   *  RelationalOperator: '<S38>/Relational Operator'
   *  Switch: '<S38>/Switch1'
   *  Switch: '<S38>/Switch2'
   */
  if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
    rtb_SinCos1 = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Switch: '<S38>/Switch'
   */
  rtDW.Integrator_DSTATE += 0.0001F * rtb_SinCos1;

  /* Switch: '<S90>/Switch1' incorporates:
   *  Constant: '<S90>/Clamping_zero'
   *  Constant: '<S90>/Constant'
   *  Constant: '<S90>/Constant2'
   *  RelationalOperator: '<S90>/fix for DT propagation issue'
   */
  if (rtb_DeadZone_i > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S90>/Switch2' incorporates:
   *  Constant: '<S90>/Clamping_zero'
   *  Constant: '<S90>/Constant3'
   *  Constant: '<S90>/Constant4'
   *  RelationalOperator: '<S90>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut_b > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S90>/Switch' incorporates:
   *  Constant: '<S90>/Clamping_zero'
   *  Constant: '<S90>/Constant1'
   *  Logic: '<S90>/AND3'
   *  RelationalOperator: '<S90>/Equal1'
   *  RelationalOperator: '<S90>/Relational Operator'
   *  Switch: '<S90>/Switch1'
   *  Switch: '<S90>/Switch2'
   */
  if ((rtb_DeadZone_i != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut_b = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S100>/Integrator' incorporates:
   *  Switch: '<S90>/Switch'
   */
  rtDW.Integrator_DSTATE_j += 0.0001F * rtb_IProdOut_b;

  /* End of Outputs for SubSystem: '<S2>/currloop' */
  /* End of Outputs for SubSystem: '<S1>/Speed and Current Loop5' */
  /* End of Outputs for SubSystem: '<Root>/Current' */

  /* Outport: '<Root>/iq' */
  rtY.iq = rtb_Integrator_c;

  /* Outport: '<Root>/id' */
  rtY.id = rtb_Add_h;
}

/* Model initialize function */
void Current_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
