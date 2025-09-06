/*
 * File: Robust.c
 *
 * Code generated for Simulink model 'Robust'.
 *
 * Model version                  : 6.132
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Wed Feb 26 19:58:22 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Robust.h"
#include "rtwtypes.h"
#include <math.h>
#include "mw_cmsis.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                  real32_T rty_tABC[3]);
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 99) {/* Sample time: [0.01s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Output and update for atomic system: '<S2>/SVPWM' */
static void SVPWM(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                  real32_T rty_tABC[3])
{
  real32_T tmp[3];
  real32_T tmp_0[3];
  real32_T rtb_Min;
  real32_T rtb_Sum1_o;
  real32_T rtb_Sum_g;

  /* Gain: '<S8>/Gain' */
  rtb_Min = -0.5F * rtu_Valpha;

  /* Gain: '<S8>/Gain1' */
  rtb_Sum1_o = 0.866025388F * rtu_Vbeta;

  /* Sum: '<S8>/Sum' */
  rtb_Sum_g = rtb_Min + rtb_Sum1_o;

  /* Sum: '<S8>/Sum1' */
  rtb_Sum1_o = rtb_Min - rtb_Sum1_o;

  /* Gain: '<S9>/Gain' incorporates:
   *  MinMax: '<S9>/Min'
   *  MinMax: '<S9>/Min1'
   *  Sum: '<S9>/Sum'
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_g), rtb_Sum1_o) + fmaxf(fmaxf
              (rtu_Valpha, rtb_Sum_g), rtb_Sum1_o)) * -0.5F;

  /* Sum: '<S6>/Sum' */
  rty_tABC[0] = rtb_Min + rtu_Valpha;
  rty_tABC[1] = rtb_Min + rtb_Sum_g;
  rty_tABC[2] = rtb_Min + rtb_Sum1_o;

  /* Gain: '<S6>/Gain' */
  mw_arm_scale_1_f32(&rtConstP.Gain_Gain, &rty_tABC[0], &tmp[0], 3U);

  /* Sum: '<S6>/Sum1' incorporates:
   *  Constant: '<S6>/Constant'
   *  Gain: '<S6>/Gain'
   *  Product: '<S6>/Divide'
   */
  tmp[0] /= rtu_v_bus;
  tmp[1] /= rtu_v_bus;
  tmp[2] /= rtu_v_bus;
  mw_arm_bias_2_f32(&tmp[0], &rtConstP.Constant_Value, &tmp_0[0], 3U);

  /* Gain: '<S6>/PWM_HalfPeriod' incorporates:
   *  Sum: '<S6>/Sum1'
   */
  mw_arm_scale_1_f32(&rtConstP.PWM_HalfPeriod_Gain, &tmp_0[0], &rty_tABC[0], 3U);
}

/* Model step function */
void Robust_step(void)
{
  real_T denAccum;
  real32_T rtb_DeadZone;
  real32_T rtb_IProdOut;
  real32_T rtb_IProdOut_b;
  real32_T rtb_Integrator_n;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_n;
  real32_T rtb_SinCos;
  real32_T rtb_SinCos1;
  int8_T tmp;
  int8_T tmp_0;

  /* Outputs for Atomic SubSystem: '<Root>/Robust control' */
  /* Outputs for Atomic SubSystem: '<S1>/currloop' */
  /* Trigonometry: '<S2>/SinCos1' incorporates:
   *  Inport: '<Root>/theta_e'
   */
  rtb_SinCos1 = cosf(rtU.theta_e);

  /* Sum: '<S4>/Add1' incorporates:
   *  Gain: '<S4>/Gain'
   *  Gain: '<S4>/Gain1'
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S4>/Add'
   */
  rtb_Integrator_n = 0.666666687F * rtU.ia - (rtU.ib + rtU.ic) * 0.333333343F;

  /* Gain: '<S4>/Gain2' incorporates:
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S4>/Add2'
   */
  rtb_IProdOut_b = (rtU.ib - rtU.ic) * 0.577350259F;

  /* Trigonometry: '<S2>/SinCos' incorporates:
   *  Inport: '<Root>/theta_e'
   */
  rtb_SinCos = sinf(rtU.theta_e);

  /* Sum: '<S7>/Sum1' incorporates:
   *  Constant: '<S7>/Constant'
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product1'
   *  Sum: '<S5>/Add'
   */
  rtb_IProdOut = 0.0F - (rtb_Integrator_n * rtb_SinCos1 + rtb_IProdOut_b *
    rtb_SinCos);

  /* Sum: '<S56>/Sum' incorporates:
   *  DiscreteIntegrator: '<S47>/Integrator'
   *  Inport: '<Root>/P'
   *  Product: '<S52>/PProd Out'
   */
  rtb_DeadZone = rtb_IProdOut * rtU.P_d + rtDW.Integrator_DSTATE;

  /* Saturate: '<S54>/Saturation' */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_Saturation = 24.9415321F;
  } else if (rtb_DeadZone < -24.9415321F) {
    rtb_Saturation = -24.9415321F;
  } else {
    rtb_Saturation = rtb_DeadZone;
  }

  /* End of Saturate: '<S54>/Saturation' */

  /* Sum: '<S7>/Sum7' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion10'
   *  DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn'
   *  Product: '<S5>/Product2'
   *  Product: '<S5>/Product3'
   *  Sum: '<S5>/Add1'
   */
  rtb_IProdOut_b = (real32_T)(1.313 * rtDW.DiscreteTransferFcn_states[0] +
    -1.313 * rtDW.DiscreteTransferFcn_states[1]) - (rtb_IProdOut_b * rtb_SinCos1
    - rtb_Integrator_n * rtb_SinCos);

  /* Sum: '<S108>/Sum' incorporates:
   *  DiscreteIntegrator: '<S99>/Integrator'
   *  Inport: '<Root>/P'
   *  Product: '<S104>/PProd Out'
   */
  rtb_Integrator_n = rtb_IProdOut_b * rtU.P_d + rtDW.Integrator_DSTATE_a;

  /* Saturate: '<S106>/Saturation' */
  if (rtb_Integrator_n > 24.9415321F) {
    rtb_Saturation_n = 24.9415321F;
  } else if (rtb_Integrator_n < -24.9415321F) {
    rtb_Saturation_n = -24.9415321F;
  } else {
    rtb_Saturation_n = rtb_Integrator_n;
  }

  /* End of Saturate: '<S106>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S2>/SVPWM' */
  /* Sum: '<S3>/Add' incorporates:
   *  Inport: '<Root>/v_bus'
   *  Outport: '<Root>/tABC'
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product1'
   *  Product: '<S3>/Product2'
   *  Product: '<S3>/Product3'
   *  Sum: '<S3>/Add1'
   */
  SVPWM(rtb_Saturation * rtb_SinCos1 - rtb_Saturation_n * rtb_SinCos,
        rtb_Saturation * rtb_SinCos + rtb_Saturation_n * rtb_SinCos1, rtU.v_bus,
        rtY.tABC);

  /* End of Outputs for SubSystem: '<S2>/SVPWM' */

  /* DeadZone: '<S91>/DeadZone' */
  if (rtb_Integrator_n > 24.9415321F) {
    rtb_Integrator_n -= 24.9415321F;
  } else if (rtb_Integrator_n >= -24.9415321F) {
    rtb_Integrator_n = 0.0F;
  } else {
    rtb_Integrator_n -= -24.9415321F;
  }

  /* End of DeadZone: '<S91>/DeadZone' */

  /* Product: '<S96>/IProd Out' incorporates:
   *  Inport: '<Root>/I'
   */
  rtb_IProdOut_b *= rtU.I;

  /* DeadZone: '<S39>/DeadZone' */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_DeadZone -= 24.9415321F;
  } else if (rtb_DeadZone >= -24.9415321F) {
    rtb_DeadZone = 0.0F;
  } else {
    rtb_DeadZone -= -24.9415321F;
  }

  /* End of DeadZone: '<S39>/DeadZone' */

  /* Product: '<S44>/IProd Out' incorporates:
   *  Inport: '<Root>/I'
   */
  rtb_IProdOut *= rtU.I;

  /* Switch: '<S37>/Switch1' incorporates:
   *  Constant: '<S37>/Clamping_zero'
   *  Constant: '<S37>/Constant'
   *  Constant: '<S37>/Constant2'
   *  RelationalOperator: '<S37>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S37>/Switch2' incorporates:
   *  Constant: '<S37>/Clamping_zero'
   *  Constant: '<S37>/Constant3'
   *  Constant: '<S37>/Constant4'
   *  RelationalOperator: '<S37>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S37>/Switch' incorporates:
   *  Constant: '<S37>/Clamping_zero'
   *  Constant: '<S37>/Constant1'
   *  Logic: '<S37>/AND3'
   *  RelationalOperator: '<S37>/Equal1'
   *  RelationalOperator: '<S37>/Relational Operator'
   *  Switch: '<S37>/Switch1'
   *  Switch: '<S37>/Switch2'
   */
  if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S47>/Integrator' incorporates:
   *  Switch: '<S37>/Switch'
   */
  rtDW.Integrator_DSTATE += 0.0001F * rtb_IProdOut;

  /* Switch: '<S89>/Switch1' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant'
   *  Constant: '<S89>/Constant2'
   *  RelationalOperator: '<S89>/fix for DT propagation issue'
   */
  if (rtb_Integrator_n > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S89>/Switch2' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant3'
   *  Constant: '<S89>/Constant4'
   *  RelationalOperator: '<S89>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut_b > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S89>/Switch' incorporates:
   *  Constant: '<S89>/Clamping_zero'
   *  Constant: '<S89>/Constant1'
   *  Logic: '<S89>/AND3'
   *  RelationalOperator: '<S89>/Equal1'
   *  RelationalOperator: '<S89>/Relational Operator'
   *  Switch: '<S89>/Switch1'
   *  Switch: '<S89>/Switch2'
   */
  if ((rtb_Integrator_n != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut_b = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S99>/Integrator' incorporates:
   *  Switch: '<S89>/Switch'
   */
  rtDW.Integrator_DSTATE_a += 0.0001F * rtb_IProdOut_b;

  /* End of Outputs for SubSystem: '<S1>/currloop' */

  /* Update for DiscreteTransferFcn: '<S1>/Discrete Transfer Fcn' incorporates:
   *  Inport: '<Root>/theta_output'
   *  Inport: '<Root>/theta_ref'
   *  Sum: '<S1>/Sum35'
   */
  denAccum = ((rtU.theta_ref - rtU.theta_output) -
              (-rtDW.DiscreteTransferFcn_states[0])) - 4.276E-5 *
    rtDW.DiscreteTransferFcn_states[1];
  rtDW.DiscreteTransferFcn_states[1] = rtDW.DiscreteTransferFcn_states[0];
  rtDW.DiscreteTransferFcn_states[0] = denAccum;

  /* End of Outputs for SubSystem: '<Root>/Robust control' */
  rate_scheduler();
}

/* Model initialize function */
void Robust_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
