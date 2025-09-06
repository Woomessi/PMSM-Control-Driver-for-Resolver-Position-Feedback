/*
 * File: Current_IF2.c
 *
 * Code generated for Simulink model 'Current_IF2'.
 *
 * Model version                  : 6.43
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sat Jan 11 18:41:34 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Current_IF2.h"
#include "rtwtypes.h"
#include <math.h>
#include "mw_cmsis.h"
#include <float.h>
#include "math.h"

/* Named constants for Chart: '<S1>/Chart' */
#define IN_AlignStage                  ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_RunStage                    ((uint8_T)3U)

/* Exported block signals */
real32_T ThetaOpen;                    /* '<S1>/Merge' */

/* Exported data definition */

/* Definition for custom storage class: Struct */
motor_type motor = {
  /* Pn */
  8.0F
};

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
extern real32_T rt_modf_snf(real32_T u0, real32_T u1);
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3]);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);
extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
real_T rtNaN = -(real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = -(real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

/* Return rtNaN needed by the generated code. */
static real_T rtGetNaN(void)
{
  return rtNaN;
}

/* Return rtNaNF needed by the generated code. */
static real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  return (boolean_T)(isnan(value) != 0);
}

/* Output and update for atomic system: '<S10>/SVPWM1' */
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3])
{
  real32_T tmp[3];
  real32_T tmp_0[3];
  real32_T rtb_Min;
  real32_T rtb_Sum1_d;
  real32_T rtb_Sum_i;

  /* Gain: '<S119>/Gain' */
  rtb_Min = -0.5F * rtu_Valpha;

  /* Gain: '<S119>/Gain1' */
  rtb_Sum1_d = 0.866025388F * rtu_Vbeta;

  /* Sum: '<S119>/Sum' */
  rtb_Sum_i = rtb_Min + rtb_Sum1_d;

  /* Sum: '<S119>/Sum1' */
  rtb_Sum1_d = rtb_Min - rtb_Sum1_d;

  /* Gain: '<S120>/Gain' incorporates:
   *  MinMax: '<S120>/Min'
   *  MinMax: '<S120>/Min1'
   *  Sum: '<S120>/Sum'
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_i), rtb_Sum1_d) + fmaxf(fmaxf
              (rtu_Valpha, rtb_Sum_i), rtb_Sum1_d)) * -0.5F;

  /* Sum: '<S118>/Sum' */
  rty_tABC[0] = rtb_Min + rtu_Valpha;
  rty_tABC[1] = rtb_Min + rtb_Sum_i;
  rty_tABC[2] = rtb_Min + rtb_Sum1_d;

  /* Gain: '<S118>/Gain' */
  mw_arm_scale_1_f32(&rtConstP.Gain_Gain, &rty_tABC[0], &tmp[0], 3U);

  /* Sum: '<S118>/Sum1' incorporates:
   *  Constant: '<S118>/Constant'
   *  Gain: '<S118>/Gain'
   *  Product: '<S118>/Divide'
   */
  tmp[0] /= rtu_v_bus;
  tmp[1] /= rtu_v_bus;
  tmp[2] /= rtu_v_bus;
  mw_arm_bias_2_f32(&tmp[0], &rtConstP.Constant_Value, &tmp_0[0], 3U);

  /* Gain: '<S118>/PWM_HalfPeriod' incorporates:
   *  Sum: '<S118>/Sum1'
   */
  mw_arm_scale_1_f32(&rtConstP.PWM_HalfPeriod_Gain, &tmp_0[0], &rty_tABC[0], 3U);
}

real32_T rt_modf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  y = u0;
  if (u1 == 0.0F) {
    if (u0 == 0.0F) {
      y = u1;
    }
  } else if (rtIsNaNF(u0) || rtIsNaNF(u1) || rtIsInfF(u0)) {
    y = (rtNaNF);
  } else if (u0 == 0.0F) {
    y = 0.0F / u1;
  } else if (rtIsInfF(u1)) {
    if ((u1 < 0.0F) != (u0 < 0.0F)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = fmodf(u0, u1);
    yEq = (y == 0.0F);
    if ((!yEq) && (u1 > floorf(u1))) {
      real32_T q;
      q = fabsf(u0 / u1);
      yEq = !(fabsf(q - floorf(q + 0.5F)) > FLT_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0F;
    } else if ((u0 < 0.0F) != (u1 < 0.0F)) {
      y += u1;
    }
  }

  return y;
}

/* Model step function */
void Current_IF2_step(void)
{
  real32_T rtb_Add_p;
  real32_T rtb_DeadZone;
  real32_T rtb_DeadZone_b_tmp;
  real32_T rtb_DeadZone_h;
  real32_T rtb_IProdOut_d;
  real32_T rtb_IProdOut_h_tmp;
  real32_T rtb_Integrator_h;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_g;
  int8_T tmp;
  int8_T tmp_0;

  /* Outputs for Atomic SubSystem: '<Root>/Current_IF2' */
  /* Sum: '<S3>/Add1' incorporates:
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S3>/Add'
   */
  rtb_Saturation_g = 0.666666687F * rtU.ia - (rtU.ib + rtU.ic) * 0.333333343F;

  /* Chart: '<S1>/Chart' incorporates:
   *  Inport: '<Root>/Motor_OnOff'
   */
  if (rtDW.temporalCounter_i1 < 1023U) {
    rtDW.temporalCounter_i1++;
  }

  if (rtDW.is_active_c3_Current_IF2 == 0U) {
    rtDW.is_active_c3_Current_IF2 = 1U;
    rtDW.is_c3_Current_IF2 = IN_IDLE;
  } else {
    switch (rtDW.is_c3_Current_IF2) {
     case IN_AlignStage:
      if (rtDW.temporalCounter_i1 >= 1000) {
        rtDW.is_c3_Current_IF2 = IN_RunStage;
      } else if (rtU.Motor_OnOff == 0.0F) {
        rtDW.is_c3_Current_IF2 = IN_IDLE;
      } else {
        rtDW.Motor_state = 2.0;
      }
      break;

     case IN_IDLE:
      if (rtU.Motor_OnOff == 1.0F) {
        rtDW.temporalCounter_i1 = 0U;
        rtDW.is_c3_Current_IF2 = IN_AlignStage;
      } else {
        rtDW.Motor_state = 1.0;
      }
      break;

     default:
      /* case IN_RunStage: */
      if (rtU.Motor_OnOff == 0.0F) {
        rtDW.is_c3_Current_IF2 = IN_IDLE;
      } else {
        rtDW.Motor_state = 4.0;
      }
      break;
    }
  }

  /* End of Chart: '<S1>/Chart' */

  /* SwitchCase: '<S1>/Switch Case' */
  switch ((int32_T)rtDW.Motor_state) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Constant: '<S4>/Constant'
     *  SignalConversion generated from: '<S4>/theta_fd'
     */
    ThetaOpen = 0.0F;

    /* Merge: '<S1>/Merge1' incorporates:
     *  Constant: '<S4>/Constant1'
     *  SignalConversion generated from: '<S4>/iq_ref'
     */
    rtDW.Merge1 = 0.0F;

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Constant: '<S5>/Constant'
     *  SignalConversion generated from: '<S5>/theta_fd'
     */
    ThetaOpen = 0.0F;

    /* Merge: '<S1>/Merge1' incorporates:
     *  Inport: '<Root>/iq_ref0'
     *  SignalConversion generated from: '<S5>/Input'
     */
    rtDW.Merge1 = rtU.iq_ref0;

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem1' */
    break;

   case 3:
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Constant: '<S6>/Constant1'
     *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator1'
     *  Math: '<S6>/Mod'
     */
    ThetaOpen = rt_modf_snf(rtDW.DiscreteTimeIntegrator1_DSTAT_g, 6.28318548F);

    /* Merge: '<S1>/Merge1' incorporates:
     *  Inport: '<Root>/iq_ref'
     *  SignalConversion generated from: '<S6>/iq_ref'
     */
    rtDW.Merge1 = rtU.iq_ref;

    /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator1' incorporates:
     *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
     */
    rtDW.DiscreteTimeIntegrator1_DSTAT_g += 0.0001F *
      rtDW.DiscreteTimeIntegrator_DSTATE;

    /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S6>/Constant4'
     *  Gain: '<S6>/Gain'
     *  Gain: '<S6>/Gain1'
     *  Inport: '<Root>/OPSpd'
     *  Inport: '<Root>/OPTime'
     *  Product: '<S6>/Divide'
     *  Product: '<S6>/Product'
     */
    rtDW.DiscreteTimeIntegrator_DSTATE += 0.104719758F * rtU.OPSpd * motor.Pn *
      (1.0F / rtU.OPTime) * 0.0001F;

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem2' */
    break;

   case 4:
    /* Outputs for IfAction SubSystem: '<S1>/If Action Subsystem3' incorporates:
     *  ActionPort: '<S7>/Action Port'
     */
    /* Merge: '<S1>/Merge' incorporates:
     *  Constant: '<S7>/Constant1'
     *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
     *  Math: '<S7>/Mod'
     */
    ThetaOpen = rt_modf_snf(rtDW.DiscreteTimeIntegrator1_DSTATE, 6.28318548F);

    /* Merge: '<S1>/Merge1' incorporates:
     *  Inport: '<Root>/iq_ref'
     *  SignalConversion generated from: '<S7>/iq_ref'
     */
    rtDW.Merge1 = rtU.iq_ref;

    /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
     *  Gain: '<S7>/Gain'
     *  Gain: '<S7>/Gain1'
     *  Inport: '<Root>/OPSpd'
     */
    rtDW.DiscreteTimeIntegrator1_DSTATE += 0.104719758F * rtU.OPSpd * motor.Pn *
      0.0001F;

    /* End of Outputs for SubSystem: '<S1>/If Action Subsystem3' */
    break;
  }

  /* End of SwitchCase: '<S1>/Switch Case' */

  /* Trigonometry: '<S1>/SinCos1' incorporates:
   *  Trigonometry: '<S117>/SinCos1'
   */
  rtb_DeadZone_b_tmp = cosf(ThetaOpen);

  /* Gain: '<S3>/Gain2' incorporates:
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S3>/Add2'
   */
  rtb_Integrator_h = (rtU.ib - rtU.ic) * 0.577350259F;

  /* Trigonometry: '<S1>/SinCos' incorporates:
   *  Trigonometry: '<S117>/SinCos'
   */
  rtb_IProdOut_h_tmp = sinf(ThetaOpen);

  /* Sum: '<S8>/Add' incorporates:
   *  Product: '<S8>/Product'
   *  Product: '<S8>/Product1'
   *  Trigonometry: '<S1>/SinCos'
   *  Trigonometry: '<S1>/SinCos1'
   */
  rtb_Add_p = rtb_Saturation_g * rtb_DeadZone_b_tmp + rtb_Integrator_h *
    rtb_IProdOut_h_tmp;

  /* Sum: '<S57>/Sum' incorporates:
   *  Constant: '<S9>/Constant'
   *  DiscreteIntegrator: '<S48>/Integrator'
   *  Inport: '<Root>/P_current'
   *  Product: '<S53>/PProd Out'
   *  Sum: '<S9>/Sum1'
   */
  rtb_DeadZone = (0.0F - rtb_Add_p) * rtU.P_current + rtDW.Integrator_DSTATE;

  /* Saturate: '<S55>/Saturation' */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_Saturation = 24.9415321F;
  } else if (rtb_DeadZone < -24.9415321F) {
    rtb_Saturation = -24.9415321F;
  } else {
    rtb_Saturation = rtb_DeadZone;
  }

  /* End of Saturate: '<S55>/Saturation' */

  /* Sum: '<S8>/Add1' incorporates:
   *  Product: '<S8>/Product2'
   *  Product: '<S8>/Product3'
   *  Trigonometry: '<S1>/SinCos'
   *  Trigonometry: '<S1>/SinCos1'
   */
  rtb_Integrator_h = rtb_Integrator_h * rtb_DeadZone_b_tmp - rtb_Saturation_g *
    rtb_IProdOut_h_tmp;

  /* Sum: '<S9>/Sum7' */
  rtb_IProdOut_d = rtDW.Merge1 - rtb_Integrator_h;

  /* Sum: '<S109>/Sum' incorporates:
   *  DiscreteIntegrator: '<S100>/Integrator'
   *  Inport: '<Root>/P_current'
   *  Product: '<S105>/PProd Out'
   */
  rtb_DeadZone_h = rtb_IProdOut_d * rtU.P_current + rtDW.Integrator_DSTATE_i;

  /* Saturate: '<S107>/Saturation' */
  if (rtb_DeadZone_h > 24.9415321F) {
    rtb_Saturation_g = 24.9415321F;
  } else if (rtb_DeadZone_h < -24.9415321F) {
    rtb_Saturation_g = -24.9415321F;
  } else {
    rtb_Saturation_g = rtb_DeadZone_h;
  }

  /* End of Saturate: '<S107>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S10>/SVPWM1' */
  /* Sum: '<S117>/Add' incorporates:
   *  Inport: '<Root>/v_bus'
   *  Outport: '<Root>/tABC'
   *  Product: '<S117>/Product'
   *  Product: '<S117>/Product1'
   *  Product: '<S117>/Product2'
   *  Product: '<S117>/Product3'
   *  Sum: '<S117>/Add1'
   */
  SVPWM1(rtb_Saturation * rtb_DeadZone_b_tmp - rtb_Saturation_g *
         rtb_IProdOut_h_tmp, rtb_Saturation * rtb_IProdOut_h_tmp +
         rtb_Saturation_g * rtb_DeadZone_b_tmp, rtU.v_bus, rtY.tABC);

  /* End of Outputs for SubSystem: '<S10>/SVPWM1' */

  /* DeadZone: '<S92>/DeadZone' */
  if (rtb_DeadZone_h > 24.9415321F) {
    rtb_DeadZone_h -= 24.9415321F;
  } else if (rtb_DeadZone_h >= -24.9415321F) {
    rtb_DeadZone_h = 0.0F;
  } else {
    rtb_DeadZone_h -= -24.9415321F;
  }

  /* End of DeadZone: '<S92>/DeadZone' */

  /* Product: '<S97>/IProd Out' incorporates:
   *  Inport: '<Root>/I_current'
   */
  rtb_IProdOut_d *= rtU.I_current;

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
   *  Constant: '<S9>/Constant'
   *  Inport: '<Root>/I_current'
   *  Sum: '<S9>/Sum1'
   */
  rtb_Saturation_g = (0.0F - rtb_Add_p) * rtU.I_current;

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
  if (rtb_Saturation_g > 0.0F) {
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
    rtb_Saturation_g = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Switch: '<S38>/Switch'
   */
  rtDW.Integrator_DSTATE += 0.0001F * rtb_Saturation_g;

  /* Switch: '<S90>/Switch1' incorporates:
   *  Constant: '<S90>/Clamping_zero'
   *  Constant: '<S90>/Constant'
   *  Constant: '<S90>/Constant2'
   *  RelationalOperator: '<S90>/fix for DT propagation issue'
   */
  if (rtb_DeadZone_h > 0.0F) {
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
  if (rtb_IProdOut_d > 0.0F) {
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
  if ((rtb_DeadZone_h != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut_d = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S100>/Integrator' incorporates:
   *  Switch: '<S90>/Switch'
   */
  rtDW.Integrator_DSTATE_i += 0.0001F * rtb_IProdOut_d;

  /* End of Outputs for SubSystem: '<Root>/Current_IF2' */

  /* Outport: '<Root>/id' */
  rtY.id = rtb_Add_p;

  /* Outport: '<Root>/iq' */
  rtY.iq = rtb_Integrator_h;
}

/* Model initialize function */
void Current_IF2_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
