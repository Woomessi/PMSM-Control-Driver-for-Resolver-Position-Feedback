/*
 * File: transformation.c
 *
 * Code generated for Simulink model 'transformation'.
 *
 * Model version                  : 6.276
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Tue Mar 25 16:16:50 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "transformation.h"
#include <math.h>
#include "rtwtypes.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void transformation_step(void)
{
  real32_T rtb_Add_h;
  real32_T rtb_DeadZone;
  real32_T rtb_IProdOut;
  real32_T rtb_Integrator;
  real32_T rtb_Saturation;
  real32_T rtb_Saturation_i;
  real32_T rtb_SinCos;
  real32_T rtb_SinCos1;
  int16_T rtb_IProdOut_d;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_AND3;

  /* Outputs for Atomic SubSystem: '<Root>/transformation' */
  /* Sum: '<S3>/Add1' incorporates:
   *  Gain: '<S3>/Gain'
   *  Gain: '<S3>/Gain1'
   *  Inport: '<Root>/ia'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S3>/Add'
   */
  rtb_Saturation = 0.666666687F * rtU.ia - (rtU.ib + rtU.ic) * 0.333333343F;

  /* Trigonometry: '<S1>/SinCos1' incorporates:
   *  Inport: '<Root>/theta_e'
   */
  rtb_SinCos1 = cosf(rtU.theta_e);

  /* Gain: '<S3>/Gain2' incorporates:
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Sum: '<S3>/Add2'
   */
  rtb_DeadZone = (rtU.ib - rtU.ic) * 0.577350259F;

  /* Trigonometry: '<S1>/SinCos' incorporates:
   *  Inport: '<Root>/theta_e'
   */
  rtb_SinCos = sinf(rtU.theta_e);

  /* Sum: '<S4>/Add' incorporates:
   *  Product: '<S4>/Product'
   *  Product: '<S4>/Product1'
   */
  rtb_Add_h = rtb_Saturation * rtb_SinCos1 + rtb_DeadZone * rtb_SinCos;

  /* Sum: '<S5>/Sum1' incorporates:
   *  Inport: '<Root>/IdRef'
   */
  rtb_Integrator = rtU.IdRef - rtb_Add_h;

  /* Sum: '<S52>/Sum' incorporates:
   *  DiscreteIntegrator: '<S43>/Integrator'
   *  Inport: '<Root>/P_current'
   *  Product: '<S48>/PProd Out'
   */
  rtb_Saturation_i = rtb_Integrator * rtU.P_current + (real32_T)
    rtDW.Integrator_DSTATE_e * 0.0001F;

  /* DeadZone: '<S35>/DeadZone' */
  if (rtb_Saturation_i > 24.9415321F) {
    rtb_IProdOut = rtb_Saturation_i - 24.9415321F;
  } else if (rtb_Saturation_i >= -24.9415321F) {
    rtb_IProdOut = 0.0F;
  } else {
    rtb_IProdOut = rtb_Saturation_i - -24.9415321F;
  }

  /* End of DeadZone: '<S35>/DeadZone' */

  /* Product: '<S40>/IProd Out' incorporates:
   *  Inport: '<Root>/I_current'
   */
  rtb_IProdOut_d = (int16_T)floorf(rtb_Integrator * rtU.I_current);

  /* Switch: '<S33>/Switch1' incorporates:
   *  Constant: '<S33>/Constant'
   *  Constant: '<S33>/Constant2'
   *  RelationalOperator: '<S33>/fix for DT propagation issue'
   */
  if (rtb_IProdOut > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S33>/Switch2' incorporates:
   *  Constant: '<S33>/Clamping_zero'
   *  Constant: '<S33>/Constant3'
   *  Constant: '<S33>/Constant4'
   *  RelationalOperator: '<S33>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut_d > 0) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Logic: '<S33>/AND3' incorporates:
   *  RelationalOperator: '<S33>/Equal1'
   *  RelationalOperator: '<S33>/Relational Operator'
   *  Switch: '<S33>/Switch1'
   *  Switch: '<S33>/Switch2'
   */
  rtb_AND3 = ((rtb_IProdOut != 0.0F) && (tmp == tmp_0));

  /* Saturate: '<S50>/Saturation' */
  if (rtb_Saturation_i > 24.9415321F) {
    rtb_Saturation_i = 24.9415321F;
  } else if (rtb_Saturation_i < -24.9415321F) {
    rtb_Saturation_i = -24.9415321F;
  }

  /* End of Saturate: '<S50>/Saturation' */

  /* Sum: '<S4>/Add1' incorporates:
   *  Product: '<S4>/Product2'
   *  Product: '<S4>/Product3'
   */
  rtb_Integrator = rtb_DeadZone * rtb_SinCos1 - rtb_Saturation * rtb_SinCos;

  /* Sum: '<S5>/Sum7' incorporates:
   *  Inport: '<Root>/IqRef'
   */
  rtb_IProdOut = rtU.IqRef - rtb_Integrator;

  /* Sum: '<S104>/Sum' incorporates:
   *  DiscreteIntegrator: '<S95>/Integrator'
   *  Inport: '<Root>/P_current'
   *  Product: '<S100>/PProd Out'
   */
  rtb_DeadZone = rtb_IProdOut * rtU.P_current + rtDW.Integrator_DSTATE;

  /* Saturate: '<S102>/Saturation' incorporates:
   *  DeadZone: '<S87>/DeadZone'
   */
  if (rtb_DeadZone > 24.9415321F) {
    rtb_Saturation = 24.9415321F;
    rtb_DeadZone -= 24.9415321F;
  } else {
    if (rtb_DeadZone < -24.9415321F) {
      rtb_Saturation = -24.9415321F;
    } else {
      rtb_Saturation = rtb_DeadZone;
    }

    if (rtb_DeadZone >= -24.9415321F) {
      rtb_DeadZone = 0.0F;
    } else {
      rtb_DeadZone -= -24.9415321F;
    }
  }

  /* End of Saturate: '<S102>/Saturation' */

  /* Product: '<S92>/IProd Out' incorporates:
   *  Inport: '<Root>/I_current'
   */
  rtb_IProdOut *= rtU.I_current;

  /* Switch: '<S33>/Switch' incorporates:
   *  Constant: '<S33>/Constant1'
   */
  if (rtb_AND3) {
    rtb_IProdOut_d = 0;
  }

  /* Update for DiscreteIntegrator: '<S43>/Integrator' incorporates:
   *  Switch: '<S33>/Switch'
   */
  rtDW.Integrator_DSTATE_e += rtb_IProdOut_d;

  /* Switch: '<S85>/Switch1' incorporates:
   *  Constant: '<S85>/Clamping_zero'
   *  Constant: '<S85>/Constant'
   *  Constant: '<S85>/Constant2'
   *  RelationalOperator: '<S85>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S85>/Switch2' incorporates:
   *  Constant: '<S85>/Clamping_zero'
   *  Constant: '<S85>/Constant3'
   *  Constant: '<S85>/Constant4'
   *  RelationalOperator: '<S85>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S85>/Switch' incorporates:
   *  Constant: '<S85>/Clamping_zero'
   *  Constant: '<S85>/Constant1'
   *  Logic: '<S85>/AND3'
   *  RelationalOperator: '<S85>/Equal1'
   *  RelationalOperator: '<S85>/Relational Operator'
   *  Switch: '<S85>/Switch1'
   *  Switch: '<S85>/Switch2'
   */
  if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S95>/Integrator' incorporates:
   *  Switch: '<S85>/Switch'
   */
  rtDW.Integrator_DSTATE += 0.0001F * rtb_IProdOut;

  /* End of Outputs for SubSystem: '<Root>/transformation' */

  /* Outport: '<Root>/id' */
  rtY.id = rtb_Add_h;

  /* Outport: '<Root>/iq' */
  rtY.iq = rtb_Integrator;

  /* Outputs for Atomic SubSystem: '<Root>/transformation' */
  /* Outport: '<Root>/ualpha' incorporates:
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product1'
   *  Sum: '<S2>/Add'
   */
  rtY.ualpha = rtb_Saturation_i * rtb_SinCos1 - rtb_Saturation * rtb_SinCos;

  /* Outport: '<Root>/ubeta' incorporates:
   *  Product: '<S2>/Product2'
   *  Product: '<S2>/Product3'
   *  Sum: '<S2>/Add1'
   */
  rtY.ubeta = rtb_Saturation_i * rtb_SinCos + rtb_Saturation * rtb_SinCos1;

  /* End of Outputs for SubSystem: '<Root>/transformation' */
}

/* Model initialize function */
void transformation_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
