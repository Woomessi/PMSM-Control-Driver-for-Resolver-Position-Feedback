/*
 * File: Current.h
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

#ifndef Current_h_
#define Current_h_
#ifndef Current_COMMON_INCLUDES_
#define Current_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* Current_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S48>/Integrator' */
  real32_T Integrator_DSTATE_j;        /* '<S100>/Integrator' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S7>/Constant'
   */
  real32_T Constant_Value;

  /* Computed Parameter: Gain_Gain
   * Referenced by: '<S7>/Gain'
   */
  real32_T Gain_Gain;

  /* Computed Parameter: PWM_HalfPeriod_Gain
   * Referenced by: '<S7>/PWM_HalfPeriod'
   */
  real32_T PWM_HalfPeriod_Gain;
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T v_bus;                      /* '<Root>/v_bus' */
  real32_T iq_Ref;                     /* '<Root>/iq_Ref' */
  real32_T PositionFd_e;               /* '<Root>/PositionFd_e' */
  real32_T P_id;                       /* '<Root>/P_id' */
  real32_T I_id;                       /* '<Root>/I_id' */
  real32_T P_iq;                       /* '<Root>/P_iq' */
  real32_T I_iq;                       /* '<Root>/I_iq' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T tABC[3];                    /* '<Root>/tABC' */
  real32_T iq;                         /* '<Root>/iq' */
  real32_T id;                         /* '<Root>/id' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Current_initialize(void);
extern void Current_step(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Scope' : Unused code path elimination
 * Block '<S2>/Rate Transition20' : Eliminated since input and output rates are identical
 * Block '<S3>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S3>/Data Type Conversion4' : Eliminate redundant data type conversion
 */

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
 * hilite_system('positionloop/Current')    - opens subsystem positionloop/Current
 * hilite_system('positionloop/Current/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'positionloop'
 * '<S1>'   : 'positionloop/Current'
 * '<S2>'   : 'positionloop/Current/Speed and Current Loop5'
 * '<S3>'   : 'positionloop/Current/Speed and Current Loop5/currloop'
 * '<S4>'   : 'positionloop/Current/Speed and Current Loop5/currloop/AntiPark'
 * '<S5>'   : 'positionloop/Current/Speed and Current Loop5/currloop/Clark'
 * '<S6>'   : 'positionloop/Current/Speed and Current Loop5/currloop/Park'
 * '<S7>'   : 'positionloop/Current/Speed and Current Loop5/currloop/SVPWM'
 * '<S8>'   : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller'
 * '<S9>'   : 'positionloop/Current/Speed and Current Loop5/currloop/SVPWM/AntiClark'
 * '<S10>'  : 'positionloop/Current/Speed and Current Loop5/currloop/SVPWM/ei_t'
 * '<S11>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1'
 * '<S12>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2'
 * '<S13>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Anti-windup'
 * '<S14>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/D Gain'
 * '<S15>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/External Derivative'
 * '<S16>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Filter'
 * '<S17>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Filter ICs'
 * '<S18>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/I Gain'
 * '<S19>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Ideal P Gain'
 * '<S20>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S21>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Integrator'
 * '<S22>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Integrator ICs'
 * '<S23>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/N Copy'
 * '<S24>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/N Gain'
 * '<S25>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/P Copy'
 * '<S26>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Parallel P Gain'
 * '<S27>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Reset Signal'
 * '<S28>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Saturation'
 * '<S29>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Saturation Fdbk'
 * '<S30>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Sum'
 * '<S31>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Sum Fdbk'
 * '<S32>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tracking Mode'
 * '<S33>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tracking Mode Sum'
 * '<S34>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tsamp - Integral'
 * '<S35>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tsamp - Ngain'
 * '<S36>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/postSat Signal'
 * '<S37>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/preSat Signal'
 * '<S38>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S39>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S40>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S41>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/D Gain/Disabled'
 * '<S42>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/External Derivative/Disabled'
 * '<S43>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Filter/Disabled'
 * '<S44>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Filter ICs/Disabled'
 * '<S45>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/I Gain/External Parameters'
 * '<S46>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S47>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Integrator/Discrete'
 * '<S49>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S50>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S51>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/N Gain/Disabled'
 * '<S52>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/P Copy/Disabled'
 * '<S53>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Parallel P Gain/External Parameters'
 * '<S54>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Reset Signal/Disabled'
 * '<S55>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Saturation/Enabled'
 * '<S56>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S57>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Sum/Sum_PI'
 * '<S58>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S59>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tracking Mode/Disabled'
 * '<S60>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S61>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S62>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S63>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S64>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S65>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Anti-windup'
 * '<S66>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/D Gain'
 * '<S67>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/External Derivative'
 * '<S68>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Filter'
 * '<S69>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Filter ICs'
 * '<S70>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/I Gain'
 * '<S71>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Ideal P Gain'
 * '<S72>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S73>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Integrator'
 * '<S74>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Integrator ICs'
 * '<S75>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/N Copy'
 * '<S76>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/N Gain'
 * '<S77>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/P Copy'
 * '<S78>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Parallel P Gain'
 * '<S79>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Reset Signal'
 * '<S80>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Saturation'
 * '<S81>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Saturation Fdbk'
 * '<S82>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Sum'
 * '<S83>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Sum Fdbk'
 * '<S84>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tracking Mode'
 * '<S85>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tracking Mode Sum'
 * '<S86>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tsamp - Integral'
 * '<S87>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tsamp - Ngain'
 * '<S88>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/postSat Signal'
 * '<S89>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/preSat Signal'
 * '<S90>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
 * '<S91>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S92>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S93>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/D Gain/Disabled'
 * '<S94>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/External Derivative/Disabled'
 * '<S95>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Filter/Disabled'
 * '<S96>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Filter ICs/Disabled'
 * '<S97>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/I Gain/External Parameters'
 * '<S98>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S99>'  : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S100>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Integrator/Discrete'
 * '<S101>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S102>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/N Copy/Disabled wSignal Specification'
 * '<S103>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/N Gain/Disabled'
 * '<S104>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/P Copy/Disabled'
 * '<S105>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Parallel P Gain/External Parameters'
 * '<S106>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Reset Signal/Disabled'
 * '<S107>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Saturation/Enabled'
 * '<S108>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S109>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Sum/Sum_PI'
 * '<S110>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S111>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tracking Mode/Disabled'
 * '<S112>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S113>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S114>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S115>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S116>' : 'positionloop/Current/Speed and Current Loop5/currloop/idq_Controller/PID Controller2/preSat Signal/Forward_Path'
 */
#endif                                 /* Current_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
