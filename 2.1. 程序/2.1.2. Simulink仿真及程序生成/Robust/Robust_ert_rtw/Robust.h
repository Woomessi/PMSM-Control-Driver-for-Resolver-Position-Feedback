/*
 * File: Robust.h
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

#ifndef Robust_h_
#define Robust_h_
#ifndef Robust_COMMON_INCLUDES_
#define Robust_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* Robust_COMMON_INCLUDES_ */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states[2];/* '<S1>/Discrete Transfer Fcn' */
  real32_T Integrator_DSTATE;          /* '<S47>/Integrator' */
  real32_T Integrator_DSTATE_a;        /* '<S99>/Integrator' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S6>/Constant'
   */
  real32_T Constant_Value;

  /* Computed Parameter: Gain_Gain
   * Referenced by: '<S6>/Gain'
   */
  real32_T Gain_Gain;

  /* Computed Parameter: PWM_HalfPeriod_Gain
   * Referenced by: '<S6>/PWM_HalfPeriod'
   */
  real32_T PWM_HalfPeriod_Gain;
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T v_bus;                      /* '<Root>/v_bus' */
  real32_T theta_e;                    /* '<Root>/theta_e' */
  real32_T P_d;                        /* '<Root>/P' */
  real32_T I;                          /* '<Root>/I' */
  real_T theta_ref;                    /* '<Root>/theta_ref' */
  real32_T theta_output;               /* '<Root>/theta_output' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T tABC[3];                    /* '<Root>/tABC' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void Robust_initialize(void);
extern void Robust_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Scope' : Unused code path elimination
 * Block '<S7>/Scope' : Unused code path elimination
 * Block '<S1>/Rate Transition12' : Eliminated since input and output rates are identical
 * Block '<S2>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S2>/Data Type Conversion4' : Eliminate redundant data type conversion
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
 * hilite_system('positionloop1/Robust control')    - opens subsystem positionloop1/Robust control
 * hilite_system('positionloop1/Robust control/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'positionloop1'
 * '<S1>'   : 'positionloop1/Robust control'
 * '<S2>'   : 'positionloop1/Robust control/currloop'
 * '<S3>'   : 'positionloop1/Robust control/currloop/AntiPark'
 * '<S4>'   : 'positionloop1/Robust control/currloop/Clark'
 * '<S5>'   : 'positionloop1/Robust control/currloop/Park'
 * '<S6>'   : 'positionloop1/Robust control/currloop/SVPWM'
 * '<S7>'   : 'positionloop1/Robust control/currloop/idq_Controller'
 * '<S8>'   : 'positionloop1/Robust control/currloop/SVPWM/AntiClark'
 * '<S9>'   : 'positionloop1/Robust control/currloop/SVPWM/ei_t'
 * '<S10>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1'
 * '<S11>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2'
 * '<S12>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Anti-windup'
 * '<S13>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/D Gain'
 * '<S14>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/External Derivative'
 * '<S15>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Filter'
 * '<S16>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Filter ICs'
 * '<S17>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/I Gain'
 * '<S18>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Ideal P Gain'
 * '<S19>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S20>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Integrator'
 * '<S21>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Integrator ICs'
 * '<S22>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/N Copy'
 * '<S23>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/N Gain'
 * '<S24>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/P Copy'
 * '<S25>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Parallel P Gain'
 * '<S26>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Reset Signal'
 * '<S27>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Saturation'
 * '<S28>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Saturation Fdbk'
 * '<S29>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Sum'
 * '<S30>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Sum Fdbk'
 * '<S31>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tracking Mode'
 * '<S32>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tracking Mode Sum'
 * '<S33>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tsamp - Integral'
 * '<S34>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tsamp - Ngain'
 * '<S35>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/postSat Signal'
 * '<S36>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/preSat Signal'
 * '<S37>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S38>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S39>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S40>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/D Gain/Disabled'
 * '<S41>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/External Derivative/Disabled'
 * '<S42>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Filter/Disabled'
 * '<S43>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Filter ICs/Disabled'
 * '<S44>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/I Gain/External Parameters'
 * '<S45>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S46>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S47>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Integrator/Discrete'
 * '<S48>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S49>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S50>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/N Gain/Disabled'
 * '<S51>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/P Copy/Disabled'
 * '<S52>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Parallel P Gain/External Parameters'
 * '<S53>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Reset Signal/Disabled'
 * '<S54>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Saturation/Enabled'
 * '<S55>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S56>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Sum/Sum_PI'
 * '<S57>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S58>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tracking Mode/Disabled'
 * '<S59>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S60>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S61>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S62>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S63>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S64>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Anti-windup'
 * '<S65>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/D Gain'
 * '<S66>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/External Derivative'
 * '<S67>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Filter'
 * '<S68>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Filter ICs'
 * '<S69>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/I Gain'
 * '<S70>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Ideal P Gain'
 * '<S71>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S72>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Integrator'
 * '<S73>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Integrator ICs'
 * '<S74>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/N Copy'
 * '<S75>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/N Gain'
 * '<S76>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/P Copy'
 * '<S77>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Parallel P Gain'
 * '<S78>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Reset Signal'
 * '<S79>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Saturation'
 * '<S80>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Saturation Fdbk'
 * '<S81>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Sum'
 * '<S82>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Sum Fdbk'
 * '<S83>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tracking Mode'
 * '<S84>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tracking Mode Sum'
 * '<S85>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tsamp - Integral'
 * '<S86>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tsamp - Ngain'
 * '<S87>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/postSat Signal'
 * '<S88>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/preSat Signal'
 * '<S89>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
 * '<S90>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S91>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S92>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/D Gain/Disabled'
 * '<S93>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/External Derivative/Disabled'
 * '<S94>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Filter/Disabled'
 * '<S95>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Filter ICs/Disabled'
 * '<S96>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/I Gain/External Parameters'
 * '<S97>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S98>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S99>'  : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Integrator/Discrete'
 * '<S100>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S101>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/N Copy/Disabled wSignal Specification'
 * '<S102>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/N Gain/Disabled'
 * '<S103>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/P Copy/Disabled'
 * '<S104>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Parallel P Gain/External Parameters'
 * '<S105>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Reset Signal/Disabled'
 * '<S106>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Saturation/Enabled'
 * '<S107>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S108>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Sum/Sum_PI'
 * '<S109>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S110>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tracking Mode/Disabled'
 * '<S111>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S112>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S113>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S114>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S115>' : 'positionloop1/Robust control/currloop/idq_Controller/PID Controller2/preSat Signal/Forward_Path'
 */
#endif                                 /* Robust_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
