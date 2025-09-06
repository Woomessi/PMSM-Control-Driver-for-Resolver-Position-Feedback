/*
 * File: transformation.h
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

#ifndef transformation_h_
#define transformation_h_
#ifndef transformation_COMMON_INCLUDES_
#define transformation_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* transformation_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T Integrator_DSTATE;          /* '<S95>/Integrator' */
  int16_T Integrator_DSTATE_e;         /* '<S43>/Integrator' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T IqRef;                      /* '<Root>/IqRef' */
  real32_T theta_e;                    /* '<Root>/theta_e' */
  real32_T P_current;                  /* '<Root>/P_current' */
  real32_T I_current;                  /* '<Root>/I_current' */
  real32_T IdRef;                      /* '<Root>/IdRef' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T id;                         /* '<Root>/id' */
  real32_T iq;                         /* '<Root>/iq' */
  real32_T ualpha;                     /* '<Root>/ualpha' */
  real32_T ubeta;                      /* '<Root>/ubeta' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void transformation_initialize(void);
extern void transformation_step(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Scope' : Unused code path elimination
 * Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion4' : Eliminate redundant data type conversion
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
 * hilite_system('positionloop/transformation')    - opens subsystem positionloop/transformation
 * hilite_system('positionloop/transformation/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'positionloop'
 * '<S1>'   : 'positionloop/transformation'
 * '<S2>'   : 'positionloop/transformation/AntiPark'
 * '<S3>'   : 'positionloop/transformation/Clark'
 * '<S4>'   : 'positionloop/transformation/Park'
 * '<S5>'   : 'positionloop/transformation/idq_Controller'
 * '<S6>'   : 'positionloop/transformation/idq_Controller/PID Controller1'
 * '<S7>'   : 'positionloop/transformation/idq_Controller/PID Controller2'
 * '<S8>'   : 'positionloop/transformation/idq_Controller/PID Controller1/Anti-windup'
 * '<S9>'   : 'positionloop/transformation/idq_Controller/PID Controller1/D Gain'
 * '<S10>'  : 'positionloop/transformation/idq_Controller/PID Controller1/External Derivative'
 * '<S11>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Filter'
 * '<S12>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Filter ICs'
 * '<S13>'  : 'positionloop/transformation/idq_Controller/PID Controller1/I Gain'
 * '<S14>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Ideal P Gain'
 * '<S15>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S16>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Integrator'
 * '<S17>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Integrator ICs'
 * '<S18>'  : 'positionloop/transformation/idq_Controller/PID Controller1/N Copy'
 * '<S19>'  : 'positionloop/transformation/idq_Controller/PID Controller1/N Gain'
 * '<S20>'  : 'positionloop/transformation/idq_Controller/PID Controller1/P Copy'
 * '<S21>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Parallel P Gain'
 * '<S22>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Reset Signal'
 * '<S23>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Saturation'
 * '<S24>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Saturation Fdbk'
 * '<S25>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Sum'
 * '<S26>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Sum Fdbk'
 * '<S27>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tracking Mode'
 * '<S28>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tracking Mode Sum'
 * '<S29>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tsamp - Integral'
 * '<S30>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tsamp - Ngain'
 * '<S31>'  : 'positionloop/transformation/idq_Controller/PID Controller1/postSat Signal'
 * '<S32>'  : 'positionloop/transformation/idq_Controller/PID Controller1/preSat Signal'
 * '<S33>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S34>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S35>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S36>'  : 'positionloop/transformation/idq_Controller/PID Controller1/D Gain/Disabled'
 * '<S37>'  : 'positionloop/transformation/idq_Controller/PID Controller1/External Derivative/Disabled'
 * '<S38>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Filter/Disabled'
 * '<S39>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Filter ICs/Disabled'
 * '<S40>'  : 'positionloop/transformation/idq_Controller/PID Controller1/I Gain/External Parameters'
 * '<S41>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S42>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S43>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Integrator/Discrete'
 * '<S44>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S45>'  : 'positionloop/transformation/idq_Controller/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S46>'  : 'positionloop/transformation/idq_Controller/PID Controller1/N Gain/Disabled'
 * '<S47>'  : 'positionloop/transformation/idq_Controller/PID Controller1/P Copy/Disabled'
 * '<S48>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Parallel P Gain/External Parameters'
 * '<S49>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Reset Signal/Disabled'
 * '<S50>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Saturation/Enabled'
 * '<S51>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S52>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Sum/Sum_PI'
 * '<S53>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S54>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tracking Mode/Disabled'
 * '<S55>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S56>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S57>'  : 'positionloop/transformation/idq_Controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S58>'  : 'positionloop/transformation/idq_Controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S59>'  : 'positionloop/transformation/idq_Controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S60>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Anti-windup'
 * '<S61>'  : 'positionloop/transformation/idq_Controller/PID Controller2/D Gain'
 * '<S62>'  : 'positionloop/transformation/idq_Controller/PID Controller2/External Derivative'
 * '<S63>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Filter'
 * '<S64>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Filter ICs'
 * '<S65>'  : 'positionloop/transformation/idq_Controller/PID Controller2/I Gain'
 * '<S66>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Ideal P Gain'
 * '<S67>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S68>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Integrator'
 * '<S69>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Integrator ICs'
 * '<S70>'  : 'positionloop/transformation/idq_Controller/PID Controller2/N Copy'
 * '<S71>'  : 'positionloop/transformation/idq_Controller/PID Controller2/N Gain'
 * '<S72>'  : 'positionloop/transformation/idq_Controller/PID Controller2/P Copy'
 * '<S73>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Parallel P Gain'
 * '<S74>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Reset Signal'
 * '<S75>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Saturation'
 * '<S76>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Saturation Fdbk'
 * '<S77>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Sum'
 * '<S78>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Sum Fdbk'
 * '<S79>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Tracking Mode'
 * '<S80>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Tracking Mode Sum'
 * '<S81>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Tsamp - Integral'
 * '<S82>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Tsamp - Ngain'
 * '<S83>'  : 'positionloop/transformation/idq_Controller/PID Controller2/postSat Signal'
 * '<S84>'  : 'positionloop/transformation/idq_Controller/PID Controller2/preSat Signal'
 * '<S85>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
 * '<S86>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S87>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S88>'  : 'positionloop/transformation/idq_Controller/PID Controller2/D Gain/Disabled'
 * '<S89>'  : 'positionloop/transformation/idq_Controller/PID Controller2/External Derivative/Disabled'
 * '<S90>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Filter/Disabled'
 * '<S91>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Filter ICs/Disabled'
 * '<S92>'  : 'positionloop/transformation/idq_Controller/PID Controller2/I Gain/External Parameters'
 * '<S93>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S94>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S95>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Integrator/Discrete'
 * '<S96>'  : 'positionloop/transformation/idq_Controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S97>'  : 'positionloop/transformation/idq_Controller/PID Controller2/N Copy/Disabled wSignal Specification'
 * '<S98>'  : 'positionloop/transformation/idq_Controller/PID Controller2/N Gain/Disabled'
 * '<S99>'  : 'positionloop/transformation/idq_Controller/PID Controller2/P Copy/Disabled'
 * '<S100>' : 'positionloop/transformation/idq_Controller/PID Controller2/Parallel P Gain/External Parameters'
 * '<S101>' : 'positionloop/transformation/idq_Controller/PID Controller2/Reset Signal/Disabled'
 * '<S102>' : 'positionloop/transformation/idq_Controller/PID Controller2/Saturation/Enabled'
 * '<S103>' : 'positionloop/transformation/idq_Controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S104>' : 'positionloop/transformation/idq_Controller/PID Controller2/Sum/Sum_PI'
 * '<S105>' : 'positionloop/transformation/idq_Controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S106>' : 'positionloop/transformation/idq_Controller/PID Controller2/Tracking Mode/Disabled'
 * '<S107>' : 'positionloop/transformation/idq_Controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S108>' : 'positionloop/transformation/idq_Controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S109>' : 'positionloop/transformation/idq_Controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S110>' : 'positionloop/transformation/idq_Controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S111>' : 'positionloop/transformation/idq_Controller/PID Controller2/preSat Signal/Forward_Path'
 */
#endif                                 /* transformation_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
