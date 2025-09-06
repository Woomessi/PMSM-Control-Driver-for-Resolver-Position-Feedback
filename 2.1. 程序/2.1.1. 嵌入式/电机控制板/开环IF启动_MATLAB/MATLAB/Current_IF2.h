/*
 * File: Current_IF2.h
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

#ifndef Current_IF2_h_
#define Current_IF2_h_
#ifndef Current_IF2_COMMON_INCLUDES_
#define Current_IF2_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* Current_IF2_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define Current_IF2_M                  (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Motor_state;                  /* '<S1>/Chart' */
  real32_T Merge1;                     /* '<S1>/Merge1' */
  real32_T Integrator_DSTATE;          /* '<S48>/Integrator' */
  real32_T Integrator_DSTATE_i;        /* '<S100>/Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S7>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator1_DSTAT_g;/* '<S6>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S6>/Discrete-Time Integrator' */
  uint16_T temporalCounter_i1;         /* '<S1>/Chart' */
  uint8_T is_active_c3_Current_IF2;    /* '<S1>/Chart' */
  uint8_T is_c3_Current_IF2;           /* '<S1>/Chart' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S118>/Constant'
   */
  real32_T Constant_Value;

  /* Computed Parameter: Gain_Gain
   * Referenced by: '<S118>/Gain'
   */
  real32_T Gain_Gain;

  /* Computed Parameter: PWM_HalfPeriod_Gain
   * Referenced by: '<S118>/PWM_HalfPeriod'
   */
  real32_T PWM_HalfPeriod_Gain;
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T v_bus;                      /* '<Root>/v_bus' */
  real32_T Motor_OnOff;                /* '<Root>/Motor_OnOff' */
  real32_T P_current;                  /* '<Root>/P_current' */
  real32_T I_current;                  /* '<Root>/I_current' */
  real32_T OPSpd;                      /* '<Root>/OPSpd' */
  real32_T iq_ref;                     /* '<Root>/iq_ref' */
  real32_T OPTime;                     /* '<Root>/OPTime' */
  real32_T iq_ref0;                    /* '<Root>/iq_ref0' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T tABC[3];                    /* '<Root>/tABC' */
  real32_T id;                         /* '<Root>/id' */
  real32_T iq;                         /* '<Root>/iq' */
} ExtY;

/* Type definition for custom storage class: Struct */
typedef struct motor_tag {
  real32_T Pn;                         /* Referenced by:
                                        * '<S6>/Gain'
                                        * '<S7>/Gain'
                                        */
} motor_type;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T ThetaOpen;             /* '<S1>/Merge' */

/* Model entry point functions */
extern void Current_IF2_initialize(void);
extern void Current_IF2_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Struct */
extern motor_type motor;

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Scope' : Unused code path elimination
 * Block '<S9>/Scope' : Unused code path elimination
 * Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion4' : Eliminate redundant data type conversion
 * Block '<S1>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition4' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition6' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition7' : Eliminated since input and output rates are identical
 * Block '<S1>/Rate Transition8' : Eliminated since input and output rates are identical
 * Block '<S117>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S117>/Data Type Conversion4' : Eliminate redundant data type conversion
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
 * hilite_system('IF/Current_IF2')    - opens subsystem IF/Current_IF2
 * hilite_system('IF/Current_IF2/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'IF'
 * '<S1>'   : 'IF/Current_IF2'
 * '<S2>'   : 'IF/Current_IF2/Chart'
 * '<S3>'   : 'IF/Current_IF2/Clark'
 * '<S4>'   : 'IF/Current_IF2/If Action Subsystem'
 * '<S5>'   : 'IF/Current_IF2/If Action Subsystem1'
 * '<S6>'   : 'IF/Current_IF2/If Action Subsystem2'
 * '<S7>'   : 'IF/Current_IF2/If Action Subsystem3'
 * '<S8>'   : 'IF/Current_IF2/Park'
 * '<S9>'   : 'IF/Current_IF2/idq_Controller'
 * '<S10>'  : 'IF/Current_IF2/svpwm'
 * '<S11>'  : 'IF/Current_IF2/idq_Controller/PID Controller1'
 * '<S12>'  : 'IF/Current_IF2/idq_Controller/PID Controller2'
 * '<S13>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Anti-windup'
 * '<S14>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/D Gain'
 * '<S15>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/External Derivative'
 * '<S16>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Filter'
 * '<S17>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Filter ICs'
 * '<S18>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/I Gain'
 * '<S19>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Ideal P Gain'
 * '<S20>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S21>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Integrator'
 * '<S22>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Integrator ICs'
 * '<S23>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/N Copy'
 * '<S24>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/N Gain'
 * '<S25>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/P Copy'
 * '<S26>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Parallel P Gain'
 * '<S27>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Reset Signal'
 * '<S28>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Saturation'
 * '<S29>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Saturation Fdbk'
 * '<S30>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Sum'
 * '<S31>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Sum Fdbk'
 * '<S32>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tracking Mode'
 * '<S33>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tracking Mode Sum'
 * '<S34>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tsamp - Integral'
 * '<S35>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tsamp - Ngain'
 * '<S36>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/postSat Signal'
 * '<S37>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/preSat Signal'
 * '<S38>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S39>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S40>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S41>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/D Gain/Disabled'
 * '<S42>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/External Derivative/Disabled'
 * '<S43>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Filter/Disabled'
 * '<S44>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Filter ICs/Disabled'
 * '<S45>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/I Gain/External Parameters'
 * '<S46>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S47>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Integrator/Discrete'
 * '<S49>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S50>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S51>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/N Gain/Disabled'
 * '<S52>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/P Copy/Disabled'
 * '<S53>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Parallel P Gain/External Parameters'
 * '<S54>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Reset Signal/Disabled'
 * '<S55>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Saturation/Enabled'
 * '<S56>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S57>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Sum/Sum_PI'
 * '<S58>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S59>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tracking Mode/Disabled'
 * '<S60>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S61>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S62>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S63>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S64>'  : 'IF/Current_IF2/idq_Controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S65>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Anti-windup'
 * '<S66>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/D Gain'
 * '<S67>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/External Derivative'
 * '<S68>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Filter'
 * '<S69>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Filter ICs'
 * '<S70>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/I Gain'
 * '<S71>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Ideal P Gain'
 * '<S72>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S73>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Integrator'
 * '<S74>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Integrator ICs'
 * '<S75>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/N Copy'
 * '<S76>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/N Gain'
 * '<S77>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/P Copy'
 * '<S78>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Parallel P Gain'
 * '<S79>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Reset Signal'
 * '<S80>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Saturation'
 * '<S81>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Saturation Fdbk'
 * '<S82>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Sum'
 * '<S83>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Sum Fdbk'
 * '<S84>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Tracking Mode'
 * '<S85>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Tracking Mode Sum'
 * '<S86>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Tsamp - Integral'
 * '<S87>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Tsamp - Ngain'
 * '<S88>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/postSat Signal'
 * '<S89>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/preSat Signal'
 * '<S90>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
 * '<S91>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S92>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S93>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/D Gain/Disabled'
 * '<S94>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/External Derivative/Disabled'
 * '<S95>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Filter/Disabled'
 * '<S96>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Filter ICs/Disabled'
 * '<S97>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/I Gain/External Parameters'
 * '<S98>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S99>'  : 'IF/Current_IF2/idq_Controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S100>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Integrator/Discrete'
 * '<S101>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S102>' : 'IF/Current_IF2/idq_Controller/PID Controller2/N Copy/Disabled wSignal Specification'
 * '<S103>' : 'IF/Current_IF2/idq_Controller/PID Controller2/N Gain/Disabled'
 * '<S104>' : 'IF/Current_IF2/idq_Controller/PID Controller2/P Copy/Disabled'
 * '<S105>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Parallel P Gain/External Parameters'
 * '<S106>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Reset Signal/Disabled'
 * '<S107>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Saturation/Enabled'
 * '<S108>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S109>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Sum/Sum_PI'
 * '<S110>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S111>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Tracking Mode/Disabled'
 * '<S112>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S113>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S114>' : 'IF/Current_IF2/idq_Controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S115>' : 'IF/Current_IF2/idq_Controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S116>' : 'IF/Current_IF2/idq_Controller/PID Controller2/preSat Signal/Forward_Path'
 * '<S117>' : 'IF/Current_IF2/svpwm/AntiPark1'
 * '<S118>' : 'IF/Current_IF2/svpwm/SVPWM1'
 * '<S119>' : 'IF/Current_IF2/svpwm/SVPWM1/AntiClark'
 * '<S120>' : 'IF/Current_IF2/svpwm/SVPWM1/ei_t'
 */
#endif                                 /* Current_IF2_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
