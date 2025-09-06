/*
 * File: svpwm.h
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

#ifndef svpwm_h_
#define svpwm_h_
#ifndef svpwm_COMMON_INCLUDES_
#define svpwm_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* svpwm_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define svpwm_M                        (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Constant_Value
   * Referenced by: '<S2>/Constant'
   */
  real32_T Constant_Value;

  /* Computed Parameter: Gain_Gain
   * Referenced by: '<S2>/Gain'
   */
  real32_T Gain_Gain;

  /* Computed Parameter: PWM_HalfPeriod_Gain
   * Referenced by: '<S2>/PWM_HalfPeriod'
   */
  real32_T PWM_HalfPeriod_Gain;
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ud;                         /* '<Root>/ud' */
  real32_T uq;                         /* '<Root>/uq' */
  real32_T position_e;                 /* '<Root>/position_e' */
  real32_T v_bus;                      /* '<Root>/v_bus' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T tABC[3];                    /* '<Root>/tABC' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void svpwm_initialize(void);
extern void svpwm_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion4' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'svpwm'
 * '<S1>'   : 'svpwm/AntiPark1'
 * '<S2>'   : 'svpwm/SVPWM1'
 * '<S3>'   : 'svpwm/SVPWM1/AntiClark'
 * '<S4>'   : 'svpwm/SVPWM1/ei_t'
 */
#endif                                 /* svpwm_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
