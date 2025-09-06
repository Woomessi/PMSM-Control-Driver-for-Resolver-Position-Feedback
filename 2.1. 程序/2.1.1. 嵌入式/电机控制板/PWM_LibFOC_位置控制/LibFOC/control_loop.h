#ifndef __CONTROL_LOOP_H__
#define __CONTROL_LOOP_H__

#include "lowpass_filter.h"
#include "pid.h"
#include "math.h"
#include "stm32g4xx_hal.h"

/********************************/
/********** 常量宏定义 **********/
/********************************/

#define _SQRT3 1.73205080757f   // sqrt(3)
#define _SQRT3_2 0.86602540378f // sqrt(3)/2
#define _1_SQRT3 0.57735026919f // 1/sqrt(3)
#define _2_SQRT3 1.15470053838f // 2/sqrt(3)
//#define PI		3.14159265358979323846f

/* 三阶控制器系数 */
#define coefficient_a1 -2.228f
#define coefficient_a2 1.471f
#define coefficient_a3 -0.2429f
#define coefficient_b1 0.9757f
#define coefficient_b2 -1.951f
#define coefficient_b3 0.9751f

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE);

void setPWM(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE);

float cal_Iq(float current_a,float current_b,float position_e);

float cal_Id(float current_a,float current_b,float position_e);

float _electricalAngle(float position);

float getFullAngle(float position);

float _normalizeAngle(float angle);

void Sin_CORDIC_INT(void);

void Cos_CORDIC_INT(void);

float sin_f(float angles);

float cos_f(float angles);

float thirdOrderController (float input);

#endif
