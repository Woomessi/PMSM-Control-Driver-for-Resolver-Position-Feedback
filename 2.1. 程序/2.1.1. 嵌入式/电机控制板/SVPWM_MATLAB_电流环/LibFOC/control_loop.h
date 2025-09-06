#include "lowpass_filter.h"
#include "pid.h"
#include "math.h"
#include "stm32g4xx_hal.h"


#define M_PI		3.14159265358979323846f

float velocityOpenloop(float target_velocity, float Ts);

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE);

void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE);

float cal_Iq(float current_a,float current_b,float position_e);

float cal_Id(float current_a,float current_b,float position_e);

float _electricalAngle(float position);

float getAngle(float position);

float _normalizeAngle(float angle);

float cal_angular_vel(float angle_now, float Ts);

float positionloop(float Kp, float target_pos, float feedback_pos, float limit);

float velocityloop_closed(float target_vel, float angle_now, float Ts, struct LowPassFilter filter, struct PIDController pid);

float velocityloop_closed_resolver(float target_vel, float feedback_vel, struct LowPassFilter filter, struct PIDController pid);
