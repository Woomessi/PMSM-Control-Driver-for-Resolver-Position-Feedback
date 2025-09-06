#include "control_loop.h"

extern float DIR;
extern float PP;
extern float angle_0;
extern float angle_prev;
extern float shaft_angle;
extern float zero_electric_angle;
extern float full_rotations;
extern float dc_a, dc_b, dc_c;
extern int period;
extern float voltage_limit;
extern float voltage_power_supply;

/* 速度开环 */
float velocityOpenloop(float target_velocity, float Ts){
	
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
	
  /* 关节电角度（rad）*/
  float angle_el = _electricalAngle(shaft_angle);
	
	return angle_el;
}

void setPhaseVoltage(float Uq,float Ud, float angle_el, TIM_TypeDef * TIM_BASE) {
	
	Uq = _constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
//  angle_el = _normalizeAngle(angle_el);
	angle_el = _normalizeAngle(angle_el + zero_electric_angle);

  // 帕克逆变换(不考虑Ud）
//  float Ualpha =  -Uq*sin(angle_el);
//  float Ubeta =   Uq*cos(angle_el);
  
	// 帕克逆变换(考虑Ud)
	float Ualpha = Ud*cos(angle_el) - Uq*sin(angle_el);
  float Ubeta = Ud*sin(angle_el) + Uq*cos(angle_el);
	
  // 克拉克逆变换
  float Ua = Ualpha + voltage_power_supply/2;
  float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc,TIM_BASE);
}

void setPwm(float Ua, float Ub, float Uc, TIM_TypeDef * TIM_BASE) {

  // 限制上限
	Ua = _constrain(Ua, 0.0f, voltage_power_supply);
	Ub = _constrain(Ub, 0.0f, voltage_power_supply);
	Uc = _constrain(Uc, 0.0f, voltage_power_supply);
	
	// 计算占空比
	// 限制占空比从0到1
	float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
	float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
	float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

	//写入PWM到PWM 0 1 2 通道
	TIM_BASE->CCR1 = (float) roundf(dc_a*period);
	TIM_BASE->CCR2 = (float) roundf(dc_b*period);
	TIM_BASE->CCR3 = (float) roundf(dc_c*period);
}

float cal_Iq(float current_a,float current_b,float position_e)
{
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = cos(position_e);
  float st = sin(position_e);
  
	float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

float cal_Id(float current_a,float current_b,float position_e)
{
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  float ct = cos(position_e);
  float st = sin(position_e);
 
	float I_d = I_alpha * ct + I_beta * st;
  return I_d;
}

float _electricalAngle(float position){
  return  _normalizeAngle(((float)(DIR * PP) * position) - zero_electric_angle);
}

/* 带圈数的角度 */
float getAngle(float position)
{
    float d_angle = position - angle_0;
    //计算旋转的总圈数
    //通过判断角度变化是否大于80%的一圈(0.8f*6.28318530718f)来判断是否发生了溢出，如果发生了，则将full_rotations增加1（如果d_angle小于0）或减少1（如果d_angle大于0）。
    if(fabs(d_angle) > (0.8f*6.28318530718f) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_0 = position;
    return (float)full_rotations * 6.28318530718f + angle_0;
}

/* 角度归一化 */
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*M_PI);   //取余运算用于归一化
  return a >= 0 ? a : (a + 2*M_PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。
  //可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2M_PI 的符号相反。
  //也就是说，如果 angle 的值小于 0 且 _2M_PI 的值为正数，则 fmod(angle, _2M_PI) 的余数将为负数。
  //例如，当 angle 的值为 -M_PI/2，_2M_PI 的值为 2M_PI 时，fmod(angle, _2M_PI) 将返回一个负数。
  //在这种情况下，可以通过将负数的余数加上 _2M_PI 来将角度归一化到 [0, 2M_PI] 的范围内，以确保角度的值始终为正数。
}

/* 基于位置数据计算速度 */
float cal_angular_vel(float angle_now, float Ts){ // 0 <= angle_now <= 2*PI
    if (angle_prev < 0){
    	angle_prev = angle_now;
    	return 0;
    }
    float delta_angle = angle_now - angle_prev;
    if (delta_angle >= 1.6*M_PI){              // 如果角度变化量的绝对值超过了0.8倍的2PI，则认为发生了一圈的变化。
    	delta_angle -= 2*M_PI;
    }
    if (delta_angle <= -1.6*M_PI){             // angle_prev = 1.6*PI -> angle_now = 0*PI : 正向
        	delta_angle += 2*M_PI;
    }
    angle_prev = angle_now;
    return delta_angle/Ts;
}

/* 位置闭环 */
float positionloop(float Kp, float target_pos, float feedback_pos, float limit){

	  float angle_error = target_pos - feedback_pos;
    angle_error =_normalizeAngle(angle_error);
    if (angle_error > M_PI){
    	angle_error -= 2*M_PI;
    }
		float iq_ref = _constrain(Kp*(angle_error),-limit,limit);
		return iq_ref;
}

/* 速度闭环 （基于位置计算速度） */
float velocityloop_closed(float target_vel, float angle_now, float Ts, struct LowPassFilter filter, struct PIDController pid){
    float angular_vel=cal_angular_vel(angle_now, Ts);
    float filtered_vel=LowPassFilter_operator(angular_vel,&filter);
    float contorller_out=PID_operator(target_vel-filtered_vel,&pid);
	  return contorller_out;
}

/* 速度闭环 （基于旋转变压器） */
float velocityloop_closed_resolver(float target_vel, float feedback_vel, struct LowPassFilter filter, struct PIDController pid){
    float filtered_vel=LowPassFilter_operator(feedback_vel,&filter);
    float contorller_out=PID_operator(target_vel-filtered_vel,&pid);
	  return contorller_out;
}