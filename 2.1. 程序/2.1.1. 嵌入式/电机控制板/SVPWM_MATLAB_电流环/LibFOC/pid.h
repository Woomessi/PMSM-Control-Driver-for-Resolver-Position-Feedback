/*
 * pid.h
 *
 *  Created on: Jun 18, 2023
 *      Author: hht
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "math.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _SQRT2 1.41421356237f
#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f
#define M_PI		3.14159265358979323846f

struct PIDController{
	float P; //!< 比例增益(P环增益)
	float I; //!< 积分增益（I环增益）
	float D; //!< 微分增益（D环增益）
	float output_ramp; // PID控制器加速度限幅
	float limit;       // PID控制器输出限幅
  float error_prev; //!< 最后的跟踪误差值
  float output_prev;  //!< 最后一个 pid 输出值
  float integral_prev; //!< 最后一个积分分量值
	float Ts; //执行频率
};
float PID_operator(float error, struct PIDController* pid);
#endif /* INC_PID_H_ */
