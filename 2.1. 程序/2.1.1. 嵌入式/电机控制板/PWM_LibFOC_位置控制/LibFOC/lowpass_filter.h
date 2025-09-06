/*
 * lowpass_filter.h
 *
 *  Created on: Dec 23, 2023
 *      Author: haotian
 */

#ifndef INC_LOWPASS_FILTER_H_
#define INC_LOWPASS_FILTER_H_

struct LowPassFilter{
	float alpha;
	float y_prev; //!< 上一个循环中的过滤后的值
};

float LowPassFilter_operator(float x, struct LowPassFilter* filter);

#endif /* INC_LOWPASS_FILTER_H_ */
