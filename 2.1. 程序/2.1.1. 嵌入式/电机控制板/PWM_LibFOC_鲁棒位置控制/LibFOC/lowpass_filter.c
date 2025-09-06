/*
 * lowpass_filter.c
 *
 *  Created on: Jun 18, 2023
 *      Author: hht
 */
#include "lowpass_filter.h"
float LowPassFilter_operator(float x, struct LowPassFilter* filter)
{
	float y = filter->alpha * filter->y_prev + (1.0f - filter->alpha) * x;
	filter->y_prev = y;
	return y;
}
