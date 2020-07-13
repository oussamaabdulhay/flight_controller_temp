#include "ButterFilter_120hz.hpp"

float ButterFilter_120hz::perform(float x) {
	float y = -coeff_120Hz_2nd_butter_5hz[0] * prev_y - coeff_120Hz_2nd_butter_5hz[1] * prev2_y + coeff_120Hz_2nd_butter_5hz[2] * x + coeff_120Hz_2nd_butter_5hz[3] * prev_x + coeff_120Hz_2nd_butter_5hz[4] * prev2_x;
	prev2_y = prev_y;
	prev_y = y;
	prev2_x = prev_x;
	prev_x = x;
	return y;
}