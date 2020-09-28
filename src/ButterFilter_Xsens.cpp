#include "ButterFilter_Xsens.hpp"

float ButterFilter_Xsens::perform(float x) {
	float y = -coeff_200Hz_2nd_butter_50hz[0] * prev_y - coeff_200Hz_2nd_butter_50hz[1] * prev2_y + coeff_200Hz_2nd_butter_50hz[2] * x + coeff_200Hz_2nd_butter_50hz[3] * prev_x + coeff_200Hz_2nd_butter_50hz[4] * prev2_x;
	prev2_y = prev_y;
	prev_y = y;
	prev2_x = prev_x;
	prev_x = x;
	return y;
}