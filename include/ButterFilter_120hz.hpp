#pragma once
#include "common_types.hpp"

class ButterFilter_120hz {

private:
	float prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;
	const float coeff_120Hz_2nd_butter_5hz[5] = { -1.279632424997809,0.477592250072517,0.049489956268677,0.098979912537354,0.049489956268677 };

public:
	float perform(float input);

};
