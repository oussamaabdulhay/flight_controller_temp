#pragma once


//THIS IS A FILTER WITH FS = 200Hz and FC = 10Hz
class ButterFilter_Xsens{

private:
	float prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;
    const float coeff_200Hz_2nd_butter_10hz[5] = {-1.561018075800718, 0.641351538057563, 0.020083365564211, 0.040166731128423, 0.020083365564211};

public:
	float perform(float input);

};