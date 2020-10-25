#pragma once
#include <stdint.h>
#include "common_srv/common_types.hpp"

class PID_parameters{

public:
	float kp=0, ki=0, kd=0, kdd=0, anti_windup=0, dt = -1;
	block_id id = block_id::NULL_ID;
	uint8_t en_pv_derivation=0;	   
};


class PID_data {

public:

	float err, pv_first, pv_second;
	   
};