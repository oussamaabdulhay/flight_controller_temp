#pragma once
#include <stdint.h>
#include "common_types.hpp"

class SM_parameters{

public:
	double alpha1=0, alpha2=0, h1=0, h2=0;
	block_id id = block_id::NULL_ID;
};