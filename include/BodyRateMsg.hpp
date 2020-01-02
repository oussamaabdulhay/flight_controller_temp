#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class BodyRateMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();

	float x, y, z;	
};