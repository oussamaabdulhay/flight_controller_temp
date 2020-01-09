#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class AccelerationMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();
	DataMessage* Clone(){ return new AccelerationMsg(*this); }
	float ddx, ddy, ddz;	
};