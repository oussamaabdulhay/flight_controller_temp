#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class VelocityMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();
	DataMessage* Clone(){ return new VelocityMsg(*this); }
	float dx, dy, dz;	
};