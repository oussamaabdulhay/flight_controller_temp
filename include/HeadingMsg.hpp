#pragma once 

#include "DataMessage.hpp"
#include "common_types.hpp"

class HeadingMsg : public DataMessage
{
public:

	msg_type getType();
	const int getSize();
	DataMessage* Clone(){ return new HeadingMsg(*this); }

	float yaw=0;	 
};