#pragma once
#include "DataMessage.hpp"

class SwitchOutMsg : public DataMessage{

private:
    msg_type _type;
    float _data;

public:

    msg_type getType();
	const int getSize();
    void setSwitchOutMsg(float);
    float getSwitchOutMsg();
    
    SwitchOutMsg();
    ~SwitchOutMsg();
};