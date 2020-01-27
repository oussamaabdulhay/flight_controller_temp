#pragma once
#include "DataMessage.hpp"

class EmptyMsg : public DataMessage
{
public:

    EmptyMsg();
    ~EmptyMsg();
    DataMessage* Clone(){ return new EmptyMsg(*this); }
    msg_type getType();
    const int getSize();    
};