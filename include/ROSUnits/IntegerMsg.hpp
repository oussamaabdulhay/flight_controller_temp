#pragma once
#include "DataMessage.hpp"

class IntegerMsg : public DataMessage
{
public:

    IntegerMsg();
    ~IntegerMsg();
    DataMessage* Clone(){ return new IntegerMsg(*this); }
    msg_type getType();
    const int getSize();

    int data;
    
};