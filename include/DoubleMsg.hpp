#pragma once
#include "DataMessage.hpp"

class DoubleMsg : public DataMessage
{
public:

    DoubleMsg();
    ~DoubleMsg();

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new DoubleMsg(*this); }

    double data;
    
};