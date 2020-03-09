#pragma once
#include "DataMessage.hpp"

class DoublePointerMsg : public DataMessage
{
public:

    DoublePointerMsg();
    ~DoublePointerMsg();

    msg_type getType();
    const int getSize();

    double* data_ptr;
    
};