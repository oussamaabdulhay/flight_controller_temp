#pragma once

#include <cstdint>
#include "DataMessage.hpp"

class TestMsg : public DataMessage
{

public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new TestMsg(*this); }
    int data;
};