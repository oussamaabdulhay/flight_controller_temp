#pragma once
#include "DataMessage.hpp"
#include <vector>

class VectorDoubleMsg : public DataMessage
{
public:

    VectorDoubleMsg();
    ~VectorDoubleMsg();

    msg_type getType();
    const int getSize();

    std::vector<double> data;
    
};