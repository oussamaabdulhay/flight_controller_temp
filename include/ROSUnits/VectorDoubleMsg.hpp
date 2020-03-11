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

    std::vector<double> data {0};   //Adding at least one element to the vector makes it possible to assign a new value
                                    //to the first element by using "vector[0] = n;" otherwise, you would have to do 
                                    //vector.push_back() to be able to change the first element.
    
};