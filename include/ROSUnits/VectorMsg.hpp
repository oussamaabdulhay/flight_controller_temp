#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"

class VectorMsg : public DataMessage{

public:

    VectorMsg();
    ~VectorMsg();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new VectorMsg(*this); }
    Vector3D<float> p1, p2;
};