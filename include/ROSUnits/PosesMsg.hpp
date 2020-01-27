#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"
#include "Poses.hpp"

class PosesMsg : public DataMessage
{

public:

    PosesMsg();
    ~PosesMsg();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new PosesMsg(*this); }
    Poses p;
};