#pragma once
#include "DataMessage.hpp"
#include "Pose.hpp"

class PoseMsg : public DataMessage{

public:

    PoseMsg();
    ~PoseMsg();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new PoseMsg(*this); }
    Pose pose;
};