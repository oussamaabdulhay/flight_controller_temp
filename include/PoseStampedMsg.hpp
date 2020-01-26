#pragma once
#include "DataMessage.hpp"
#include "PoseStamped.hpp"

class PoseStampedMsg : public DataMessage{

public:

    PoseStampedMsg();
    ~PoseStampedMsg();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new PoseStampedMsg(*this); }

    PoseStamped pose;
};