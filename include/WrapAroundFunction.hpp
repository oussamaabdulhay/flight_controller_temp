#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"

class WrapAroundFunction : public MsgEmitter, public MsgReceiver
{
    double min_val,max_val,span;

public:
    
    void assignParametersRange(double t_min_val,double t_max_val);
    double wrapAround(double input);
    void receiveMsgData(DataMessage*);
    void receiveMsgData(DataMessage* rec_msg, int ch);
};