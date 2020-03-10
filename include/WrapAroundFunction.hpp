#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3D.hpp"
#include "Vector3DMessage.hpp"

class WrapAroundFunction : public MsgEmitter, public MsgReceiver
{
    double min_val,max_val,span;

public:
    
    void assignParametersRange(double t_min_val,double t_max_val);
    double wrapAround(double input);
    void receiveMsgData(DataMessage*);
    void receiveMsgData(DataMessage* rec_msg, int ch);
};