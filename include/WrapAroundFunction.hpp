#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3D.hpp"
#include "Vector3DMessage.hpp"

class WrapAroundFunction : public msg_emitter, public msg_receiver
{
    double min_val,max_val,span;

public:
    
    void assignParametersRange(double t_min_val,double t_max_val);
    double wrapAround(double input);
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage* rec_msg, int ch);
};