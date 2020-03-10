#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "cmath"
#include "FloatMsg.hpp"
#include "ControlSystem.hpp"

class Saturation : public msg_emitter, public msg_receiver {

private:
    float _clip_value;

public:
    void receive_msg_data(DataMessage*, int);
    void clip(float);
    Saturation(float);
    ~Saturation();
};