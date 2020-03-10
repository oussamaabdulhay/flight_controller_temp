#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "cmath"
#include "FloatMsg.hpp"
#include "ControlSystem.hpp"

class Saturation : public MsgEmitter, public MsgReceiver {

private:
    float _clip_value;

public:
    void receiveMsgData(DataMessage*, int);
    void clip(float);
    Saturation(float);
    ~Saturation();
};