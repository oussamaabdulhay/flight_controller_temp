#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/FloatMsg.hpp"
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