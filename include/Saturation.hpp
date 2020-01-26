#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"
#include "ROSMsg.hpp"
#include "cmath"
#include "ControlSystemMessage.hpp"

class Saturation : public msg_emitter, public msg_receiver {

private:
    float _clip_value;
    ControlSystemMessage m_output_msg;

public:
    void receive_msg_data(DataMessage*);
    void clip(float);
    Saturation(float);
    ~Saturation();
};