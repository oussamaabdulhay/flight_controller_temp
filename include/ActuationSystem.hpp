#pragma once

#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "ControlSystemMessage.hpp"
#include "Actuator.hpp"
#include <vector>

class ActuationSystem : public MsgEmitter, public MsgReceiver{

public:

    virtual void receiveMsgData(DataMessage* t_msg) = 0;
    
    ActuationSystem(std::vector<Actuator*>) {};
};