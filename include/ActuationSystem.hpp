#pragma once

#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "Actuator.hpp"
#include <vector>

class ActuationSystem : public MsgEmitter, public MsgReceiver{

public:

    virtual void receiveMsgData(DataMessage* t_msg) = 0;
    
    ActuationSystem(std::vector<Actuator*>) {};
};