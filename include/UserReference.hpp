#pragma once
#include "common_types.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"
#include "FlightScenarioMessage.hpp"
#include "ControlSystem.hpp"

class UserReference : public MsgEmitter, public MsgReceiver{

private:
    UpdatePoseMessage _user_msg;

public:
    enum unicast_addresses {broadcast, x, y, z, yaw};
    virtual void receiveMsgData(DataMessage*) = 0;

    UserReference();
    ~UserReference();
};