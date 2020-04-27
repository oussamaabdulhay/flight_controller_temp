#pragma once

#include "common_srv/DataMessage.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"


class  Actuator {
    public:
        virtual bool initialize() = 0;
        virtual void applyCommand(int command) = 0;

        Actuator() {};
};
