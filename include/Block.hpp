#pragma once
#include "common_types.hpp"
#include <iostream>
#include "DataMessage.hpp"
#include "MsgReceiver.hpp"

class Block : public MsgReceiver{   //TODO check MsgEmitter MsgReceiver

    public:
        
        virtual block_id getID() = 0;
        virtual block_type getType() = 0;
        virtual void switchIn(DataMessage*) = 0;
        virtual DataMessage* switchOut() = 0;
        virtual DataMessage* runTask(DataMessage*) = 0; 
        virtual void receiveMsgData(DataMessage* t_msg) = 0;

        Block();
        ~Block();
};