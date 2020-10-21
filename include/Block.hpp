#pragma once
class Block;
#include "Port.hpp"
#include "common_types.hpp"
#include <iostream>
#include "common_srv/DataMessage.hpp"
#include "common_srv/MsgReceiver.hpp"

class Block : public MsgReceiver{   //TODO check MsgEmitter MsgReceiver

    public:
        
        virtual block_id getID() = 0;
        virtual block_type getType() = 0;
        virtual void switchIn(DataMessage*) = 0;
        virtual DataMessage* switchOut() = 0;
        virtual DataMessage* runTask(DataMessage*) = 0; 
        virtual void receiveMsgData(DataMessage* t_msg) = 0;
        virtual void process(DataMessage* t_msg, Port* t_port) = 0;
        Block();
        ~Block();
};