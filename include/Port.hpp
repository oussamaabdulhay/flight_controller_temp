#pragma once
class Port;
#include "Block.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"

class Port : public MsgReceiver, public MsgEmitter{

private:
    int _id;
    Block* _block;

public:
    
    virtual void receiveMsgData(DataMessage* t_msg) = 0;
    virtual void receiveMsgData(DataMessage* t_msg, int channel_id) = 0;
    int getID();
    Port(int t_id, Block* t_block);
    ~Port();
};