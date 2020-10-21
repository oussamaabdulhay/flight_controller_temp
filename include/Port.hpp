#pragma once
class Port;
#include "Block.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "common_srv/MsgEmitter.hpp"

class Port : public MsgReceiver, public MsgEmitter{

private:
    DataMessage* _data;
    int _id;
    Block* _block;

public:
    
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int channel_id);
    int getID();
    Port(int t_id, Block* t_block);
    ~Port();
};