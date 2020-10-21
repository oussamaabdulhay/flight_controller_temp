#pragma once
#include "Port.hpp"

class InputPort : public Port{

private:
    DataMessage* _data;
    int _id;
    Block* _block;

public:
    
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int channel_id);
    InputPort(int t_id, Block* t_block);
    ~InputPort();
};