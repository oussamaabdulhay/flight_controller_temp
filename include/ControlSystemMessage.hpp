#pragma once
#include "common_srv/DataMessage.hpp"
#include "Block.hpp"

class ControlSystemMessage : public DataMessage{

private:
    Block* _to_add;
    msg_type _type;
    control_system_msg_type _control_system_msg_type;
    
public:
    msg_type getType();
    const int getSize();
    Block* getBlockToAdd();
    control_system_msg_type getControlSystemMsgType();

    ControlSystemMessage();

    void setControlSystemMessage(control_system_msg_type, Block*);

    ~ControlSystemMessage();
};