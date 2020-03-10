// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#pragma once

#include <iostream>
#include "common_types.hpp"
#include "DataMessage.hpp"
const int msg_broadcast_channel=-1;
class MsgReceiver {
private:
    msg_type _msg_type;
public:
    MsgReceiver();
    void assignMsgTypeID(msg_type i);
    msg_type getMsgTypeID();
    virtual void receiveMsgData(DataMessage* t_msg);
    virtual void receiveMsgData(DataMessage* t_msg, int channel_id);
};
