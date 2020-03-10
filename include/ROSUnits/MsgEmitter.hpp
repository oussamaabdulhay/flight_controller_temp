// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#pragma once
#include <vector>
#include <iterator> 
#include <iostream>
#include "MsgReceiver.hpp"
#include "DataMessage.hpp"

const int DEFAULT_UNICAST = -1;
class MsgReceiver;
class MsgEmitter{
private:

protected:
    std::vector<MsgReceiver*> _list_of_msg_receivers;
    std::vector<int> _list_of_receivers_mask_unicast;
    int emitting_channel=msg_broadcast_channel;
public:
    void setEmittingChannel(int);
    int getEmittingChannel();
    MsgEmitter();
    void addCallbackMsgReceiver(MsgReceiver* _callback_msg_receiver, int t_associated_unicast_mask);
    void addCallbackMsgReceiver(MsgReceiver* _callback_msg_receiver);
    void emitMsgUnicast(DataMessage* t_msg, int t_unicast_mask);
    void emitMsgUnicast(DataMessage* t_msg, int t_unicast_mask, int t_channel_id);
    void emitMsgUnicastDefault(DataMessage* t_msg);
    void emitMsgUnicastDefault(DataMessage* t_msg, int t_channel_id);
};
