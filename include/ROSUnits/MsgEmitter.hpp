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

class msg_receiver;
class msg_emitter{
private:

protected:
    std::vector<msg_receiver*> _list_of_msg_receivers;
    std::vector<int> _list_of_receivers_mask_unicast;
    int emitting_channel=msg_broadcast_channel;
public:
    void setEmittingChannel(int);
    int getEmittingChannel();
    msg_emitter();
    void add_callback_msg_receiver(msg_receiver* _callback_msg_receiver,int t_associated_unicast_mask);
    void emit_message_unicast(DataMessage* t_msg,int t_unicast_mask);
    void emit_message_unicast(DataMessage* t_msg,int t_unicast_mask,int t_channel_id);
};
