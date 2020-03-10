// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#pragma once

#include <iostream>
#include "common_types.hpp"
#include "DataMessage.hpp"
const int msg_broadcast_channel=-1;
class msg_receiver {
private:
    msg_type _msg_type;
public:
    msg_receiver();
    void assign_message_type_ID(msg_type i);
    msg_type get_message_type_ID();
    virtual void receive_msg_data(DataMessage* t_msg);
    virtual void receive_msg_data(DataMessage* t_msg, int channel_id);
};
