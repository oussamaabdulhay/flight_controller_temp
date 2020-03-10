// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#include "MsgReceiver.hpp"

void msg_receiver::assign_message_type_ID(msg_type i){
    this->_msg_type=i;
}

msg_type msg_receiver::get_message_type_ID(){
    return this->_msg_type;
}
msg_receiver::msg_receiver(){
}

void msg_receiver::receive_msg_data(DataMessage* t_msg){

}
void msg_receiver::receive_msg_data(DataMessage* t_msg,int channel_id){

}
