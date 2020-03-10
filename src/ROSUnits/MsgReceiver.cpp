// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#include "MsgReceiver.hpp"

void MsgReceiver::assignMsgTypeID(msg_type i){
    this->_msg_type=i;
}

msg_type MsgReceiver::getMsgTypeID(){
    return this->_msg_type;
}
MsgReceiver::MsgReceiver(){
}

void MsgReceiver::receiveMsgData(DataMessage* t_msg){

}
void MsgReceiver::receiveMsgData(DataMessage* t_msg,int channel_id){

}
