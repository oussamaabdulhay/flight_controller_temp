// Version: 3.0
// Author: P. Silva
// Date: 09 Mar 2020
// Release note: Removed broadcast channels

#include "MsgEmitter.hpp"
using namespace std;

void MsgEmitter::addCallbackMsgReceiver(MsgReceiver* _callback_msg_receiver){
    this->addCallbackMsgReceiver(_callback_msg_receiver, DEFAULT_UNICAST);
}

void MsgEmitter::addCallbackMsgReceiver(MsgReceiver* _callback_msg_receiver, int associated_publishing_channel){
    _list_of_msg_receivers.push_back(_callback_msg_receiver);
    _list_of_receivers_mask_unicast.push_back(associated_publishing_channel);
}

void MsgEmitter::emitMsgUnicastDefault(DataMessage* t_msg, int t_channel_id){
    this->emitMsgUnicast(t_msg, DEFAULT_UNICAST, t_channel_id);
}

void MsgEmitter::emitMsgUnicastDefault(DataMessage* t_msg){

    if (this->emitting_channel!=msg_broadcast_channel){
        this->emitMsgUnicast(t_msg, DEFAULT_UNICAST, this->emitting_channel);
    }
    else{
        this->emitMsgUnicast(t_msg, DEFAULT_UNICAST);
    }

}

void MsgEmitter::emitMsgUnicast(DataMessage* t_msg,int t_unicast_mask){
    if (this->emitting_channel!=msg_broadcast_channel){
        this->emitMsgUnicast(t_msg, t_unicast_mask, this->emitting_channel);
    }else{
        for (int i = 0; i < _list_of_msg_receivers.size(); ++i){
            if (t_unicast_mask==_list_of_receivers_mask_unicast[i]){
                _list_of_msg_receivers[i]->receiveMsgData(t_msg);
            }
        }
    }
}
void MsgEmitter::emitMsgUnicast(DataMessage* t_msg,int t_unicast_mask,int t_channel_id){
    for (int i = 0; i < _list_of_msg_receivers.size(); ++i){
        if (t_unicast_mask==_list_of_receivers_mask_unicast[i]){
             _list_of_msg_receivers[i]->receiveMsgData(t_msg,t_channel_id);
        }
    }
}
MsgEmitter::MsgEmitter(){
}

void MsgEmitter::setEmittingChannel(int t_ch){
    this->emitting_channel=t_ch;
}

int MsgEmitter::getEmittingChannel(){
    return emitting_channel;
}