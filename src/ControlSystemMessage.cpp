#include "ControlSystemMessage.hpp"

ControlSystemMessage::ControlSystemMessage(){
}

ControlSystemMessage::~ControlSystemMessage() {

}

msg_type ControlSystemMessage::getType(){
    return _type;
}

const int ControlSystemMessage::getSize()
{
    return sizeof(this);
}

Block* ControlSystemMessage::getBlockToAdd(){
    return _to_add;
}
control_system_msg_type ControlSystemMessage::getControlSystemMsgType(){
    return _control_system_msg_type;
}
void ControlSystemMessage::setControlSystemMessage(control_system_msg_type t_type, Block* t_to_add) {
    _to_add = t_to_add;
    _type = msg_type::control_system;
    _control_system_msg_type = t_type;
}
