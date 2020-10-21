#include "Port.hpp"

Port::Port(int t_id, Block* t_block) {
    this->_id = t_id;
    this->_block = t_block;
}

Port::~Port() {

}

void Port::receiveMsgData(DataMessage* t_msg){

    this->_block->process(t_msg, this);
}

void Port::receiveMsgData(DataMessage* t_msg, int channel_id){

    this->emitMsgUnicastDefault(t_msg, channel_id);
}

int Port::getID(){
    return _id;
}