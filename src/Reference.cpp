#include "Reference.hpp"

Reference::Reference(){
    _type = block_type::reference;
}

Reference::~Reference() {

}

block_type Reference::getType() {
    return _type;
}

void Reference::switchIn(DataMessage* data){
}


DataMessage* Reference::switchOut(){
    
    DataMessage* msg;
    
    return msg;
}    

void Reference::receive_msg_data(DataMessage* t_msg){


}
