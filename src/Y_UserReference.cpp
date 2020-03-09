#include "Y_UserReference.hpp"

Y_UserReference::Y_UserReference() {

}

Y_UserReference::~Y_UserReference() {

}

void Y_UserReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::FLOAT){   
        FloatMsg* user_msg = (FloatMsg*)t_msg;
        if(user_msg->getRefType() == msg_type_reference::Y){
            DoubleMsg y_reference_msg;
            y_reference_msg.data = user_msg->getY();
            std::cout << "Setting Y Reference: " << user_msg->getY() << std::endl;
            this->emit_message_unicast((DataMessage*) &y_reference_msg, 
                                        UserReference::unicast_addresses::y,
                                        ControlSystem::receiving_channels::ch_Reference);
        }    
    }
}