#include "X_UserReference.hpp"

X_UserReference::X_UserReference() {

}

X_UserReference::~X_UserReference() {

}

void X_UserReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::USERREFERENCE){   
        UpdatePoseMessage* user_msg = (UpdatePoseMessage*)t_msg;
        if(user_msg->getRefType() == msg_type_reference::X){
            DoubleMsg x_reference_msg;
            x_reference_msg.data = user_msg->getX();
            std::cout << "Setting X Reference: " << user_msg->getX() << std::endl;
            this->emit_message_unicast((DataMessage*) &x_reference_msg, 
                                        UserReference::unicast_addresses::x,
                                        ControlSystem::receiving_channels::ch_Reference);
        }    
    }
}
