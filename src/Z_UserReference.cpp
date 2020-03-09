#include "Z_UserReference.hpp"

Z_UserReference::Z_UserReference() {

}

Z_UserReference::~Z_UserReference() {

}

void Z_UserReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::USERREFERENCE){   
        UpdatePoseMessage* user_msg = (UpdatePoseMessage*)t_msg;
        if(user_msg->getRefType() == msg_type_reference::Z){
            DoubleMsg z_reference_msg;
            z_reference_msg.data = user_msg->getZ();
            std::cout << "Setting Z Reference: " << user_msg->getZ() << std::endl;
            this->emit_message_unicast((DataMessage*) &z_reference_msg, 
                                        UserReference::unicast_addresses::z,
                                        ControlSystem::receiving_channels::ch_Reference);
        }    
    }
}