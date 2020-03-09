#include "Yaw_UserReference.hpp"

Yaw_UserReference::Yaw_UserReference() {

}

Yaw_UserReference::~Yaw_UserReference() {

}

void Yaw_UserReference::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::USERREFERENCE){   
        UpdatePoseMessage* user_msg = (UpdatePoseMessage*)t_msg;
        if(user_msg->getRefType() == msg_type_reference::YAW){
            DoubleMsg yaw_reference_msg;
            yaw_reference_msg.data = user_msg->getYaw();
            //TODO make a logger
            std::cout << "Setting Yaw Reference: " << user_msg->getYaw() << std::endl;
            this->emit_message_unicast((DataMessage*) &yaw_reference_msg, 
                                        UserReference::unicast_addresses::yaw,
                                        ControlSystem::receiving_channels::ch_Reference);
        }    
    }
}