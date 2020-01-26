#include "Saturation.hpp"

Saturation::Saturation(float t_clip_value) {
    _clip_value = t_clip_value;
}

Saturation::~Saturation() {

}

void Saturation::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* ctrl_sys_msg = (ControlSystemMessage*)t_msg;

        if(ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            this->clip(ctrl_sys_msg->getData());
        }
    }
}

void Saturation::clip(float t_value_to_clip){

    if(t_value_to_clip > _clip_value){
        m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, _clip_value);
    }else if(t_value_to_clip < -_clip_value){
        m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, -_clip_value);
    }else{
        m_output_msg.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, t_value_to_clip);
    }   
    this->emit_message((DataMessage*) &m_output_msg);
    
    
}