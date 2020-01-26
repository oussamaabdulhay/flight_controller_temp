#include "Transform_InertialToBody.hpp"

Transform_InertialToBody::Transform_InertialToBody(control_system t_control_system, Vector3D<float>* t_inertial_command) {
    _source = t_control_system;
    _inertial_command = t_inertial_command;
}

Transform_InertialToBody::~Transform_InertialToBody() {

}
//TODO refactor to remove ifs
void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* ctrl_sys_msg = (ControlSystemMessage*)t_msg;

        if(_source == control_system::x && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command->x = ctrl_sys_msg->getData();
        } else if (_source == control_system::y && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command->y = ctrl_sys_msg->getData(); 
        }

    }else if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* ros_msg = (Vector3DMessage*)t_msg;
        
        Vector3D<float> yawpv = ros_msg->getData();

        Vector3D<float> yaw_rotation;
        yaw_rotation.x = 0.0;
        yaw_rotation.y = 0.0;
        yaw_rotation.z = -yawpv.x;

        _rotation_matrix.Update(yaw_rotation);
        this->transform();
    }
}

void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg, int t_channel){

    if(t_channel == (int)control_system::yaw_rate){
        
    } 
}


void Transform_InertialToBody::transform(){

    _body_command = _rotation_matrix.TransformVector(*_inertial_command);

    if(_source == control_system::x){
        m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.x);
        this->emit_message((DataMessage*) &m_output_msg);
    } else if(_source == control_system::y){
        m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.y);
        this->emit_message((DataMessage*) &m_output_msg);
    }
    
    
}