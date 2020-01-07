#include "Transform_InertialToBody.hpp"

Transform_InertialToBody::Transform_InertialToBody(control_system t_control_system) {
    _source = t_control_system;
}

Transform_InertialToBody::~Transform_InertialToBody() {

}
//TODO refactor to remove ifs
void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* ctrl_sys_msg = (ControlSystemMessage*)t_msg;

        if(_source == control_system::x && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command.x = ctrl_sys_msg->getData();
            std::cout << "COMMAND X INERTIAL: " << _inertial_command.x << std::endl;
        } else if (_source == control_system::y && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command.y = ctrl_sys_msg->getData();
            std::cout << "COMMAND Y INERTIAL: " << _inertial_command.y << std::endl;
        }

    } else if (t_msg->getType() == msg_type::ROS){
        ROSMsg* ros_msg = (ROSMsg*)t_msg;
        
        if(ros_msg->getROSMsgType() == ros_msg_type::YAW_PV){
            Vector3D<float> yawpv = ros_msg->getYaw_PV();

            Vector3D<float> yaw_rotation;
            yaw_rotation.x = 0.0;
            yaw_rotation.y = 0.0;
            yaw_rotation.z = yawpv.x;

            _rotation_matrix.Update(yaw_rotation);
            this->transform();
        }
    }
}

void Transform_InertialToBody::transform(){

    _body_command = _rotation_matrix.TransformVector(_inertial_command);

    if(_source == control_system::x){
        m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.x);
        std::cout << "COMMAND X BODY: " << _body_command.x << std::endl;
        this->emit_message((DataMessage*) &m_output_msg);
    } else if(_source == control_system::y){
        m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.y);
        std::cout << "COMMAND Y BODY: " << _body_command.y << std::endl;
        this->emit_message((DataMessage*) &m_output_msg);
    }
    
    
}