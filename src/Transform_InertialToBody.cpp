#include "Transform_InertialToBody.hpp"

std::atomic<float>  Transform_InertialToBody::_inertial_command_x;
std::atomic<float>  Transform_InertialToBody::_inertial_command_y;
std::atomic<float>  Transform_InertialToBody::_inertial_command_z;

Transform_InertialToBody::Transform_InertialToBody(control_system t_control_system) {
    _source = t_control_system;
    // _inertial_command_x=t_inertial_command->x; //TODO-Chehadeh
    // _inertial_command_y=t_inertial_command->y;
    // _inertial_command_z=t_inertial_command->z;
}

Transform_InertialToBody::~Transform_InertialToBody() {

}
//TODO refactor to remove ifs
void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){ // TODO-Chehadeh: emit-message must be here to complete the loop at the rate of X and Y

        ControlSystemMessage* ctrl_sys_msg = (ControlSystemMessage*)t_msg;
        //std::cout << "void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){" << std::endl; 
        if(_source == control_system::x && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command_x = ctrl_sys_msg->getData();
        } else if (_source == control_system::y && ctrl_sys_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            _inertial_command_y = ctrl_sys_msg->getData(); 
        }
        

    }
}

void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg, int t_channel){ //TODO-Chehadeh: Check channel info

    //if(t_channel == (int)control_system::yaw){
        Vector3DMessage* yaw_msg = (Vector3DMessage*)t_msg;
        //std::cout << "void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg, int t_channel){" << std::endl; 

        Vector3D<float> yawpv = yaw_msg->getData();

        Vector3D<float> yaw_rotation;
        yaw_rotation.x = 0.0;
        yaw_rotation.y = 0.0;
        yaw_rotation.z = -yawpv.x;

        _rotation_matrix.Update(yaw_rotation);
        // this->transform();
        Vector3D<float> _body_command;
        Vector3D<float> _inertial_command;
        _inertial_command.x=_inertial_command_x;
        _inertial_command.y=_inertial_command_y;
        _inertial_command.z=_inertial_command_z;
        _body_command = _rotation_matrix.TransformVector(_inertial_command);

        if(_source == control_system::x){
            m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.x);
            this->emit_message((DataMessage*) &m_output_msg);
        } else if(_source == control_system::y){
            m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.y);
            this->emit_message((DataMessage*) &m_output_msg);
        }
    //} 
}


// void Transform_InertialToBody::transform(){
//     // Vector3D<float> _body_command;
//     // _body_command = _rotation_matrix.TransformVector(*_inertial_command);

//     // if(_source == control_system::x){
//     //     m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.x);
//     //     this->emit_message((DataMessage*) &m_output_msg);
//     // } else if(_source == control_system::y){
//     //     m_output_msg.setControlSystemMessage(_source, control_system_msg_type::to_system, _body_command.y);
//     //     this->emit_message((DataMessage*) &m_output_msg);
//     // }
    
    
// }