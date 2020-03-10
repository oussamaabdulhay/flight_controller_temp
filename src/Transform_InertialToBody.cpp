#include "Transform_InertialToBody.hpp"

std::atomic<float>  Transform_InertialToBody::_inertial_command_x;
std::atomic<float>  Transform_InertialToBody::_inertial_command_y;
std::atomic<float>  Transform_InertialToBody::_inertial_command_z;

Transform_InertialToBody::Transform_InertialToBody(control_system t_control_system) {
    _source = t_control_system;
}

Transform_InertialToBody::~Transform_InertialToBody() {

}

void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::FLOAT){

        FloatMsg* float_msg = (FloatMsg*)t_msg;

        FloatMsg output;
        Vector3D<float> body_command;
        Vector3D<float> inertial_command;
        if(_source == control_system::x){
            _inertial_command_x = float_msg->data;

            inertial_command.x=_inertial_command_x;
            inertial_command.y=_inertial_command_y;
            inertial_command.z=_inertial_command_z;
            body_command = _rotation_matrix.TransformVector(inertial_command);

            output.data = _body_command.x;
            this->emit_message_unicast((DataMessage*) &output,
                                        -1,
                                        ControlSystem::receiving_channels::ch_reference);

        } else if (_source == control_system::y){
            _inertial_command_y = float_msg->data; 

            inertial_command.x=_inertial_command_x;
            inertial_command.y=_inertial_command_y;
            inertial_command.z=_inertial_command_z;
            body_command = _rotation_matrix.TransformVector(inertial_command);

            output.data = _body_command.y;
            this->emit_message_unicast((DataMessage*) &output,
                                        -1,
                                        ControlSystem::receiving_channels::ch_reference);
        }   
    }
}

void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg, int t_channel){ 

        Vector3DMessage* yaw_msg = (Vector3DMessage*)t_msg;
        Vector3D<float> yawpv = yaw_msg->getData();

        Vector3D<float> yaw_rotation;
        yaw_rotation.x = 0.0;
        yaw_rotation.y = 0.0;
        yaw_rotation.z = -yawpv.x;

        _rotation_matrix.Update(yaw_rotation);
}
