#include "Transform_InertialToBody.hpp"

Transform_InertialToBody::Transform_InertialToBody() {

}

Transform_InertialToBody::~Transform_InertialToBody() {

}

void Transform_InertialToBody::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::USERREFERENCE){   
        UpdatePoseMessage* user_msg = (UpdatePoseMessage*)t_msg;
        
        if(user_msg->getRefType() == msg_type_reference::X){
            _inertial_command.x = user_msg->getX();
        } else if (user_msg->getRefType() == msg_type_reference::Y){
            _inertial_command.y = user_msg->getY();
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

    _body_xy.setPoseX(_body_command.x);
    this->emit_message((DataMessage*) &_body_xy);
    _body_xy.setPoseY(_body_command.y);
    this->emit_message((DataMessage*) &_body_xy);
}