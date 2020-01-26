#include "ProcessVariableDifferentiator.hpp"
#include <iostream>

ProcessVariableDifferentiator::ProcessVariableDifferentiator(){
    Quaternion _bodyAtt;
    Vector3D<float> _bodyPos;
    _prev_time = 0;

}

ProcessVariableDifferentiator::~ProcessVariableDifferentiator() {

}

void ProcessVariableDifferentiator::updateVelocity(double _dt){

    _bodyVel.x = (_bodyPos.x - _prev_pos.x) / _dt;
    _bodyVel.y = (_bodyPos.y - _prev_pos.y) / _dt;
    _bodyVel.z = (_bodyPos.z - _prev_pos.z) / _dt;  
}

void ProcessVariableDifferentiator::updateAcceleration(double _dt){
    _bodyAcc.x = (_bodyVel.x - _prev_vel.x) / _dt;
    _bodyAcc.y = (_bodyVel.y - _prev_vel.y) / _dt;
    _bodyAcc.z = (_bodyVel.z - _prev_vel.z) / _dt;
}

void ProcessVariableDifferentiator::updateYawRate(double _dt){
    _bodyYawRate = (_bodyHeading - _prev_heading) / _dt;
}

void ProcessVariableDifferentiator::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::POSE){

        PoseStampedMsg* pose_msg = (PoseStampedMsg*)t_msg;
        
        _bodyPos.x = pose_msg->pose.x;
        _bodyPos.y = pose_msg->pose.y;
        _bodyPos.z = pose_msg->pose.z;
        _bodyHeading = pose_msg->pose.yaw;
        _time = pose_msg->pose.time;

        _dt = (_time - _prev_time);

        this->updateVelocity(_dt);
        this->updateAcceleration(_dt);
        this->updateYawRate(_dt);
        _prev_pos = _bodyPos;
        _prev_vel = _bodyVel;
        _prev_heading = _bodyHeading;
        
        Vector3D<float> x_pv;
        x_pv.x = _bodyPos.x;
        x_pv.y = _bodyVel.x;
        x_pv.z = _bodyAcc.x;
        _x_pv_msg.setVector3DMessage(x_pv);
        this->emit_message_unicast((DataMessage*) &_x_pv_msg, (int)control_system::x, (int)control_system::x);

        Vector3D<float> y_pv;
        y_pv.x = _bodyPos.y;
        y_pv.y = _bodyVel.y;
        y_pv.z = _bodyAcc.y;
        _y_pv_msg.setVector3DMessage(y_pv);
        this->emit_message_unicast((DataMessage*) &_y_pv_msg, (int)control_system::y, (int)control_system::y);

        Vector3D<float> z_pv;
        z_pv.x = _bodyPos.z;
        z_pv.y = _bodyVel.z;
        z_pv.z = _bodyAcc.z;
        _z_pv_msg.setVector3DMessage(z_pv);
        this->emit_message_unicast((DataMessage*) &_z_pv_msg, (int)control_system::z, (int)control_system::z);

        Vector3D<float> yaw_pv;
        yaw_pv.x = _bodyHeading;
        yaw_pv.y = 0.0;
        yaw_pv.z = 0.0;
        _yaw_pv_msg.setVector3DMessage(yaw_pv);
        this->emit_message_unicast((DataMessage*) &_yaw_pv_msg, (int)control_system::yaw, (int)control_system::yaw);

        Vector3D<float> yaw_rate_pv;
        yaw_rate_pv.x = _bodyYawRate;
        yaw_rate_pv.y = 0.0;
        yaw_rate_pv.z = 0.0;
        _yaw_rate_pv_msg.setVector3DMessage(yaw_rate_pv);
        this->emit_message_unicast((DataMessage*) &_yaw_rate_pv_msg, (int)control_system::yaw_rate, (int)control_system::yaw_rate);

        _prev_time = _time;
    }
}
