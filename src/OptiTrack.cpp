#include "OptiTrack.hpp"
#include <iostream>

OptiTrack::OptiTrack(){
    Quaternion _bodyAtt;
    Vector3D<float> _bodyPos;
    _prev_time = 0;

}

OptiTrack::~OptiTrack() {

}

AttitudeMsg OptiTrack::getAttitude(){
    Vector3D<float> rpy = getEulerfromQuaternion(_bodyAtt);
    AttitudeMsg t_att_msg;
    t_att_msg.roll = rpy.x;
    t_att_msg.pitch = rpy.y;

    // std::cout << "getAttitude"<< std::endl;
    // std::cout << "roll: " << rpy.x << std::endl;
    // std::cout << "pitch: " << rpy.y << std::endl;

    return t_att_msg;
}

HeadingMsg OptiTrack::getHeading(){
    Vector3D<float> rpy = getEulerfromQuaternion(_bodyAtt);
    HeadingMsg t_heading_msg;
    t_heading_msg.yaw = rpy.z;

    // std::cout << "getHeading"<< std::endl;
    // std::cout << "yaw: " << rpy.z << std::endl;

    return t_heading_msg;
}

Quaternion OptiTrack::getAttitudeHeading(){

    // std::cout << "getAttitudeHeading"<< std::endl;
    // std::cout << "x: " << _bodyAtt.x << std::endl;
    // std::cout << "y: " << _bodyAtt.y << std::endl;
    // std::cout << "z: " << _bodyAtt.z << std::endl;
    // std::cout << "w: " << _bodyAtt.w << std::endl;
    
    return _bodyAtt;
}

PositionMsg OptiTrack::getPosition(){

    PositionMsg t_pos_msg;
    t_pos_msg.x = _bodyPos.x;
    t_pos_msg.y = _bodyPos.y;
    t_pos_msg.z = _bodyPos.z;
    // std::cout << "getPosition"<< std::endl;
    // std::cout << "x: " << _bodyPos.x << std::endl;
    // std::cout << "y: " << _bodyPos.y << std::endl;
    // std::cout << "z: " << _bodyPos.z << std::endl;

    double t_dt = (_time - _prev_time);
    
    if(t_dt != 0) {
        this->updateVelocity(t_dt);
        this->updateAcceleration(t_dt);
        this->updateYawRate(t_dt);
        _prev_pos = _bodyPos;
        _prev_vel = _bodyVel;
        _prev_heading = _bodyHeading;
        _prev_time = _time;
    }

    return t_pos_msg;
}

void OptiTrack::updateVelocity(double t_dt){

    _bodyVel.x = (_bodyPos.x - _prev_pos.x) / t_dt;
    _bodyVel.y = (_bodyPos.y - _prev_pos.y) / t_dt;
    _bodyVel.z = (_bodyPos.z - _prev_pos.z) / t_dt;  
}

void OptiTrack::updateAcceleration(double t_dt){
    _bodyAcc.x = (_bodyVel.x - _prev_vel.x) / t_dt;
    _bodyAcc.y = (_bodyVel.y - _prev_vel.y) / t_dt;
    _bodyAcc.z = (_bodyVel.z - _prev_vel.z) / t_dt;
}

void OptiTrack::updateYawRate(double t_dt){
    _bodyYawRate = (_bodyHeading - _prev_heading) / t_dt;
}

VelocityMsg OptiTrack::getVelocity(){
    VelocityMsg t_vel_msg;    
    t_vel_msg.dx = _bodyVel.x;
    t_vel_msg.dy = _bodyVel.y;
    t_vel_msg.dz = _bodyVel.z;

    // std::cout << "getVelocity"<< std::endl;
    // std::cout << "x: " << _bodyVel.x << std::endl;
    // std::cout << "y: " << _bodyVel.y << std::endl;
    // std::cout << "z: " << _bodyVel.z << std::endl;
    
    return t_vel_msg;
}

AccelerationMsg OptiTrack::getAcceleration(){
    AccelerationMsg t_accel_msg;
    t_accel_msg.ddx = _bodyAcc.x;
    t_accel_msg.ddy = _bodyAcc.y;
    t_accel_msg.ddz = _bodyAcc.z;

    // std::cout << "getAccel"<< std::endl;
    // std::cout << "x: " << _bodyAcc.x << std::endl;
    // std::cout << "y: " << _bodyAcc.y << std::endl;
    // std::cout << "z: " << _bodyAcc.z << std::endl;

    return t_accel_msg;
}

Vector3D<float> OptiTrack::getBodyRate(){
    
    Vector3D<float> t_body_rate;
    t_body_rate.x = 0.0;
    t_body_rate.y = 0.0;
    t_body_rate.z = _bodyYawRate;

    return t_body_rate;
}

void OptiTrack::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::optitrack){

        OptitrackMessage* opti_msg = (OptitrackMessage*)t_msg;
        
        _bodyPos = opti_msg->getPosition();
        _bodyAtt = opti_msg->getAttitudeHeading();
        _bodyHeading = this->getHeading().yaw;
        _time = opti_msg->getTime();

        double t_dt = (_time - _prev_time);

        this->updateVelocity(t_dt);
        this->updateAcceleration(t_dt);
        this->updateYawRate(t_dt);
        _prev_pos = _bodyPos;
        _prev_vel = _bodyVel;
        _prev_heading = _bodyHeading;
        _prev_time = _time;
        
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

    }
}

Vector3D<float> OptiTrack::getEulerfromQuaternion(Quaternion q){

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    _euler.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        _euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    _euler.z = atan2(siny_cosp, cosy_cosp);

    return _euler;
}

Quaternion OptiTrack::getQuaternionfromEuler(Vector3D<float> euler){
    
    double roll = euler.x;
    double pitch = euler.y;
    double yaw = euler.z;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    _quat.w = cy * cp * cr + sy * sp * sr;
    _quat.x = cy * cp * sr - sy * sp * cr;
    _quat.y = sy * cp * sr + cy * sp * cr;
    _quat.z = sy * cp * cr - cy * sp * sr;
    
    return _quat;
}