#include "XSens_IMU.hpp"
#include <fstream>
#include <vector>
#include <iostream>

//std::ofstream write_data("/home/pedrohrpbs/catkin_ws_NAVIO//orientation_control_data.txt"); 


XSens_IMU::XSens_IMU() {

}

XSens_IMU::~XSens_IMU() {

}

void XSens_IMU::receive_msg_data(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::QUATERNION){

        QuaternionMessage* att_msg = (QuaternionMessage*)t_msg;

        Quaternion _quaternion = att_msg->getData();
        last_euler_angles=getEulerfromQuaternion(_quaternion);
        Roll_PVProvider::getProcessVariable();

    }else if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* br_msg = (Vector3DMessage*)t_msg;

        _bodyrate = br_msg->getData();
    }
}

AttitudeMsg XSens_IMU::getAttitude(){
    AttitudeMsg t_att_msg;
    
    t_att_msg.roll = last_euler_angles.x;
    t_att_msg.pitch = last_euler_angles.y;

    return t_att_msg;
}

Vector3D<float> XSens_IMU::getBodyRate(){
    BodyRateMsg _transform;
    Vector3D<float> v3d_bodyrate;

    v3d_bodyrate.y = _bodyrate.x;
    v3d_bodyrate.x = _bodyrate.y;

    return v3d_bodyrate;
}

Vector3D<float> XSens_IMU::getEulerfromQuaternion(Quaternion q){
    // write_data <<q.x << ", ";
    // write_data << q.y << ", ";
    // write_data <<q.z << ", ";
    // write_data << q.w << ", ";
    
    Vector3D<float> _euler;
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

    // write_data <<_euler.x << ", ";
    // write_data <<_euler.y << ", ";
    // write_data <<_euler.z << ", \n";

    return _euler;
}