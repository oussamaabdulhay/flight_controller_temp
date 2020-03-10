#include "XSens_IMU.hpp"
#include <fstream>
#include <vector>
#include <iostream>


XSens_IMU::XSens_IMU() {

}

XSens_IMU::~XSens_IMU() {

}

void XSens_IMU::receiveMsgData(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::XSENS){
        XSensMessage* xsens_msg = (XSensMessage*)t_msg;

        Vector3D<float> orientation_euler = xsens_msg->getOrientationEuler(); 
        Vector3D<float> body_rate = xsens_msg->getAngularVelocity(); 

        Vector3D<float> roll_pv;
        roll_pv.x = orientation_euler.x;
        roll_pv.y = body_rate.x;
        roll_pv.z = 0.0;
        _roll_pv_msg.setVector3DMessage(roll_pv);
        this->emitMsgUnicast((DataMessage*) &_roll_pv_msg, (int)control_system::roll, (int)control_system::roll);

        Vector3D<float> pitch_pv;
        pitch_pv.x = orientation_euler.y;
        pitch_pv.y = body_rate.y;
        pitch_pv.z = 0.0;
        _pitch_pv_msg.setVector3DMessage(pitch_pv);
        this->emitMsgUnicast((DataMessage*) &_pitch_pv_msg, (int)control_system::pitch, (int)control_system::pitch);
    }

}