#include "XSens_IMU.hpp"
#include <fstream>
#include <vector>
#include <iostream>


XSens_IMU::XSens_IMU() {

}

XSens_IMU::~XSens_IMU() {

}

void XSens_IMU::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::XSENS){
        XSensMessage* xsens_msg = (XSensMessage*)t_msg;

        Vector3D<float> orientation_euler = xsens_msg->getOrientationEuler(); 
        Vector3D<float> body_rate = xsens_msg->getAngularVelocity(); 

        Vector3D<float> roll_pv;
        roll_pv.x = orientation_euler.x;
        roll_pv.y = body_rate.x;
        roll_pv.z = 0.0;
        _roll_pv_msg.setRollProviderMessage(roll_pv);
        this->emit_message((DataMessage*) &_roll_pv_msg);

        Vector3D<float> pitch_pv;
        pitch_pv.x = orientation_euler.y;
        pitch_pv.y = body_rate.y;
        pitch_pv.z = 0.0;
        _pitch_pv_msg.setPitchProviderMessage(pitch_pv);
        this->emit_message((DataMessage*) &_pitch_pv_msg);
    }

}

AttitudeMsg XSens_IMU::getAttitude(){
    // AttitudeMsg t_att_msg;
    
    // t_att_msg.roll = last_euler_angles.x;
    // t_att_msg.pitch = last_euler_angles.y;

    // return t_att_msg;
}

Vector3D<float> XSens_IMU::getBodyRate(){
    // BodyRateMsg _transform;
    // Vector3D<float> v3d_bodyrate;

    // v3d_bodyrate.y = _bodyrate.x;
    // v3d_bodyrate.x = _bodyrate.y;

    // return v3d_bodyrate;
}
