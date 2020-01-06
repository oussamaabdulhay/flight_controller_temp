#include "XSens_IMU.hpp"

XSens_IMU::XSens_IMU() {

}

XSens_IMU::~XSens_IMU() {

}

void XSens_IMU::receive_msg_data(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::ATTITUDE){
        AttitudeMsg* att_msg = (AttitudeMsg*)t_msg;

        _attitude = *att_msg;

    }else if(t_msg->getType() == msg_type::BODYRATE){
        BodyRateMsg* br_msg = (BodyRateMsg*)t_msg;

        _bodyrate = *br_msg;
    }
}

AttitudeMsg XSens_IMU::getAttitude(){
    AttitudeMsg _transform;

    _transform.roll = _attitude.pitch;
    _transform.pitch = _attitude.roll;

    // std::cout << "getAttitude"<< std::endl;
    // std::cout << "roll: " << _transform.roll << std::endl;
    // std::cout << "pitch: " << _transform.pitch << std::endl;
    return _transform;
}

Vector3D<float> XSens_IMU::getBodyRate(){
    BodyRateMsg _transform;
    Vector3D<float> v3d_bodyrate;

    v3d_bodyrate.y = _bodyrate.x;
    v3d_bodyrate.x = _bodyrate.y;

    // std::cout << "getBodyRate"<< std::endl;
    // std::cout << "x: " << v3d_bodyrate.x << std::endl;
    // std::cout << "y: " << v3d_bodyrate.y << std::endl;

    return v3d_bodyrate;
}