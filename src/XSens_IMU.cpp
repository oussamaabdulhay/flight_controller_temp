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
        //TODO
    }
}

AttitudeMsg XSens_IMU::getAttitude(){
    std::cout << "getAttitude"<< std::endl;
    std::cout << "roll: " << _attitude.roll << std::endl;
    std::cout << "pitch: " << _attitude.pitch << std::endl;
    return _attitude;
}

Vector3D<float> XSens_IMU::getBodyRate(){
    //TODO
}