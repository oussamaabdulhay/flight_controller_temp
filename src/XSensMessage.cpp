#include "XSensMessage.hpp"

XSensMessage::XSensMessage() {
    _type = msg_type::XSENS;
}

XSensMessage::~XSensMessage() {

}

msg_type XSensMessage::getType(){
    return _type;
}

const int XSensMessage::getSize()
{
    return sizeof(this);
}
    
Vector3D<float> XSensMessage::getAngularVelocity(){
    return _angular_velocity;
}

Quaternion XSensMessage::getOrientation(){
    return _orientation;
}

Vector3D<float> XSensMessage::getOrientationEuler(){
    return _orientation_euler;
}

void XSensMessage::setAngularVelocity(Vector3D<float> t_angular_velocity) {
    _angular_velocity = t_angular_velocity;
}

void XSensMessage::setOrientation(Quaternion t_orientation) {
    _orientation = t_orientation;
}

void XSensMessage::setOrientationEuler(Vector3D<float>  t_orientation_euler) {
    _orientation_euler = t_orientation_euler;
}

void XSensMessage::setXSensMessage(Vector3D<float> t_angular_velocity, Quaternion t_orientation) {
    _orientation = t_orientation;
    _angular_velocity = t_angular_velocity;
}