#include "ROSMsg.hpp"

ROSMsg::ROSMsg() {
    _type = msg_type::ROS;
}

ROSMsg::~ROSMsg() {

}

msg_type ROSMsg::getType(){
    return _type;
}
ros_msg_type ROSMsg::getROSMsgType(){
    return _ros_type;
}
const int ROSMsg::getSize(){
    return sizeof(this);
}
// void ROSMsg::setPosition(PositionMsg tmp){
//     _ros_type = ros_msg_type::POSITION;
//     _position = tmp;
// }
// void ROSMsg::setAttitude(AttitudeMsg tmp){
//     _ros_type = ros_msg_type::ORIENTATION;
//     _attitude = tmp;
// }
// void ROSMsg::setHeading(HeadingMsg tmp){
//     _ros_type = ros_msg_type::ORIENTATION;
//     _heading = tmp;
// }
void ROSMsg::setX_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::X_PV;
    _x_pv = tmp;
}
void ROSMsg::setY_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::Y_PV;
    _y_pv = tmp;
}
void ROSMsg::setZ_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::Z_PV;
    _z_pv = tmp;
}
void ROSMsg::setRoll_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::ROLL_PV;
    _roll_pv = tmp;
}
void ROSMsg::setPitch_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::PITCH_PV;
    _pitch_pv = tmp;
}
void ROSMsg::setYaw_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::YAW_PV;
    _yaw_pv = tmp;
}
void ROSMsg::setYawRate_PV(Vector3D<float> tmp){
    _ros_type = ros_msg_type::YAW_RATE_PV;
    _yaw_rate_pv = tmp;
}
void ROSMsg::setControlSystem(float tmp, control_system t_source){
    _ros_type = ros_msg_type::CONTROLSYSTEM;
    _control_system = tmp;
    _source = t_source;
}
void ROSMsg::setControlSystemReference(float tmp, control_system t_source){
    _ros_type = ros_msg_type::CONTROLSYSTEMREFERENCE;
    _control_system = tmp;
    _source = t_source;
}
void ROSMsg::setActuation(float* tmp){
    _ros_type = ros_msg_type::ACTUATION;
    _actuation = tmp;
}
void ROSMsg::setArmed(bool tmp){
    _ros_type = ros_msg_type::ARMED;
    _armed = tmp;
}
void ROSMsg::setNumberOfWaypoints(int tmp){
    _ros_type = ros_msg_type::NUMBER_OF_WAYPOINTS;
    _number_of_waypoints = tmp;
}
