#include "Roll_PVProvider.hpp"

Roll_PVProvider::Roll_PVProvider() {

}

Roll_PVProvider::~Roll_PVProvider() {

}

Vector3D<float> Roll_PVProvider::getProcessVariable(){

    AttitudeMsg att = this->getAttitude();

    Vector3D<float> t_process_variable;
    t_process_variable.x = att.roll;
    t_process_variable.y = (this->getBodyRate()).y;
    t_process_variable.z = 0.0; //TODO roll_dot_dot

    ros_msg.setRoll_PV(t_process_variable);
    this->PVProvider::emit_message((DataMessage*) &ros_msg);

    return t_process_variable;
    
}