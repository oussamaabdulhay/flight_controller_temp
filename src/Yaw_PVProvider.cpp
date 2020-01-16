#include "Yaw_PVProvider.hpp"

Yaw_PVProvider::Yaw_PVProvider() {

}

Yaw_PVProvider::~Yaw_PVProvider() {

}

Vector3D<float> Yaw_PVProvider::getProcessVariable(){

    Vector3D<float> t_process_variable;
    t_process_variable.x = this->getHeading().yaw;
    t_process_variable.y = 0.0; 
    t_process_variable.z = 0.0; //TODO yaw_dot_dot

    ros_msg.setYaw_PV(t_process_variable);
    this->PVProvider::emit_message((DataMessage*) &ros_msg);

    return t_process_variable;
}