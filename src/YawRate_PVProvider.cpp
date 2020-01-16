#include "YawRate_PVProvider.hpp"

YawRate_PVProvider::YawRate_PVProvider() {

}

YawRate_PVProvider::~YawRate_PVProvider() {

}

Vector3D<float> YawRate_PVProvider::getProcessVariable(){

    Vector3D<float> t_process_variable;
    t_process_variable.x = this->getBodyRate().z; 
    t_process_variable.y = 0.0; 
    t_process_variable.z = 0.0;

    ros_msg.setYawRate_PV(t_process_variable);
    this->PVProvider::emit_message((DataMessage*) &ros_msg);

    return t_process_variable;
}