#include "Roll_PVProvider.hpp"

Roll_PVProvider::Roll_PVProvider() {

}

Roll_PVProvider::~Roll_PVProvider() {

}

Vector3D<float> Roll_PVProvider::getProcessVariable(){

    Vector3D<float> t_process_variable;
    t_process_variable.x = this->getAttitude().roll;
    if(this->getBodyRateProvider()){
        t_process_variable.y = (this->getBodyRateProvider()->getBodyRate().y) * (M_PI/180);
    }else{
        t_process_variable.y = 0.0;
    }
    t_process_variable.z = 0.0; //TODO roll_dot_dot

    ros_msg.setRoll_PV(t_process_variable);
    this->PVProvider::emit_message((DataMessage*) &ros_msg);

    ros_msg.setAttitude(this->getAttitude());
    this->AttitudeProvider::emit_message((DataMessage*) &ros_msg);

    return t_process_variable;
    
}