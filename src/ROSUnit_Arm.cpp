#include "ROSUnit_Arm.hpp"
ROSUnit_Arm* ROSUnit_Arm::_instance_ptr = NULL;
BooleanMsg ROSUnit_Arm::_bool_msg;
//TODO change to a Service
ROSUnit_Arm::ROSUnit_Arm(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_armed = t_main_handler.advertiseService("arm", callbackArm);
    _instance_ptr = this;
}   

ROSUnit_Arm::~ROSUnit_Arm() {

}

void ROSUnit_Arm::receiveMsgData(DataMessage* t_msg){


}

bool ROSUnit_Arm::callbackArm(flight_controller::Arm::Request &req, flight_controller::Arm::Response &res){

    bool data;
    data = req.armed;

    _bool_msg.data = data;
    _instance_ptr->emitMsgUnicastDefault((DataMessage*) &_bool_msg);
    
    return true;
}