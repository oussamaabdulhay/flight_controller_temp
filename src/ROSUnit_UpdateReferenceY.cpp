#include "ROSUnit_UpdateReferenceY.hpp"

//TODO rename the topics and class
ROSUnit_UpdateReferenceY* ROSUnit_UpdateReferenceY::_instance_ptr = NULL;
UpdatePoseMessage ROSUnit_UpdateReferenceY::_pose_ref_msg;

ROSUnit_UpdateReferenceY::ROSUnit_UpdateReferenceY(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_reference/y", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_UpdateReferenceY::~ROSUnit_UpdateReferenceY() {

}

void ROSUnit_UpdateReferenceY::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateReferenceY::callbackSetpoint(flight_controller::Update_Y_Reference::Request  &req, 
                                                flight_controller::Update_Y_Reference::Response &res){

    float setpoint = req.setpoint_y;

    _pose_ref_msg.setPoseY(setpoint);

    _instance_ptr->emit_message((DataMessage*) &_pose_ref_msg);

    return true;

}