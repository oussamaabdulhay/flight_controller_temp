#include "ROSUnit_UpdateReferenceX.hpp"

//TODO rename the topics and class
ROSUnit_UpdateReferenceX* ROSUnit_UpdateReferenceX::_instance_ptr = NULL;
UpdatePoseMessage ROSUnit_UpdateReferenceX::_pose_ref_msg;

ROSUnit_UpdateReferenceX::ROSUnit_UpdateReferenceX(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_reference/x", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_UpdateReferenceX::~ROSUnit_UpdateReferenceX() {

}

void ROSUnit_UpdateReferenceX::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateReferenceX::callbackSetpoint(flight_controller::Update_X_Reference::Request  &req, 
                                                flight_controller::Update_X_Reference::Response &res){

    float setpoint = req.setpoint_x;

    _pose_ref_msg.setPoseX(setpoint);

    _instance_ptr->emit_message((DataMessage*) &_pose_ref_msg);

    return true;

}