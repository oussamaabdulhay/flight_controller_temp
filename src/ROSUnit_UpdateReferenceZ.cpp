#include "ROSUnit_UpdateReferenceZ.hpp"

//TODO rename the topics and class
ROSUnit_UpdateReferenceZ* ROSUnit_UpdateReferenceZ::_instance_ptr = NULL;
UpdatePoseMessage ROSUnit_UpdateReferenceZ::_pose_ref_msg;

ROSUnit_UpdateReferenceZ::ROSUnit_UpdateReferenceZ(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_reference/z", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_UpdateReferenceZ::~ROSUnit_UpdateReferenceZ() {

}

void ROSUnit_UpdateReferenceZ::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateReferenceZ::callbackSetpoint(flight_controller::Update_Z_Reference::Request  &req, 
                                                flight_controller::Update_Z_Reference::Response &res){

    float setpoint = req.setpoint_z;

    _pose_ref_msg.setPoseZ(setpoint);

    _instance_ptr->emit_message((DataMessage*) &_pose_ref_msg);

    return true;

}