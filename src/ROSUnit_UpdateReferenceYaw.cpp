#include "ROSUnit_UpdateReferenceYaw.hpp"

//TODO rename the topics and class
ROSUnit_UpdateReferenceYaw* ROSUnit_UpdateReferenceYaw::_instance_ptr = NULL;
UpdatePoseMessage ROSUnit_UpdateReferenceYaw::_pose_ref_msg;

ROSUnit_UpdateReferenceYaw::ROSUnit_UpdateReferenceYaw(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_reference/yaw", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_UpdateReferenceYaw::~ROSUnit_UpdateReferenceYaw() {

}

void ROSUnit_UpdateReferenceYaw::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_UpdateReferenceYaw::callbackSetpoint(flight_controller::Update_Yaw_Reference::Request  &req, 
                                                flight_controller::Update_Yaw_Reference::Response &res){

    float setpoint = req.setpoint_yaw;

    _pose_ref_msg.setPoseYaw(setpoint);

    _instance_ptr->emit_message((DataMessage*) &_pose_ref_msg);

    return true;

}