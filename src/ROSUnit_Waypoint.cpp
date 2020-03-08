#include "ROSUnit_Waypoint.hpp"

//TODO rename the topics and class
ROSUnit_Waypoint* ROSUnit_Waypoint::_instance_ptr = NULL;
WaypointMsg ROSUnit_Waypoint::_waypoint_msg;

ROSUnit_Waypoint::ROSUnit_Waypoint(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_setpoint = t_main_handler.advertiseService("update_pose_reference", callbackSetpoint);    
    _instance_ptr = this;
}   

ROSUnit_Waypoint::~ROSUnit_Waypoint() {

}

void ROSUnit_Waypoint::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_Waypoint::callbackSetpoint( flight_controller::Update_Pose_Reference::Request  &req, 
                                                flight_controller::Update_Pose_Reference::Response &res){

    //TODO change to receive only one reference at a time
    Waypoint t_waypoint;
    t_waypoint.position.x = req.setpoint_pose.x;
    t_waypoint.position.y = req.setpoint_pose.y;
    t_waypoint.position.z = req.setpoint_pose.z;
    t_waypoint.yaw = req.setpoint_pose.yaw;

    _waypoint_msg.waypoint = t_waypoint;

    _instance_ptr->emit_message((DataMessage*) &_waypoint_msg);

    return true;

}