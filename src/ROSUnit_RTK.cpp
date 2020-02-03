#include "ROSUnit_RTK.hpp"
#include <iostream>
ROSUnit_RTK* ROSUnit_RTK::_instance_ptr = NULL;

ROSUnit_RTK::ROSUnit_RTK(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){
    _sub_fix = t_main_handler.subscribe("/tcpfix", 1, callbackRTK1);
    _sub_vel = t_main_handler.subscribe("/tcpvel", 1, callbackRTK2);
    _instance_ptr = this;
}

ROSUnit_RTK::~ROSUnit_RTK() {

}

void ROSUnit_RTK::callbackRTK1(const sensor_msgs::NavSatFix::ConstPtr & msg){
    
    Vector3DMessage position_msg;
    Vector3D<float> pos_data;
    pos_data.x = msg->latitude;
    pos_data.y = msg->longitude;
    pos_data.z = msg->altitude;

    position_msg.x=pos_data.x;
    position_msg.y=pos_data.y;
    position_msg.z=pos_data.z;
    _instance_ptr->emit_message((DataMessage*) &position_msg,(int)Global2Inertial::receiving_channels::ch_RTK_pos);

}


void ROSUnit_RTK::callbackRTK2(const geometry_msgs::TwistStamped::ConstPtr& msg){
    
    Vector3DMessage velocity_msg;
    Vector3D<float> vel_data;
    vel_data.x = msg->twist.linear.x;
    vel_data.y = msg->twist.linear.y;
    vel_data.z = msg->twist.linear.z;

   
    velocity_msg.x=vel_data.x;
    velocity_msg.y=vel_data.y;
    velocity_msg.z=vel_data.z;
    _instance_ptr->emit_message((DataMessage*) &velocity_msg); 
    
    
}

void ROSUnit_RTK::receive_msg_data(DataMessage* t_msg){


}