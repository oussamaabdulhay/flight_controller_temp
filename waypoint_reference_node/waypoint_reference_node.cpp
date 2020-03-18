#include "ros/ros.h"
#include <iostream>
#include "ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "providers_node");

    ros::NodeHandle nh;
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* myROSRestNormSettings = new ROSUnit_RestNormSettings(nh);
    ROSUnit* ROSUnit_uav_control_set_path = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                                ROSUnit_msg_type::ROSUnit_Poses,
                                                                                "uav_control/set_path");


    RestrictedNormWaypointRefGenerator* myWaypoint = new RestrictedNormWaypointRefGenerator();

    //myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)myWaypoint, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos); 
    ROSUnit_uav_control_set_path->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);
    myROSRestNormSettings->addCallbackMsgReceiver((MsgReceiver*)myWaypoint);
    //myWaypoint->addCallbackMsgReceiver((MsgReceiver*)myROSBroadcastData);

    //******************SETTING TRAJECTORY GENERATION TOOL******************

    // myWaypoint->add_x_control_system(X_ControlSystem);
    // myWaypoint->add_y_control_system(Y_ControlSystem);
    // myWaypoint->add_z_control_system(Z_ControlSystem);
    // myWaypoint->add_yaw_control_system(Yaw_ControlSystem);

    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}