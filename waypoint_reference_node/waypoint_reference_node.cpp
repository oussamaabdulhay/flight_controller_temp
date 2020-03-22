#include "ros/ros.h"
#include <iostream>
#include "ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "waypoint_reference_node");

    ros::NodeHandle nh;
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* rosunit_restricted_norm_settings = new ROSUnit_RestNormSettings(nh);
    ROSUnit* rosunit_uav_control_set_path = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                                ROSUnit_msg_type::ROSUnit_Poses,
                                                                                "uav_control/set_path");
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Subscriber, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_waypoint_counter = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/counter");
    ROSUnit* rosunit_waypoint_x = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/x");
    ROSUnit* rosunit_waypoint_y = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/y");
    ROSUnit* rosunit_waypoint_z = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/z");
    ROSUnit* rosunit_waypoint_yaw = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Float,
                                                                    "waypoint_reference/yaw");

    RestrictedNormWaypointRefGenerator* waypoint_generator = new RestrictedNormWaypointRefGenerator();

    rosunit_g2i_position->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator); 
    rosunit_uav_control_set_path->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator);
    rosunit_restricted_norm_settings->addCallbackMsgReceiver((MsgReceiver*)waypoint_generator);
    waypoint_generator->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_counter);

    //******************SETTING TRAJECTORY GENERATION TOOL******************

    waypoint_generator->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_x, (int)RestrictedNormWaypointRefGenerator::unicast_addresses::x);
    waypoint_generator->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_y, (int)RestrictedNormWaypointRefGenerator::unicast_addresses::y);
    waypoint_generator->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_z, (int)RestrictedNormWaypointRefGenerator::unicast_addresses::z);
    waypoint_generator->addCallbackMsgReceiver((MsgReceiver*)rosunit_waypoint_yaw, (int)RestrictedNormWaypointRefGenerator::unicast_addresses::yaw);

    std::cout  << "###### WAYPOINT REFERENCE NODE ######" "\n";

    while(ros::ok()){
        ros::spinOnce();
        usleep( 10 );
    }

    return 0;
}