#include "ros/ros.h"
#include <iostream>
#include "ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "RestrictedNormWaypointRefGenerator.hpp"
#include "ROSUnit_RestNormSettings.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "Global2Inertial.hpp"

int main(int argc, char **argv){

    ros::init(argc, argv, "global2inertial_node");

    ros::NodeHandle nh;
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* myROSOptitrack = new ROSUnit_Optitrack(nh);
    ROSUnit* rosunit_g2i_position = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/position");
    ROSUnit* rosunit_g2i_orientation = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "global2inertial/orientation");                                                                
    ROSUnit* rosunit_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,
                                                                            ROSUnit_msg_type::ROSUnit_Float,
                                                                            "set_height_offset");

    
    Global2Inertial* myGlobal2Inertial = new Global2Inertial();

    myROSOptitrack->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)rosunit_g2i_position, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)rosunit_g2i_orientation, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    rosunit_set_height_offset->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);

    std::cout  << "###### GLOBAL2INERTIAL NODE ######" "\n";

    while(ros::ok()){
        ros::spinOnce();
        usleep( 10 );
    }

    return 0;
}