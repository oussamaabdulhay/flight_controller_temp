#include "ros/ros.h"
#include <iostream>
#include "ROSUnit_Factory.hpp"
#include "std_logger.hpp"
#include "Global2Inertial.hpp"
#include "PVConcatenator.hpp"
#include "Differentiator.hpp"
#include "WrapAroundFunction.hpp"
#include "ROSUnit_Optitrack.hpp"
#include "ROSUnit_Xsens.hpp"
#include "Timer.hpp"

const int OPTITRACK_FREQUENCY = 120;

int main(int argc, char **argv){

    ros::init(argc, argv, "providers_node");

    ros::NodeHandle nh;
    //ros::Rate rate(120);
    
    ROSUnit_Factory ROSUnit_Factory_main{nh};

    ROSUnit* ROSUnit_set_height_offset = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Server,ROSUnit_msg_type::ROSUnit_Float,"set_height_offset");


    ROSUnit* rosunit_x_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/x");
    ROSUnit* rosunit_y_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/y");
    ROSUnit* rosunit_z_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/z");
    ROSUnit* rosunit_roll_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/roll");
    ROSUnit* rosunit_pitch_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/pitch");
    ROSUnit* rosunit_yaw_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw");
    ROSUnit* rosunit_yaw_rate_provider_pub = ROSUnit_Factory_main.CreateROSUnit(ROSUnit_tx_rx_type::Publisher, 
                                                                    ROSUnit_msg_type::ROSUnit_Point,
                                                                    "/providers/yaw_rate");

   
    //***********************ADDING SENSORS********************************
    ROSUnit* myROSOptitrack = new ROSUnit_Optitrack(nh);
    ROSUnit* myROSUnit_Xsens = new ROSUnit_Xsens(nh);

    //***********************SETTING PROVIDERS**********************************
    
    Global2Inertial* myGlobal2Inertial = new Global2Inertial();
    PVConcatenator* CsX_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsY_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsZ_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsRoll_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_x_axis, act_on::pv);
    PVConcatenator* CsPitch_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_y_axis, act_on::pv);
    PVConcatenator* CsYaw_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);
    PVConcatenator* CsYawRate_PVConcatenator = new PVConcatenator(PVConcatenator::concatenation_axes::conc_z_axis, act_on::pv);

    WrapAroundFunction* wrap_around_yaw = new WrapAroundFunction();
    wrap_around_yaw->assignParametersRange(-M_PI, M_PI);
    
    Differentiator* velocityFromPosition = new Differentiator(1./OPTITRACK_FREQUENCY);
    velocityFromPosition->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv_dot);
    
    Differentiator* yawRateFromYaw = new Differentiator(1./OPTITRACK_FREQUENCY);
    yawRateFromYaw->setEmittingChannel((int)PVConcatenator::receiving_channels::ch_pv);
    
    myROSOptitrack->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)velocityFromPosition, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)yawRateFromYaw, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator);
    velocityFromPosition->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator);
    yawRateFromYaw->addCallbackMsgReceiver((MsgReceiver*)CsYawRate_PVConcatenator);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsX_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsY_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)CsZ_PVConcatenator, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos);
    myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)wrap_around_yaw, (int)Global2Inertial::unicast_addresses::uni_Optitrack_heading);
    wrap_around_yaw->addCallbackMsgReceiver((MsgReceiver*)CsYaw_PVConcatenator);
    
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_attitude_rate);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_attitude_rate);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsRoll_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_orientation);
    myROSUnit_Xsens->addCallbackMsgReceiver((MsgReceiver*)CsPitch_PVConcatenator,(int)ROSUnit_Xsens::unicast_addresses::unicast_XSens_orientation);

    //TODO after adding G2I
    //myGlobal2Inertial->addCallbackMsgReceiver((MsgReceiver*)myWaypoint, (int)Global2Inertial::unicast_addresses::uni_Optitrack_pos); 
    //ROSUnit_set_height_offset->addCallbackMsgReceiver((MsgReceiver*)myGlobal2Inertial);
    
    //******************PROVIDERS TO CONTROL SYSTEMS******************************

    CsX_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_x_provider_pub);
    CsY_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_y_provider_pub);
    CsZ_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_z_provider_pub);
    CsPitch_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_pitch_provider_pub);
    CsRoll_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_roll_provider_pub);
    CsYaw_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_yaw_provider_pub);
    CsYawRate_PVConcatenator->addCallbackMsgReceiver((MsgReceiver*)rosunit_yaw_rate_provider_pub);

    Timer tempo;
    while(ros::ok()){
        //tempo.tick();
        ros::spinOnce();
        //std::cout  << "Prov: " << tempo.tockMicroSeconds() << "\n";
        
        //rate.sleep();
    }

    return 0;
}