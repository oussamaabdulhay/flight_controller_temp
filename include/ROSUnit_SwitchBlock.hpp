#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include <std_msgs/Bool.h>
#include "BooleanMsg.hpp"
#include "Vector3D.hpp"
#include "SwitchBlockMsg.hpp"
#include <flight_controller/SwitchBlock.h>

class ROSUnit_SwitchBlock :  public ROSUnit{

    private:
        static ROSUnit_SwitchBlock* _instance_ptr;
        static SwitchBlockMsg _switch_msg; 
        ros::ServiceServer _srv_switch;
        static bool callbackSwitchBlocks(flight_controller::SwitchBlock::Request  &req,
                                         flight_controller::SwitchBlock::Response &res);
    
    public:
        void receive_msg_data(DataMessage* t_msg); 
        ROSUnit_SwitchBlock(ros::NodeHandle&);
        ~ROSUnit_SwitchBlock();
};