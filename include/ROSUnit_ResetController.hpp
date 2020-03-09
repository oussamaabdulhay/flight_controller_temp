#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include <std_msgs/Bool.h>
#include "BooleanMsg.hpp"
#include "Vector3D.hpp"
#include "ResetControllerMsg.hpp"
#include <flight_controller/Reset_Controller.h>

class ROSUnit_ResetController :  public ROSUnit{

    private:

        static ROSUnit_ResetController* _instance_ptr;
        static ResetControllerMsg _reset_msg; 
        ros::ServiceServer _srv_reset_controller;
        static bool callbackResetController(flight_controller::Reset_Controller::Request  &req, flight_controller::Reset_Controller::Response &res);
        void receive_msg_data(DataMessage* t_msg);  

    public:
        ROSUnit_ResetController(ros::NodeHandle&);
        ~ROSUnit_ResetController();
};