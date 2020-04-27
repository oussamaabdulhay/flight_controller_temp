#pragma once
#include "common_srv/ROSUnit.hpp"
#include <std_msgs/Bool.h>
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/IntegerMsg.hpp"
#include <flight_controller/Reset_Controller.h>

class ROSUnit_ResetController :  public ROSUnit{

    private:

        static ROSUnit_ResetController* _instance_ptr;
        static IntegerMsg _reset_msg; 
        ros::ServiceServer _srv_reset_controller;
        static bool callbackResetController(flight_controller::Reset_Controller::Request  &req, flight_controller::Reset_Controller::Response &res);
        void receiveMsgData(DataMessage* t_msg);  

    public:
        ROSUnit_ResetController(ros::NodeHandle&);
        ~ROSUnit_ResetController();
};