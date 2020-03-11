#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include <geometry_msgs/PoseStamped.h>
#include "Vector3D.hpp"
#include "PID_values.hpp"
#include <flight_controller/Controller_param.h>
#include <flight_controller/Update_Controller.h>
#include "common_types.hpp"
#include "ControllerMessage.hpp"

class ROSUnit_UpdateController :  public ROSUnit{

    private:
        static control_system _id;
        static ROSUnit_UpdateController* _instance_ptr;
        static ControllerMessage _update_controller_msg; 
        ros::ServiceServer _srv_update_controller;
        static bool callbackUpdateController(flight_controller::Update_Controller::Request  &req, 
                                             flight_controller::Update_Controller::Response &res);
        void receiveMsgData(DataMessage* t_msg);  

    public:
        ROSUnit_UpdateController(ros::NodeHandle&);
        ~ROSUnit_UpdateController();
};