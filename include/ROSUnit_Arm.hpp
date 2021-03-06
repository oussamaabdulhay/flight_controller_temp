#pragma once
#include "common_srv/ROSUnit.hpp"
#include <std_msgs/Bool.h>
#include "common_srv/BooleanMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/BooleanMsg.hpp"
#include <flight_controller/Arm.h>

class ROSUnit_Arm :  public ROSUnit{

    private:
        //TODO receive msgs from a service through a callback 
        static ROSUnit_Arm* _instance_ptr;
        static BooleanMsg _bool_msg; 
        ros::ServiceServer _srv_armed;
        static bool callbackArm(flight_controller::Arm::Request  &req, flight_controller::Arm::Response &res);
        void receiveMsgData(DataMessage* t_msg);  

    public:
        ROSUnit_Arm(ros::NodeHandle&);
        ~ROSUnit_Arm();
};