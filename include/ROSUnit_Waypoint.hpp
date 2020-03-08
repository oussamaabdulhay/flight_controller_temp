#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include "WaypointMsg.hpp"
#include <flight_controller/Waypoint.h>
#include <flight_controller/Update_Pose_Reference.h>
#include "Vector3D.hpp"

class ROSUnit_Waypoint :  public ROSUnit{

    private:
        static ROSUnit_Waypoint* _instance_ptr;
        static WaypointMsg _waypoint_msg; 
        ros::ServiceServer _srv_setpoint;
        static bool callbackSetpoint(flight_controller::Update_Pose_Reference::Request  &req, 
                                     flight_controller::Update_Pose_Reference::Response &res);
        void receive_msg_data(DataMessage* t_msg);  

    public:
        ROSUnit_Waypoint(ros::NodeHandle&);
        ~ROSUnit_Waypoint();
};