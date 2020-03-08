#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include "UpdatePoseMessage.hpp"
#include <flight_controller/Update_Y_Reference.h>
#include "Vector3D.hpp"

class ROSUnit_UpdateReferenceY :  public ROSUnit{

    private:
        //TODO receive msgs from a service through a callback 
        static ROSUnit_UpdateReferenceY* _instance_ptr;
        static UpdatePoseMessage _pose_ref_msg; 
        ros::ServiceServer _srv_setpoint;
        static bool callbackSetpoint(flight_controller::Update_Y_Reference::Request  &req, 
                                     flight_controller::Update_Y_Reference::Response &res);
        void receive_msg_data(DataMessage* t_msg);  

    public:
        ROSUnit_UpdateReferenceY(ros::NodeHandle&);
        ~ROSUnit_UpdateReferenceY();
};