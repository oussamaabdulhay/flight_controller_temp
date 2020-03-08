#pragma once
#include "ROSUnit.hpp"
#include "FlightScenarioMessage.hpp"
#include "UpdatePoseMessage.hpp"
#include <flight_controller/Update_Yaw_Reference.h>
#include "Vector3D.hpp"

class ROSUnit_UpdateReferenceYaw :  public ROSUnit{

    private:
        //TODO receive msgs from a service through a callback 
        static ROSUnit_UpdateReferenceYaw* _instance_ptr;
        static UpdatePoseMessage _pose_ref_msg; 
        ros::ServiceServer _srv_setpoint;
        static bool callbackSetpoint(flight_controller::Update_Yaw_Reference::Request  &req, 
                                     flight_controller::Update_Yaw_Reference::Response &res);
        void receive_msg_data(DataMessage* t_msg);  

    public:
        ROSUnit_UpdateReferenceYaw(ros::NodeHandle&);
        ~ROSUnit_UpdateReferenceYaw();
};