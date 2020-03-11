#pragma once
#include "ROSUnit.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include <flight_controller/Restricted_Norm_Settings.h>
#include "Vector3D.hpp"

class ROSUnit_RestNormSettings :  public ROSUnit{

    private:
        static ROSUnit_RestNormSettings* _instance_ptr;
        static RestrictedNormRefSettingsMsg _settings_msg; 
        ros::ServiceServer _srv_rest_norm_settings;
        static bool callbackSettings(flight_controller::Restricted_Norm_Settings::Request  &req, 
                                     flight_controller::Restricted_Norm_Settings::Response &res);
        void receiveMsgData(DataMessage* t_msg);  

    public:
        
        ROSUnit_RestNormSettings(ros::NodeHandle&);
        ~ROSUnit_RestNormSettings();
};