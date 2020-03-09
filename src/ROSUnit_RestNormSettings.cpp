#include "ROSUnit_RestNormSettings.hpp"

ROSUnit_RestNormSettings* ROSUnit_RestNormSettings::_instance_ptr = NULL;
RestrictedNormRefSettingsMsg ROSUnit_RestNormSettings::_settings_msg;

ROSUnit_RestNormSettings::ROSUnit_RestNormSettings(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_rest_norm_settings = t_main_handler.advertiseService("restricted_norm_settings", callbackSettings);    
    _instance_ptr = this;
}   

ROSUnit_RestNormSettings::~ROSUnit_RestNormSettings() {

}

void ROSUnit_RestNormSettings::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_RestNormSettings::callbackSettings(flight_controller::Restricted_Norm_Settings::Request  &req, 
                                                flight_controller::Restricted_Norm_Settings::Response &res){

    _settings_msg.enabled = req.enabled;
    _settings_msg.delete_existing_waypoints = req.delete_existing_waypoints;
    _settings_msg.setMaxNorm(req.max_norm);

    _instance_ptr->emit_message_unicast((DataMessage*) &_settings_msg, ROSUnit_RestNormSettings::unicast_addresses::broadcast);

    return true;

}