#include "ROSUnit_ResetController.hpp"
ROSUnit_ResetController* ROSUnit_ResetController::_instance_ptr = NULL;
IntegerMsg ROSUnit_ResetController::_reset_msg;

ROSUnit_ResetController::ROSUnit_ResetController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_reset_controller = t_main_handler.advertiseService("reset_controller", callbackResetController);
    _instance_ptr = this;
}   

ROSUnit_ResetController::~ROSUnit_ResetController() {

}

void ROSUnit_ResetController::receive_msg_data(DataMessage* t_msg){


}

bool ROSUnit_ResetController::callbackResetController(flight_controller::Reset_Controller::Request &req, flight_controller::Reset_Controller::Response &res){

    int data;
    data = req.id;

    _reset_msg.data = data;
    _instance_ptr->emit_message_unicast((DataMessage*) &_reset_msg, -1);
    
    return true;
}