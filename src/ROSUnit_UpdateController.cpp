#include "ROSUnit_UpdateController.hpp"

//TODO rename the topics and class
ROSUnit_UpdateController* ROSUnit_UpdateController::_instance_ptr = NULL;
ControllerMessage ROSUnit_UpdateController::_update_controller_msg;
control_system ROSUnit_UpdateController::_id;

ROSUnit_UpdateController::ROSUnit_UpdateController(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler) {
    _srv_update_controller = t_main_handler.advertiseService("update_controller", callbackUpdateController);
    _instance_ptr = this;
}   

ROSUnit_UpdateController::~ROSUnit_UpdateController() {

}

void ROSUnit_UpdateController::receiveMsgData(DataMessage* t_msg){


}

bool ROSUnit_UpdateController::callbackUpdateController(flight_controller::Update_Controller::Request  &req, 
                                                        flight_controller::Update_Controller::Response &res){

    block_id _id = static_cast<block_id>((int)req.controller_parameters.id);
    
    PID_parameters pid_data;
    pid_data.kp = req.controller_parameters.pid_kp;
    pid_data.ki = req.controller_parameters.pid_ki;
    pid_data.kd = req.controller_parameters.pid_kd;
    pid_data.kdd = req.controller_parameters.pid_kdd;
    pid_data.anti_windup = req.controller_parameters.pid_anti_windup;
    pid_data.en_pv_derivation = req.controller_parameters.pid_en_pv_derivation;
    pid_data.id = _id;

    MRFT_parameters mrft_data;
    mrft_data.beta = req.controller_parameters.mrft_beta;
    mrft_data.relay_amp = req.controller_parameters.mrft_relay_amp;
    mrft_data.bias = req.controller_parameters.mrft_bias;
    mrft_data.id = _id;

    _update_controller_msg.setPIDParam(pid_data);
    _update_controller_msg.setMRFTParam(mrft_data);
    _instance_ptr->emitMsgUnicast((DataMessage*) &_update_controller_msg, -1);
    
    return true;
}