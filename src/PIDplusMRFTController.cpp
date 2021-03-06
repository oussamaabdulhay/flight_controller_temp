#include "PIDplusMRFTController.hpp"

PIDplusMRFTController::PIDplusMRFTController(block_id t_id, PIDController* pid_ctr, MRFTController* mrft_ctr) {
    _controller_type = controller_type::pid_mrft;
    _id = t_id;
    _pid_controller = pid_ctr;
    _mrft_controller = mrft_ctr;
}

PIDplusMRFTController::~PIDplusMRFTController() {

}

void PIDplusMRFTController::switchIn(DataMessage* t_msg){
	Logger::getAssignedLogger()->log("SWITCH IN PID+MRFT CONTROLLER", LoggerLevel::Warning);
}

DataMessage* PIDplusMRFTController::switchOut(){
    Logger::getAssignedLogger()->log("SWITCH OUT PID+MRFT CONTROLLER",LoggerLevel::Warning);
    DataMessage* msg;
    return msg;
}

DataMessage* PIDplusMRFTController::runTask(DataMessage* t_msg){

    FloatMsg* mrft_output_msg = (FloatMsg*)(_mrft_controller->runTask(t_msg));

    if(!_PID_enabled || _current_pv >= z_min){
        _command_msg.data = _last_PID + mrft_output_msg->data;
        _PID_enabled = false;
        
    }else if(_PID_enabled){
        FloatMsg* pid_output_msg = (FloatMsg*)(_pid_controller->runTask(t_msg));
        _last_PID = pid_output_msg->data;
        _command_msg.data = _last_PID;
    }

	return (DataMessage*) &_command_msg;
}

void PIDplusMRFTController::receiveMsgData(DataMessage* t_msg){
    
    _pid_controller->receiveMsgData(t_msg);
    _mrft_controller->receiveMsgData(t_msg);
    
}

void PIDplusMRFTController::receiveMsgData(DataMessage* t_msg, int t_channel){
    // std::cout << "_current_pv " << _current_pv<<std::endl;
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* provider = (Vector3DMessage*)t_msg;
        _current_pv = provider->getData().x;
        _current_pv_dot = provider->getData().y;
        _current_pv_dot_dot = provider->getData().z;

    }
}
