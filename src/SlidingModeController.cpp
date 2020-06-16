#include "SlidingModeController.hpp"

SlidingModeController::SlidingModeController(block_id t_id) {  
    _controller_type = controller_type::sliding_mode;
	_id = t_id;

}

SlidingModeController::~SlidingModeController() {

}

void SlidingModeController::switchIn(DataMessage* t_msg){
	Logger::getAssignedLogger()->log("SWITCH IN SLIDING_MODE CONTROLLER", LoggerLevel::Warning);
}

DataMessage* SlidingModeController::switchOut(){
	Logger::getAssignedLogger()->log("SWITCH OUT SLIDING_MODE CONTROLLER", LoggerLevel::Warning);
    
	_switchout_msg.setSwitchOutMsg(0.0);

    return (DataMessage*)&_switchout_msg;
} 

void SlidingModeController::receiveMsgData(DataMessage* t_msg){

	if(t_msg->getType() == msg_type::UPDATECONTROLLER){
		ControllerMessage* sm_msg = (ControllerMessage*)t_msg;
		SM_parameters params = sm_msg->getSMParam();

		if(params.id == this->_id){		
			this->initialize(&params);	
		}

	}else if(t_msg->getType() == msg_type::INTEGER){
		IntegerMsg* integer_msg = (IntegerMsg*)t_msg;

		if(static_cast<block_id>(integer_msg->data) == this->_id){
			Logger::getAssignedLogger()->log("RESET CONTROLLER: %.0f", (int)this->_id, LoggerLevel::Warning);
			this->reset();
		}
	}
}

void SlidingModeController::reset(){
}

void SlidingModeController::initialize(SM_parameters* t_params){
	_id = t_params->id;
	_alpha1 = t_params->alpha1;
	_alpha2 = t_params->alpha2;
	_h1 = t_params->h1;
	_h2 = t_params->h2;

	Logger::getAssignedLogger()->log("SLIDING_MODE SETTINGS: ID_%.0f", static_cast<int>(_id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Alpha1: %.2f", _alpha1, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Alpha2: %.2f", _alpha2, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("H1: %.2f", _h1, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("H2: %.2f", _h2, LoggerLevel::Info);
}

DataMessage* SlidingModeController::runTask(DataMessage* t_msg){
        
	Vector3DMessage* controller_msg = (Vector3DMessage*)t_msg;

    Vector3D<float> data = controller_msg->getData();
	
	// data.x is Error
	// data.y is PV_First
	// data.z is PV_Second

    float command;
	
	command = sliding_mode_algorithm(data.x);
	
    _command_msg.data = command;

	return (DataMessage*) &_command_msg;
}

float SlidingModeController::sliding_mode_algorithm(float t_error){
	double command = 0;

    if(-t_error > _h1){
		command = -_alpha1;
	}else if(-t_error < -_h1){
		command = _alpha1;
	}

	if(-t_error > _h2){
		command = -_alpha2;
	}else if(-t_error < -_h2){
		command = _alpha2;
	}

	//std::cout << "Error: " << t_error << " Command: " << command << std::endl;

	return command;

}