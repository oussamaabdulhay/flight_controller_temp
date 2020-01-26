#include "MRFTController.hpp"

MRFTController::MRFTController(block_id t_id) {  
    _controller_type = controller_type::mrft;
	_id = t_id;
}

MRFTController::~MRFTController() {

}

void MRFTController::switchIn(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::SWITCHOUT){
		SwitchOutMsg* switch_out_msg = (SwitchOutMsg*)t_msg;
		parameters.bias = switch_out_msg->getSwitchOutMsg();
	}
	Logger::getAssignedLogger()->log("SWITCH IN MRFT CONTROLLER - Bias: %f", parameters.bias, LoggerLevel::Warning);
}

DataMessage* MRFTController::switchOut(){
	Logger::getAssignedLogger()->log("SWITCH OUT MRFT CONTROLLER",LoggerLevel::Warning);
    DataMessage* msg;
    return msg;
} 

void MRFTController::receive_msg_data(DataMessage* t_msg){

	if(t_msg->getType() == msg_type::UPDATECONTROLLER){
		ControllerMessage* mrft_msg = (ControllerMessage*)t_msg;
		MRFT_parameters _params = mrft_msg->getMRFTParam();

		if(mrft_msg->getID() == this->_id){		
			this->initialize(&_params);	
		}
		
	}else if(t_msg->getType() == msg_type::RESETCONTROLLER){
		ResetControllerMsg* reset_msg = (ResetControllerMsg*)t_msg;

		if(static_cast<block_id>(reset_msg->getData()) == this->_id){
			Logger::getAssignedLogger()->log("RESET CONTROLLER: %.0f", (int)this->_id, LoggerLevel::Warning);
			this->reset();
		}
	}

}

void MRFTController::reset(){
	first_run = true;
	prev_err = 0;
	mem1 = false;
	mem2 = 0;
	minpeak_out = 0;
	maxpeak_out = 0;
}

DataMessage* MRFTController::receive_msg_internal(DataMessage* t_msg){
        
	SwitcherMessage* controller_msg = (SwitcherMessage*)t_msg;

    Vector3D<float> data = controller_msg->getVector3DData();
	
	// data.x is Error
	// data.y is PV_First
	// data.z is PV_Second

    float command;
	bool mrft_bag_ready=false;
	
	command = mrft_with_antilock(data.x, mrft_bag_ready, _mrft_period);
	
	if (mrft_bag_ready){
		//local_comm.send_packet((void*)&mrft_period, sizeof(mrft_bag), local_comm.mrft_bag);
		//TODO add a Warning
	}

    m_output_msg.data = command;
	return (DataMessage*) &m_output_msg;
}

// Start of Chehadeh's Code
float MRFTController::maxpeak(bool cntrl, float in){
	if (cntrl == 1){
		if (in>0)
		{
			maxpeak_out = in;
		}
		else
		{
			maxpeak_out = 0;
		}
	}
	else
	{
		if (in>maxpeak_out){
			maxpeak_out = in;
		}
	}
	return maxpeak_out;

}

float MRFTController::minpeak(bool cntrl, float in){
	if (cntrl == 1){
		if (in <= 0)
		{
			minpeak_out = in;
		}
		else
		{
			minpeak_out = 0;
		}
	}
	else
	{
		if (in<minpeak_out){
			minpeak_out = in;
		}
	}
	return minpeak_out;
}

bool MRFTController::algorithm(float err, bool& mrft_bag_ready_para, MRFT_bag& mrft_period){
	float buff1, outmin, outmax, outswitch;
	bool op1, op2;
	if (mrft_bag_ready_para){
		mrft_bag_ready_para = false;
	}
	buff1 = ((-err) - (prev_err)) / _dt;//TODO: use Gyro
	prev_err = -err;
	if (buff1 >= 0)
	{
		op1 = true;
	}
	else
	{
		op1 = false;
	}
	op2 = ((!op1) && (mem1 ^ op1));
	outmax = maxpeak(op2, err);
	op2 = ((op1) && (mem1 ^ op1));
	outmin = minpeak(op2, err);
	if (op1 == true){
		outswitch = outmax;
	}
	else{
		outswitch = outmin;
	}
	buff1 = outswitch*parameters.beta + err;
	if (buff1 >= 0){
		op2 = true;
	}
	else{
		op2 = false;
	}
	if (mem2 != op2){
		if (op2)
		{
			mrft_period.amplitude = mrft_period.amplitude - outmin;
			mrft_period.duration = _timer.tockMicroSeconds();
			mrft_bag_ready_para = true;
			_timer.tick();
		}
		else
		{
			mrft_period.amplitude = outmax;
		}
	}
	mem1 = op1;
	mem2 = op2;
	return op2;
}

void MRFTController::initialize(MRFT_parameters* para){
	
	parameters.beta = para->beta;
	parameters.relay_amp = para->relay_amp;
	parameters.bias = para->bias;
	parameters.id = para->id;
	if(para->dt > 0){
		_dt = para->dt;
	}

	Logger::getAssignedLogger()->log("MRFT SETTINGS: ID_%.0f", static_cast<int>(parameters.id), LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Beta: %.2f", parameters.beta, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Relay_amp: %.2f", parameters.relay_amp, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("Bias: %.6f", parameters.bias, LoggerLevel::Info);
	Logger::getAssignedLogger()->log("dt: %.6f", _dt, LoggerLevel::Info);

}

float MRFTController::mrft_with_antilock(float err, bool& mrft_bag_ready_para, MRFT_bag& mrft_period){
	bool mrft_res;
	mrft_res = algorithm(err, mrft_bag_ready_para, mrft_period);
	if (first_run){
		if (mrft_res)
		{ 
			realy_output = parameters.relay_amp; 
		}
		else
		{ 
			realy_output = -parameters.relay_amp; 
		}
		iteration_number = iterations_lock_count;
		prev_res = mrft_res;
		first_run = false;
		return (realy_output);
	}
	if (mrft_res){
		if ((prev_res == !mrft_res) && (iteration_number == 0)) {
			prev_res = mrft_res;
			realy_output = parameters.relay_amp;
			iteration_number = iterations_lock_count;
		}
		/*if (angle_pitch>max_angle_pitch[cycle]){
		max_angle_pitch[cycle] = angle_pitch;
		}*/

	}
	else
	{
		if ((prev_res == !mrft_res) && (iteration_number == 0)){
			prev_res = mrft_res;
			realy_output = -parameters.relay_amp;
			iteration_number = iterations_lock_count;
		}
		/*if (angle_pitch<min_angle_pitch[cycle]){
		min_angle_pitch[cycle] = angle_pitch;
		}*/
	}
	if (iteration_number > 0){ 
		iteration_number--; 
	}
	return (realy_output+parameters.bias);
}