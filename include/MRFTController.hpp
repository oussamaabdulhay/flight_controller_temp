#pragma once
#include <math.h>
#include "ControllerMessage.hpp"
#include "FloatMessage.hpp"
#include "MRFT_values.hpp"
#include "Controller.hpp"
#include "ResetControllerMsg.hpp"
#include "Timer.hpp"
#include "SwitcherMessage.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"

class MRFTController : public Controller{

private:
	Timer _timer;
	block_id _id;
    controller_type _controller_type;
	FloatMessage m_output_msg;
	float _dt;
    //Chehadeh's code
	//TODO add to MRFT parameters
	const uint8_t iterations_lock_count = 10; //COMMIT change from 20 to 5
	bool prev_res, first_run = true;
	float prev_err = 0;
	/*
	last_switch: to detect the previous switch state; 
	period_switch: detect two switchings, i.e. one period
	*/
	uint32_t last_peak_micros;
	bool mem1=false,mem2 = 0;
	uint8_t iteration_number;
	float realy_output;
	float minpeak_out = 0;
	float maxpeak_out = 0;
	float maxpeak(bool, float);
	float minpeak(bool, float);
	bool algorithm(float, bool&, MRFT_bag&);
    //---------------
public:
    //Chehadeh's code
    MRFT_parameters parameters;
	void initialize(MRFT_parameters*);
	float mrft_with_antilock(float, bool&, MRFT_bag&);
	MRFT_bag _mrft_period;
    //---------------
	void switchIn(DataMessage*);
    DataMessage* switchOut();
	void receive_msg_data(DataMessage* t_msg); 
    void reset();
	DataMessage* receive_msg_internal(DataMessage*);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }

    MRFTController(block_id t_id);
    ~MRFTController();
};
