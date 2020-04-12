#pragma once
#include <math.h>
#include "FloatMsg.hpp"
#include "Controller.hpp"
#include "Timer.hpp"
#include "Vector3DMessage.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"
#include "IntegerMsg.hpp"
#include "SM_values.hpp"

class SlidingModeController : public Controller{

private:
	Timer _timer;
	block_id _id;
    controller_type _controller_type;
	float _dt;
	FloatMsg _command_msg;
    double _alpha1, _alpha2, _h1, _h2;
    
public:
	void switchIn(DataMessage*);
    DataMessage* switchOut();
	void receiveMsgData(DataMessage* t_msg); 
    void reset();
    void initialize(SM_parameters*);
	DataMessage* runTask(DataMessage*);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }
    float sliding_mode_algorithm(float);

    SlidingModeController(block_id t_id);
    ~SlidingModeController();
};
