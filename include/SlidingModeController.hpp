#pragma once
#include <math.h>
#include "common_srv/FloatMsg.hpp"
#include "Controller.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "SwitchOutMsg.hpp"
#include "logger.hpp"
#include "common_srv/IntegerMsg.hpp"
#include "SM_values.hpp"

class SlidingModeController : public Controller{

private:
	Timer _timer;
	block_id _id;
    controller_type _controller_type;
	float _dt;
	FloatMsg _command_msg;
    double _alpha1, _alpha2, _h1, _h2;
    SwitchOutMsg _switchout_msg;
	double _command = 0;


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
