#pragma once
#include "Controller.hpp"
#include "PIDController.hpp"
#include "MRFTController.hpp"
#include "Vector3DMessage.hpp"
#include "FloatMsg.hpp"

class PIDplusMRFTController : public Controller{

private:
    PIDController* _pid_controller;
    MRFTController* _mrft_controller;
    float _current_pv, _current_pv_dot;
    float z_max, z_dot_max;
    FloatMsg _command_msg;
    controller_type _controller_type;
    block_id _id;

public:
    void switchIn(DataMessage*);
    DataMessage* switchOut();
    DataMessage* runTask(DataMessage*);
    void receiveMsgData(DataMessage* t_msg);

    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }

    PIDplusMRFTController(block_id t_id, PIDController* pid_ctr, MRFTController* mrft_ctr);
    ~PIDplusMRFTController();
};