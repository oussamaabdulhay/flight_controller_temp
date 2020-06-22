#pragma once
#include "Controller.hpp"
#include "PIDController.hpp"
#include "MRFTController.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/FloatMsg.hpp"

class PIDplusMRFTController : public Controller{

private:
    PIDController* _pid_controller;
    MRFTController* _mrft_controller;
    float _current_pv, _current_pv_dot;
    float z_max = 1.65, z_dot_max = 0.6;
    FloatMsg _command_msg;
    controller_type _controller_type;
    block_id _id;
    bool _PID_enabled = true;
    float _last_PID;
public:
    void switchIn(DataMessage*);
    DataMessage* switchOut();
    DataMessage* runTask(DataMessage*);
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);
    controller_type getControllerType(){ return _controller_type; }
    block_id getID(){ return _id; }

    PIDplusMRFTController(block_id t_id, PIDController* pid_ctr, MRFTController* mrft_ctr);
    ~PIDplusMRFTController();
};