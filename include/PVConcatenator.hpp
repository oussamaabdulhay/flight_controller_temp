#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3DMessage.hpp"

class PVConcatenator : public msg_emitter, public msg_receiver{

private:
    Vector3D<float> _stored_diff_input;
    Vector3D<double> _position;
    Vector3D<double> _velocity;
    Vector3D<float> _angle;
    Vector3D<float> _angle_rate;
    control_system _cs;

public:
    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int);
    void concatenate();
    PVConcatenator(control_system);
    ~PVConcatenator();
};