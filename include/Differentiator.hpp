#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "cmath"
#include "Vector3DMessage.hpp"
#include "Timer.hpp"
#include "FloatMsg.hpp"
#include "PVConcatenator.hpp"

class Differentiator : public MsgEmitter, public MsgReceiver {

private:
    float _old_float_data;
    Vector3D<float> _old_vector3d_data;
    Timer timer;
    float _dt;
    
public:
    
    void receiveMsgData(DataMessage*, int);
    Differentiator(float);
    ~Differentiator();
};