#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/FloatMsg.hpp"
#include "PVConcatenator.hpp"

class Differentiator : public MsgEmitter, public MsgReceiver {

private:
    float _old_float_data;
    Vector3D<float> _old_vector3d_data;
    Timer timer;
    float _dt;
    Vector3D<float> diff_values;
    
public:
    
    void receiveMsgData(DataMessage*, int);
    Differentiator(float);
    ~Differentiator();
};