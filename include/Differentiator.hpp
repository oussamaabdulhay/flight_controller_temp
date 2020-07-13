#pragma once
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/MsgReceiver.hpp"
#include "cmath"
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/Timer.hpp"
#include "common_srv/FloatMsg.hpp"
#include "PVConcatenator.hpp"
#include "ButterFilter_120hz.hpp"

class Differentiator : public MsgEmitter, public MsgReceiver {

private:
    float _old_float_data;
    Vector3D<float> _old_vector3d_data;
    Timer timer;
    float _dt;
    Vector3D<float> diff_values;
    ButterFilter_120hz low_pass_filter_x, low_pass_filter_y, low_pass_filter_z;
    
public:
    
    void receiveMsgData(DataMessage*, int);
    Differentiator(float);
    ~Differentiator();
};