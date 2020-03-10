#pragma once
#include "Vector3DMessage.hpp"
#include "MsgEmitter.hpp"
#include "XSensMessage.hpp"
class XSens_IMU : public MsgReceiver, public MsgEmitter{

private:
    Vector3D<float> _bodyrate;
    Vector3D<float> last_euler_angles;
    Vector3DMessage _roll_pv_msg;
    Vector3DMessage _pitch_pv_msg;

public:

    void receiveMsgData(DataMessage* t_msg);
    XSens_IMU();
    ~XSens_IMU();
};