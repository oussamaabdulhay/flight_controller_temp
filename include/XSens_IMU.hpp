#pragma once
#include "common_srv/Vector3DMessage.hpp"
#include "common_srv/MsgEmitter.hpp"
#include "XSensMessage.hpp"
#include "common_types.hpp"

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