#pragma once
#include "AttitudeMsg.hpp"
#include "BodyRateMsg.hpp"
#include "QuaternionMessage.hpp"
#include "Vector3DMessage.hpp"
#include "thread_terminal_unit.hpp"
#include "MsgEmitter.hpp"
#include "XSensMessage.hpp"
#include "RollProviderMessage.hpp"
#include "PitchProviderMessage.hpp"

class XSens_IMU : public msg_receiver, public msg_emitter{

private:
    Vector3D<float> _bodyrate;
    Vector3D<float> last_euler_angles;
    Vector3DMessage _roll_pv_msg;
    Vector3DMessage _pitch_pv_msg;

public:

    void receive_msg_data(DataMessage* t_msg);
    AttitudeMsg getAttitude();
    Vector3D<float> getBodyRate();
    Vector3D<float> getEulerfromQuaternion(Quaternion);
    XSens_IMU();
    ~XSens_IMU();
};