#pragma once
#include "Roll_PVProvider.hpp"
#include "Pitch_PVProvider.hpp"
#include "AttitudeMsg.hpp"
#include "BodyRateMsg.hpp"

class XSens_IMU : public Roll_PVProvider, public Pitch_PVProvider, public msg_receiver{

private:
    AttitudeMsg _attitude;
    BodyRateMsg _bodyrate;
public:

    void receive_msg_data(DataMessage* t_msg);
    AttitudeMsg getAttitude();
    Vector3D<float> getBodyRate();
    XSens_IMU();
    ~XSens_IMU();
};