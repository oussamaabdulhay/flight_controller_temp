#pragma once
#include "Quaternion.hpp"
#include "OptitrackMessage.hpp"
#include <math.h>
#include "ROSMsg.hpp"
#include "VelocityMsg.hpp"
#include "AccelerationMsg.hpp"
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3DMessage.hpp"

class OptiTrack : public msg_emitter, msg_receiver {

    private:
        Quaternion _bodyAtt;
        Vector3D<float> _bodyPos, _bodyVel, _bodyAcc;
        Vector3D<float> _prev_pos, _prev_vel; 
        Vector3D<float> _euler;
        float _bodyYawRate, _prev_heading, _bodyHeading;
        Quaternion _quat;
        int j = 0;
        double _time, _prev_time;
        ROSMsg m_ros_msg;
        Vector3DMessage _x_pv_msg, _y_pv_msg, _z_pv_msg, _yaw_pv_msg, _yaw_rate_pv_msg;
    public:
        PositionMsg getPosition();
        AttitudeMsg getAttitude(); 
        HeadingMsg getHeading(); 
        VelocityMsg getVelocity();
        AccelerationMsg getAcceleration();
        Vector3D<float> getBodyRate();

        void updateVelocity(double);
        void updateAcceleration(double);
        void updateYawRate(double);
        
        void receive_msg_data(DataMessage* t_msg);
        Vector3D<float> getEulerfromQuaternion(Quaternion);
        Quaternion getQuaternionfromEuler(Vector3D<float>);
        Quaternion getAttitudeHeading(); 

        OptiTrack();
        ~OptiTrack();
};