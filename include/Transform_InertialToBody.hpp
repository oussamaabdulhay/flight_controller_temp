#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"
#include "Yaw_PVProvider.hpp"
#include "ROSMsg.hpp"
#include "cmath"
#include "RotationMatrix3by3.hpp"

class Transform_InertialToBody : public msg_emitter, public msg_receiver {

private:
    Vector3D<float> _inertial_command, _body_command;
    bool _opti_x_received = false, _opti_y_received = false, _current_yaw_received = false;
    UpdatePoseMessage _body_xy;
    RotationMatrix3by3 _rotation_matrix;
    
public:

    void receive_msg_data(DataMessage*);
    void transform();
    Transform_InertialToBody();
    ~Transform_InertialToBody();
};