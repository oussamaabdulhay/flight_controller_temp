#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"

#include "cmath"
#include "RotationMatrix3by3.hpp"
#include "ControlSystemMessage.hpp"
#include <atomic>
#include "Vector3DMessage.hpp"
#include "FloatMsg.hpp"
class Transform_InertialToBody : public msg_emitter, public msg_receiver {

private:
    static std::atomic<float>  _inertial_command_x;
    static std::atomic<float>  _inertial_command_y;
    static std::atomic<float>  _inertial_command_z;
    bool _opti_x_received = false, _opti_y_received = false, _current_yaw_received = false;
    UpdatePoseMessage _body_xy;
    RotationMatrix3by3 _rotation_matrix;
    ControlSystemMessage m_output_msg;
    control_system _source;

public:

    void receive_msg_data(DataMessage*);
    void receive_msg_data(DataMessage*, int);
    Transform_InertialToBody(control_system);
    ~Transform_InertialToBody();
};