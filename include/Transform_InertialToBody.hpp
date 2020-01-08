#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "UpdatePoseMessage.hpp"
#include "Yaw_PVProvider.hpp"
#include "ROSMsg.hpp"
#include "cmath"
#include "RotationMatrix3by3.hpp"
#include "ControlSystemMessage.hpp"

class Transform_InertialToBody : public msg_emitter, public msg_receiver {

private:
    Vector3D<float>* _inertial_command;
    Vector3D<float> _body_command;
    bool _opti_x_received = false, _opti_y_received = false, _current_yaw_received = false;
    UpdatePoseMessage _body_xy;
    RotationMatrix3by3 _rotation_matrix;
    ControlSystemMessage m_output_msg;
    control_system _source;

public:

    void receive_msg_data(DataMessage*);
    void transform();
    Transform_InertialToBody(control_system,  Vector3D<float>* );
    ~Transform_InertialToBody();
};