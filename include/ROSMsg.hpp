#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"
#include "PositionMsg.hpp"
#include "AttitudeMsg.hpp"
#include "HeadingMsg.hpp"

class ROSMsg : public DataMessage {

private:
    msg_type _type;
    ros_msg_type _ros_type;
    PositionMsg _position;
    AttitudeMsg _attitude;
    HeadingMsg _heading;
    Vector3D<float> _x_pv;
    Vector3D<float> _y_pv;
    Vector3D<float> _z_pv;
    Vector3D<float> _roll_pv;
    Vector3D<float> _pitch_pv;
    Vector3D<float> _yaw_pv;
    Vector3D<float> _yaw_rate_pv;
    float _control_system;
    control_system _source;
    float* _actuation;

public:
    msg_type getType();
    ros_msg_type getROSMsgType();
    const int getSize();
    DataMessage* Clone(){ return new ROSMsg(*this); }
    void setPosition(PositionMsg);
    void setAttitude(AttitudeMsg);
    void setHeading(HeadingMsg);
    void setX_PV(Vector3D<float>);
    void setY_PV(Vector3D<float>);
    void setZ_PV(Vector3D<float>);
    void setRoll_PV(Vector3D<float>);
    void setPitch_PV(Vector3D<float>);
    void setYaw_PV(Vector3D<float>);
    void setYawRate_PV(Vector3D<float>);
    void setControlSystem(float, control_system);
    void setActuation(float*);
    void setSource(control_system);
    PositionMsg getPosition() { return _position; }
    AttitudeMsg getAttitude() { return _attitude; }
    HeadingMsg getHeading() { return _heading; }
    Vector3D<float> getX_PV() { return _x_pv; }
    Vector3D<float> getY_PV() { return _y_pv; }
    Vector3D<float> getZ_PV() { return _z_pv; }
    Vector3D<float> getRoll_PV() { return _roll_pv; }
    Vector3D<float> getPitch_PV() { return _pitch_pv; }
    Vector3D<float> getYaw_PV() { return _yaw_pv; }
    Vector3D<float> getYawRate_PV() { return _yaw_rate_pv; }
    float getControlSystem() { return _control_system; }
    float* getActuation() { return _actuation; }
    control_system getSource() { return _source; }
    

    ROSMsg();
    ~ROSMsg();
};