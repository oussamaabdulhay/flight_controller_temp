#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"
#include "Quaternion.hpp"

class XSensMessage : public DataMessage{


private:
    Vector3D<float> _angular_velocity;
    Quaternion _orientation; 
    msg_type _type;

public:
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new XSensMessage(*this); }
    Vector3D<float> getAngularVelocity();
    Quaternion getOrientation();
    void setXSensMessage(Vector3D<float>, Quaternion);
    void setAngularVelocity(Vector3D<float>);
    void setOrientation(Quaternion);

    XSensMessage();
    ~XSensMessage();
};