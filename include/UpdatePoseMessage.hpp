#pragma once
#include "DataMessage.hpp"

class UpdatePoseMessage : public DataMessage{

private:
    float _x, _y, _z, _yaw;
    msg_type _type;
    msg_type_reference _ref_type;

public:

    float getX();
    float getY();
    float getZ();
    float getYaw();
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new UpdatePoseMessage(*this); }
    void setPoseMessage(float, float, float, float);
    void setPoseX(float);
    void setPoseY(float);
    void setPoseZ(float);
    void setPoseYaw(float);
    msg_type_reference getRefType();
    
    UpdatePoseMessage(float,float,float,float);
    UpdatePoseMessage();
    ~UpdatePoseMessage();
};