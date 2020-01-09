#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"
#include "Quaternion.hpp"

class SwitcherMessage : public DataMessage{

private:
    msg_type _type;
    Vector3D<float> _v3D_data;
    float _float_data;
    
public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new SwitcherMessage(*this); }
    Vector3D<float> getVector3DData();
    float getFloatData();
    void setSwitcherMessage(Vector3D<float>);
    void setSwitcherMessage(float);

    SwitcherMessage();
    ~SwitcherMessage();
};