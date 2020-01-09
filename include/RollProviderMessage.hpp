#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"

class RollProviderMessage : public DataMessage {
private:
    msg_type _type;
    Vector3D<float> _data;

public:
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new RollProviderMessage(*this); }
    Vector3D<float> getData();
    void setRollProviderMessage(Vector3D<float>);

    RollProviderMessage();
    ~RollProviderMessage();
};
