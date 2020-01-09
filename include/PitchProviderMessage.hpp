#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"

class PitchProviderMessage : public DataMessage {
private:
    msg_type _type;
    Vector3D<float> _data;

public:
    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new PitchProviderMessage(*this); }
    Vector3D<float> getData();
    void setPitchProviderMessage(Vector3D<float>);

    PitchProviderMessage();
    ~PitchProviderMessage();
};
