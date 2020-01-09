#include "PitchProviderMessage.hpp"

PitchProviderMessage::PitchProviderMessage() {
}

PitchProviderMessage::~PitchProviderMessage() {

}

msg_type PitchProviderMessage::getType(){
    return _type;
}

const int PitchProviderMessage::getSize()
{
    return sizeof(this);
}

Vector3D<float> PitchProviderMessage::getData(){
    return _data;
}

void PitchProviderMessage::setPitchProviderMessage(Vector3D<float> t_data) {
    _type = msg_type::PITCH_PROVIDER;
    _data = t_data;
}
