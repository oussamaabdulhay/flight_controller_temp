#include "RollProviderMessage.hpp"

RollProviderMessage::RollProviderMessage() {
}

RollProviderMessage::~RollProviderMessage() {

}

msg_type RollProviderMessage::getType(){
    return _type;
}

const int RollProviderMessage::getSize()
{
    return sizeof(this);
}

Vector3D<float> RollProviderMessage::getData(){
    return _data;
}

void RollProviderMessage::setRollProviderMessage(Vector3D<float> t_data) {
    _type = msg_type::ROLL_PROVIDER;
    _data = t_data;
}
