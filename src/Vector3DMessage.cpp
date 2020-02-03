#include "Vector3DMessage.hpp"

Vector3DMessage::Vector3DMessage() {
}

Vector3DMessage::~Vector3DMessage() {

}

msg_type Vector3DMessage::getType(){
    return _type;
}

const int Vector3DMessage::getSize()
{
    return sizeof(this);
}

Vector3D<float> Vector3DMessage::getData(){
    return _data;
}

Vector3D<double> Vector3DMessage::getDataDouble(){
    return _data_double;
}

void Vector3DMessage::setVector3DMessage(Vector3D<float> t_data) {
    _type = msg_type::VECTOR3D;
    _data = t_data;
}


void Vector3DMessage::setVector3DMessage(Vector3D<double> t_data) {
    _type = msg_type::VECTOR3D;
    _data_double = t_data;
}