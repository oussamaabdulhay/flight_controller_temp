#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"

class Vector3DMessage : public DataMessage{

private:
    msg_type _type;
    Vector3D<float> _data;
    Vector3D<double> _data_double;
public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new Vector3DMessage(*this); }
    Vector3D<float> getData();
    Vector3D<double> getDataDouble();

    Vector3DMessage();
    ~Vector3DMessage();

    void setVector3DMessage(Vector3D<float>);
    void setVector3DMessage(Vector3D<double>);
};