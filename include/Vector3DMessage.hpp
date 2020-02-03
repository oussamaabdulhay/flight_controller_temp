#pragma once
#include "DataMessage.hpp"
#include "Vector3D.hpp"

class Vector3DMessage : public DataMessage{

private:
    msg_type _type;
    Vector3D<double> _data;
public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new Vector3DMessage(*this); }
    Vector3D<double> getData();

    Vector3DMessage();
    ~Vector3DMessage();
    
    void setVector3DMessage(Vector3D<float>);
    void setVector3DMessage(Vector3D<double>);
};