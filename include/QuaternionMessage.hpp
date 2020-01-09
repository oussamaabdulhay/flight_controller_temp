#pragma once
#include "DataMessage.hpp"
#include "Quaternion.hpp"

class QuaternionMessage : public DataMessage{

private:
    msg_type _type;
    Quaternion _data;

public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new QuaternionMessage(*this); }
    Quaternion getData();

    QuaternionMessage();
    ~QuaternionMessage();

    void setQuaternionMessage(Quaternion);
};