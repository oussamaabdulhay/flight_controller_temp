#pragma once
#include "DataMessage.hpp"

class ResetControllerMsg : public DataMessage{

private:
    msg_type _type;
    int _data;

public:

    msg_type getType();
    const int getSize();
    DataMessage* Clone(){ return new ResetControllerMsg(*this); }
    int getData();
    void setResetControllerMessage(int);

    ResetControllerMsg();
    ~ResetControllerMsg();
};