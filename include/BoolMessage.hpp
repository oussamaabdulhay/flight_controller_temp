#pragma once
#include "DataMessage.hpp"

class BoolMessage : public DataMessage{

private:
    msg_type _type;
    bool _data;

public:

    msg_type getType();
    const int getSize();
    bool getData();
    void setBoolMessage(bool);
    DataMessage* Clone(){ return new BoolMessage(*this); }
    BoolMessage();
    ~BoolMessage();
};