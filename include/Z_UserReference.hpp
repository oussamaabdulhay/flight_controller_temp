#pragma once
#include "UserReference.hpp"
#include "DoubleMsg.hpp"

class Z_UserReference : public UserReference{

public:

    void receive_msg_data(DataMessage*);
    
    Z_UserReference();
    ~Z_UserReference();
};