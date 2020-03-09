#pragma once
#include "UserReference.hpp"
#include "DoubleMsg.hpp"

class Y_UserReference : public UserReference{

public:

    void receive_msg_data(DataMessage*);
    
    Y_UserReference();
    ~Y_UserReference();
};