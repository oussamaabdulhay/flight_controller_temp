#pragma once
#include "UserReference.hpp"
#include "DoubleMsg.hpp"

class Yaw_UserReference : public UserReference{

public:

    void receive_msg_data(DataMessage*);
    
    Yaw_UserReference();
    ~Yaw_UserReference();
};