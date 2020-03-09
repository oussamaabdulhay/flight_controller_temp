#pragma once
#include "UserReference.hpp"
#include "DoubleMsg.hpp"

class X_UserReference : public UserReference{

public:
    void receive_msg_data(DataMessage*);
    
    X_UserReference();
    ~X_UserReference();
};