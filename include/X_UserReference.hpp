#pragma once
#include "UserReference.hpp"
#include "ControlSystemMessage.hpp"
#include "DoubleMsg.hpp"

class X_UserReference : public UserReference{

private:
    ControlSystemMessage m_output_msg;

public:
    enum unicast_addresses {broadcast};
    void receive_msg_data(DataMessage*);
    
    X_UserReference();
    ~X_UserReference();
};