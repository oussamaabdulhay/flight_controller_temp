#pragma once
#include "DataMessage.hpp"
#include <vector>
#include "PID_values.hpp"
#include "MRFT_values.hpp"

class ControllerMessage : public DataMessage{

private:
    msg_type _type;
    block_id _id;
    PID_parameters _pid_param;
    MRFT_parameters _mrft_param;

public:
   
    const int getSize();
    msg_type getType();
    controller_msg_type getControllerMsgType();
    void setPIDParam(PID_parameters);
    void set_dt(float);
    void setMRFTParam(MRFT_parameters);
    DataMessage* Clone(){ return new ControllerMessage(*this); }
    MRFT_parameters getMRFTParam(){ return _mrft_param; }
    PID_parameters getPIDParam(){ return _pid_param; }
    block_id getID() { return _id; };

    ControllerMessage();
    ~ControllerMessage();
};