#pragma once
#include "DataMessage.hpp"

class FlightScenarioMessage : public DataMessage{

private:
    float _x, _y, _z, _yaw;
    msg_type _type;
    msg_type_flight_scenario _fs_msg_type;

public:

    float getX();
    float getY();
    float getZ();
    float getYaw();
    msg_type getType();
    msg_type_flight_scenario getFlightScenarioMsgType();
    void setFSUserMessage(float, float, float, float);
    const int getSize();
    DataMessage* Clone(){ return new FlightScenarioMessage(*this); }

    FlightScenarioMessage(float, float, float, float);
    FlightScenarioMessage();
    ~FlightScenarioMessage();
};