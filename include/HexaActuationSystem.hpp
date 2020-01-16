#pragma once
#include "ActuationSystem.hpp"
#include "ControlSystemMessage.hpp"
#include "BoolMessage.hpp"
#include "ROSMsg.hpp"
#include <pthread.h>

// GEOMETRY
//      CW(3) (5)CCW                y
//          \ /                     ↑
// CCW(2) -- X -- (1)CW             |
//          / \                     +----→ x
//      CW(6) (4)CCW               z up
//
// For Positive Roll, all motors with negative X should be increased
// For Positive Pitch, all motors with negative Y should be increased
// For Positive Yaw, all motors with CW should be increased
// Mx = [x, y, direction, thottle]

class HexaActuationSystem : public ActuationSystem {

private:    
    std::vector<Actuator*> _actuators;
    ROSMsg ros_msg;
    const int _escMin = 1000;
    const int _escMin_armed = 1150;
    const int _escMax = 2000;
    bool _armed = false;
    float _movements[4]; //[roll, pitch, yaw, throttle]
    float _commands[6];
    float _geometry[6][4] = {{  1  * -1,         0 * -1,  1, 1},
                             { -1  * -1,         0 * -1, -1, 1},
                             {-0.5 * -1,  0.866025 * -1,  1, 1},
                             { 0.5 * -1, -0.866025 * -1, -1, 1},
                             { 0.5 * -1,  0.866025 * -1, -1, 1},
                             {-0.5 * -1, -0.866025 * -1,  1, 1}};
    static pthread_mutex_t lock;
public:

    void receive_msg_data(DataMessage* t_msg);
    void command();
    int constrain(float value, int min_value, int max_value);

    HexaActuationSystem(std::vector<Actuator*>);
    ~HexaActuationSystem();
};