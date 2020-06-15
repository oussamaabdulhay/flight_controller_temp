#include "QuadActuationSystem.hpp"
pthread_mutex_t QuadActuationSystem::lock;

QuadActuationSystem::QuadActuationSystem(std::vector<Actuator*> t_actuators) : ActuationSystem(t_actuators){
    _actuators = t_actuators;
}

QuadActuationSystem::~QuadActuationSystem() {

}

void QuadActuationSystem::command(){


    for(int i = 0; i < NUM_MOTORS; i++){
        _commands[i] = 0.0;
    }

    //Update pulse values
    for(int i = 0; i < NUM_MOTORS; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _movements[j];
        }
    }

    //Let's limit the PID output considering that a value of 0 is the minimum output and 1 is the maximum output. 
    //Thus, we have to adjust for the range 1150 to 2000 on _commands.
    //Normalize and Constrain

    for(int i = 0; i < NUM_MOTORS; i++){
        if(_armed){
            _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
            _commands[i] = this->constrain(_commands[i], _escMin_armed, _escMax);
        }else{
            _commands[i] = _escMin;
        }
    }

    //Actuate
    for(int i = 0; i < NUM_MOTORS; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }

    VectorDoubleMsg commands_msg;
    commands_msg.data = _commands;
    this->emitMsgUnicast((DataMessage*) &commands_msg, 
                                QuadActuationSystem::unicast_addresses::unicast_ActuationSystem_commands,
                                ROSUnit_BroadcastData::ros_broadcast_channels::actuation);

    BooleanMsg armed_msg;
    armed_msg.data = _armed;
    this->emitMsgUnicast((DataMessage*) &armed_msg,
                                QuadActuationSystem::unicast_addresses::unicast_ActuationSystem_armed,
                                ROSUnit_BroadcastData::ros_broadcast_channels::armed);

}

int QuadActuationSystem::constrain(float value, int min_value, int max_value) {
    
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return int(value);
}

void QuadActuationSystem::receiveMsgData(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::BOOLEAN){

        BooleanMsg* bool_msg = (BooleanMsg*)t_msg;
        _armed = bool_msg->data;

    }
}

void QuadActuationSystem::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;

        if(_armed){
            _movements[t_channel] = float_msg->data;
            if(t_channel == (int)receiving_channels::ch_pitch){ //This sends the commands to the motors on the fastest loop, avoiding thread issues.
                this->command();
            }
        }else{
            _movements[0] = 0.0;
            _movements[1] = 0.0;
            _movements[2] = 0.0;
            _movements[3] = 0.0;
            this->command();
        }     
    }
}
