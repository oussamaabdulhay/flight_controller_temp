#include "HexaActuationSystem.hpp"
pthread_mutex_t HexaActuationSystem::lock;

HexaActuationSystem::HexaActuationSystem(std::vector<Actuator*> t_actuators) : ActuationSystem(t_actuators){
    _actuators = t_actuators;
}

HexaActuationSystem::~HexaActuationSystem() {

}

void HexaActuationSystem::command(){

    for(int i = 0; i < 6; i++){
        _commands[i] = 0.0;
    }

    //Update pulse values
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _u[j];
        }
    }

    //_u (PID outputs) should be between 0 and 1. Thus, we have to adjust for the range 1150 to 2000 on _commands.
    //Normalize and Constrain

    for(int i = 0; i < 6; i++){
        _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
    }

    float min_command = _commands[0];

    for(int i = 1; i < 6; i++){
        if(_commands[i] < min_command){
            min_command = _commands[i];
        }
    }

    float bias = 0;

    if(min_command < _escMin_armed){
        bias = _escMin_armed - min_command;
        
        for(int i = 0; i < 6; i++){
            _commands[i] = _commands[i] + bias;
        }
    }

    for(int i = 0; i < 6; i++){
        if(_armed){
            _commands[i] = this->constrain(_commands[i], _escMin_armed, _escMax);
        }else{
            _commands[i] = _escMin;
        }  
    }

    //Actuate
    for(int i = 0; i < 6; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }

    VectorDoubleMsg commands_msg;
    commands_msg.data = _commands;
    this->emitMsgUnicast((DataMessage*) &commands_msg, 
                                HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_commands,
                                ROSUnit_BroadcastData::ros_broadcast_channels::actuation);

    BooleanMsg armed_msg;
    armed_msg.data = _armed;
    this->emitMsgUnicast((DataMessage*) &armed_msg,
                                HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_armed,
                                ROSUnit_BroadcastData::ros_broadcast_channels::armed);

}

int HexaActuationSystem::constrain(float value, int min_value, int max_value) {
    
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return int(value);
}

void HexaActuationSystem::receiveMsgData(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::BOOLEAN){

        BooleanMsg* bool_msg = (BooleanMsg*)t_msg;
        _armed = bool_msg->data;

    }
}

void HexaActuationSystem::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* float_msg = (FloatMsg*)t_msg;

        if(_armed){
            _u[t_channel] = float_msg->data;
            if(t_channel == (int)receiving_channels::ch_pitch){ //This sends the commands to the motors on the fastest loop, avoiding thread issues.
                this->command();
            }
        }else{
            _u[0] = 0.0;
            _u[1] = 0.0;
            _u[2] = 0.0;
            _u[3] = 0.0;
            this->command();
        }     
    }
}
