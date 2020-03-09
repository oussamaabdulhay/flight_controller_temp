#include "HexaActuationSystem.hpp"
pthread_mutex_t HexaActuationSystem::lock;

HexaActuationSystem::HexaActuationSystem(std::vector<Actuator*> t_actuators) : ActuationSystem(t_actuators){
    _actuators = t_actuators;
}

HexaActuationSystem::~HexaActuationSystem() {

}

void HexaActuationSystem::command(){

    for(int i = 0; i < 6; i++){
        _commands[i] = 0;
    }
    
    //Update pulse values
    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 4; j++){
            _commands[i] += _geometry[i][j] * _movements[j];
        }
    }

    //_movements (PID outputs) should be between 0 and 1. Thus, we have to adjust for the range 1150 to 2000 on _commands.
    //Normalize and Constrain

    for(int i = 0; i < 6; i++){
        if(_armed){
            _commands[i] = (_commands[i] * (_escMax-_escMin_armed)) + _escMin_armed;
            _commands[i] = this->constrain(_commands[i], _escMin_armed, _escMax);
        }else{
            _commands[i] = _escMin;
        }
    }
    
    //Actuate
    for(int i = 0; i < 6; i++){
        _actuators[i]->applyCommand(_commands[i]);
    }

    ros_msg.setActuation(&_commands[0]);
    this->emit_message_unicast((DataMessage*) &ros_msg, 
                                HexaActuationSystem::unicast_addresses::unicast_ActuationSystem_commands,
                                ROSUnit_BroadcastData::ros_broadcast_channels::actuation);

    ros_msg.setArmed(_armed);
    this->emit_message_unicast((DataMessage*) &ros_msg,
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

void HexaActuationSystem::receive_msg_data(DataMessage* t_msg){

    if(t_msg->getType() == msg_type::control_system){
        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            
            if(_armed){
                switch (control_system_msg->getSource())
                {
                case control_system::roll:
                {
                    _movements[0] = control_system_msg->getData();
                    break;
                }
                case control_system::pitch:
                {
                    _movements[1] = control_system_msg->getData();
                    this->command();
                    break;
                }
                case control_system::yaw_rate:
                {
                    _movements[2] = control_system_msg->getData();
                    break;
                }
                case control_system::z:
                {
                    _movements[3] = control_system_msg->getData();
                    break;
                }
                default:
                    break;
                }
            }else{
                _movements[0] = 0.0;
                _movements[1] = 0.0;
                _movements[2] = 0.0;
                _movements[3] = 0.0;
                this->command();
            }
        }
          
    }else if(t_msg->getType() == msg_type::BOOLEAN){

        BoolMessage* bool_msg = (BoolMessage*)t_msg;
        _armed = bool_msg->getData();

    }
}
