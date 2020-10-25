#include "Switch.hpp"
#include "common_srv/FloatMsg.hpp"

Switch::Switch(std::function<bool(float,float)> t_operation, float t_trigger_value) {
    this->_operation = t_operation;
    this->_trigger_value = t_trigger_value;

    this->_input_port = new InputPort(ports_id::IP_0_DATA, this);
    this->_trigger_port = new InputPort(ports_id::IP_1_TRIGGER, this);
    this->_output_port_0 = new OutputPort(ports_id::OP_0_DATA, this);
    this->_output_port_1 = new OutputPort(ports_id::OP_1_DATA, this);
    this->_active_output_port = _output_port_0;
    _ports = {_input_port, _trigger_port, _output_port_0, _output_port_1};
}

Switch::~Switch() {

}

void Switch::triggerCallback(float t_current_value){

    if (this->_operation(t_current_value, _trigger_value)){
        _active_output_port = _output_port_1;
    }else{
        _active_output_port = _output_port_0;
    }
}

DataMessage* Switch::runTask(DataMessage* t_msg){
    _active_output_port->receiveMsgData(t_msg);
    return t_msg;
}

void Switch::process(DataMessage* t_msg, Port* t_port) {
    
    if(t_port->getID() == ports_id::IP_0_DATA){
        this->runTask(t_msg);

    }else if(t_port->getID() == ports_id::IP_1_TRIGGER){
        float data = 0.0;

        if(t_msg->getType() == msg_type::VECTOR3D){
            Vector3DMessage* vector_3D_msg = (Vector3DMessage*)t_msg;
            data = vector_3D_msg->getData().x; //TODO check if Z comes from a Vector3D
        }

        this->triggerCallback(data) ;
    }
}

std::vector<Port*> Switch::getPorts(){
    return _ports;
}
