#include "ControlSystem.hpp"
#include <fstream>
//std::ofstream write_data("/home/pedrohrpbs/catkin_ws_NAVIO//orientation_control_data_control.txt"); 

ControlSystem::ControlSystem(control_system t_control_system, block_frequency t_bf) : TimedBlock(t_bf) {
    // timer.tick();
    _control_system = t_control_system;
    
    controllerSwitcher = new Switcher(switcher_type::controller);
    referenceSwitcher = new Switcher(switcher_type::reference);
    _switchers = {controllerSwitcher, referenceSwitcher};
    _frequency = t_bf;
    _dt = 1.0f / (int)_frequency;

    this->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    this->add_callback_msg_receiver((msg_receiver*)referenceSwitcher);
    referenceSwitcher->add_callback_msg_receiver((msg_receiver*)controllerSwitcher);
    controllerSwitcher->add_callback_msg_receiver((msg_receiver*)this);
}

ControlSystem::~ControlSystem() {

}

void ControlSystem::receive_msg_data(DataMessage* t_msg){
    // (2)
    if(t_msg->getType() == msg_type::switcher){

        SwitcherMessage* switcher_msg = (SwitcherMessage*)t_msg;

        m_output_msg.setControlSystemMessage(this->getControlSystemType(), control_system_msg_type::to_system, switcher_msg->getFloatData());

        this->emit_message((DataMessage*) &m_output_msg);

        //Emiting msg to ROSUnit
        m_ros_msg.setControlSystem(switcher_msg->getFloatData(), this->getControlSystemType());
        this->emit_message((DataMessage*) &m_ros_msg);
            
    // (3)
    }else if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        //TODO make the naming more clear
        //std::cout << "MESSAGE RECEIVED: " << control_system_msg->getData() <<std::endl;
        if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::to_system){
            m_output_msg.setControlSystemMessage(this->getControlSystemType(), control_system_msg_type::SETREFERENCE, control_system_msg->getData());
            this->emit_message((DataMessage*) &m_output_msg);
        }//TODO add the update parameters msg

    }else if(t_msg->getType() == msg_type::SWITCHBLOCK){

        SwitchBlockMsg* switch_msg = (SwitchBlockMsg*)t_msg;
        this->emit_message((DataMessage*) switch_msg);
    
    }
}

void ControlSystem::receive_msg_data(DataMessage* t_msg, int t_channel){
    //std::cout <<" you shouldn't be here " << std::endl;
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* provider = (Vector3DMessage*)t_msg;
        Vector3D<float> pv_data = provider->getData();
        std::cout << "pv_data.x " << pv_data.x << ", pv_data.y " << pv_data.y << ", pv_data.z " << pv_data.z << std::endl;
        m_provider_data_msg.setControlSystemMessage(this->getControlSystemType(), control_system_msg_type::PROVIDER, pv_data);
        this->emit_message((DataMessage*) &m_provider_data_msg);
    }
}

control_system ControlSystem::getControlSystemType(){
    return _control_system;
}
//TODO remove
void ControlSystem::getStatus(){
    
    for(Switcher* s : _switchers){
        if(s->getActiveBlock() != nullptr){
            std::cout << "For Control System " << static_cast<int>(_control_system) << std::endl;
            std::cout << "For switcher " << (int)(s->getType()) << " the active block is " << (int)(s->getActiveBlock()->getID()) << std::endl;
        }     
    }
}


void ControlSystem::loopInternal(){
}

void ControlSystem::addBlock(Block* t_block){
    m_add_block_msg.setControlSystemMessage(control_system_msg_type::add_block, t_block);

    this->emit_message((DataMessage*) &m_add_block_msg);
}

void ControlSystem::runTasks(){
}

// write_data << roll_provider->getData().y << ", " << timer.tockMilliSeconds() <<"\n";
// timer.tick();