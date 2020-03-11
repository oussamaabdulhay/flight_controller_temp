#include "Switcher.hpp"

Switcher::Switcher(switcher_type t_type) {
    _type = t_type;
    _active_block = nullptr;
}

Switcher::~Switcher() {

}

void Switcher::addBlock(Block* b){   
    _blocks.push_back(b);
}

switcher_type Switcher::getType(){
    return _type;
}

Block* Switcher::getActiveBlock(){
    return _active_block;
}

void Switcher::receiveMsgData(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        Block* block_to_add = control_system_msg->getBlockToAdd(); 
        
        if (control_system_msg->getControlSystemMsgType() == control_system_msg_type::add_block
                    && static_cast<int>(this->getType()) == static_cast<int>(block_to_add->getType())){ //TODO Refactor
                
            if(this->_blocks.empty()){
                _active_block = block_to_add;
            }
            this->addBlock(block_to_add);
        }

    }else if(t_msg->getType() == msg_type::SWITCHBLOCK){
        SwitchBlockMsg* switch_msg = (SwitchBlockMsg*)t_msg;

        block_id block_in_id = static_cast<block_id>(switch_msg->getBlockToSwitchIn());
        block_id block_out_id = static_cast<block_id>(switch_msg->getBlockToSwitchOut());

        Block* block_in=nullptr;
        Block* block_out=nullptr;

        for (_it = _blocks.begin(); _it != _blocks.end(); ++_it){

            block_id this_id = (*_it)->getID();

            if(this_id == block_in_id || this_id == block_out_id){

                if(this_id == block_in_id){
                    block_in = *_it;
                }else if(this_id == block_out_id){
                    block_out = *_it;
                }
            }
        }

        if(block_in && block_out){

            block_in->switchIn(block_out->switchOut());
            _active_block = block_in;

        }else{
            //TODO make logger
        }
    }
}

void Switcher::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::VECTOR3D){
       Vector3DMessage* vector3D_msg = (Vector3DMessage*)t_msg;

        if(t_channel == receiving_channels::ch_provider){
            std::cout << "Switcher ch_provider vector3D_msg.x " << vector3D_msg->getData().x << "\n";
            DataMessage* ref_output_msg = _active_block->runTask((DataMessage*) vector3D_msg);
            std::cout << "Switcher ch_provider ref_output_msg.x " << ((Vector3DMessage*)ref_output_msg)->getData().x << "\n";
            this->emitMsgUnicast((DataMessage*) ref_output_msg,
                                Switcher::unicast_addresses::unicast_controller_switcher,
                                Switcher::receiving_channels::ch_error);

        }else if(t_channel == receiving_channels::ch_error){ 
            Controller* controller_block = (Controller*)_active_block; 
            std::cout << "Switcher ch_error vector3D_msg.x " << vector3D_msg->getData().x << "\n";                    
            DataMessage* ctrl_output_msg = _active_block->runTask((DataMessage*) vector3D_msg);
            std::cout << "Switcher ch_provider ctrl_output_msg.data " << ((FloatMsg*)ctrl_output_msg)->data << "\n";
            this->emitMsgUnicast((DataMessage*) ctrl_output_msg,
                                Switcher::unicast_addresses::unicast_control_system,
                                ControlSystem::receiving_channels::ch_controller);

        }
    }else if(t_msg->getType() == msg_type::FLOAT){ //ch_reference, no need for checks because it's the only FLOAT msg.
        FloatMsg* float_msg = (FloatMsg*)t_msg;
        std::cout << "Switcher ch_reference float_msg->data " << float_msg->data << "\n";
        ((Reference*)_active_block)->setReferenceValue(float_msg->data);
        std::cout << "REFERENCE SET" << "\n";
    }
}


