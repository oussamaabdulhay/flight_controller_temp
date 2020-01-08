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

void Switcher::receive_msg_data(DataMessage* t_msg){
    
    if(t_msg->getType() == msg_type::control_system){

        ControlSystemMessage* control_system_msg = (ControlSystemMessage*)t_msg;
        Block* block_to_add = control_system_msg->getBlockToAdd();
        
        if (control_system_msg->getControlSystemMsgType() == control_system_msg_type::add_block
                    && static_cast<int>(this->getType()) == static_cast<int>(block_to_add->getType())){ //TODO Refactor
                
            Block* block_to_add = control_system_msg->getBlockToAdd();
            if(this->_blocks.empty()){
                _active_block = block_to_add;
            }
            this->addBlock(block_to_add);
        //(7)
        } else if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::PROVIDER){
            
            Vector3D<float> data_provided = control_system_msg->getV3DData();
            
            if(_active_block->getType() == block_type::reference){
                
                Reference* reference_block = (Reference*)_active_block;

                m_process_variable.setVector3DMessage(data_provided);
                DataMessage* ref_output_msg = reference_block->receive_msg_internal((DataMessage*) &m_process_variable);
                Vector3DMessage* v3d_ref_output_msg = (Vector3DMessage*)ref_output_msg;

                m_reference_msg.setSwitcherMessage(v3d_ref_output_msg->getData()); 
                this->emit_message((DataMessage*) &m_reference_msg);
            }
        
        //(9)
        } else if(control_system_msg->getControlSystemMsgType() == control_system_msg_type::SETREFERENCE){
            float reference = control_system_msg->getData();
            if(_active_block->getType() == block_type::reference){
                //std::cout << "REFERENCE RECEIVED: " << reference << std::endl;
                Reference* _reference_block = (Reference*)_active_block;
                _reference_block->setReferenceValue(reference);
            }
        }

    //(8)  //Controller Switcher
    }else if(t_msg->getType() == msg_type::switcher){

        SwitcherMessage* switcher_msg = (SwitcherMessage*)t_msg;
                      
        DataMessage* output = _active_block->receive_msg_internal((DataMessage*)switcher_msg);

        FloatMessage* data = (FloatMessage*)output;
        m_out_switcher_msg.setSwitcherMessage(data->getData());
        
        this->emit_message((DataMessage*) &m_out_switcher_msg);

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
