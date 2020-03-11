#include "Saturation.hpp"

Saturation::Saturation(float t_clip_value) {
    _clip_value = t_clip_value;
}

Saturation::~Saturation() {

}

void Saturation::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::FLOAT){

        FloatMsg* float_msg = (FloatMsg*)t_msg;
        FloatMsg output;
        output.data = float_msg->data;

        if(output.data > _clip_value){
            output.data = _clip_value;
        }else if(output.data < -_clip_value){
            output.data = -_clip_value;
        }

        this->emitMsgUnicast((DataMessage*) &output,
                                    -1,
                                    ControlSystem::receiving_channels::ch_reference);   
    }
}
