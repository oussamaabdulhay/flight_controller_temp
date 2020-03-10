#include "Differentiator.hpp"

#undef Differentiator_debug

Differentiator::Differentiator(float t_dt) {
    _dt = t_dt;
}

Differentiator::~Differentiator() {

}

void Differentiator::receiveMsgData(DataMessage* t_msg, int t_channel){
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_data = (Vector3DMessage*)t_msg;

        Vector3D<float> diff_values;
        Vector3DMessage output_msg;

        diff_values.x = (vector3d_data->getData().x - _old_vector3d_data.x) / _dt;
        diff_values.y = (vector3d_data->getData().y - _old_vector3d_data.y) / _dt;
        diff_values.z = (vector3d_data->getData().z - _old_vector3d_data.z) / _dt;
        
        std::cout << "Differentiator diff_values.x " << diff_values.x << "\n";

        output_msg.setVector3DMessage(diff_values);
        this->emitMsgUnicastDefault((DataMessage*) &output_msg, 
                            PVConcatenator::receiving_channels::ch_pv_dot);
        
        _old_vector3d_data = vector3d_data->getData();

    }else if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* float_data = (FloatMsg*)t_msg;

        float diff_value;
        FloatMsg output_msg;

        diff_value = (float_data->data - _old_float_data) / _dt;
        output_msg.data = diff_value;
        this->emitMsgUnicastDefault((DataMessage*) &output_msg, 
                            PVConcatenator::receiving_channels::ch_pv_dot);
                            
        _old_float_data = float_data->data;
    }
}