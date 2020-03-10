#include "Differentiator.hpp"

#undef Differentiator_debug

Differentiator::Differentiator(float t_dt) {
    _dt = t_dt;
}

Differentiator::~Differentiator() {

}

void Differentiator::receiveMsgData(DataMessage* t_msg, int t_channel){
    #ifdef Differentiator_debug
    std::cout << "Differentiator::receiveMsgData(DataMessage* t_msg)" << std::endl;
    #endif
    if(t_msg->getType() == msg_type::VECTOR3D){
        #ifdef Differentiator_debug
        std::cout << "t_msg->getType() == msg_type::VECTOR3D" << std::endl;
        #endif
        Vector3DMessage* vector3d_data = (Vector3DMessage*)t_msg;

        this->differentiate(vector3d_data->getData());

    }else if(t_msg->getType() == msg_type::FLOAT){
        #ifdef Differentiator_debug
        std::cout << "t_msg->getType() == msg_type::FLOAT" << std::endl;
        #endif
        FloatMsg* float_data = (FloatMsg*)t_msg;

        this->differentiate(float_data->data);
    }
}

void Differentiator::differentiate(float t_float_data){
    
    float diff_value;
    FloatMsg output_msg;

    diff_value = (t_float_data - _old_float_data) / _dt;
    #ifdef Differentiator_debug
    std::cout << "this->emitMsgUnicastDefault((DataMessage*) &output_msg, PVConcatenator::receiving_channels::ch_pv_dot)" << std::endl;
    #endif
    this->emitMsgUnicast((DataMessage*) &output_msg, 
                                -1, 
                                PVConcatenator::receiving_channels::ch_pv_dot);
    _old_float_data = t_float_data;
}

void Differentiator::differentiate(Vector3D<float> t_vector3d_data){

    Vector3D<float> diff_values;
    Vector3DMessage output_msg;

    diff_values.x = (t_vector3d_data.x - _old_vector3d_data.x) / _dt;
    diff_values.y = (t_vector3d_data.y - _old_vector3d_data.y) / _dt;
    diff_values.z = (t_vector3d_data.z - _old_vector3d_data.z) / _dt;
    output_msg.setVector3DMessage(diff_values);
    #ifdef Differentiator_debug
    std::cout << "diff_values.x " << diff_values.x << ", diff_values.y " << diff_values.y << ", diff_values.z " << diff_values.z << std::endl;
    #endif
    this->emitMsgUnicast((DataMessage*) &output_msg, 
                                -1, 
                                PVConcatenator::receiving_channels::ch_pv_dot);
    _old_vector3d_data = t_vector3d_data;
}
    
    