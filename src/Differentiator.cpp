#include "Differentiator.hpp"

Differentiator::Differentiator(float t_dt) {
    _dt = t_dt;
}

Differentiator::~Differentiator() {

}

void Differentiator::receive_msg_data(DataMessage* t_msg){
    std::cout << "Differentiator::receive_msg_data(DataMessage* t_msg)" << std::endl;
    if(t_msg->getType() == msg_type::VECTOR3D){

        Vector3DMessage* vector3d_data = (Vector3DMessage*)t_msg;

        this->differentiate(vector3d_data->getData());

    }else if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* float_data = (FloatMsg*)t_msg;

        this->differentiate(float_data->data);
    }
}

void Differentiator::differentiate(float t_float_data){
    
    float diff_value;
    FloatMsg output_msg;

    diff_value = (t_float_data - _old_float_data) / _dt;

    this->emit_message((DataMessage*) &output_msg);
    _old_float_data = t_float_data;
}

void Differentiator::differentiate(Vector3D<float> t_vector3d_data){

    Vector3D<float> diff_values;
    Vector3DMessage output_msg;

    diff_values.x = (t_vector3d_data.x - _old_vector3d_data.x) / _dt;
    diff_values.y = (t_vector3d_data.y - _old_vector3d_data.y) / _dt;
    diff_values.z = (t_vector3d_data.z - _old_vector3d_data.z) / _dt;

    this->emit_message((DataMessage*) &output_msg);
    _old_vector3d_data = t_vector3d_data;
}
    
    