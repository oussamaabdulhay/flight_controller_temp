#include "ProcessVariableReference.hpp"

ProcessVariableReference::ProcessVariableReference(block_id t_id) {
    _reference_type = reference_type::process_variable_ref;
    _reference_value = 0.0;
    _id = t_id;
}

ProcessVariableReference::~ProcessVariableReference() {

}

reference_type ProcessVariableReference::getReferenceType(){
    return _reference_type;
}

DataMessage* ProcessVariableReference::runTask(DataMessage* t_msg){
    // std::cout << " My current Reference Variable Value: " << _reference_value << std::endl;

    Vector3DMessage* pos_msg = (Vector3DMessage*)t_msg;
    // std::cout << "Data received (Process Variable): " << pos_msg->getData().x << std::endl;
    Vector3D<float> error;

    error.x = _reference_value - pos_msg->getData().x;
    error.y = 0.0 - pos_msg->getData().y ;
    error.z = 0.0 - pos_msg->getData().z;
  
    m_error_msg.setVector3DMessage(error);

    return (DataMessage*) &m_error_msg;
}

void ProcessVariableReference::setReferenceValue(float t_reference_value){
    _reference_value = t_reference_value;
}