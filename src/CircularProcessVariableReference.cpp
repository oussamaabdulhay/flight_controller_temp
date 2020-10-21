#include "CircularProcessVariableReference.hpp"
#include <math.h>

CircularProcessVariableReference::CircularProcessVariableReference(block_id t_id) {
    _reference_type = reference_type::process_variable_ref;
    _reference_value = 0.0;
    _id = t_id;
}

CircularProcessVariableReference::~CircularProcessVariableReference() {

}

reference_type CircularProcessVariableReference::getReferenceType(){
    return _reference_type;
}

DataMessage* CircularProcessVariableReference::runTask(DataMessage* t_msg){

    Vector3DMessage* pos_msg = (Vector3DMessage*)t_msg;
    Vector3D<float> error;
    
    float pv = pos_msg->getData().x;

    //Adjust the reference to be between +PI and -PI
    if(fabs(_reference_value - pv) > M_PI){
        float a = fabs(_reference_value - pv);
        float b = 2 * M_PI - a;
        float sign_a;
        if(pv-_reference_value > 0){
            sign_a = 1;
        }else{
            sign_a = -1;
        }
        float ref_a = pv + b * sign_a;
        error.x = ref_a - pv;
    }else{
        error.x = _reference_value - pv;
    }

    error.y = 0.0 - pos_msg->getData().y;
    error.z = 0.0 - pos_msg->getData().z;
    
    m_error_msg.setVector3DMessage(error);

    return (DataMessage*) &m_error_msg;
}

void CircularProcessVariableReference::setReferenceValue(float t_reference_value){
    _reference_value = t_reference_value;
}