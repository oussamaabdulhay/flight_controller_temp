#pragma once
#include "Reference.hpp"
#include "FloatMessage.hpp"
#include "Vector3D.hpp"
#include "Vector3DMessage.hpp"

class CircularProcessVariableReference : public Reference{

private:
    reference_type _reference_type;
    float _reference_value;
    Vector3DMessage m_error_msg;
    block_id _id;

public:
    reference_type getReferenceType();
      
    void setReferenceValue(float);
    DataMessage* receive_msg_internal(DataMessage*);
    block_id getID(){ return _id; }
    CircularProcessVariableReference(block_id t_id);
    ~CircularProcessVariableReference();
};