#pragma once
#include "Reference.hpp"
#include "common_srv/FloatMsg.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include <atomic>
class ProcessVariableReference : public Reference{

private:
    reference_type _reference_type;
    std::atomic<float> _reference_value; // TODO-Chehadeh: make thread-safe
    Vector3DMessage m_error_msg;
    block_id _id;

public:
    reference_type getReferenceType();
      
    void setReferenceValue(float);
    DataMessage* runTask(DataMessage*);
    block_id getID(){ return _id; }
    ProcessVariableReference(block_id t_id);
    ~ProcessVariableReference();
};