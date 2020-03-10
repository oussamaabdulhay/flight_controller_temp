#include "WrapAroundFunction.hpp"

void WrapAroundFunction::receiveMsgData(DataMessage* rec_msg){
    //this->receiveMsgData(rec_msg,msg_broadcast_channel);
}
void WrapAroundFunction::receiveMsgData(DataMessage* rec_msg, int ch){
    if (rec_msg->getType()==msg_type::VECTOR3D){
        Vector3D<double> msg_data=((Vector3DMessage*)rec_msg)->getData();
        msg_data.z=wrapAround(msg_data.z);
        Vector3DMessage emit_msg;
        emit_msg.setVector3DMessage(msg_data);
        this->emitMsgUnicastDefault(&emit_msg,
                            ch);
    }
}

double WrapAroundFunction::wrapAround(double input){ //TODO handle cases for abs(input)>2span
    if (input>max_val){
        return input-span;
    }
    else if (input<min_val){
        return input+span;
    }
    return input;
}

void WrapAroundFunction::assignParametersRange(double t_min_val,double t_max_val){
    min_val = t_min_val;
    max_val =t_max_val;
    span=max_val-min_val;
}