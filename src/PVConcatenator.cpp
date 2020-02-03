#include "PVConcatenator.hpp"

PVConcatenator::PVConcatenator(concatenation_axes t_selected_concatenation_axes) {
    _selected_concatenation_axes = t_selected_concatenation_axes;
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg){

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel){
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        if (t_channel==(int)ch_pv){
            if (_selected_concatenation_axes==conc_x_axis){
                pv_vector.x=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_vector.x=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_vector.x=v3d_msg->getData().z;
            }
            Vector3DMessage pv_vector_msg;
            pv_vector_msg.setVector3DMessage(pv_vector);
            emit_message(&pv_vector_msg);
        }
        else if(t_channel==(int)ch_pv_dot){
            if (_selected_concatenation_axes==conc_x_axis){
                pv_vector.y=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_vector.y=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_vector.y=v3d_msg->getData().z;
            }
        }
        else if(t_channel==(int)ch_pv_dot_dot){
            if (_selected_concatenation_axes==conc_x_axis){
                pv_vector.z=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_vector.z=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_vector.z=v3d_msg->getData().z;
            }
        }
    }    
}
