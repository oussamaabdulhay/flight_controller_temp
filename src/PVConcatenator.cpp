#include "PVConcatenator.hpp"

PVConcatenator::PVConcatenator(concatenation_axes t_selected_concatenation_axes) {
    _selected_concatenation_axes = t_selected_concatenation_axes;
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg){
    std::cout << "PVConcatenator::receive_msg_data(DataMessage* t_msg)" << std::endl;
}

void PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel){
    //std::cout << "PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel)" << std::endl;
    if(t_msg->getType() == msg_type::VECTOR3D){
        //std::cout << "t_msg->getType() == msg_type::VECTOR3D" << std::endl;
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        if (t_channel==(int)ch_pv){
            //std::cout << "t_channel==(int)ch_pv" << std::endl;
            if (_selected_concatenation_axes==conc_x_axis){
                //std::cout << "_selected_concatenation_axes==conc_x_axis " << v3d_msg->getData().x << std::endl;
                pv_vector.x=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_vector.x=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_vector.x=v3d_msg->getData().z;
            }
            Vector3DMessage pv_vector_msg;
            pv_vector_msg.setVector3DMessage(pv_vector);
            std::cout << "pv_vector.x " << pv_vector.x << ", pv_vector.y " << pv_vector.y << ", pv_vector.z " << pv_vector.z << std::endl;
            this->emit_message((DataMessage*) &pv_vector_msg);
        }
        else if(t_channel==(int)ch_pv_dot){
            //std::cout << "t_channel==(int)ch_pv_dot" << std::endl;
            if (_selected_concatenation_axes==conc_x_axis){
                //std::cout << "_selected_concatenation_axes==conc_x_axis " << v3d_msg->getData().x << std::endl;
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
