#include "PVConcatenator.hpp"

#define PVConc_debug

PVConcatenator::PVConcatenator(concatenation_axes t_selected_concatenation_axes, act_on t_act_on) {
    _selected_concatenation_axes = t_selected_concatenation_axes;
    _act_on = t_act_on;
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg){
    #ifdef PVConc_debug
    std::cout << "PVConcatenator::receive_msg_data(DataMessage* t_msg)" << std::endl;
    #endif
}

void PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel){
    #ifdef PVConc_debug
    //std::cout << "PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel)" << std::endl;
    #endif
    if(t_msg->getType() == msg_type::VECTOR3D){
        #ifdef PVConc_debug
        //std::cout << "t_msg->getType() == msg_type::VECTOR3D" << std::endl;
        #endif
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        if (t_channel==(int)ch_pv){
            #ifdef PVConc_debug
            //std::cout << "t_channel==(int)ch_pv" << std::endl;
            #endif
            if (_selected_concatenation_axes==conc_x_axis){
                #ifdef PVConc_debug
                //std::cout << "_selected_concatenation_axes==conc_x_axis " << v3d_msg->getData().x << std::endl;
                #endif
                pv=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv=v3d_msg->getData().z;
            }
            if(_act_on == act_on::pv){
                pv_vector.x = pv;
                pv_vector.y = pv_dot;
                pv_vector.z = 0.0; //TODO add pv_dot_dot if needed
                Vector3DMessage pv_vector_msg;
                pv_vector_msg.setVector3DMessage(pv_vector);
                #ifdef PVConc_debug
                //std::cout << "pv_vector.x " << pv_vector.x << ", pv_dot " << pv_dot << ", pv_dot_dot " << pv_dot_dot << std::endl;
                #endif
                this->emit_message((DataMessage*) &pv_vector_msg);
            }
        }
        else if(t_channel==(int)ch_pv_dot){
            #ifdef PVConc_debug
            //std::cout << "t_channel==(int)ch_pv_dot" << std::endl;
            #endif
            if (_selected_concatenation_axes==conc_x_axis){
                #ifdef PVConc_debug
                //std::cout << "_selected_concatenation_axes==conc_x_axis " << v3d_msg->getData().x << std::endl;
                #endif
                pv_dot=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_dot=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_dot=v3d_msg->getData().z;
            }
            if(_act_on == act_on::pv_dot){
                pv_vector.x = pv;
                pv_vector.y = pv_dot;
                pv_vector.z = 0.0; //TODO add pv_dot_dot if needed
                Vector3DMessage pv_vector_msg;
                pv_vector_msg.setVector3DMessage(pv_vector);
                this->emit_message((DataMessage*) &pv_vector_msg);
            }

        }
        else if(t_channel==(int)ch_pv_dot_dot){
            if (_selected_concatenation_axes==conc_x_axis){
                pv_dot_dot=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv_dot_dot=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv_dot_dot=v3d_msg->getData().z;
            }
        }
    }    
}
