#include "PVConcatenator.hpp"

#define PVConc_debug

PVConcatenator::PVConcatenator(concatenation_axes t_selected_concatenation_axes, act_on t_act_on) {
    _selected_concatenation_axes = t_selected_concatenation_axes;
    _act_on = t_act_on;
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::receiveMsgData(DataMessage* t_msg){
    std::cout << "PVConcatenator::receiveMsgData(DataMessage* t_msg)" << std::endl;
}

void PVConcatenator::receiveMsgData(DataMessage* t_msg, int t_channel){
    #ifdef PVConc_debug
    //std::cout << "PVConcatenator::receiveMsgData(DataMessage* t_msg, int t_channel)" << std::endl;
    #endif
    if(t_msg->getType() == msg_type::VECTOR3D){
        #ifdef PVConc_debug
        //std::cout << "t_msg->getType() == msg_type::VECTOR3D" << std::endl;
        #endif
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        if (t_channel==(int)ch_pv){
            if (_selected_concatenation_axes==conc_x_axis){
                pv=v3d_msg->getData().x;
            }else if (_selected_concatenation_axes==conc_y_axis){
                pv=v3d_msg->getData().y;
            }else if (_selected_concatenation_axes==conc_z_axis){
                pv=v3d_msg->getData().z;
                std::cout << "PVConcatenator pv.z " << pv << "\n";
            }
            if(_act_on == act_on::pv){
                pv_vector.x = pv;
                pv_vector.y = pv_dot;
                pv_vector.z = 0.0; //TODO add pv_dot_dot if needed
                Vector3DMessage pv_vector_msg;
                pv_vector_msg.setVector3DMessage(pv_vector);
                this->emitMsgUnicastDefault((DataMessage*) &pv_vector_msg);
            }
        }
        else if(t_channel==(int)ch_pv_dot){
            if (_selected_concatenation_axes==conc_x_axis){
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
                this->emitMsgUnicastDefault((DataMessage*) &pv_vector_msg);
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
