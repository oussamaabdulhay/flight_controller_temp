#include "PVConcatenator.hpp"

PVConcatenator::PVConcatenator(control_system t_cs) {
    _cs = t_cs;
}

PVConcatenator::~PVConcatenator() {

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg){

}

void PVConcatenator::receive_msg_data(DataMessage* t_msg, int t_channel){
    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* v3d_msg = (Vector3DMessage*)t_msg;
        if (t_channel==(int)ch_pv){
            if (selected_concatenation_axes==conc_x_axis){
                pv_vector.x=v3d_msg->getData().x;
            }else if (selected_concatenation_axes==conc_y_axis){
                pv_vector.x=v3d_msg->getData().y;
            }else if (selected_concatenation_axes==conc_z_axis){
                pv_vector.x=v3d_msg->getData().z;
            }
            Vector3DMessage pv_vector_msg;
            pv_vector_msg.setVector3DMessage(pv_vector);
            emit_message(&pv_vector_msg);
        }
        else if(t_channel==(int)ch_pv_dot){
            if (selected_concatenation_axes==conc_x_axis){
                pv_vector.y=v3d_msg->getData().x;
            }else if (selected_concatenation_axes==conc_y_axis){
                pv_vector.y=v3d_msg->getData().y;
            }else if (selected_concatenation_axes==conc_z_axis){
                pv_vector.y=v3d_msg->getData().z;
            }
        }
        else if(t_channel==(int)ch_pv_dot_dot){
            if (selected_concatenation_axes==conc_x_axis){
                pv_vector.z=v3d_msg->getData().x;
            }else if (selected_concatenation_axes==conc_y_axis){
                pv_vector.z=v3d_msg->getData().y;
            }else if (selected_concatenation_axes==conc_z_axis){
                pv_vector.z=v3d_msg->getData().z;
            }
        }
    }    
}

void PVConcatenator::concatenate(){
    switch (_cs){
        case control_system::x:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _position.x;
            output.y = _velocity.x;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::x, (int)control_system::x);
            break;
        }  
        case control_system::y:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _position.y;
            output.y = _velocity.y;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::y, (int)control_system::y);
            break;
        }  
        case control_system::z:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _position.z;
            output.y = _velocity.z;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::z, (int)control_system::z);
            break;
        }
        case control_system::roll:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _angle.y;
            output.y = _angle_rate.y;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::roll, (int)control_system::roll);
            break;
        }
        case control_system::pitch:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = -_angle.x;
            output.y = -_angle_rate.x;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::pitch, (int)control_system::pitch);
            break;
        }
            
        case control_system::yaw:
        {   
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _angle.z;
            output.y = 0.0;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::yaw, (int)control_system::yaw);
            break;
        }
            
        case control_system::yaw_rate:
        {
            Vector3D<float> output;
            Vector3DMessage output_msg;
            output.x = _angle_rate.z;
            output.y = 0.0;
            output.z = 0.0;
            output_msg.setVector3DMessage(output);
            this->emit_message_unicast(&output_msg, (int)control_system::yaw_rate, (int)control_system::yaw_rate);
            break;
        }     
        default:
            break;
    }
}