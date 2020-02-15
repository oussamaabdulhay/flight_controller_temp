#include "Global2Inertial.hpp"
#define DEBUG_HR_LR_DECOUPLED
// const float PI = atan(1.0)*4.0;

// float MeasureGPStoMeters(float lat1, float lon1, float lat2, float lon2)
// {  
//     float R    = 6378.137; // Radius of earth in km
//     float dLat = lat2*(PI/180) - lat1*(PI/180);
//     float dLon = lon2*(PI/180) - lon1*(PI/180);
//     float a    = sin(dLat/2)*sin(dLat/2) + cos(lat1*PI/180)*cos(lat2*PI/180)*sin(dLon/2)*sin(dLon/2);
//     float c    = 2 * atan2(sqrt(a), sqrt(1-a));
//     float d    = R * c;
//     return d*1000; // meters
// }
Global2Inertial::Global2Inertial(){
    //TODO: Ensure altitude is calibrated
    calib_point1.x = 24.44828723;
    calib_point1.y = 54.39683993;
    calib_point1.z = 0.0;

    calib_point2.x = 24.44814808;
    calib_point2.y = 54.39666318;
    calib_point2.z = 0.0;

    // calib_point1.x=0;
    // calib_point1.y=0;
    // calib_point1.z=0;
    // calib_point2.x=0;
    // calib_point2.y=0;
    // calib_point2.z=0;
    calibrated_reference_inertial_heading=180.*(M_PI/180.);
    Vector3D<double> calib_points_diff = calib_point2 - calib_point1;
    calibrated_global_to_inertial_angle = atan2(calib_points_diff.x, calib_points_diff.y);
    antenna_pose.x=0.;
    antenna_pose.y=0.1;
    antenna_pose.z=0.1;
}
void Global2Inertial::receive_msg_data(DataMessage* t_msg)
{
    if(t_msg->getType() == msg_type::optitrack){ //TODO: Refactor message types

        //TODO implement for GPS
        OptitrackMessage* opti_msg = ((OptitrackMessage*)t_msg);
        Vector3D<double> pos_point = opti_msg->getPosition();
        Quaternion _bodyAtt = opti_msg->getAttitudeHeading();
        //HeadingMsg _bodyHeading = this->getHeading(_bodyAtt);
        //Vector3D<float> results = transformPoint(pos_point); //TODO uncomment
        Vector3D<float> results = pos_point; //TODO uncomment
        Vector3D<float> att_vec = getEulerfromQuaternion(_bodyAtt);
        AttitudeMsg _eulerAtt;

        HeadingMsg _bodyHeading;
        _eulerAtt.pitch = att_vec.y;
        _eulerAtt.roll = att_vec.x;
        _bodyHeading.yaw = att_vec.z;

        Vector3DMessage results_msg;
        results_msg.setVector3DMessage(results);
        FloatMsg yaw_msg;
        yaw_msg.data = att_vec.z - calibrated_reference_inertial_heading;
        
        // FloatMsg time_msg;
        // time_msg.data = opti_msg->getTime();

        // this->emit_message((DataMessage*)&time_msg);
        this->emit_message_unicast(&results_msg,Global2Inertial::unicast_addresses::uni_Optitrack_pos);
        this->emit_message((DataMessage*)&yaw_msg);
        //this->emit_message((DataMessage*)&_eulerAtt);
        //this->emit_message((DataMessage*)&_bodyHeading);
    }
    else if (t_msg->getType()==msg_type::rtkposition){
        #ifdef planC_dual_RTK
        RTKMessagePosition* rtk_msg = ((RTKMessagePosition*)t_msg);
        #error planC_dual_RTK is not implemented yet
        //TODO: PlanC is not implemented
        #elif defined(planB_single_RTK)
        RTKMessagePosition* rtk_msg = ((RTKMessagePosition*)t_msg);
        if (rtk_msg->id==1){
            RotationMatrix3by3 antenna_body_aligner;
            antenna_body_aligner.Update(last_known_orientation);
            Vector3D<double> rotated_antenna_position=antenna_body_aligner.TransformVector(antenna_pose);
            Vector3D<double> t_global_antenna_position=rtk_msg->position;
            Vector3D<double> t_global_drone_position=t_global_antenna_position-rotated_antenna_position;
            Vector3D<float> t_inertial_drone_position = transformPoint(t_global_drone_position);
            PoseStampedMsg results_msg;
            results_msg.pose.x = t_inertial_drone_position.x;
            results_msg.pose.y = t_inertial_drone_position.y;
            results_msg.pose.z = t_inertial_drone_position.z;
            this->emit_message((DataMessage*)&results_msg);
        }
        #endif
        
    }
    else if (t_msg->getType()==msg_type::POSITION){//TODO: XSesns position

    }
    else if (t_msg->getType()==msg_type::ATTITUDE){
        AttitudeMsg* att_msg = ((AttitudeMsg*)t_msg);
        last_known_orientation.x=att_msg->pitch;
        last_known_orientation.y=att_msg->roll;
    }
    else if (t_msg->getType()==msg_type::HEADING){
        HeadingMsg* hdng_msg = ((HeadingMsg*)t_msg);
        last_known_orientation.z=hdng_msg->yaw;
        HeadingMsg calibrated_heading_msg;
        calibrated_heading_msg.yaw=last_known_orientation.z-calibrated_reference_inertial_heading;
        emit_message((DataMessage*)&calibrated_heading_msg);
    }
    else if (t_msg->getType()==msg_type::FLOAT){
        calib_point1.z = ((FloatMsg*)t_msg)->data;
        std::cout << "NEW HEIGHT OFFSET = " << calib_point1.z << std::endl;
    }
}

void Global2Inertial::receive_msg_data(DataMessage* t_msg,int ch){
    if (t_msg->getType()==msg_type::VECTOR3D){
        if (ch==Global2Inertial::receiving_channels::ch_RTK_pos){
            //std::cout << "RTK RAW results.x=" << ((Vector3DMessage*)t_msg)->getData().x << " results.y=" << ((Vector3DMessage*)t_msg)->getData().y << " results.z=" << ((Vector3DMessage*)t_msg)->getData().z << std::endl;
            Vector3D<double> results = changeLLAtoMeters(calib_point1, ((Vector3DMessage*)t_msg)->getData()); //TODO uncoment
            Vector3D<double> results_elev=offsetElevation(results,-calib_point1.z);
            //std::cout << "RTK BEFORE results.x=" << results.x << " results.y=" << results.y << " results.z=" << results.z << std::endl;
            Vector3D<double> results_rot=rotatePoint(results_elev);
            //std::cout << "RTK AFTER  results.x=" << results_rot.x << " results.y=" << results_rot.y << " results.z=" << results_rot.z << std::endl;  
            Vector3DMessage res_msg;
            res_msg.setVector3DMessage(results_rot);
            #ifndef DEBUG_HR_LR_DECOUPLED
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_RTK_pos);
            #else
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_RTK_pos_pv, (int)PVConcatenator::receiving_channels::ch_pv);
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_RTK_pos_wp);
            #endif
        }
        else if (ch==Global2Inertial::receiving_channels::ch_XSens_pos){
            //std::cout << "RAW.x=" << ((Vector3DMessage*)t_msg)->getData().x << " RAW.y=" << ((Vector3DMessage*)t_msg)->getData().y << " RAW.z=" << ((Vector3DMessage*)t_msg)->getData().z << std::endl;
            Vector3D<double> results = changeLLAtoMeters(calib_point1,((Vector3DMessage*)t_msg)->getData());
            //std::cout << "BEFORE results.x=" << results.x << " results.y=" << results.y << " results.z=" << results.z << std::endl;
            Vector3D<double> results_rot=rotatePoint(results);
            //std::cout << "AFTER  results.x=" << results_rot.x << " results.y=" << results_rot.y << " results.z=" << results_rot.z << std::endl;  
            Vector3DMessage res_msg;
            res_msg.setVector3DMessage(results_rot);
            #ifdef RTK
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_XSens_pos);
            #else
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_XSens_pos,(int)PVConcatenator::receiving_channels::ch_pv);
            #endif
        }
        else if (ch==Global2Inertial::receiving_channels::ch_XSens_vel){
            Vector3D<double> results = transformVelocity(((Vector3DMessage*)t_msg)->getData());
            Vector3DMessage res_msg;
            res_msg.setVector3DMessage(results);
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_XSens_vel,(int)PVConcatenator::receiving_channels::ch_pv_dot);
        }
        else if (ch==Global2Inertial::receiving_channels::ch_XSens_ori){
            Vector3D<double> results = ((Vector3DMessage*)t_msg)->getData();
		//std::cout << "raw results.z " << results.z << "\n";
            results.z = results.z - calibrated_reference_inertial_heading - calibrated_global_to_inertial_angle;
            //std::cout << "results.z " << results.z << ", "<<calibrated_reference_inertial_heading<<","<<calibrated_global_to_inertial_angle<<"\n";
            Vector3DMessage res_msg;
            res_msg.setVector3DMessage(results);
            emit_message_unicast(&res_msg,Global2Inertial::unicast_addresses::uni_XSens_ori,(int)PVConcatenator::receiving_channels::ch_pv);
        }
    }
}

Vector3D<double> Global2Inertial::translatePoint(Vector3D<double> t_input_point){
        

    Vector3D<double> origin = calib_point1;
    Vector3D<double> calibrated_input_point;
    
    calibrated_input_point = t_input_point - origin;
    
    return calibrated_input_point;
}

Vector3D<double> Global2Inertial::offsetElevation(Vector3D<double> t_input,double elev_offset){
    Vector3D<double> t_res;
    t_res=t_input;
    t_res.z=t_res.z+elev_offset;
    return t_res;
}


Vector3D<double> Global2Inertial::rotatePoint(Vector3D<double> t_input_point){
        
    Vector3D<double> euler_calib;
    euler_calib.z = calibrated_global_to_inertial_angle;
    Vector3D<double> calibrated_input_point;
    RotationMatrix3by3 G_I_rot_matrix;

    G_I_rot_matrix.Update(euler_calib);

    //calibrated_input_point = G_I_rot_matrix.TransformVector(calibrated_input_point);
 
    G_I_rot_matrix.Transpose();
    calibrated_input_point = G_I_rot_matrix.TransformVector(t_input_point);

    return calibrated_input_point;
}

Vector3D<double> Global2Inertial::transformVelocity(Vector3D<double> t_input_point){
    RotationMatrix3by3 G_I_rot_matrix;
    Vector3D<double> euler_calib;
    euler_calib.z = calibrated_global_to_inertial_angle;
    G_I_rot_matrix.Update(euler_calib);
    G_I_rot_matrix.Transpose();
    return G_I_rot_matrix.TransformVector(t_input_point);
}

Vector3D<double> Global2Inertial::getEulerfromQuaternion(Quaternion q){

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    Vector3D<float> _euler;
    _euler.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        _euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _euler.y = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    _euler.z = atan2(siny_cosp, cosy_cosp);

    return _euler;
}

HeadingMsg Global2Inertial::getHeading(Quaternion t_bodyAtt)
{
    Vector3D<float> rpy = getEulerfromQuaternion(t_bodyAtt);
    HeadingMsg t_heading_msg;
    t_heading_msg.yaw = rpy.z;

    return t_heading_msg;
}

Vector3D<double> Global2Inertial::changeLLAtoMeters(Vector3D<double> t_origin,Vector3D<double> t_input2){
    //StackOverflow: "How to convert latitude or longitude to meters". Answer by JJones
    Vector3D<double> res;
    double latMid,m_per_deg_lat,m_per_deg_long,deltaLat,deltaLong,dist_m;
    latMid=(t_origin.x+t_input2.x)/2.;
    m_per_deg_lat=111132.954-559.822 * cos(2.*latMid)+1.175*cos(4.0*latMid);
    m_per_deg_long=(M_PI/180.)*6367449.0*cos(latMid);
    deltaLat=t_input2.x-t_origin.x;
    deltaLong=t_input2.y-t_origin.y;
    res.y=deltaLat*m_per_deg_lat;
    res.x=deltaLong*m_per_deg_long;
    res.z=t_input2.z;
    
    return res;
}
