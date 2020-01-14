#include "RestrictedNormWaypointRefGenerator.hpp"
void RestrictedNormWaypointRefGenerator::receive_msg_data(DataMessage* t_msg){
    if (t_msg->getType()==msg_type::WAYPOINT)
    {
        WaypointMsg* t_pos_msg=(WaypointMsg*) t_msg;
        Waypoint t_waypoint;
        t_waypoint.position.x=t_pos_msg->waypoint.position.x;
        t_waypoint.position.y=t_pos_msg->waypoint.position.y;
        t_waypoint.position.z=t_pos_msg->waypoint.position.z;
        t_waypoint.yaw=t_pos_msg->waypoint.yaw;
        std::cout << "waypoint X : " << t_waypoint.position.x << std::endl;
        std::cout << "waypoint Y : " << t_waypoint.position.y << std::endl;
        std::cout << "waypoint Z : " << t_waypoint.position.z << std::endl;
        std::cout << "waypoint Yaw : " << t_waypoint.yaw << std::endl;
        Waypoints.push_back(t_waypoint);
    }
    else if(t_msg->getType()==msg_type::RESTNORMREF_SETTINGS)
    {
        RestrictedNormRefSettingsMsg* t_settings_msg=(RestrictedNormRefSettingsMsg*) t_msg;
        max_norm=t_settings_msg->getMaxNorm();
        enabled=t_settings_msg->enabled;
        if (t_settings_msg->delete_existing_waypoints){
            Waypoints.clear();
        }
    }
    else if(t_msg->getType()==msg_type::POSITION){
        PositionMsg* t_current_pos=(PositionMsg*) t_msg;
        Vector3D<double> t_current_pos_vec;
        t_current_pos_vec.x=t_current_pos->x;
        t_current_pos_vec.y=t_current_pos->y;
        t_current_pos_vec.z=t_current_pos->z;
        
        while (Waypoints.size()>0){
            Vector3D<double> diff_pos_waypoint=Waypoints[0].position-t_current_pos_vec;
            double t_dist= Vector3D<double>::getL2Norm(diff_pos_waypoint);
            if (t_dist>=max_norm){
                Vector3D<double> restricted_ref;
                restricted_ref=t_current_pos_vec+(diff_pos_waypoint/t_dist)*max_norm;
                updateControlSystemsReferences(restricted_ref,Waypoints[0].yaw);
                break;
            }
            else
            {
                if (Waypoints.size()==1){
                    updateControlSystemsReferences(Waypoints[0].position,Waypoints[0].yaw);
                    auto firstWaypoint=Waypoints.begin();
                    Waypoints.erase(firstWaypoint);
                    break;
                }
                auto firstWaypoint=Waypoints.begin();
                Waypoints.erase(firstWaypoint);
            }
        }
    }
    
}

void RestrictedNormWaypointRefGenerator::updateControlSystemsReferences(Vector3D<double> t_pos_ref, double t_yaw){
    
    ControlSystemMessage x_cont_ref;
    x_cont_ref.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, t_pos_ref.x);
    std::cout << "Setting X Reference: " << t_pos_ref.x << std::endl;
    ((msg_receiver*)x_control_system)->receive_msg_data(&x_cont_ref);

    ControlSystemMessage y_cont_ref;
    y_cont_ref.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, t_pos_ref.y);
    std::cout << "Setting Y Reference: " << t_pos_ref.y << std::endl;
    ((msg_receiver*)y_control_system)->receive_msg_data(&y_cont_ref);

    ControlSystemMessage z_cont_ref;
    z_cont_ref.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, t_pos_ref.z);
    std::cout << "Setting Z Reference: " << t_pos_ref.z << std::endl;
    ((msg_receiver*)z_control_system)->receive_msg_data(&z_cont_ref);

    ControlSystemMessage yaw_cont_ref;
    yaw_cont_ref.setControlSystemMessage(control_system::null_type, control_system_msg_type::to_system, t_yaw);
    std::cout << "Setting Yaw Reference: " << t_yaw << std::endl;
    ((msg_receiver*)yaw_control_system)->receive_msg_data(&yaw_cont_ref);
}

void RestrictedNormWaypointRefGenerator::add_x_control_system(ControlSystem* t_sys){
    x_control_system=t_sys;
}
void RestrictedNormWaypointRefGenerator::add_y_control_system(ControlSystem* t_sys){
    y_control_system=t_sys;
}
void RestrictedNormWaypointRefGenerator::add_z_control_system(ControlSystem* t_sys){
    z_control_system=t_sys;
}
void RestrictedNormWaypointRefGenerator::add_yaw_control_system(ControlSystem* t_sys){
    yaw_control_system=t_sys;
}