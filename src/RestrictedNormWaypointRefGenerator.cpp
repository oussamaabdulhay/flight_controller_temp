#include "RestrictedNormWaypointRefGenerator.hpp"
void RestrictedNormWaypointRefGenerator::receive_msg_data(DataMessage* t_msg){
    if (t_msg->getType()==msg_type::WAYPOINT)// TODO: add to msg_type
    {
        WaypointMsg* t_pos_msg=(WaypointMsg*) t_msg;
        Waypoint t_waypoint;
        t_waypoint.position.x=t_pos_msg->waypoint.position.x;
        t_waypoint.position.y=t_pos_msg->waypoint.position.y;
        t_waypoint.position.z=t_pos_msg->waypoint.position.z;
        t_waypoint.yaw=t_pos_msg->waypoint.yaw;
        Waypoints.push_back(t_waypoint);
        delete t_pos_msg;
    }
    else if(t_msg->getType()==msg_type::RESTNORMREF_SETTINGS)// TODO: add to msg_type
    {
        RestrictedNormRefSettingsMsg* t_settings_msg=(RestrictedNormRefSettingsMsg*) t_msg;
        max_norm=t_settings_msg->getMaxNorm();
        enabled=t_settings_msg->enabled;
        if (t_settings_msg->delete_existing_waypoints){
            Waypoints.clear();
        }
        delete t_settings_msg;
    }
    else if(t_msg->getType()==msg_type::POSITION){
        PositionMsg* t_current_pos=(PositionMsg*) t_msg;
        Vector3D<double> t_current_pos_vec;
        t_current_pos_vec.x=t_current_pos->x;
        t_current_pos_vec.y=t_current_pos->y;
        t_current_pos_vec.z=t_current_pos->z;
        delete t_current_pos;
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

void RestrictedNormWaypointRefGenerator::updateControlSystemsReferences(Vector3D<double> t_pos_ref,double yaw){
    ReferenceMessage x_cont_ref;
    x_cont_ref.setReferenceMessage(t_pos_ref.x);
    ((msg_receiver*)x_control_system)->receive_msg_data(&x_cont_ref);
    ReferenceMessage y_cont_ref;
    y_cont_ref.setReferenceMessage(t_pos_ref.y);
    ((msg_receiver*)y_control_system)->receive_msg_data(&y_cont_ref);
    ReferenceMessage z_cont_ref;
    z_cont_ref.setReferenceMessage(t_pos_ref.z);
    ((msg_receiver*)z_control_system)->receive_msg_data(&z_cont_ref);
    ReferenceMessage yaw_cont_ref;
    yaw_cont_ref.setReferenceMessage(yaw);
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