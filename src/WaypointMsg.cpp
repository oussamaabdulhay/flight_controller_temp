#include "WaypointMsg.hpp"
    msg_type WaypointMsg::getType(){
        return msg_type::WAYPOINT;
    }
    const int WaypointMsg::getSize(){
        return sizeof(*this);
    }