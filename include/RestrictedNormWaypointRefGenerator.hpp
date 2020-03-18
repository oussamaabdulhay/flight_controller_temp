#pragma once
#include "MsgEmitter.hpp"
#include "MsgReceiver.hpp"
#include "Vector3D.hpp"
#include "RestrictedNormRefSettingsMsg.hpp"
#include "PosesMsg.hpp"
#include <vector>
#include "ControlSystem.hpp"
#include "Waypoint.hpp"
#include "DoubleMsg.hpp"

class RestrictedNormWaypointRefGenerator : public MsgEmitter, public MsgReceiver{

    private:
    static pthread_mutex_t lock;
    std::vector<Waypoint> Waypoints;
    double max_norm = 0.2;
    bool enabled=false;
    int old_size = 0;
    void updateControlSystemsReferences(Vector3D<double> position,double yaw);
public:
    enum unicast_addresses {x, y, z, yaw};
    void receiveMsgData(DataMessage* t_msg);
    void receiveMsgData(DataMessage* t_msg, int t_channel);
};