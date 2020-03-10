#pragma once

#include "ROSUnit.hpp"
#include "geometry_msgs/Point.h"
//Change the msg type to match your received msg
#include "Vector3DMessage.hpp"

class ROSUnit_PointPub : public ROSUnit
{
    public:

        ROSUnit_PointPub(std::string, ros::NodeHandle&);
        ~ROSUnit_PointPub();
        //Change the receiveMsgData code to reflect your system
        void receiveMsgData(DataMessage* t_msg);

    private:

        ros::Publisher m_pub;
};