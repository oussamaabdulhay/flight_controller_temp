#pragma once
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include "common_srv/ROSUnit.hpp"
#include "common_srv/Vector3D.hpp"
#include "common_types.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "Global2Inertial.hpp"

class ROSUnit_RTK : public ROSUnit{

    private:
        static ROSUnit_RTK* _instance_ptr;
        ros::Subscriber _sub_fix;
        ros::Subscriber _sub_vel;
        static void callbackRTK1(const sensor_msgs::NavSatFix::ConstPtr & msg);
        static void callbackRTK2(const geometry_msgs::TwistStamped::ConstPtr & msg);
        void receiveMsgData(DataMessage* t_msg);  
        
    public:
        ROSUnit_RTK(ros::NodeHandle&);
        ~ROSUnit_RTK();

};