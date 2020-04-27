#pragma once
#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "common_srv/ROSUnit.hpp"
#include "common_srv/Vector3D.hpp"
#include "Quaternion.hpp"
#include "common_srv/Vector3DMessage.hpp"
#include "PVConcatenator.hpp"
#include "Global2Inertial.hpp"
#include "common_srv/Timer.hpp"

class ROSUnit_Xsens : public ROSUnit{

    private:
        static ROSUnit_Xsens* _instance_ptr;
        ros::Subscriber _sub_attitude;
        ros::Subscriber _sub_position;
        ros::Subscriber _sub_velocity;
        ros::Subscriber _sub_body_rate;
        static void callbackXsensPosition(const geometry_msgs::Vector3Stamped& msg_position);
        static void callbackXsensAttitude(const geometry_msgs::QuaternionStamped& msg_attitude);
        static void callbackXsensVelocity(const geometry_msgs::TwistStamped& msg_velocity);
        static void callbackXsensBodyRate(const geometry_msgs::Vector3Stamped& msg_bodyrate);
        
        static Timer t_pedro;

    public:
        enum unicast_addresses {broadcast,unicast_XSens_translation,unicast_XSens_orientation,unicast_XSens_attitude_rate,unicast_XSens_yaw_rate,unicast_XSens_translation_rate};
        void receiveMsgData(DataMessage*);
        ROSUnit_Xsens(ros::NodeHandle&);
        ~ROSUnit_Xsens();

};