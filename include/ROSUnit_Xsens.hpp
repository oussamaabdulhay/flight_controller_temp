#pragma once
#include "ros/ros.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include "ROSUnit.hpp"
#include "Vector3D.hpp"
#include "Quaternion.hpp"
#include "AttitudeMsg.hpp"
#include "VelocityMsg.hpp"
#include "HeadingMsg.hpp"
#include "PositionMsg.hpp"
#include "AccelerationMsg.hpp"
#include "BodyRateMsg.hpp"

class ROSUnit_Xsens : public ROSUnit{

    private:
        static ROSUnit_Xsens* _instance_ptr;
        ros::Subscriber _sub_attitude;
        ros::Subscriber _sub_position;
        ros::Subscriber _sub_velocity;
        ros::Subscriber _sub_body_rate;
        static AttitudeMsg attitude_msg;
        static VelocityMsg velocity_msg; 
        static HeadingMsg heading_msg; 
        static PositionMsg position_msg; 
        static BodyRateMsg bodyrate_msg;
        static AccelerationMsg acceleration_msg;  
        static void callbackXsensPosition(const geometry_msgs::Vector3Stamped& msg_position);
        static void callbackXsensAttitude(const sensor_msgs::Imu& msg_attitude);
        static void callbackXsensVelocity(const geometry_msgs::Vector3Stamped& msg_velocity);
        static void callbackXsensBodyRate(const geometry_msgs::TwistStamped& msg_bodyrate);
        void receive_msg_data(DataMessage* t_msg);  
        
    public:
        ROSUnit_Xsens(ros::NodeHandle&);
        ~ROSUnit_Xsens();

};