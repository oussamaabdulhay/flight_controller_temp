#pragma once
#include "ROSUnit.hpp"
#include "ROSMsg.hpp"
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include "PositionMsg.hpp"

class ROSUnit_BroadcastData : public ROSUnit{

private:
    ros::Publisher _pos_prov_pub;
    ros::Publisher _ori_prov_pub;
    ros::Publisher _xpv_prov_pub;
    ros::Publisher _ypv_prov_pub;
    ros::Publisher _zpv_prov_pub;
    ros::Publisher _rollpv_prov_pub;
    ros::Publisher _pitchpv_prov_pub;
    ros::Publisher _yawpv_prov_pub;
    ros::Publisher _cs_prov_pub;
    ros::Publisher _act_prov_pub;
    ros::Publisher _yawratepv_prov_pub;
    bool roll_received = false, pitch_received = false, yaw_received = false;
    bool x_received = false, y_received = false, z_received = false;
    static ROSUnit_BroadcastData* _instance_ptr;
    void receive_msg_data(DataMessage* t_msg); 
    int _seq_pos = 0, _seq_ori = 0, _seq_xpv = 0, _seq_ypv = 0, _seq_zpv = 0;
    int _seq_rollpv = 0, _seq_pitchpv = 0, _seq_yawpv = 0, _seq_cs = 0, _seq_act = 0;
    int _seq_yawratepv = 0;
    vector<double> _cs_outputs{ 0, 0, 0, 0, 0, 0, 0 }; 
    vector<double> _act_outputs{ 0, 0, 0, 0, 0, 0 }; 
    PositionMsg _position;
    AttitudeMsg _att;
    HeadingMsg _head;

public:
    ROSUnit_BroadcastData(ros::NodeHandle&);
    ~ROSUnit_BroadcastData();
};