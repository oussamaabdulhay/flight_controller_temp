#include "ROSUnit_BroadcastData.hpp"
ROSUnit_BroadcastData* ROSUnit_BroadcastData::_instance_ptr = NULL;

ROSUnit_BroadcastData::ROSUnit_BroadcastData(ros::NodeHandle& t_main_handler) : ROSUnit(t_main_handler){

    _pos_prov_pub = t_main_handler.advertise<geometry_msgs::Point>("uav_control/uav_position", 1);
    _ori_prov_pub = t_main_handler.advertise<geometry_msgs::Point>("uav_control/uav_orientation", 1);
    _xpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("x_provider", 1);
    _ypv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("y_provider", 1);
    _zpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("z_provider", 1);
    _rollpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("roll_provider", 1);
    _pitchpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("pitch_provider", 1);
    _yawpv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("yaw_provider", 1);
    _yawratepv_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("yaw_rate_provider", 1);
    _cs_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("control_system_output", 1);
    _csr_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("control_system_reference", 1);
    _act_prov_pub = t_main_handler.advertise<std_msgs::Float64MultiArray>("actuation_output", 1);
    _info_prov_pub = t_main_handler.advertise<flight_controller::Info>("info", 1);
    _error_prov_pub = t_main_handler.advertise<geometry_msgs::PointStamped>("error", 1);

    _att.x = 0;
    _head = 0;

    _instance_ptr = this;
}

ROSUnit_BroadcastData::~ROSUnit_BroadcastData() {

}


void ROSUnit_BroadcastData::receiveMsgData(DataMessage* t_msg){
    if(t_msg->getType() == msg_type::FLOAT){
        FloatMsg* voltage_msg = (FloatMsg*)t_msg;
        _voltage = voltage_msg->data;

    }else if(t_msg->getType() == msg_type::BOOLEAN){
        BooleanMsg* armed_msg = (BooleanMsg*)t_msg;
        _armed = armed_msg->data;
        flight_controller::Info msg;
        msg.header.seq = ++_seq_info;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "";
        msg.number_of_waypoints = _number_of_waypoints;
        msg.armed = _armed;
        msg.battery_voltage = _voltage;
        _info_prov_pub.publish(msg);

    }
}

void ROSUnit_BroadcastData::receiveMsgData(DataMessage* t_msg, int t_channel){

    if(t_msg->getType() == msg_type::VECTOR3D){
        Vector3DMessage* vector3d_msg = (Vector3DMessage*)t_msg;
        if(t_channel == (int)ros_broadcast_channels::x){
            Vector3D<float> xpv = vector3d_msg->getData();
            _position.x = xpv.x;
            x_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_xpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = xpv.x;
            msg.point.y = xpv.y;
            msg.point.z = xpv.z;
            _xpv_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::y){
            Vector3D<float> ypv = vector3d_msg->getData();
            _position.y = ypv.x;
            y_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_ypv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = ypv.x;
            msg.point.y = ypv.y;
            msg.point.z = ypv.z;
            _ypv_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::z){
            Vector3D<float> zpv = vector3d_msg->getData();
            _position.z = zpv.x;
            z_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_zpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = zpv.x;
            msg.point.y = zpv.y;
            msg.point.z = zpv.z;
            _zpv_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::yaw){
            Vector3D<float> yawpv = vector3d_msg->getData();
            _head = yawpv.x;
            yaw_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_yawpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = yawpv.x;
            msg.point.y = yawpv.y;
            msg.point.z = yawpv.z;
            _yawpv_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::yaw_rate){
            Vector3D<float> yawratepv = vector3d_msg->getData();
            std::cout << "ROSUNIT yawratepv.x " << yawratepv.x << "\n";
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_yawratepv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = yawratepv.x;
            msg.point.y = yawratepv.y;
            msg.point.z = yawratepv.z;
            _yawratepv_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::roll){
            Vector3D<float> rollpv = vector3d_msg->getData();
            _att.x = rollpv.x;
            roll_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_rollpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = rollpv.x;
            msg.point.y = rollpv.y;
            msg.point.z = rollpv.z;
            _rollpv_prov_pub.publish(msg);
        }else if(t_channel == (int)ros_broadcast_channels::pitch){
            Vector3D<float> pitchpv = vector3d_msg->getData();
            _att.y = pitchpv.x;
            pitch_received = true;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_pitchpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = pitchpv.x;
            msg.point.y = pitchpv.y;
            msg.point.z = pitchpv.z;
            _pitchpv_prov_pub.publish(msg);
        }
    }else if(t_msg->getType() == msg_type::VECTORDOUBLE){
        VectorDoubleMsg* vector_double_msg = (VectorDoubleMsg*)t_msg;
        
        if(t_channel == (int)ros_broadcast_channels::actuation){
            _act_outputs = vector_double_msg->data;

            std_msgs::Float64MultiArray msg;
            msg.data = _act_outputs;
            _act_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::references){
             int i = (int)(vector_double_msg->data[0]);
            _cs_references[i] = vector_double_msg->data[1];

            std_msgs::Float64MultiArray msg;
            msg.data = _cs_references;
            _csr_prov_pub.publish(msg);

        }else if(t_channel == (int)ros_broadcast_channels::control_outputs){
            int i = (int)(vector_double_msg->data[0]);
            _cs_outputs[i] = vector_double_msg->data[1];

            std_msgs::Float64MultiArray msg;
            msg.data = _cs_outputs;
            _cs_prov_pub.publish(msg);
        }
    }else if(t_msg->getType() == msg_type::INTEGER){
        IntegerMsg* integer_msg = (IntegerMsg*)t_msg;
        if(t_channel == ros_broadcast_channels::waypoints){
            _number_of_waypoints = integer_msg->data; 
            
        }else if(t_channel == ros_broadcast_channels::error){
            _error_accumulator += integer_msg->data;
            geometry_msgs::PointStamped msg;
            msg.header.seq = ++_seq_xpv;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "";
            msg.point.x = _error_accumulator;
            msg.point.y = 0.;
            msg.point.z = 0.;
            _error_prov_pub.publish(msg);
        }
    }

    if(x_received && y_received && z_received){
        geometry_msgs::Point msg;
        msg.x = _position.x;
        msg.y = _position.y;
        msg.z = _position.z;
        _pos_prov_pub.publish(msg);
        x_received = false;
        y_received = false;
        z_received = false;
    }

    if(roll_received && pitch_received && yaw_received){
        geometry_msgs::Point msg;
        msg.x = _att.x;
        msg.y = _att.y;
        msg.z = _head;
        _ori_prov_pub.publish(msg);
        roll_received = false;
        pitch_received = false;
        yaw_received = false;
    }

}