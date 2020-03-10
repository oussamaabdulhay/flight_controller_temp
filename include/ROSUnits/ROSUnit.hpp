#pragma once
#include "ros/ros.h"
#include <iostream>
#include <string>
#include "std_msgs/String.h"
#include "MsgReceiver.hpp"
#include "MsgEmitter.hpp"
#include "common_types.hpp"

const int ROSUnit_capacity=6;
class ROSUnit : public MsgEmitter, public MsgReceiver{

    private:
        ros::NodeHandle _main_handler;

    public:
        ros::NodeHandle getNodeHandle();
        virtual void receiveMsgData(DataMessage* t_msg) = 0;

        ROSUnit(ros::NodeHandle&);
        ~ROSUnit();
    protected:
        
        
};