#pragma once

#include "ROSUnit.hpp"
#include "common_srv/set_float.h"
//Change the msg type to match your received msg
#include "FloatMsg.hpp"

class ROSUnit_SetFloatClnt : public ROSUnit
{
    public:

        ROSUnit_SetFloatClnt(std::string, ros::NodeHandle&);
        ~ROSUnit_SetFloatClnt();
        //Change the receiveMsgData code to reflect your system
        void receiveMsgData(DataMessage* t_msg);

    private:

        ros::ServiceClient m_client;
};