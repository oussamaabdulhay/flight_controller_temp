#pragma once

#include "ROSUnit.hpp"
#include "std_srvs/Empty.h"
//Change the msg type to match your received msg
#include "EmptyMsg.hpp"

class ROSUnit_EmptyClnt : public ROSUnit
{
    public:
    
        ROSUnit_EmptyClnt(std::string, ros::NodeHandle&);
        ~ROSUnit_EmptyClnt();
        //Change the receiveMsgData code to reflect your system
        void receiveMsgData(DataMessage* t_msg);

    private:

        ros::ServiceClient m_client;
};