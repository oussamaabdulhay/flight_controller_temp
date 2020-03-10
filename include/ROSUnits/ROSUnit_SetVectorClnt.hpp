#pragma once

#include "ROSUnit.hpp"
#include "common_srv/set_vector.h"
//Change the msg type code to reflect your system
#include "VectorMsg.hpp"

class ROSUnit_SetVectorClnt : public ROSUnit
{
    public:
    
        ROSUnit_SetVectorClnt(std::string, ros::NodeHandle&);
        ~ROSUnit_SetVectorClnt();
        //Change the receiveMsgData code to reflect your system
        void receiveMsgData(DataMessage* t_msg);

    private:

        ros::ServiceClient m_client;
};