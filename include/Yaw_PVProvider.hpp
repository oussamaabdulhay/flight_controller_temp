#pragma once
#include "PVProvider.hpp"
#include "HeadingProvider.hpp"
#include "BodyRateProvider.hpp"
#include "ROSMsg.hpp"

class Yaw_PVProvider :  public PVProvider, 
                        public HeadingProvider{

public:

    Vector3D<float> getProcessVariable();
    virtual HeadingMsg getHeading() = 0;
    ROSMsg ros_msg;

    Yaw_PVProvider();
    ~Yaw_PVProvider();
};