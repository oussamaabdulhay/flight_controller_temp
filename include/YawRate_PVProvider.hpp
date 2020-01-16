#pragma once
#include "PVProvider.hpp"
#include "HeadingProvider.hpp"
#include "BodyRateProvider.hpp"
#include "ROSMsg.hpp"

class YawRate_PVProvider :  public PVProvider, 
                            public BodyRateProvider{

public:

    Vector3D<float> getProcessVariable();
    virtual Vector3D<float> getBodyRate() = 0;
    ROSMsg ros_msg;

    YawRate_PVProvider();
    ~YawRate_PVProvider();
};