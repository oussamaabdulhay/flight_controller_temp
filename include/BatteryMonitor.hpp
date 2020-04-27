#pragma once
#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>
#include <memory>
#include "common_srv/MsgEmitter.hpp"
#include "common_srv/FloatMsg.hpp"

#define READ_FAILED -1
int const BATTERY_VOLTAGE_PIN = 2;
float const SCALE = 0.0108; //SCALE and OFFSET were obtained through constant measurements between Navio data and Voltimeter
float const OFFSET = 0.416;
class BatteryMonitor : public MsgEmitter{

private:
    std::unique_ptr<ADC> adc;
    float results[6] = {0.0f};
    FloatMsg _voltage_reading;

public:
    float getVoltageReading();
    BatteryMonitor();
    ~BatteryMonitor();
};