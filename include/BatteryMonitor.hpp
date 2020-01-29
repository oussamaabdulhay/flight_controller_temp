#pragma once
#include <unistd.h>
#include <cstdio>
#include <Common/Util.h>
#include <Navio2/ADC_Navio2.h>
#include <memory>
#include "MsgEmitter.hpp"
#include "FloatMsg.hpp"

#define READ_FAILED -1
int const BATTERY_VOLTAGE_PIN = 2;
float const SCALE = 0.00986; //SCALE and OFFSET were obtained through constant measurements between Navio data and Voltimeter
float const OFFSET = 1.71;
class BatteryMonitor : public msg_emitter{

private:
    std::unique_ptr<ADC> adc;
    float results[6] = {0.0f};
    FloatMsg _voltage_reading;

public:
    float getVoltageReading();
    BatteryMonitor();
    ~BatteryMonitor();
};