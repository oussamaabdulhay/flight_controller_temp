#include "BatteryMonitor.hpp"

BatteryMonitor::BatteryMonitor() {
    adc = std::unique_ptr <ADC>{ new ADC_Navio2() };
    adc->initialize();
}

BatteryMonitor::~BatteryMonitor() {

}

float BatteryMonitor::getVoltageReading(){
    results[BATTERY_VOLTAGE_PIN] = adc->read(BATTERY_VOLTAGE_PIN);

    if (results[BATTERY_VOLTAGE_PIN] == READ_FAILED){
        return EXIT_FAILURE;
    }

    _voltage_reading.data = results[BATTERY_VOLTAGE_PIN] * SCALE + OFFSET;

    this->emit_message_unicast((DataMessage*)&_voltage_reading, -1); 
}