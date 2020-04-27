#pragma once 

#include "common_srv/Vector3D.hpp"

enum sens_type {ACCELEROMETER = 0, GYROSCOPE = 1, MAGNETOMETER = 2};
enum setting_type {FSR = 0, SENS = 1, SAMPLERATE = 2, LPF = 3};
enum sen_status {CALIBRATED = 0, NEEDSCALIBRATION = 1, CALIBRATING = 2, MALFUNCTION = 3};

class ThreeAxisSensor
{
public:
	
	virtual sens_type getSensType() = 0;
	virtual void setSettings(setting_type, float) = 0;
	virtual void startCalibration() = 0;
	virtual void stopCalibration() = 0;
	virtual void resetCalibration() = 0;
	virtual sen_status getSensorStatus() = 0;
	virtual Vector3D<float> getCalibratedData() = 0;
};