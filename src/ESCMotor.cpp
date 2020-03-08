/*
Motors connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect motor to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your motor correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include "ESCMotor.hpp"

ESCMotor::ESCMotor(int t_pin, int t_freq){
    _pwmPin = t_pin;
    _freq = t_freq;
    this->initialize();
}


bool ESCMotor::initialize(){

    _pwm = new RCOutput_Navio2();

    if(!(_pwm->initialize(_pwmPin)) ) {
        return 1;
    }

    _pwm->set_frequency(_pwmPin, _freq);

	if ( !(_pwm->enable(_pwmPin)) ) {
	    return 1;
	}

}

void ESCMotor::applyCommand(int t_command){

    //std::cout << "Received Command on PIN: " << _pwmPin << " Value :" << t_command << "\r"; //std::endl;
    _pwm->set_duty_cycle(_pwmPin, t_command);

}


