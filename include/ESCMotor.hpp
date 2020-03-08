#pragma once

#include "Actuator.hpp"
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <unistd.h>
#include <memory>

class ESCMotor : public Actuator {

    private:
        int _pwmPin;
        int _freq;
        RCOutput* _pwm;

    public:
        bool initialize();
        void applyCommand(int command);
        RCOutput* get_rcout();

        ESCMotor(int, int);
};