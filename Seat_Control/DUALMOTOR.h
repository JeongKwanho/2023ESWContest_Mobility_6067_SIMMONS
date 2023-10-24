#ifndef DUALMOTOR_h
#define DUALMOTOR_h
#include "mbed.h"
#include <cstdint>
class DUALMOTOR{
    public:
        DUALMOTOR(PinName PWMA, PinName PWMB);
        void setSpeed(float pwmVal);
        void forward();
        void backward();
        void stop();
        void run(float velocity);
    private:
        PwmOut _pwma;
        PwmOut _pwmb;
        int _DIR;
};
#endif