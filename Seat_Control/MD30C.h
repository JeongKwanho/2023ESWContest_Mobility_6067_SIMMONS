#ifndef MD30C_h
#define MD30C_h
#include "mbed.h"
#include <cstdint>
class MD30C{
    public:
        MD30C(PinName DIR, PinName PWM);
        void setSpeed(float pwmVal);
        float getSpeed();
        void forward();
        void backward();
        void stop();
        void run(float velocity);
        uint8_t get_dir();
    private:
        PwmOut _pwm;
        DigitalOut _DIR;
};
#endif