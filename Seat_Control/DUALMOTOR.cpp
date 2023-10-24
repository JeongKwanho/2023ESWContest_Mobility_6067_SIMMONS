#include "DUALMOTOR.h"

DUALMOTOR::DUALMOTOR(PinName PWMA,PinName PWMB):_pwma(PWMA),_pwmb(PWMB)
{
    _pwma.period_us(66); // 15kHz
    _pwmb.period_us(66); // 15kHz
    _pwma = 0.;
    _pwmb = 0.;
    _DIR = 1;//기본 세팅 전진
}
void DUALMOTOR::setSpeed(float pwmVal){
    if (_DIR ==1){
        _pwma = pwmVal;
        _pwmb = 0;
    }
    else{
        _pwma = 0;
        _pwmb = pwmVal;
    }
}
void DUALMOTOR::stop(){
    _pwma=0;
    _pwmb=0;
}
void DUALMOTOR::forward(){
    _DIR = 1;
}
void DUALMOTOR::backward(){
    _DIR = 0;
}
void DUALMOTOR::run(float vel){
    if(abs(vel)<=0.05) vel=0;
    if (vel>=0){
        forward();
        setSpeed((vel>1.0f) ? 1.0f:vel);
    }
    else{
        backward();
        setSpeed((vel<-1.0f) ? 1.0f:-vel);
    }
}