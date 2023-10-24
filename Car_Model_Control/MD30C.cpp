#include "MD30C.h"

MD30C::MD30C(PinName DIR, PinName PWM):_DIR(DIR),_pwm(PWM)
{
    _pwm.period_us(66); // 15kHz
    _pwm = 0.;
    _DIR = 1;//기본 세팅 전진
}
void MD30C::setSpeed(float pwmVal){
    _pwm = pwmVal;
}
float MD30C::getSpeed(){
    return _pwm;
}
void MD30C::forward(){
    _DIR = 1;
}
void MD30C::backward(){
    _DIR = 0;
}
void MD30C::run(float vel){
    if(abs(vel)<=0.05) vel=0;
    if (vel>=0){
        setSpeed((vel>1.0f) ? 1.0f:vel);
        forward();
    }
    else{
        setSpeed((vel<-1.0f) ? 1.0f:-vel);
        backward();
    }
}
uint8_t MD30C::get_dir(){
    return _DIR.read();
}