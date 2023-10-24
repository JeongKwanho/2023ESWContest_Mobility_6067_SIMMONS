/* Qencoder Version 1.0 */
#pragma once //instead of safe include guard
#include "mbed.h"
#include <cstdint>

#ifndef _PI
#define _PI 3.14159265358979323846
#endif

class QEncoder{
    private:
        InterruptIn _pinA;
        InterruptIn _pinB;
        volatile uint8_t _previousState;
        volatile int32_t pulse_count;
        volatile uint32_t _errorCount;
        const float alpha;
        const uint32_t dt;
        float las_rpm,las_rate, fil_rpm, fil_rate;
        uint16_t CPR;
        float gear_ratio;
        const char dir;

        void decode(){
            uint8_t pinA_state, pinB_state;
            pinA_state= _pinA.read();
            pinB_state= _pinB.read();
            uint8_t newState=(pinA_state<<1|pinB_state);
            switch((_previousState <<2)|newState){
            case 0b0001: // 0x01 PREV|CURR
            case 0b0111: // 0x07
            case 0b1110: // 0x0E
            case 0b1000: // 0x08
                if(dir == 'L')
                    {pulse_count++; break;}
                else
                    {pulse_count--; break;}
            case 0b0010: // 0x02
            case 0b1011: // 0x0B
            case 0b1101: // 0x0D
            case 0b0100: // 0x04
                if(dir == 'L')
                    {pulse_count--; break;}
                else
                    {pulse_count++; break;}
            default:
                _errorCount++; break;
            }
            _previousState = newState;
        }
    
    public:
    QEncoder(PinName pinA, PinName pinB, float _alpha, uint32_t _dt, char _dir)
        :_pinA(pinA),_pinB(pinB), alpha(_alpha), dt(_dt), dir(_dir) {
            _pinA.mode(PullUp);
            _pinB.mode(PullUp);
            uint8_t pinA_state, pinB_state;
            pinA_state= _pinA.read();
            pinB_state= _pinB.read();
            _previousState = (pinA_state<<1|pinB_state);
            pulse_count = 0;
            _errorCount=0;
            CPR = 600;
            gear_ratio = 1;
            las_rpm = 0;
            las_rate = 0;
        }

    void init(){ // callbacks for all edges
        _pinA.enable_irq ();
        _pinB.enable_irq ();
        _pinA.rise(callback(this,&QEncoder::decode));
        _pinA.fall(callback(this,&QEncoder::decode));
        _pinB.rise(callback(this,&QEncoder::decode));
        _pinB.fall(callback(this,&QEncoder::decode));
        // _pinA.enable_irq ();
        // _pinB.enable_irq ();
    }

    float CalRPM(){
        float enc_rpm = (float)pulse_count*60*1000/(dt*CPR*gear_ratio*4);
        fil_rpm = LPF(&las_rpm,enc_rpm);
        pulse_count = 0;
        return fil_rpm;
    }

    float CalRATE(){
        ///RATE = RPM*
        float enc_rate = (float)pulse_count*1000*2*_PI/(dt*CPR*gear_ratio*4);//각속도
        fil_rate = LPF(&las_rate,enc_rate);
        pulse_count = 0;
        return fil_rate;
    }
    
    float LPF(float* prev_data, float current_data){
        if(isnan(current_data)){
            return *prev_data;
        }
        else{
            float fil_data;
            fil_data = alpha*(*prev_data)+(1-alpha)*current_data;
            *prev_data = fil_data;
            return fil_data;
        }
    }

    void disable_Interrupt(){
        _pinA.disable_irq ();
        _pinB.disable_irq ();
    }

    void enable_Interrupt(){
        _pinA.enable_irq ();
        _pinB.enable_irq ();
    }

    void setCount(int32_t EncoderVal){
        _pinA.disable_irq ();
        _pinB.disable_irq ();
        pulse_count = EncoderVal;
        _pinA.enable_irq ();
        _pinB.enable_irq ();
    }

    int32_t getErrorCount(){
        return _errorCount;
    }

    int32_t get_pulse_Count(){
        int32_t pulse = pulse_count;
        pulse_count=0;
        return pulse;
    }
};
