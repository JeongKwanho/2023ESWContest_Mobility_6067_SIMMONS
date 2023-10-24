#pragma once
#include "mbed.h"
#include <cstdint>

class PIDBasic {
    public:
        PIDBasic(float kp=0.,float ki=0.,float kd=0.,float imax=0):
        _kp(kp),_ki(ki),_kd(kd),_imax(fabs(imax)){//멤버초기화리스트
        _I_term=0.0f; _last_output=0.0f;
    }
    void operator()(float kp=0.,float ki=0.,float kd=0.,float imax=0.){
        _kp = kp; _ki = ki; _kd = kd; _imax = fabs(imax);
    }
    float constrain(float x, float x_min, float x_max){
        return (x>x_max)? x_max: ((x<x_min)? x_min:x);
    }

    float computePID(float target, float output, uint32_t _dt, float max){
        float ref;
        float error = target - output; // error
        float _P_term= _kp*error; //P error = kp*error
        float dt = _dt*0.001f;
        
        if (target == 0.0) _I_term = 0.0;
        _I_term =_I_term+ _ki*error*dt; //I error = I error + ki*error*dt  (적분)
        
         // I windup 방지
        _I_term=constrain(_I_term,-_imax,_imax);

        float _D_term =_kd*(-output+_last_output)/dt;//D error = (error - previous error) / (이전 시간 - 현재 시간)  (미분), 여기선 error 대신 아웃풋 변화량 -> 계단파 방지
        _last_output=output;

        ref =_P_term + _I_term + _D_term;
        ref = constrain(ref,-max, max);
        return ref; // 이 속도로 돌려라
    }
    private:
        float _kp, _ki, _kd;
        float _imax;
        float _I_term; // integrator term
        float _last_output; // last error for derivative
};