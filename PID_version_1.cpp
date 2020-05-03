#include "PID_version_1.h"

PID::PID() {
    this->I = 0;
    this-> D = 0;
    this-> prev_error = 0;
    this-> error_d1 = 0;
    this-> error_d2 = 0;
    t0 = micros()*1.0e-6;
    this->tp = 0;
}

PID::~PID() = default;

void PID::set_gains(float kp, float ki, float kd){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PID::set_input_range(float up_limit, float low_limit){
    this->up_limit = up_limit;
    this->low_limit = low_limit;
}

float PID::saturation_check(float output){
    if (output > up_limit) return up_limit;
    else if (output < low_limit) return low_limit;
    else return output;
}

float PID::integrator_anti_windup(float error){
    // set max integral bounds
    if(ki > 0.0) float I_max = 0.30*up_limit;  // 30% of max output
    else float I_max = 0;

    // use appropriate integration
    // returns // appropriate error using anti-windup for integral
    if((I > I_max) && (error > 0)) return 0;   // stop integration
    else if((I < -I_max) && (error < 0)) return 0; // stop
    else return error;
}

float PID::calculate(float y_d, float y) {
    float error = y_d - y;
    
    float t = micros()*1.0e-6 - t0;
    float dt = t - tp;
    this->tp = t;

    this->I += integrator_anti_windup(error)*dt;
    this->D = (error - prev_error)/dt;

    this->prev_error = error;

    // unsaturated output
    output = kp*error + ki*I + kd*D;

    return saturation_check(output);
}

float PID::calculate_digital(float y_d, float y) {
    
    // current error
    float error = y_d - y;

    K1 = kp + ki + kd;
    K2 = -kp - 2*kd;
    
    // unsaturated output
    output = y_d1 + K1*error + K2*error_d1 + kd*error_d2;

    this->error_d2 = error_d1;
    this->error_d1 = error;

    this->y_d1 = saturation_check(output);
    return y_d1;    // return saturated output
};
