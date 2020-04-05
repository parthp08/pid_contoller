#include "PID_arduino.h"

PID::PID(){};

PID::~PID() = default;


float PID::calculate(float y_d, float y, float dt, float kp, float ki, float kd, float limit) {
    float error = y_d - y;
    
    this->I += error*dt;
    this->D = (error - prev_error)/dt;

    this->prev_error = error;

    // unsaturated output
    float output_unsat = kp*error + ki*I + kd*D;

    // check for saturation
    if (output_unsat > limit) {return limit;}
    else if (output_unsat < -limit) {return -limit;}
    else {return output_unsat;}
};
