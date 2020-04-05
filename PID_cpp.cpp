class PID {
    protected:
        double output;
        double I = 0; // integrator
        double D = 0; // differentiator
        double prev_error = 0;
        double error_d1; // for digital implementation
        double error_d2;
        double y_d1; 

    public:
    PID();

    virtual ~PID();

    /**
     * simple pid contoller
     * @param y_d Desired output
     * @param y Current Output
     * @param dt Time Step
     * @param kp Proportional Gain
     * @param ki Integral Gain
     * @param kd Derivative Gain
     * @param limit Saturation limit
     * @return Output Command
    */
    double pid_simple(double y_d, double y, double dt, double kp, double ki, double kd, double limit);

    /**
     * Digital implementation of PID
     * @param y_d Desired output
     * @param y Current Output
     * @param kp Proportional Gain
     * @param ki Integral Gain
     * @param kd Derivative Gain
     * @param limit Saturation limit
     * @return Output Command
    */
    double pid_digital(double y_d, double y, double kp, double ki, double kd, double limit);
};

PID::PID() {
    this->prev_error = 0;
    this->I = 0;
    this->D = 0;
    this-> error_d1 = 0;
    this-> error_d2 = 0;
}

PID::~PID() = default;

double PID::pid_simple(double y_d, double y, double dt, double kp, double ki, double kd, double limit) {
    double error = y_d - y;
    
    this->I += error*dt;
    this->D = (error - prev_error)/dt;

    this->prev_error = error;

    // unsaturated output
    double output_unsat = kp*error + ki*I + kd*D;

    // check for saturation
    if (output_unsat > limit) {this->output = limit;}
    else if (output_unsat < -limit) {this->output = -limit;}
    else {this->output = output_unsat;}

    return output;
}

/*
Reference: "Digital Implementation of PID Controller for Temperature Control", PrachiRusia, 
International Journal of Scientific & Engineering Research Volume 8, Issue 5, May-2017,
ISSN 2229-5518
*/
double PID::pid_digital(double y_d, double y, double kp, double ki, double kd, double limit) {
    
    // current error
    double error = y_d - y;

    double K1 = kp + ki + kd;
    double K2 = -kp - 2*kd;
    // unsaturated output
    double output_unsat = y_d1 + K1*error + K2*error_d1 + kd*error_d2;
    
    // check for saturation
    if (output_unsat > limit) {this->output = limit;}
    else if (output_unsat < -limit) {this->output = -limit;}
    else {this->output = output_unsat;}

    this->y_d1 = output;

    this-> error_d2 = error_d1;
    this-> error_d1 = error;

    return output;
}
