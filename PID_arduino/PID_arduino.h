class PID {
    protected:
        float I = 0; // integrator
        float D = 0; // differentiator
        float prev_error = 0;

    public:
    PID();
    
    virtual ~PID();

    /**
     * calculate output
     * @param y_d Desired output
     * @param y Current Output
     * @param dt Time Step
     * @param kp Proportional Gain
     * @param ki Integral Gain
     * @param kd Derivative Gain
     * @param limit Saturation limit
     * @return Output Command
    */
    float calculate(float y_d, float y, float dt, float kp, float ki, float kd, float limit);
};