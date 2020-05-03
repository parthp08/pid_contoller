/**
 * PID Controller implemenation for Arduino.
 * 
 * Use:
 *  PID myPID;  // init pid
 *  myPID.set_gains(kp,ki,kd);
 *  myPID.set_input_range(up_limit, low_limit); // saturation limit of the input
 *  input = myPID.caclulate(y_desired, y_actual);   // calculate inputs required
 *          "or"
 *  input = myPID.calculate_digital(y_desired, y_actual); // digital implementation of PID
 *              //note: digital pid require different gains than simple pid
*/
class PID {
    public:
        PID();

        virtual ~PID();

        /**
         * set kp, ki and kd
         * @param kp Proportional Gain
         * @param ki Integral Gain
         * @param kd Derivative Gain
         */
        void set_gains(float kp, float ki, float kd);

        /**
         * set input limits (saturation)
         * @param up_limit Upper Saturation limit
         * @param low_limit lower Saturation limit
         */
        void set_input_range(float up_limit, float low_limit);

        /**
         * simple pid contoller
         * @param y_d Desired output
         * @param y Current Output
         * @return Output Command
         */
        float calculate(float y_d, float y);

        /**
         * Digital implementation of PID
         * @param y_d Desired output
         * @param y Current Output
         * @return Output Command
         * @note Equations for digital implementation of PID can be found in below reference.
         * @cite {"Digital Implementation of PID Controller for Temperature Control", PrachiRusia, 
         *      International Journal of Scientific & Engineering Research Volume 8, Issue 5, May-2017,
         *      ISSN 2229-5518}
         */
        float calculate_digital(float y_d, float y);

    protected:
        // for both implementation
        float output;
        float kp;
        float ki;
        float kd;
        float up_limit;
        float low_limit;
        
        // for simple pid
        float I; // integrator
        float I_max;    // for anti-windup
        float D; // differentiator
        float prev_error;
        float tp;   // prev_time
        //float dt = 0.01;   // time step   
        // float dt_inv = 100;
        float t0;
        
        // for digital pid
        float error_d1; // for digital implementation
        float error_d2;
        float y_d1; 
        float K1;
        float K2;

        /**
         * check for saturation of the output
         * @param output unsaturated output
         * @return output saturated output
         */
        float saturation_check(float output);

        /**
         * check anti-windup for integrator
         * @param error error between desired and actual value
         * @return error_ appropriate error for integral only
         */
        float integrator_anti_windup(float error);
};

