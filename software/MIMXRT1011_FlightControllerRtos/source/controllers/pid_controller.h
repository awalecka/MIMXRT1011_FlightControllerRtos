#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    /**
     * @brief Construct a new PIDController object
     * @param p Proportional gain
     * @param i Integral gain
     * @param d Derivative gain
     * @param alpha Derivative filter smoothing factor (0.0 - 1.0)
     */
    PIDController(float p, float i, float d, float alpha);

    /**
     * @brief Update the PID loop.
     */
    float calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit);

    /**
     * @brief Reset integral and derivative terms.
     */
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float alpha;
    float dTermFiltered;
};

#endif // PID_CONTROLLER_H
