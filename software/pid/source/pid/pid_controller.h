#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @class PIDController
 * @brief Proportional-Integral-Derivative controller with anti-windup and derivative filtering.
 *
 * Implements a standard parallel PID architecture with conditional integration
 * (clamping) to prevent windup during actuator saturation. The derivative term
 * features an exponential moving average (EMA) filter to attenuate high-frequency noise.
 */
class PIDController {
public:
    /**
     * @brief Constructs a new PIDController object.
     * * @param p     Proportional gain (Kp).
     * @param i     Integral gain (Ki).
     * @param d     Derivative gain (Kd).
     * @param alpha Derivative filter smoothing factor [0.0 - 1.0]. Lower is smoother.
     */
    PIDController(float p, float i, float d, float alpha);

    /**
     * @brief Computes the control output for the current time step.
     * * @param setpoint     Desired target value.
     * @param currentValue Current measured value.
     * @param dt           Time elapsed since the last update [seconds].
     * @param minLimit     Minimum allowable control output (saturation lower bound).
     * @param maxLimit     Maximum allowable control output (saturation upper bound).
     * @return float       Clamped control output.
     */
    float calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit);

    /**
     * @brief Resets the integral accumulator and derivative state histories.
     * * Useful when re-engaging the controller after a period of manual control
     * or when switching operational modes to prevent sudden output jumps.
     */
    void reset();

private:
    float kp;
    float ki;
    float kd;

    float integral;
    float prevError;

    float alpha;
    float dTermFiltered;
};

#endif // PID_CONTROLLER_H
