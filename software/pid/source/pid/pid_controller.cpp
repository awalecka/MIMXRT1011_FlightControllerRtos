#include "pid_controller.h"
#include <algorithm>
#include <cmath>

PIDController::PIDController(float p, float i, float d, float alpha)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f), alpha(alpha), dTermFiltered(0.0f) {
}

float PIDController::calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit) {
    // Guard against non-positive time steps to prevent division by zero in the derivative
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - currentValue;

    /* -- Proportional Term ------------------------------------------------- */
    // P_out = Kp * e(t)
    float pOut = kp * error;

    /* -- Derivative Term with EMA Filter ----------------------------------- */
    // Raw derivative: de/dt = (e(t) - e(t-1)) / dt
    // Filtered: D_f = alpha * D_raw + (1 - alpha) * D_f_prev
    float derivativeRaw = (error - prevError) / dt;
    dTermFiltered = (alpha * derivativeRaw) + ((1.0f - alpha) * dTermFiltered);
    float dOut = kd * dTermFiltered;

    /* -- Saturation Check -------------------------------------------------- */
    // Calculate the theoretical output using the *previous* integral state
    // to determine if the system is currently saturated.
    float currentTotal = pOut + (ki * integral) + dOut;

    bool isSaturatedMax = (currentTotal >= maxLimit);
    bool isSaturatedMin = (currentTotal <= minLimit);

    /* -- Integral Term with Conditional Integration (Anti-Windup) ---------- */
    // Only integrate if the system is not saturated, OR if the current error
    // is pushing the output away from the saturation limit.
    // This prevents the accumulator from winding up into a deep stall.
    if (!isSaturatedMax && !isSaturatedMin) {
        integral += error * dt;
    } else if (isSaturatedMax && error < 0.0f) {
        integral += error * dt;
    } else if (isSaturatedMin && error > 0.0f) {
        integral += error * dt;
    }

    prevError = error;

    /* -- Final Output Combination ------------------------------------------ */
    // Recalculate total with the newly updated integral term
    float finalOutput = pOut + (ki * integral) + dOut;

    return std::clamp(finalOutput, minLimit, maxLimit);
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
    dTermFiltered = 0.0f;
}
