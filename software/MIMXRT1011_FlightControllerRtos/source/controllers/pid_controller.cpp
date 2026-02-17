#include "controllers/pid_controller.h"
#include <algorithm>
#include <cmath>

PIDController::PIDController(float p, float i, float d, float alpha)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f), alpha(alpha), dTermFiltered(0.0f) {}

float PIDController::calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - currentValue;
    float pOut = kp * error;
    float derivativeRaw = (error - prevError) / dt;
    dTermFiltered = (alpha * derivativeRaw) + ((1.0f - alpha) * dTermFiltered);
    float dOut = kd * dTermFiltered;
    float currentTotal = pOut + (ki * integral) + dOut;

    bool isSaturatedMax = (currentTotal >= maxLimit);
    bool isSaturatedMin = (currentTotal <= minLimit);

    // Anti-windup: stop integrating if saturated and error is driving further into saturation
    if (!isSaturatedMax && !isSaturatedMin) {
        integral += error * dt;
    } else if (isSaturatedMax && error < 0.0f) {
        integral += error * dt;
    } else if (isSaturatedMin && error > 0.0f) {
        integral += error * dt;
    }

    prevError = error;
    float finalOutput = pOut + (ki * integral) + dOut;
    return std::clamp(finalOutput, minLimit, maxLimit);
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
    dTermFiltered = 0.0f;
}
