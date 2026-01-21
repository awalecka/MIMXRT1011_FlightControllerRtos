#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "common_types.h"
#include "controllers/pid_controller.h"

class AttitudeController {
public:
    AttitudeController();

    void setSetpoints(float roll, float pitch);
    ActuatorOutput update(const FullSensorData& sensorData, float dt);

    float getTargetPitchDeg() const { return targetPitchDeg; }
    float getTargetRollDeg() const { return targetRollDeg; }

private:
    float kpRollAngle, kpPitchAngle;
    PIDController rollRateController;
    PIDController pitchRateController;
    PIDController yawRateController;
    float kFfRoll, kFfPitch, kFfYaw;
    float targetRollDeg, targetPitchDeg;

    static constexpr float GRAVITY_MS2 = 9.80665f;
    static constexpr float NOMINAL_AIRSPEED_FOR_TUNING = 20.0f;
};

#endif // ATTITUDE_CONTROLLER_H
