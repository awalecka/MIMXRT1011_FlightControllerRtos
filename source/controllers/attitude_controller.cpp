#include "controllers/attitude_controller.h"
#include "arm_math.h" // For arm_sin_f32, arm_cos_f32
#include "utils/utils.h" // For mapFloat etc if needed, or local helper
#include <cmath>
#include <algorithm>

// Local helpers for conversion if not in utils.h
static constexpr float degToRad(float deg) { return deg * 3.1415926535f / 180.0f; }

AttitudeController::AttitudeController()
    : kpRollAngle(6.0f), kpPitchAngle(6.0f),
      rollRateController(0.1f, 0.01f, 0.0f, 0.5569f),
      pitchRateController(0.1f, 0.01f, 0.0f, 0.5569f),
      yawRateController(0.15f, 0.01f, 0.0f, 0.5569f),
      kFfRoll(0.1f), kFfPitch(0.1f), kFfYaw(0.1f),
      targetRollDeg(0.0f), targetPitchDeg(0.0f) {}

void AttitudeController::setSetpoints(float roll, float pitch) {
    targetRollDeg = roll;
    targetPitchDeg = pitch;
}

ActuatorOutput AttitudeController::update(const FullSensorData& sensorData, float dt) {
    float currentRollRad = degToRad(sensorData.rollDeg);
    float currentPitchRad = degToRad(sensorData.pitchDeg);
    float targetRollRad = degToRad(targetRollDeg);
    float targetPitchRad = degToRad(targetPitchDeg);

    float sinRoll = arm_sin_f32(currentRollRad);
    float cosRoll = arm_cos_f32(currentRollRad);
    float sinPitch = arm_sin_f32(currentPitchRad);
    float cosPitch = arm_cos_f32(currentPitchRad);
    float tanRoll = (std::abs(cosRoll) > 1e-4f) ? (sinRoll / cosRoll) : 0.0f;

    // Coordinated Turn Logic
    float coordinatedYawRateRadS = 0.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        coordinatedYawRateRadS = (GRAVITY_MS2 / sensorData.trueAirspeedMs) * tanRoll;
    }

    // Rate Setpoints
    float phiDotSpRadS = kpRollAngle * (targetRollRad - currentRollRad);
    float thetaDotSpRadS = kpPitchAngle * (targetPitchRad - currentPitchRad);
    float psiDotSpRadS = coordinatedYawRateRadS;

    // Body Rate Setpoints Transform
    float pSpRadS = phiDotSpRadS - psiDotSpRadS * sinPitch;
    float qSpRadS = thetaDotSpRadS * cosRoll + psiDotSpRadS * cosPitch * sinRoll;
    float rSpRadS = -thetaDotSpRadS * sinRoll + psiDotSpRadS * cosPitch * cosRoll;

    // Feed Forward
    float ffRollMoment = kFfRoll * pSpRadS;
    float ffPitchMoment = kFfPitch * qSpRadS;
    float ffYawMoment = kFfYaw * rSpRadS;

    float pRadS = degToRad(sensorData.rollRateDps);
    float qRadS = degToRad(sensorData.pitchRateDps);
    float rRadS = degToRad(sensorData.yawRateDps);

    // Airspeed Scaling
    float airspeedScaler = 1.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        airspeedScaler = (NOMINAL_AIRSPEED_FOR_TUNING * NOMINAL_AIRSPEED_FOR_TUNING) /
                         (sensorData.trueAirspeedMs * sensorData.trueAirspeedMs);
    }
    airspeedScaler = std::clamp(airspeedScaler, 0.1f, 10.0f);

    float scalerInv = 1.0f / airspeedScaler;
    float maxRollPID = (1.0f * scalerInv) - ffRollMoment;
    float minRollPID = (-1.0f * scalerInv) - ffRollMoment;
    float maxPitchPID = (1.0f * scalerInv) - ffPitchMoment;
    float minPitchPID = (-1.0f * scalerInv) - ffPitchMoment;
    float maxYawPID = (1.0f * scalerInv) - ffYawMoment;
    float minYawPID = (-1.0f * scalerInv) - ffYawMoment;

    float fbRollMoment = rollRateController.calculate(pSpRadS, pRadS, dt, minRollPID, maxRollPID);
    float fbPitchMoment = pitchRateController.calculate(qSpRadS, qRadS, dt, minPitchPID, maxPitchPID);
    float fbYawMoment = yawRateController.calculate(rSpRadS, rRadS, dt, minYawPID, maxYawPID);

    float totalRollMoment = (fbRollMoment + ffRollMoment) * airspeedScaler;
    float totalPitchMoment = (fbPitchMoment + ffPitchMoment) * airspeedScaler;
    float totalYawMoment = (fbYawMoment + ffYawMoment) * airspeedScaler;

    ActuatorOutput controls;
    controls.aileron = std::clamp(totalRollMoment, -1.0f, 1.0f);
    controls.elevator = std::clamp(totalPitchMoment, -1.0f, 1.0f);
    controls.rudder = std::clamp(totalYawMoment, -1.0f, 1.0f);

    return controls;
}
