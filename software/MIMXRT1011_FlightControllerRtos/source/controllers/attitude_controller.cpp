#include "attitude_controller.h"
#include "arm_math.h"
#include "utils.h"
#include <cmath>
#include <algorithm>
#include <numbers>

/* --------------------------------------------------------------------------
 * File-scope helpers
 * -------------------------------------------------------------------------- */

/**
 * @brief Converts degrees to radians.
 * @param deg  Angle in degrees.
 * @return     Angle in radians.
 */
static constexpr float degToRad(float deg)
{
    return deg * static_cast<float>(std::numbers::pi_v<double>) / 180.0f;
}

/* --------------------------------------------------------------------------
 * Construction
 * -------------------------------------------------------------------------- */

AttitudeController::AttitudeController()
    : kpRollAngle(6.0f),
      kpPitchAngle(6.0f),
      rollRateController(0.1f, 0.01f, 0.0f, RATE_PID_FILTER_CUTOFF_RADS),
      pitchRateController(0.1f, 0.01f, 0.0f, RATE_PID_FILTER_CUTOFF_RADS),
      yawRateController(0.15f, 0.01f, 0.0f, RATE_PID_FILTER_CUTOFF_RADS),
      kFfRoll(0.0f),   // FF disabled until per-axis inertia is characterised
      kFfPitch(0.0f),
      kFfYaw(0.0f),
      targetRollDeg(0.0f),
      targetPitchDeg(0.0f),
      prevPSpRadS(0.0f),
      prevQSpRadS(0.0f),
      prevRSpRadS(0.0f),
      lastOutput{}
{
}

/* --------------------------------------------------------------------------
 * Setpoint ingress
 * -------------------------------------------------------------------------- */

void AttitudeController::setSetpoints(float roll, float pitch)
{
    // Clamp to the safe flight-envelope limits defined in the header.
    // MAX_ROLL_DEG (70°) keeps tan(φ) well away from its ±90° singularity.
    // MAX_PITCH_DEG (45°) keeps cos(θ) > ~0.7, ensuring the Euler-to-body
    // rate transform remains well-conditioned.
    targetRollDeg  = std::clamp(roll,  -MAX_ROLL_DEG,  MAX_ROLL_DEG);
    targetPitchDeg = std::clamp(pitch, -MAX_PITCH_DEG, MAX_PITCH_DEG);
}

/* --------------------------------------------------------------------------
 * Main update
 * -------------------------------------------------------------------------- */

ActuatorOutput AttitudeController::update(const FullSensorData& sensorData, float dt)
{
    /* -- Guard: reject pathological time steps ----------------------------- */
    // A dt of zero or negative would produce divide-by-zero in the PID
    // derivative; an excessively large dt (scheduler overrun) would cause
    // integrator wind-up.  In either case, return the last known-good output.
    if (dt <= 0.0f || dt > MAX_DT_S) {
        return lastOutput;
    }

    /* -- Guard: reject non-finite sensor data ------------------------------ */
    // A failed IMU or disconnected sensor can produce NaN / Inf on embedded
    // targets.  Propagating these through the controller would corrupt the
    // integrators and require a full reset.
    if (!std::isfinite(sensorData.rollDeg)    ||
        !std::isfinite(sensorData.pitchDeg)   ||
        !std::isfinite(sensorData.rollRateDps) ||
        !std::isfinite(sensorData.pitchRateDps)||
        !std::isfinite(sensorData.yawRateDps)  ||
        !std::isfinite(sensorData.trueAirspeedMs)) {
        return lastOutput;
    }

    /* -- Convert current attitude to radians ------------------------------- */
    const float currentRollRad  = degToRad(sensorData.rollDeg);
    const float currentPitchRad = degToRad(sensorData.pitchDeg);
    const float targetRollRad   = degToRad(targetRollDeg);
    const float targetPitchRad  = degToRad(targetPitchDeg);

    /* -- Pre-compute trig values used in multiple places below ------------- */
    const float sinRoll  = arm_sin_f32(currentRollRad);
    const float cosRoll  = arm_cos_f32(currentRollRad);
    const float sinPitch = arm_sin_f32(currentPitchRad);
    const float cosPitch = arm_cos_f32(currentPitchRad);

    // tan(φ) is used for the coordinated-turn yaw rate.  setSetpoints() has
    // already clamped targetRollDeg to ±MAX_ROLL_DEG (70°), so the current
    // roll angle should never reach 90° in normal operation; the guard below
    // is a belt-and-braces safety net.
    const float tanRoll = (std::abs(cosRoll) > 1e-4f) ? (sinRoll / cosRoll) : 0.0f;

    /* -- Coordinated-turn yaw rate feedforward ----------------------------- */
    // ψ̇ = (g / V) · tan(φ) is the navigation-frame Euler yaw rate required to
    // maintain a balanced, coordinated turn.  No cos(θ) pitch correction is
    // applied here: this formula is already expressed in the navigation frame
    // and is valid at any pitch attitude.  The cos(θ) weighting appears
    // naturally one step later, inside the Euler-to-body-rate transform on the
    // r-axis (see rSpRadS below), so applying it here would double-count it
    // and under-drive the rudder at elevated pitch angles.
    float coordinatedYawRateRadS = 0.0f;
    if (std::abs(sensorData.trueAirspeedMs) > MIN_AIRSPEED_FOR_SCALING_MS) {
        coordinatedYawRateRadS = (GRAVITY_MS2 / sensorData.trueAirspeedMs) * tanRoll;
    }

    /* -- Outer loop: angle error → Euler rate setpoints ------------------- */
    const float phiDotSpRadS = kpRollAngle  * (targetRollRad  - currentRollRad);
    const float thetaDotSpRadS = kpPitchAngle * (targetPitchRad - currentPitchRad);
    const float psiDotSpRadS   = coordinatedYawRateRadS;

    /* -- Transform Euler rate setpoints to body-rate setpoints ------------- */
    // Standard ZYX Euler kinematic relationship (Stengel, "Flight Dynamics", §2.4):
    //
    //   | p |   |  1      0        −sin(θ)     | | φ̇ |
    //   | q | = |  0   cos(φ)   cos(θ)·sin(φ)  | | θ̇ |
    //   | r |   |  0  −sin(φ)   cos(θ)·cos(φ)  | | ψ̇ |
    //
    // The q-row ψ̇ cross-term is cos(θ)·sin(φ) — cosPitch is correct here.
    // The cos(θ) in the r-row is also where the pitch dependency of the
    // coordinated-turn yaw rate naturally enters the body-frame demand.
    const float pSpRadS = phiDotSpRadS - psiDotSpRadS * sinPitch;
    const float qSpRadS = thetaDotSpRadS * cosRoll + psiDotSpRadS * cosPitch * sinRoll;
    const float rSpRadS = -thetaDotSpRadS * sinRoll + psiDotSpRadS * cosPitch * cosRoll;

    /* -- Feed-forward: setpoint derivative → angular acceleration ---------- */
    // FF is computed from the finite-difference derivative of the body-rate
    // setpoint (i.e., the commanded angular acceleration) rather than from the
    // setpoint magnitude.  Using the magnitude would inject a constant bias
    // moment whenever a non-zero rate is commanded, causing a steady-state
    // attitude error that the integrator must fight.
    const float pSpDotRadS2  = (pSpRadS  - prevPSpRadS)  / dt;
    const float qSpDotRadS2  = (qSpRadS  - prevQSpRadS)  / dt;
    const float rSpDotRadS2  = (rSpRadS  - prevRSpRadS)  / dt;

    const float ffRollMoment  = kFfRoll  * pSpDotRadS2;
    const float ffPitchMoment = kFfPitch * qSpDotRadS2;
    const float ffYawMoment   = kFfYaw   * rSpDotRadS2;

    // Persist setpoints for the derivative calculation in the next cycle
    prevPSpRadS = pSpRadS;
    prevQSpRadS = qSpRadS;
    prevRSpRadS = rSpRadS;

    /* -- Convert measured rates to rad/s ----------------------------------- */
    const float pRadS = degToRad(sensorData.rollRateDps);
    const float qRadS = degToRad(sensorData.pitchRateDps);
    const float rRadS = degToRad(sensorData.yawRateDps);

    /* -- Dynamic-pressure inverse scaler ----------------------------------- */
    // Control surface effectiveness is proportional to dynamic pressure (½ρV²).
    // Scaling PID output by (V_nom / V)² keeps the effective loop gain roughly
    // constant across the flight envelope:
    //   scaler < 1 at high speed  → less deflection needed (surfaces more effective)
    //   scaler > 1 at low speed   → more deflection needed (surfaces less effective)
    float dynamicPressureInvScaler = 1.0f;
    if (std::abs(sensorData.trueAirspeedMs) > MIN_AIRSPEED_FOR_SCALING_MS) {
        const float vRatio = NOMINAL_AIRSPEED_FOR_TUNING / sensorData.trueAirspeedMs;
        dynamicPressureInvScaler = vRatio * vRatio;
    }
    dynamicPressureInvScaler = std::clamp(dynamicPressureInvScaler,
                                          AIRSPEED_SCALER_MIN,
                                          AIRSPEED_SCALER_MAX);

    /* -- Compute PID output clamp limits ----------------------------------- */
    // The total actuator demand is  (PID + FF) * scaler.  To prevent the
    // summed output from exceeding ±1 after scaling, the PID is clamped to
    // leave headroom for the FF contribution.  std::max/min guards ensure the
    // bounds cannot invert (which would happen if |FF| > 1/scaler).
    const float scalerInv = 1.0f / dynamicPressureInvScaler;

    const float maxRollPID  = std::max(0.0f,  scalerInv - ffRollMoment);
    const float minRollPID  = std::min(0.0f, -scalerInv - ffRollMoment);
    const float maxPitchPID = std::max(0.0f,  scalerInv - ffPitchMoment);
    const float minPitchPID = std::min(0.0f, -scalerInv - ffPitchMoment);
    const float maxYawPID   = std::max(0.0f,  scalerInv - ffYawMoment);
    const float minYawPID   = std::min(0.0f, -scalerInv - ffYawMoment);

    /* -- Inner loop: PID rate controllers ---------------------------------- */
    const float fbRollMoment  = rollRateController.calculate(
                                    pSpRadS, pRadS, dt, minRollPID,  maxRollPID);
    const float fbPitchMoment = pitchRateController.calculate(
                                    qSpRadS, qRadS, dt, minPitchPID, maxPitchPID);
    const float fbYawMoment   = yawRateController.calculate(
                                    rSpRadS, rRadS, dt, minYawPID,   maxYawPID);

    /* -- Sum FB + FF, then apply inverse scaler ---------------------------- */
    // Re-applying the scaler after summing ensures the FF and FB contributions
    // both benefit from the same dynamic-pressure compensation.
    const float totalRollMoment  = (fbRollMoment  + ffRollMoment)  * dynamicPressureInvScaler;
    const float totalPitchMoment = (fbPitchMoment + ffPitchMoment) * dynamicPressureInvScaler;
    const float totalYawMoment   = (fbYawMoment   + ffYawMoment)   * dynamicPressureInvScaler;

    /* -- Final clamp to normalised actuator range -------------------------- */
    ActuatorOutput controls;
    controls.aileron  = std::clamp(totalRollMoment,  -1.0f, 1.0f);
    controls.elevator = std::clamp(totalPitchMoment, -1.0f, 1.0f);
    controls.rudder   = std::clamp(totalYawMoment,   -1.0f, 1.0f);

    lastOutput = controls; // Cache for fault-hold on the next cycle
    return controls;
}
