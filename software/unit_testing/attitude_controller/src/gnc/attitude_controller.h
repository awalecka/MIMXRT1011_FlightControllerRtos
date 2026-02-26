#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "common_types.h"
#include "pid_controller.h"

/**
 * @file attitude_controller.h
 * @brief Cascaded attitude controller for fixed-wing aircraft.
 *
 * Implements a two-loop cascaded controller:
 *   - Outer loop: proportional angle error → Euler rate setpoints
 *   - Inner loop: PID body-rate controllers (roll/pitch/yaw)
 *
 * Coordinated turn assistance is computed from airspeed and bank angle.
 * All PID outputs are airspeed-scaled to maintain consistent handling
 * qualities across the flight envelope.
 *
 * @note Designed for the NXP RT1011. No heap allocation is performed;
 *       all state is held in statically-sized members.
 */
class AttitudeController {
public:
    /**
     * @brief Constructs the controller with default tuning gains.
     *
     * Default gains are set for a nominal 20 m/s airspeed. Re-tune via
     * the individual gain setters after construction if required.
     */
    AttitudeController();

    /**
     * @brief Sets the desired roll and pitch attitude setpoints.
     *
     * Inputs are validated and clamped to the aircraft's safe flight-envelope
     * limits before being stored.  Commands outside these limits are silently
     * saturated; callers should not rely on the controller accepting arbitrary
     * angles.
     *
     * @param roll  Desired roll angle  [degrees], clamped to ±MAX_ROLL_DEG.
     * @param pitch Desired pitch angle [degrees], clamped to ±MAX_PITCH_DEG.
     */
    void setSetpoints(float roll, float pitch);

    /**
     * @brief Runs one control cycle and returns normalised actuator demands.
     *
     * Must be called at a fixed, known rate.  The supplied @p dt is validated;
     * if it is outside the range (0, MAX_DT_S] the previous actuator output is
     * returned unchanged to prevent integrator wind-up or derivative spikes
     * following a scheduler overrun.
     *
     * Sensor inputs are checked for finiteness before use.  A non-finite value
     * in any required field causes an early return of the last known-good output.
     *
     * @param sensorData  Current IMU/airdata measurements.
     * @param dt          Time elapsed since the last call [seconds].
     * @return            Normalised actuator demands in the range [-1, 1].
     */
    ActuatorOutput update(const FullSensorData& sensorData, float dt);

    /**
     * @brief Returns the currently active pitch setpoint.
     * @return Target pitch angle [degrees].
     */
    float getTargetPitchDeg() const { return targetPitchDeg; }

    /**
     * @brief Returns the currently active roll setpoint.
     * @return Target roll angle [degrees].
     */
    float getTargetRollDeg() const { return targetRollDeg; }

private:
    /* ------------------------------------------------------------------ */
    /* Physical / tuning constants                                         */
    /* ------------------------------------------------------------------ */

    /** Gravitational acceleration [m/s²]. */
    static constexpr float GRAVITY_MS2 = 9.80665f;

    /** Airspeed at which the inner-loop PIDs were tuned [m/s]. */
    static constexpr float NOMINAL_AIRSPEED_FOR_TUNING = 20.0f;

    /**
     * @brief Low-pass filter cut-off applied inside each PID controller [rad/s].
     *
     * Value 0.5569 rad/s ≈ 0.0886 Hz — this attenuates high-frequency noise
     * on the derivative term while preserving adequate phase margin at typical
     * rate-loop crossover frequencies (~5–15 rad/s for small fixed-wing).
     */
    static constexpr float RATE_PID_FILTER_CUTOFF_RADS = 0.5569f;

    /**
     * @brief Minimum airspeed below which the dynamic-pressure inverse scaler
     *        is no longer updated [m/s].
     *
     * Prevents division-by-zero and avoids nonsensical scaling during taxi or
     * when the pitot tube is obstructed.
     */
    static constexpr float MIN_AIRSPEED_FOR_SCALING_MS = 1.0f;

    /**
     * @brief Lower bound of the dynamic-pressure inverse scaler [-].
     *
     * Prevents over-deflection at airspeeds well above the tuning point; a
     * value of 0.1 limits authority reduction to 10× the nominal tuning gain.
     */
    static constexpr float AIRSPEED_SCALER_MIN = 0.1f;

    /**
     * @brief Upper bound of the dynamic-pressure inverse scaler [-].
     *
     * Prevents excessive deflection at very low airspeeds; 10× is chosen as a
     * practical saturation that keeps the plant in a roughly linear regime.
     */
    static constexpr float AIRSPEED_SCALER_MAX = 10.0f;

    /**
     * @brief Maximum allowable roll setpoint [degrees].
     *
     * Clamps commanded bank angle to avoid the tan(φ) singularity in the
     * coordinated-turn computation, which diverges as φ → ±90°.
     */
    static constexpr float MAX_ROLL_DEG = 70.0f;

    /**
     * @brief Maximum allowable pitch setpoint [degrees].
     *
     * Chosen to keep the aircraft within a normal manoeuvring envelope and to
     * ensure the Euler-to-body-rate transform remains well-conditioned
     * (cos(θ) remains > ~0.34 at ±70°).
     */
    static constexpr float MAX_PITCH_DEG = 45.0f;

    /**
     * @brief Maximum credible loop period [seconds].
     *
     * If dt exceeds this value it is assumed a scheduler fault occurred and the
     * last output is held rather than integrating over an abnormally long window.
     */
    static constexpr float MAX_DT_S = 0.5f;

    /* ------------------------------------------------------------------ */
    /* Outer-loop (angle) gains                                            */
    /* ------------------------------------------------------------------ */

    /** Proportional gain: roll angle error → roll rate setpoint [1/s]. */
    float kpRollAngle;

    /** Proportional gain: pitch angle error → pitch rate setpoint [1/s]. */
    float kpPitchAngle;

    /* ------------------------------------------------------------------ */
    /* Inner-loop (rate) PID controllers                                   */
    /* ------------------------------------------------------------------ */

    PIDController rollRateController;   /**< Controls body roll  rate p [rad/s]. */
    PIDController pitchRateController;  /**< Controls body pitch rate q [rad/s]. */
    PIDController yawRateController;    /**< Controls body yaw   rate r [rad/s]. */

    /* ------------------------------------------------------------------ */
    /* Feed-forward gains                                                  */
    /* ------------------------------------------------------------------ */

    /**
     * @brief Feed-forward gains applied to the body-rate setpoint derivatives.
     *
     * Each gain maps commanded angular acceleration [rad/s²] to a normalised
     * moment demand [-].  Set to 0 to disable feed-forward on the corresponding
     * axis until the aircraft-specific inertia values are characterised.
     */
    float kFfRoll;   /**< Roll  axis FF gain  [normalised moment / (rad/s²)]. */
    float kFfPitch;  /**< Pitch axis FF gain  [normalised moment / (rad/s²)]. */
    float kFfYaw;    /**< Yaw   axis FF gain  [normalised moment / (rad/s²)]. */

    /* ------------------------------------------------------------------ */
    /* State                                                               */
    /* ------------------------------------------------------------------ */

    float targetRollDeg;   /**< Active roll  setpoint [deg], clamped to ±MAX_ROLL_DEG.  */
    float targetPitchDeg;  /**< Active pitch setpoint [deg], clamped to ±MAX_PITCH_DEG. */

    /**
     * @brief Body-rate setpoints from the previous control cycle [rad/s].
     *
     * Retained so that the feed-forward term can be computed from the
     * finite-difference derivative of the setpoint rather than from the
     * setpoint magnitude itself, avoiding a steady-state bias moment.
     */
    float prevPSpRadS;  /**< Previous roll  rate setpoint p [rad/s]. */
    float prevQSpRadS;  /**< Previous pitch rate setpoint q [rad/s]. */
    float prevRSpRadS;  /**< Previous yaw   rate setpoint r [rad/s]. */

    /** Last actuator output, returned unchanged on a bad dt or sensor fault. */
    ActuatorOutput lastOutput;
};

#endif // ATTITUDE_CONTROLLER_H
