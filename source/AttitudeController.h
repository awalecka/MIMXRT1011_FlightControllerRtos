#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

#include "FreeRTOS.h"

/**
 * @struct FullSensorData
 * @brief Holds all sensor data required for the advanced controller.
 */
struct FullSensorData {
    // Attitude (estimated, e.g., from an EKF)
    float roll_deg;      // Current roll angle in degrees
    float pitch_deg;     // Current pitch angle in degrees
    float yaw_deg;       // Current yaw angle in degrees (not used by controller, but needed for Jacobian)

    // Angular Rates (body frame)
    float roll_rate_dps;  // p (Body-frame roll rate in degrees per second)
    float pitch_rate_dps; // q (Body-frame pitch rate in degrees per second)
    float yaw_rate_dps;   // r (Body-frame yaw rate in degrees per second)

    // Airspeed
    float true_airspeed_ms; // V_T (True airspeed in meters per second)
};

/**
 * @struct ActuatorOutput
 * @brief Holds the final output values for the aircraft's control surfaces.
 * The values represent normalized deflections.
 */
struct ActuatorOutput {
    TickType_t timestamp;
	float aileron;   // -1.0 (left) to 1.0 (right)
    float elevator;  // -1.0 (down) to 1.0 (up)
    float rudder;    // -1.0 (left) to 1.0 (right)
};

/**
 * @class PIDController
 * @brief A standard PID controller.
 */
class PIDController {
public:
    PIDController(float p, float i, float d);
    void setGains(float p, float i, float d);
    float calculate(float setpoint, float current_value, float dt);
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prev_error;
};

/**
 * @class AttitudeController
 * @brief Implements a cascaded P-PI attitude control architecture with feed-forward
 * and turn coordination.
 */
class AttitudeController {
public:
    AttitudeController();

    /**
     * @brief Updates the controller with new sensor data and computes control outputs.
     * @param sensor_data The latest data from the aircraft's sensors.
     * @param dt The time delta in seconds since the last update.
     * @return The computed actuator control signals.
     */
    ActuatorOutput update(const FullSensorData& sensor_data, float dt);

    /**
     * @brief Sets the desired target attitude for the controller.
     * @param roll The target roll angle in degrees.
     * @param pitch The target pitch angle in degrees.
     */
    void setSetpoints(float roll, float pitch);

private:
    // --- Controller Gains ---

    // Outer P-Loop (Attitude Angle -> Angular Rate)
    // Determines how aggressively to command an angular rate to correct an attitude error.
    // Unit: (rad/s) / rad  or simply 1/s
    float Kp_roll_angle;
    float Kp_pitch_angle;

    // Inner PI-Loop (Angular Rate -> Control Moment)
    // These controllers track the desired body rates calculated by the outer loop and
    // the Jacobian transformation.
    PIDController roll_rate_controller;
    PIDController pitch_rate_controller;
    PIDController yaw_rate_controller;

    // Feed-Forward Gains (Desired Angular Rate -> Control Moment)
    // Proactively commands a control moment based on the desired rate, improving response time.
    // Unit: (normalized moment output) / (rad/s)
    float K_ff_roll;
    float K_ff_pitch;
    float K_ff_yaw;

    // --- State Variables ---

    // Target attitude setpoints in degrees.
    float target_roll_deg;
    float target_pitch_deg;

    // --- Physical Constants ---

    // Acceleration due to gravity in m/s^2. Used for turn coordination calculation.
    const float GRAVITY_MS2 = 9.80665;
};

#endif // ATTITUDE_CONTROLLER_H
