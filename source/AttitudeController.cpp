#include "AttitudeController.h"
#include <algorithm> // For std::clamp
#include <cmath>     // For mathematical functions like sin, cos, tan

// --- Utility Functions ---
constexpr float deg_to_rad(float deg) { return deg * M_PI / 180.0; }
constexpr float rad_to_deg(float rad) { return rad * 180.0 / M_PI; }

// --- PIDController Implementation ---
PIDController::PIDController(float p, float i, float d)
    : kp(p), ki(i), kd(d), integral(0.0), prev_error(0.0) {}

void PIDController::setGains(float p, float i, float d) {
    kp = p; ki = i; kd = d;
}

float PIDController::calculate(float setpoint, float current_value, float dt) {
    if (dt <= 0) return 0.0; // Prevent division by zero or negative dt

    // Calculate error
    float error = setpoint - current_value;

    // Proportional term
    float p_out = kp * error;

    // Integral term
    integral += error * dt;
    float i_out = ki * integral;

    // Derivative term
    float derivative = (dt > 1e-6) ? (error - prev_error) / dt : 0.0;
    float d_out = kd * derivative;

    // Total output
    float output = p_out + i_out + d_out;

    // Store error for next derivative calculation
    prev_error = error;

    return output;
}

void PIDController::reset() {
    integral = 0.0; prev_error = 0.0;
}


// --- AttitudeController Implementation ---

AttitudeController::AttitudeController()
    // Initialize PID controllers and gains.
    // IMPORTANT: These gains MUST be tuned for a specific airframe and flight condition.
    // The values here are placeholders and are unlikely to work on a real aircraft without tuning.

    // Outer loop P-gains (Attitude -> Rate)
    // Unit: (rad/s) / rad. Higher value means a more aggressive correction of angle error.
    : Kp_roll_angle(6.0),
      Kp_pitch_angle(6.0),

      // Inner loop rate controllers (PI). These convert a rate error into a commanded moment.
      // Kp Unit: (normalized moment) / (rad/s). The main driver for correcting rate errors.
      // Ki Unit: (normalized moment) / rad. Corrects for steady-state errors (e.g., due to CG offset or trim issues).
      // Kd Unit: (normalized moment) / (rad/s^2). Dampens oscillations (set to 0 here).
      roll_rate_controller(/*Kp*/ 0.1, /*Ki*/ 0.05, /*Kd*/ 0.0),
      pitch_rate_controller(/*Kp*/ 0.1, /*Ki*/ 0.05, /*Kd*/ 0.0),
      yaw_rate_controller(/*Kp*/ 0.1, /*Ki*/ 0.05, /*Kd*/ 0.0),

      // Feed-forward gains (Desired Rate -> Moment)
      // Unit: (normalized moment) / (rad/s). Improves response by anticipating the required control input.
      K_ff_roll(0.1),
      K_ff_pitch(0.1),
      K_ff_yaw(0.1),

      // Initialize setpoints to a stable, level flight attitude.
      target_roll_deg(0.0),
      target_pitch_deg(0.0) {}

void AttitudeController::setSetpoints(float roll, float pitch) {
    target_roll_deg = roll;
    target_pitch_deg = pitch;
}

ActuatorOutput AttitudeController::update(const FullSensorData& sensor_data, float dt) {
    // Convert current state and setpoints to radians for calculations
    float current_roll_rad = deg_to_rad(sensor_data.roll_deg);
    float current_pitch_rad = deg_to_rad(sensor_data.pitch_deg);
    float target_roll_rad = deg_to_rad(target_roll_deg);
    float target_pitch_rad = deg_to_rad(target_pitch_deg);

    // 1. --- Turn Coordination ---
    // Calculates the required yaw rate to maintain a coordinated ("no-slip") turn.
    // This prevents the aircraft from skidding or slipping during a roll.
    // Formula: coordinated_yaw_rate = (g / V) * tan(roll)
    float coordinated_yaw_rate_rad_s = 0.0;
    if (std::abs(sensor_data.true_airspeed_ms) > 1.0) { // Avoid division by zero
        // Use the roll setpoint for a smoother, more proactive turn coordination
        coordinated_yaw_rate_rad_s = (GRAVITY_MS2 / sensor_data.true_airspeed_ms) * std::tan(target_roll_rad);
    }

    // 2. --- Outer Attitude P-Loop ---
    // Calculates desired Euler rates (phi_dot, theta_dot) to correct attitude errors.
    // This is the "P" part of the outer loop.
    float roll_error_rad = target_roll_rad - current_roll_rad;
    float pitch_error_rad = target_pitch_rad - current_pitch_rad;

    float phi_dot_sp_rad_s = Kp_roll_angle * roll_error_rad;       // Desired roll rate
    float theta_dot_sp_rad_s = Kp_pitch_angle * pitch_error_rad;   // Desired pitch rate
    float psi_dot_sp_rad_s = coordinated_yaw_rate_rad_s;             // Desired yaw rate from turn coordination

    // 3. --- Jacobian Transformation (Euler Rates to Body Rates) ---
    // Converts the desired rates of change of Euler angles (phi_dot, theta_dot, psi_dot)
    // into desired body-frame angular rates (p, q, r), which is what the gyros measure.
    // p_sp = phi_dot - psi_dot * sin(pitch)
    // q_sp = theta_dot * cos(roll) + psi_dot * cos(pitch) * sin(roll)
    // r_sp = -theta_dot * sin(roll) + psi_dot * cos(pitch) * cos(roll)
    float p_sp_rad_s = phi_dot_sp_rad_s - psi_dot_sp_rad_s * std::sin(current_pitch_rad);
    float q_sp_rad_s = theta_dot_sp_rad_s * std::cos(current_roll_rad) + psi_dot_sp_rad_s * std::cos(current_pitch_rad) * std::sin(current_roll_rad);
    float r_sp_rad_s = -theta_dot_sp_rad_s * std::sin(current_roll_rad) + psi_dot_sp_rad_s * std::cos(current_pitch_rad) * std::cos(current_roll_rad);

    // 4. --- Feed-Forward (FF) Path ---
    // The feed-forward term tries to command the required moments directly from the desired body rates.
    // This makes the controller more responsive than a pure feedback controller.
    float ff_roll_moment = K_ff_roll * p_sp_rad_s;
    float ff_pitch_moment = K_ff_pitch * q_sp_rad_s;
    float ff_yaw_moment = K_ff_yaw * r_sp_rad_s;

    // 5. --- Inner Rate PI-Loop (Feedback Path) ---
    // The feedback loop corrects for any error between the desired body rates and the actual, measured body rates.
    // This is the "PI" part of the inner loop.
    float p_rad_s = deg_to_rad(sensor_data.roll_rate_dps);
    float q_rad_s = deg_to_rad(sensor_data.pitch_rate_dps);
    float r_rad_s = deg_to_rad(sensor_data.yaw_rate_dps);

    float fb_roll_moment = roll_rate_controller.calculate(p_sp_rad_s, p_rad_s, dt);
    float fb_pitch_moment = pitch_rate_controller.calculate(q_sp_rad_s, q_rad_s, dt);
    float fb_yaw_moment = yaw_rate_controller.calculate(r_sp_rad_s, r_rad_s, dt);

    // 6. --- Airspeed Scaling (Gain Scheduling) ---
    // Control surface effectiveness is proportional to the square of the airspeed. To maintain consistent
    // handling characteristics across different speeds, we scale the commanded moments.
    // The controller is tuned at a specific speed (`nominal_airspeed_for_tuning`), and the output
    // is scaled inversely with the square of the airspeed ratio.
    float airspeed_scaler = 1.0;
    // The speed (m/s) at which the P, I, and FF gains were tuned. This is critical for the scaling to work correctly.
    const float nominal_airspeed_for_tuning = 20.0;
    if (std::abs(sensor_data.true_airspeed_ms) > 1.0) {
        // Scale gains inversely with the square of airspeed ratio.
        // If current speed is higher than nominal, scaler < 1 (reduce output).
        // If current speed is lower than nominal, scaler > 1 (increase output).
        airspeed_scaler = (nominal_airspeed_for_tuning * nominal_airspeed_for_tuning) /
                          (sensor_data.true_airspeed_ms * sensor_data.true_airspeed_ms);
    }

    // 7. --- Summation ---
    // The total commanded moment is the sum of the feedback (PI) and feed-forward (FF) contributions,
    // all scaled by the airspeed correction factor.
    float total_roll_moment = (fb_roll_moment + ff_roll_moment) * airspeed_scaler;
    float total_pitch_moment = (fb_pitch_moment + ff_pitch_moment) * airspeed_scaler;
    float total_yaw_moment = (fb_yaw_moment + ff_yaw_moment) * airspeed_scaler;

    // 8. --- Control Mixer ---
    // Converts the desired roll, pitch, and yaw moments into actuator deflections.
    // This is a highly simplified 1-to-1 mixer. A real aircraft would have a more complex
    // mixing matrix here to account for cross-coupling effects (e.g., ailerons causing adverse yaw).
    ActuatorOutput controls;
    controls.aileron = total_roll_moment;
    controls.elevator = total_pitch_moment;
    controls.rudder = total_yaw_moment;

    // Clamp final outputs to the valid range [-1.0, 1.0] to prevent over-driving the actuators.
    controls.aileron = std::clamp(controls.aileron, -1.0f, 1.0f);
    controls.elevator = std::clamp(controls.elevator, -1.0f, 1.0f);
    controls.rudder = std::clamp(controls.rudder, -1.0f, 1.0f);

    return controls;
}
