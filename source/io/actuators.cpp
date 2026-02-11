#include "io/actuators.h"
#include "utils/utils.h"
#include "board.h"
#include "peripherals.h" // For PWM defines

using namespace firmware::drivers;

// Static configuration for servo channels.
static const ServoChannelConfig s_servoConfig[] = {
    { PWM1, kPWM_Module_0, kPWM_PwmA }, // Aileron - Left
	{ PWM1, kPWM_Module_0, kPWM_PwmB }, // Aileron - Right
	{ PWM1, kPWM_Module_2, kPWM_PwmA }, // Elevator
    { PWM1, kPWM_Module_2, kPWM_PwmB }, // Rudder
    { PWM1, kPWM_Module_3, kPWM_PwmA }, // Throttle
};

void Actuators::init() {
    m_servoDriver.init(s_servoConfig);

    // Set safe defaults immediately upon startup
	// Surfaces to Center, Throttle to Minimum
	setRawOutputs(CENTER_PULSE_US, CENTER_PULSE_US, CENTER_PULSE_US, THROTTLE_MIN_PULSE_US);
}

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {
    // Control Surfaces: Use extended range for max throw
    m_servoDriver.setNormalizedOutput(0, aileron, MIN_PULSE_US, MAX_PULSE_US);   // Left Aileron
    m_servoDriver.setNormalizedOutput(1, -aileron, MIN_PULSE_US, MAX_PULSE_US);  // Right Aileron
    m_servoDriver.setNormalizedOutput(2, elevator, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setNormalizedOutput(3, rudder, MIN_PULSE_US, MAX_PULSE_US);

    // Throttle: Map 0-100% directly to target microseconds
    uint16_t throttleUs = mapFloat(throttle, 0.0f, 100.0f, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);

    // Pass the calculated microseconds directly.
    // We pass the same Min/Max limits to ensure the driver doesn't re-scale it further.
    m_servoDriver.setPulseWidthUs(4, throttleUs, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);

    // Latch all values at once
    m_servoDriver.commitUpdates();
}

void Actuators::setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs) {
    // Channel 0: Left Aileron (Direct)
    m_servoDriver.setPulseWidthUs(0, aileronUs, MIN_PULSE_US, MAX_PULSE_US);

    // Channel 1: Right Aileron (Inverted around CENTER)
    // Formula: (CENTER + (CENTER - Input)) = 2*CENTER - Input
    uint16_t invertedAileron = (2 * CENTER_PULSE_US) - aileronUs;
    m_servoDriver.setPulseWidthUs(1, invertedAileron, MIN_PULSE_US, MAX_PULSE_US);

    // Channel 2: Elevator
    m_servoDriver.setPulseWidthUs(2, elevatorUs, MIN_PULSE_US, MAX_PULSE_US);

    // Channel 3: Rudder
    m_servoDriver.setPulseWidthUs(3, rudderUs, MIN_PULSE_US, MAX_PULSE_US);

    // Channel 4: Throttle (Direct 1:1 mapping typically, but clamped to limits)
    m_servoDriver.setPulseWidthUs(4, throttleUs, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);

    // Latch all values at once
    m_servoDriver.commitUpdates();
}
