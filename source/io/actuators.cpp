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
	// Surfaces to Center (1500us), Throttle to Minimum (1000us)
	setRawOutputs(1500, 1500, 1500, 1000);
}

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {
	m_servoDriver.setNormalizedOutput(0, aileron);   // Left Aileron (Normal)
	m_servoDriver.setNormalizedOutput(1, -aileron);  // Right Aileron (Inverted)
    m_servoDriver.setNormalizedOutput(2, elevator);
    m_servoDriver.setNormalizedOutput(3, rudder);

    float throttleNormalized = mapFloat(throttle, 0.0f, 100.0f, -1.0f, 1.0f);
    m_servoDriver.setNormalizedOutput(4, throttleNormalized); // Throttle

    // Latch all values at once
    m_servoDriver.commitUpdates();
}

void Actuators::setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs) {
    // Channel 0: Left Aileron (Direct)
    m_servoDriver.setPulseWidthUs(0, aileronUs);

    // Channel 1: Right Aileron (Inverted around 1500us center)
    // 3000 - 1000 = 2000
    // 3000 - 2000 = 1000
    m_servoDriver.setPulseWidthUs(1, 3000 - aileronUs);

    // Channel 2: Elevator
    m_servoDriver.setPulseWidthUs(2, elevatorUs);

    // Channel 3: Rudder
    m_servoDriver.setPulseWidthUs(3, rudderUs);

    // Channel 4: Throttle
    m_servoDriver.setPulseWidthUs(4, throttleUs);

    // Latch all values at once
    m_servoDriver.commitUpdates();
}
