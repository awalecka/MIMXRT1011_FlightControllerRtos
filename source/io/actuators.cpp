#include "io/actuators.h"
#include "utils/utils.h"
#include "board.h"
#include "peripherals.h" // For PWM defines

using namespace firmware::drivers;

// Static configuration for servo channels.
static const ServoChannelConfig s_servoConfig[] = {
    { PWM1, kPWM_Module_0, kPWM_PwmA }, // Aileron
    { PWM1, kPWM_Module_0, kPWM_PwmB }, // Elevator
    { PWM1, kPWM_Module_2, kPWM_PwmA }, // Throttle
    { PWM1, kPWM_Module_2, kPWM_PwmB }, // Rudder
};

void Actuators::init() {
    m_servoDriver.init(s_servoConfig);
}

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {
    m_servoDriver.setNormalizedOutput(0, aileron);  // Aileron
    m_servoDriver.setNormalizedOutput(1, elevator); // Elevator
    m_servoDriver.setNormalizedOutput(3, rudder);   // Rudder

    // Map throttle 0.0-100.0 -> -1.0-1.0
    float throttleNormalized = mapFloat(throttle, 0.0f, 100.0f, -1.0f, 1.0f);
    m_servoDriver.setNormalizedOutput(2, throttleNormalized);
}

void Actuators::setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs) {
    m_servoDriver.setPulseWidthUs(0, aileronUs);
    m_servoDriver.setPulseWidthUs(1, elevatorUs);
    m_servoDriver.setPulseWidthUs(2, throttleUs);
    m_servoDriver.setPulseWidthUs(3, rudderUs);
}
