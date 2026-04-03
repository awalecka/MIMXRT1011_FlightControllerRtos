#include "io/actuators.h"
#include "utils/utils.h"
#include "board.h"
#include "peripherals.h"

using namespace firmware::drivers;

// Static configuration for servo channels.
static const ServoChannelConfig s_servoConfig[] = {
    { PWM1, kPWM_Module_0, kPWM_PwmA }, // Aileron - Left
    { PWM1, kPWM_Module_0, kPWM_PwmB }, // Aileron - Right
    { PWM1, kPWM_Module_2, kPWM_PwmA }, // Elevator
    { PWM1, kPWM_Module_2, kPWM_PwmB }, // Rudder
    { PWM1, kPWM_Module_3, kPWM_PwmA }, // Throttle
    { PWM1, kPWM_Module_1, kPWM_PwmA }, // Gear - Left
    { PWM1, kPWM_Module_1, kPWM_PwmB }, // Gear - Right
};

void Actuators::init() {
    m_servoDriver.init(s_servoConfig);

    // Set safe defaults immediately upon startup
    // Surfaces to Center, Throttle to Minimum, Gear to Center
    setRawOutputs(CENTER_PULSE_US, CENTER_PULSE_US, CENTER_PULSE_US, THROTTLE_MIN_PULSE_US, CENTER_PULSE_US);
}

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle, float gear) {
    // Control Surfaces: Use extended range for max throw
    m_servoDriver.setNormalizedOutput(0, -aileron, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setNormalizedOutput(1, -aileron, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setNormalizedOutput(2, elevator, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setNormalizedOutput(3, -rudder, MIN_PULSE_US, MAX_PULSE_US);

    // Throttle: Map 0-100% directly to target microseconds
    uint16_t throttleUs = mapFloat(throttle, 0.0f, 100.0f, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);
    m_servoDriver.setPulseWidthUs(4, throttleUs, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);

    // Gear Output
    m_servoDriver.setNormalizedOutput(5, gear, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setNormalizedOutput(6, gear, MIN_PULSE_US, MAX_PULSE_US);

    m_servoDriver.commitUpdates();
}

void Actuators::setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs, uint16_t gearUs) {

    // Invert the standard 1000-2000us RC signal by subtracting from 3000
    uint16_t invertedAileronUs = 3000U - aileronUs;
    uint16_t invertedRudderUs = 3000U - rudderUs;

    // Pass identical microseconds to both ailerons to achieve opposite aerodynamic throw
    m_servoDriver.setPulseWidthUs(0, invertedAileronUs, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setPulseWidthUs(1, invertedAileronUs, MIN_PULSE_US, MAX_PULSE_US);

    m_servoDriver.setPulseWidthUs(2, elevatorUs, MIN_PULSE_US, MAX_PULSE_US);

    // Apply the inverted rudder here
    m_servoDriver.setPulseWidthUs(3, invertedRudderUs, MIN_PULSE_US, MAX_PULSE_US);

    m_servoDriver.setPulseWidthUs(4, throttleUs, THROTTLE_MIN_PULSE_US, THROTTLE_MAX_PULSE_US);
    m_servoDriver.setPulseWidthUs(5, gearUs, MIN_PULSE_US, MAX_PULSE_US);
    m_servoDriver.setPulseWidthUs(6, gearUs, MIN_PULSE_US, MAX_PULSE_US);

    m_servoDriver.commitUpdates();
}
