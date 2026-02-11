#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "drivers/servo_driver.h"
#include <cstdint>

class Actuators {
public:
    // --- Configuration Constants ---
    // Extended range for control surfaces (Max throw)
    static constexpr uint16_t MIN_PULSE_US = 500U;
    static constexpr uint16_t MAX_PULSE_US = 2500U;
    static constexpr uint16_t CENTER_PULSE_US = 1500U;

    // Standard range for ESCs (Throttle)
    static constexpr uint16_t THROTTLE_MIN_PULSE_US = 1000U;
    static constexpr uint16_t THROTTLE_MAX_PULSE_US = 2000U;

    void init();

    /**
     * @brief Sets the normalized output for control surfaces and throttle.
     * @param aileron -1.0 to 1.0
     * @param elevator -1.0 to 1.0
     * @param rudder -1.0 to 1.0
     * @param throttle 0.0 to 100.0
     */
    void setOutputs(float aileron, float elevator, float rudder, float throttle);

    /**
     * @brief Sets the raw output in microseconds (Direct Pass-Through).
     */
    void setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs);

private:
    firmware::drivers::ServoDriver m_servoDriver;
};

#endif // ACTUATORS_H
