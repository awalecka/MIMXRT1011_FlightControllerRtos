/**
 * @file servo_driver.h
 * @brief Servo driver abstraction for NXP PWM peripherals.
 * Provides high-resolution PWM control and pass-through functionality.
 */

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include <cstdint>
#include <span>
#include "fsl_pwm.h"

namespace firmware::drivers {

/**
 * @brief Configuration structure for a single servo channel.
 */
struct ServoChannelConfig {
    PWM_Type* base;              ///< PWM Peripheral Base (e.g., PWM1)
    pwm_submodule_t subModule;   ///< PWM Submodule (kPWM_Module_0, etc.)
    pwm_channels_t channel;      ///< PWM Channel (kPWM_PwmA or kPWM_PwmB)
};

/**
 * @class ServoDriver
 * @brief Manages PWM outputs for RC Servos with support for normalized and raw input.
 */
class ServoDriver {
public:
    /**
     * @brief Constants for standard RC Servo timing (50Hz / 20ms period).
     */
    static constexpr uint32_t PWM_FREQUENCY_HZ = 50U;
    static constexpr uint32_t PERIOD_US = 1000000U / PWM_FREQUENCY_HZ;
    static constexpr uint16_t MIN_PULSE_US = 500U;
    static constexpr uint16_t MAX_PULSE_US = 2500U;
    static constexpr uint16_t CENTER_PULSE_US = 1500U;

    /**
     * @brief Initializes the driver with the provided channel configuration.
     * @param channelConfig A span of servo configurations. The index in this span
     * becomes the servo ID for set methods.
     */
    void init(std::span<const ServoChannelConfig> channelConfig);

    /**
     * @brief Sets the servo output using a normalized range.
     * @param servoIndex Index corresponding to the initialization config array.
     * @param value Normalized value between -1.0 and 1.0.
     * -1.0 maps to MIN_PULSE_US.
     * 0.0 maps to CENTER_PULSE_US.
     * 1.0 maps to MAX_PULSE_US.
     */
    void setNormalizedOutput(size_t servoIndex, float value);

    /**
     * @brief Sets the servo output using raw microseconds (Direct Pass Through).
     * @param servoIndex Index corresponding to the initialization config array.
     * @param pulseWidthUs Pulse width in microseconds (typically 1000-2000).
     */
    void setPulseWidthUs(size_t servoIndex, uint16_t pulseWidthUs);

    /**
     * @brief Disables output for a specific servo (sets duty to 0).
     * @param servoIndex Index corresponding to the initialization config array.
     */
    void disableServo(size_t servoIndex);

    /**
	 * @brief Commits all pending register updates by setting the Load OK bit.
	 * Must be called after setting outputs to ensure they take effect.
	 */
	void commitUpdates();

private:
    /**
     * @brief Reference to the configuration array passed in init.
     * We store this span rather than copying to save memory (zero-copy).
     */
    std::span<const ServoChannelConfig> m_config;

    /**
     * @brief Updates the hardware register with the calculated tick count.
     * @param config Config struct for the specific channel.
     * @param pulseWidthUs Target width in microseconds.
     */
    void writeHardwareRegister(const ServoChannelConfig& config, uint16_t pulseWidthUs);
};

} // namespace firmware::drivers

#endif // SERVO_DRIVER_H
