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
     * -1.0 maps to minLimitUs.
     * 0.0 maps to center of min/max.
     * 1.0 maps to maxLimitUs.
     * @param minLimitUs The lower bound of the output pulse in microseconds.
     * @param maxLimitUs The upper bound of the output pulse in microseconds.
     */
    void setNormalizedOutput(size_t servoIndex, float value,
                             uint16_t minLimitUs,
                             uint16_t maxLimitUs);

    /**
     * @brief Sets the servo output using raw microseconds (Standard RC Input).
     * Maps the standard 1000-2000us input range to the specified output range.
     * * @param servoIndex Index corresponding to the initialization config array.
     * @param pulseWidthUs Standard RC pulse width input (typically 1000-2000).
     * @param minLimitUs The lower bound of the output pulse in microseconds.
     * @param maxLimitUs The upper bound of the output pulse in microseconds.
     */
    void setPulseWidthUs(size_t servoIndex, uint16_t pulseWidthUs,
                         uint16_t minLimitUs,
                         uint16_t maxLimitUs);

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
