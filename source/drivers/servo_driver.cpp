/**
 * @file servo_driver.cpp
 * @brief Implementation of the ServoDriver class.
 */

#include "servo_driver.h"
#include "utils.h" // For mapFloat, mapUshort
#include <algorithm> // For std::clamp

namespace firmware::drivers {

void ServoDriver::init(std::span<const ServoChannelConfig> channelConfig) {
    m_config = channelConfig;

    // Iterate through all configured channels and ensure the Load OK bit is set
    // to allow register updates. We assume the low-level PWM setup (Pins/Clocks)
    // is handled by the Board/Peripherals tool generated code.
    for (const auto& cfg : m_config) {
        // Set LDOK (Load OK) for the specific submodule to enable register updates
        PWM_SetPwmLdok(cfg.base, (1U << cfg.subModule), true);
    }
}

void ServoDriver::setNormalizedOutput(size_t servoIndex, float value) {
    if (servoIndex >= m_config.size()) {
        return;
    }

    // Clamp input for safety
    float clampedValue = std::clamp(value, -1.0f, 1.0f);

    // Map -1.0...1.0 to standard RC range 1000...2000
    // setPulseWidthUs will then map this to the physical range MIN_PULSE_US...MAX_PULSE_US
    float usFloat = mapFloat(clampedValue, -1.0f, 1.0f, 1000.0f, 2000.0f);

    setPulseWidthUs(servoIndex, static_cast<uint16_t>(usFloat));
}

void ServoDriver::setPulseWidthUs(size_t servoIndex, uint16_t pulseWidthUs) {
    if (servoIndex >= m_config.size()) {
        return;
    }

    // Map standard RC input (1000-2000) to the driver's configured physical range (500-2500)
    uint16_t mappedWidth = mapUshort(pulseWidthUs, 1000, 2000, MIN_PULSE_US, MAX_PULSE_US);

    const auto& cfg = m_config[servoIndex];
    writeHardwareRegister(cfg, mappedWidth);
}

void ServoDriver::disableServo(size_t servoIndex) {
    if (servoIndex >= m_config.size()) {
        return;
    }

    // Writing 0 duty cycle effectively disables the pulse
    writeHardwareRegister(m_config[servoIndex], 0);
}

void ServoDriver::writeHardwareRegister(const ServoChannelConfig& config, uint16_t pulseWidthUs) {
    // Get the current Period (Modulo) in ticks from the hardware.
    // VAL1 register holds the maximum count value (Period) for the submodule.
    uint16_t periodTicks = config.base->SM[config.subModule].VAL1;

    // Calculate the target tick count for the desired pulse width.
    // Logic: (TargetUS / TotalPeriodUS) * TotalPeriodTicks
    // We use uint32_t for the intermediate calculation to prevent overflow.
    uint32_t targetTicks = (static_cast<uint32_t>(pulseWidthUs) * periodTicks) / PERIOD_US;

    // Write to the Value Register.
    // For Edge-Aligned PWM (standard for Servos):
    // - PWM A uses VAL3 as the turn-off point.
    // - PWM B uses VAL5 as the turn-off point.
    // - VAL0/VAL2/VAL4 are usually 0 (start of cycle).
    if (config.channel == kPWM_PwmA) {
        config.base->SM[config.subModule].VAL3 = static_cast<uint16_t>(targetTicks);
    } else {
        config.base->SM[config.subModule].VAL5 = static_cast<uint16_t>(targetTicks);
    }

    // Set the Load OK bit to latch the new values into the active registers
    // at the next PWM reload opportunity.
    PWM_SetPwmLdok(config.base, (1U << config.subModule), true);
}

} // namespace firmware::drivers
