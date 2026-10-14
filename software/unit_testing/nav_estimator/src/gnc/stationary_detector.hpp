/**
 * @file stationary_detector.hpp
 * @brief Zero-allocation stationary detector for automotive INS.
 *
 * Employs a sliding-window statistical analysis over accelerometer specific force
 * magnitude variance and gyroscope angular rates, with hysteresis debounce.
 */

#ifndef STATIONARY_DETECTOR_HPP
#define STATIONARY_DETECTOR_HPP

#include "ins_types.hpp"
#include <array>
#include <cstdint>

namespace gnc::ins {

class StationaryDetector {
public:
    explicit StationaryDetector(const InsConfig& cfg);

    /**
     * @brief Feed a new IMU sample to the detector.
     * @param dt Time since last IMU sample [s]
     * @param gyroBody Angular rate vector [rad/s]
     * @param accelBody Specific force vector [m/s^2]
     * @param gpsSpeed Current GPS ground speed [m/s] (negative if GPS invalid/unavailable)
     * @return true if vehicle is determined to be stationary
     */
    bool update(float dt, const Vec3& gyroBody, const Vec3& accelBody, float gpsSpeed = -1.0f);

    [[nodiscard]] bool isStationary() const { return m_isStationary; }
    [[nodiscard]] float getAccelStd() const { return m_currentAccelStd; }
    [[nodiscard]] float getGyroMaxNorm() const { return m_currentGyroMax; }

    void reset();

private:
    InsConfig m_cfg;
    bool m_isStationary = false;

    float m_enterTimer = 0.0f;
    float m_exitTimer = 0.0f;

    static constexpr size_t MAX_WINDOW = 256;
    std::array<float, MAX_WINDOW> m_accelMagBuf{};
    std::array<float, MAX_WINDOW> m_gyroNormBuf{};
    size_t m_bufHead = 0;
    size_t m_bufCount = 0;

    float m_currentAccelStd = 0.0f;
    float m_currentGyroMax = 0.0f;
};

} // namespace gnc::ins

#endif // STATIONARY_DETECTOR_HPP
