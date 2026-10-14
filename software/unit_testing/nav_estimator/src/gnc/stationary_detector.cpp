/**
 * @file stationary_detector.cpp
 * @brief Implementation of the automotive stationary detector.
 */

#include "stationary_detector.hpp"
#include <cmath>
#include <algorithm>

namespace gnc::ins {

StationaryDetector::StationaryDetector(const InsConfig& cfg)
    : m_cfg(cfg)
{
    reset();
}

void StationaryDetector::reset()
{
    m_isStationary = false;
    m_enterTimer = 0.0f;
    m_exitTimer = 0.0f;
    m_bufHead = 0;
    m_bufCount = 0;
    m_currentAccelStd = 0.0f;
    m_currentGyroMax = 0.0f;
    m_accelMagBuf.fill(0.0f);
    m_gyroNormBuf.fill(0.0f);
}

bool StationaryDetector::update(float dt, const Vec3& gyroBody, const Vec3& accelBody, float gpsSpeed)
{
    if (dt <= 0.0f) return m_isStationary;

    const float aMag = accelBody.norm();
    const float gNorm = gyroBody.norm();

    // Store in circular buffer
    m_accelMagBuf[m_bufHead] = aMag;
    m_gyroNormBuf[m_bufHead] = gNorm;
    m_bufHead = (m_bufHead + 1) % MAX_WINDOW;
    if (m_bufCount < MAX_WINDOW) {
        m_bufCount++;
    }

    // Determine sample window size corresponding to m_cfg.stationaryWindowS
    size_t targetSamples = static_cast<size_t>(std::ceil(m_cfg.stationaryWindowS / dt));
    if (targetSamples < 5) targetSamples = 5;
    if (targetSamples > MAX_WINDOW) targetSamples = MAX_WINDOW;

    if (m_bufCount < targetSamples) {
        // Not enough samples yet to reliably determine stationarity
        return m_isStationary;
    }

    // Compute sample mean and variance of accel magnitude and max gyro norm over targetSamples
    float sumA = 0.0f;
    float sumA2 = 0.0f;
    float maxG = 0.0f;

    for (size_t i = 0; i < targetSamples; ++i) {
        // walk backwards from m_bufHead
        const size_t idx = (m_bufHead + MAX_WINDOW - 1 - i) % MAX_WINDOW;
        const float a = m_accelMagBuf[idx];
        const float g = m_gyroNormBuf[idx];

        sumA += a;
        sumA2 += a * a;
        if (g > maxG) {
            maxG = g;
        }
    }

    const float meanA = sumA / static_cast<float>(targetSamples);
    const float varA = std::max(0.0f, (sumA2 / static_cast<float>(targetSamples)) - meanA * meanA);
    const float stdA = std::sqrt(varA);

    m_currentAccelStd = stdA;
    m_currentGyroMax = maxG;

    // Check stationary criteria
    bool condAccel = (stdA <= m_cfg.stationaryAccelStd);
    bool condGyro = (maxG <= m_cfg.stationaryGyroMax);
    bool condGps = true;
    if (gpsSpeed >= 0.0f) {
        condGps = (gpsSpeed <= m_cfg.stationaryGpsSpeed);
    }

    const bool instantStationary = condAccel && condGyro && condGps;

    // Hysteresis debouncing
    if (instantStationary) {
        m_exitTimer = 0.0f;
        m_enterTimer += dt;
        if (m_enterTimer >= m_cfg.stationaryEnterS) {
            m_isStationary = true;
        }
    } else {
        m_enterTimer = 0.0f;
        m_exitTimer += dt;
        if (m_exitTimer >= m_cfg.stationaryExitS) {
            m_isStationary = false;
        }
    }

    return m_isStationary;
}

} // namespace gnc::ins
