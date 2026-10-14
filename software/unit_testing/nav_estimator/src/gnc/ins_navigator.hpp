/**
 * @file ins_navigator.hpp
 * @brief Top-level automotive navigation manager and policy coordinator.
 *
 * Coordinates:
 *   - InsMekf (core 15-state filter)
 *   - LocalTangentPlane (WGS-84 <-> local NED)
 *   - StationaryDetector (ZUPT/ZARU trigger)
 *   - NMEA GpsData ingestion and epoch tracking
 *   - Multi-phase alignment state machine
 */

#ifndef INS_NAVIGATOR_HPP
#define INS_NAVIGATOR_HPP

#include "ins_types.hpp"
#include "ins_mekf.hpp"
#include "local_tangent_plane.hpp"
#include "stationary_detector.hpp"
#include "../../../nmea/src/sensors/nmea.h"

namespace gnc::ins {

class InsNavigator {
public:
    explicit InsNavigator(const InsConfig& cfg);

    /**
     * @brief Ingest IMU sample.
     * @param dt Time step [s]
     * @param gyroBody Gyroscope angular rate [rad/s]
     * @param accelBody Accelerometer measurement in the AttitudeMekf "+g" convention
     *                  (reads approx [0, 0, +9.81] when level and static).
     */
    void onImu(float dt, const Vec3& gyroBody, const Vec3& accelBody);

    /**
     * @brief Ingest calibrated magnetometer sample [uT or Gauss].
     */
    void onMag(const Vec3& magBody);

    /**
     * @brief Ingest parsed GPS data (call whenever NmeaParser processes a sentence).
     */
    void onGps(const firmware::sensors::GpsData& gps);

    [[nodiscard]] NavSolution getSolution() const;
    [[nodiscard]] NavStatus getStatus() const { return m_status; }
    [[nodiscard]] bool isStationary() const { return m_detector.isStationary(); }
    [[nodiscard]] const InsMekf& getFilter() const { return m_filter; }
    [[nodiscard]] const LocalTangentPlane& getLtp() const { return m_ltp; }

    void reset();

private:
    InsConfig m_cfg;
    InsMekf m_filter;
    LocalTangentPlane m_ltp;
    StationaryDetector m_detector;

    NavStatus m_status = NavStatus::WaitStatic;

    // Alignment accumulation
    float m_alignTimer = 0.0f;
    float m_alignTimeoutTimer = 0.0f;
    Vec3 m_accelAccum = Vec3::Zero();
    Vec3 m_gyroAccum = Vec3::Zero();
    size_t m_alignSamples = 0;

    // Magnetometer reference
    float m_magRefNorm = 0.0f;
    float m_lastMagUpdateTimer = 0.0f;

    // Aiding timers / decimation
    float m_nhcTimer = 0.0f;
    float m_zuptTimer = 0.0f;
    float m_gpsAgeTimer = 0.0f;

    // GPS epoch tracking
    uint32_t m_lastGpsPosEpoch = 0;
    uint32_t m_lastGpsVelEpoch = 0;
    uint8_t m_consecutiveGpsRejects = 0;
    float m_latestGpsSpeed = -1.0f;

    void stepAlignment(float dt, const Vec3& gyroBody, const Vec3& specForce);
    void handleStationaryUpdates(float dt, const Vec3& gyroBody);
    void handleNhcUpdates(float dt);
    void checkDivergenceAndRecentering();
};

} // namespace gnc::ins

#endif // INS_NAVIGATOR_HPP
