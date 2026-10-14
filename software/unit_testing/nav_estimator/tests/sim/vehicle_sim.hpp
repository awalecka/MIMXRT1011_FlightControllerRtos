/**
 * @file vehicle_sim.hpp
 * @brief Ground truth generator and synthetic sensor synthesizer for vehicle navigation tests.
 *
 * Produces ground truth (q, v, p), synthetic IMU measurements with noise and bias,
 * and realistic NMEA GPS sentences (GGA, RMC, VTG, GST).
 */

#ifndef VEHICLE_SIM_HPP
#define VEHICLE_SIM_HPP

#include "../../src/gnc/ins_types.hpp"
#include "../../src/gnc/local_tangent_plane.hpp"
#include <vector>
#include <string>
#include <random>

namespace gnc::ins::sim {

struct Segment {
    float durationS = 0.0f;
    float longAccelMps2 = 0.0f;   ///< Forward acceleration in vehicle frame
    float yawRateRps = 0.0f;      ///< Yaw rate in vehicle frame (positive turning right)
};

struct SimConfig {
    double originLat = 37.7749;   ///< San Francisco
    double originLon = -122.4194;
    double originAlt = 15.0;

    float imuRateHz = 208.0f;
    float gpsRateHz = 5.0f;

    // Sensor errors
    float gyroNoiseStd = 0.001f;     ///< [rad/s]
    float accelNoiseStd = 0.01f;     ///< [m/s^2]
    Vec3 gyroBias = Vec3(0.002f, -0.003f, 0.001f); ///< [rad/s]
    Vec3 accelBias = Vec3(0.05f, -0.04f, 0.02f);   ///< [m/s^2]

    float gpsPosNoiseH = 1.0f;       ///< 1-sigma horizontal [m]
    float gpsPosNoiseV = 2.0f;       ///< 1-sigma vertical [m]
    float gpsSpeedNoise = 0.05f;     ///< [m/s]
    float gpsCourseNoiseDeg = 1.0f;  ///< [deg]

    bool emitGst = false;
    bool emitVtg = true;

    Eigen::Quaternionf mountRotation = Eigen::Quaternionf::Identity(); ///< body -> vehicle
};

struct TruthState {
    float time = 0.0f;
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); ///< body -> NED
    Vec3 v = Vec3::Zero();                                ///< NED [m/s]
    Vec3 p = Vec3::Zero();                                ///< NED [m]
    Vec3 w_b = Vec3::Zero();                              ///< body angular rate [rad/s]
    Vec3 a_b = Vec3::Zero();                              ///< body raw accel (AttitudeMekf convention: +g on Z at rest)
    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    float speed = 0.0f;
    float courseDeg = 0.0f;
};

class VehicleSim {
public:
    explicit VehicleSim(const SimConfig& cfg);

    void setTrajectory(const std::vector<Segment>& segments);

    /**
     * @brief Run one IMU step.
     * @param truth [out] Ground truth at this step
     * @param gyroMeas [out] Noisy IMU gyro measurement [rad/s]
     * @param accelMeas [out] Noisy IMU accel measurement in AttitudeMekf convention [m/s^2]
     * @param outNmea [out] Non-empty if GPS emitted one or more NMEA sentences at this step
     * @return true if simulation is ongoing, false if trajectory complete
     */
    bool step(TruthState& truth, Vec3& gyroMeas, Vec3& accelMeas, std::vector<std::string>& outNmea);

    static std::string formatNmeaSentence(const std::string& body);

    void setGpsOutage(float startS, float durationS);

private:
    SimConfig m_cfg;
    LocalTangentPlane m_ltp;
    std::mt19937 m_rng{42};

    std::vector<Segment> m_segments;
    size_t m_currentSeg = 0;
    float m_segElapsed = 0.0f;
    float m_simTime = 0.0f;
    float m_gpsTimer = 0.0f;

    // Outage
    float m_outageStart = -1.0f;
    float m_outageDuration = 0.0f;

    // Truth state
    Eigen::Quaternionf m_q_v = Eigen::Quaternionf::Identity(); ///< vehicle -> NED
    Vec3 m_v_ned = Vec3::Zero();
    Vec3 m_p_ned = Vec3::Zero();
    float m_speed = 0.0f;
    float m_yaw = 0.0f;

    std::string generateGga(const TruthState& truth, float timeOfDayS);
    std::string generateRmc(const TruthState& truth, float timeOfDayS);
    std::string generateVtg(const TruthState& truth);
    std::string generateGst(const TruthState& truth, float timeOfDayS);
};

} // namespace gnc::ins::sim

#endif // VEHICLE_SIM_HPP
