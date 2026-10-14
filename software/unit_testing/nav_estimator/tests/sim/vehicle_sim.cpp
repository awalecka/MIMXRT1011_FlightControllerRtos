/**
 * @file vehicle_sim.cpp
 * @brief Implementation of vehicle trajectory simulation and NMEA generator.
 */

#include "vehicle_sim.hpp"
#include <iomanip>
#include <sstream>
#include <cmath>

namespace gnc::ins::sim {

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG2RAD_F = PI_F / 180.0f;
constexpr float RAD2DEG_F = 180.0f / PI_F;

std::string formatLatNmea(double latDeg) {
    char hemi = (latDeg >= 0.0) ? 'N' : 'S';
    double absLat = std::abs(latDeg);
    int deg = static_cast<int>(absLat);
    double min = (absLat - deg) * 60.0;
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(2) << deg
       << std::fixed << std::setprecision(5) << std::setw(8) << min
       << "," << hemi;
    return ss.str();
}

std::string formatLonNmea(double lonDeg) {
    char hemi = (lonDeg >= 0.0) ? 'E' : 'W';
    double absLon = std::abs(lonDeg);
    int deg = static_cast<int>(absLon);
    double min = (absLon - deg) * 60.0;
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(3) << deg
       << std::fixed << std::setprecision(5) << std::setw(8) << min
       << "," << hemi;
    return ss.str();
}

std::string formatUtc(float timeOfDayS) {
    int totalS = static_cast<int>(timeOfDayS);
    int hh = (totalS / 3600) % 24;
    int mm = (totalS % 3600) / 60;
    int ss = totalS % 60;
    int cs = static_cast<int>((timeOfDayS - totalS) * 100.0f);
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << hh
        << std::setfill('0') << std::setw(2) << mm
        << std::setfill('0') << std::setw(2) << ss << "."
        << std::setfill('0') << std::setw(2) << cs;
    return oss.str();
}
} // namespace

VehicleSim::VehicleSim(const SimConfig& cfg)
    : m_cfg(cfg)
{
    m_ltp.setOrigin(m_cfg.originLat, m_cfg.originLon, m_cfg.originAlt);
}

void VehicleSim::setTrajectory(const std::vector<Segment>& segments)
{
    m_segments = segments;
    m_currentSeg = 0;
    m_segElapsed = 0.0f;
    m_simTime = 0.0f;
    m_gpsTimer = 0.0f;
    m_speed = 0.0f;
    m_yaw = 0.0f;
    m_q_v = Eigen::Quaternionf::Identity();
    m_v_ned.setZero();
    m_p_ned.setZero();
}

void VehicleSim::setGpsOutage(float startS, float durationS)
{
    m_outageStart = startS;
    m_outageDuration = durationS;
}

std::string VehicleSim::formatNmeaSentence(const std::string& body)
{
    uint8_t cs = 0;
    for (char c : body) {
        cs ^= static_cast<uint8_t>(c);
    }
    std::ostringstream oss;
    oss << "$" << body << "*" << std::hex << std::uppercase
        << std::setfill('0') << std::setw(2) << static_cast<int>(cs) << "\r\n";
    return oss.str();
}

bool VehicleSim::step(TruthState& truth, Vec3& gyroMeas, Vec3& accelMeas, std::vector<std::string>& outNmea)
{
    outNmea.clear();
    if (m_currentSeg >= m_segments.size()) {
        return false;
    }

    const float dt = 1.0f / m_cfg.imuRateHz;
    const Segment& seg = m_segments[m_currentSeg];

    // Update vehicle kinematics
    m_yaw += seg.yawRateRps * dt;
    while (m_yaw > PI_F) m_yaw -= 2.0f * PI_F;
    while (m_yaw < -PI_F) m_yaw += 2.0f * PI_F;

    m_q_v = Eigen::AngleAxisf(m_yaw, Eigen::Vector3f::UnitZ());

    m_speed = std::max(0.0f, m_speed + seg.longAccelMps2 * dt);

    // Vehicle frame velocity: [speed, 0, 0]^T (non-holonomic vehicle)
    const Vec3 v_vehicle(m_speed, 0.0f, 0.0f);
    m_v_ned = m_q_v * v_vehicle;
    m_p_ned += m_v_ned * dt;

    // Kinematic acceleration in vehicle frame
    // a_vehicle = [longAccel, yawRate * speed, 0]^T (centripetal accel)
    const Vec3 a_vehicle(seg.longAccelMps2, seg.yawRateRps * m_speed, 0.0f);

    // Gravity in vehicle frame: R_v_n * [0, 0, +g] = [0, 0, +g] for flat road
    const Vec3 g_vehicle(0.0f, 0.0f, 9.81f);

    // Specific force in vehicle frame: f_vehicle = a_vehicle - g_vehicle
    const Vec3 f_vehicle = a_vehicle - g_vehicle;

    // Vehicle angular rate
    const Vec3 w_vehicle(0.0f, 0.0f, seg.yawRateRps);

    // Transform to body (IMU) frame:
    // mountRotation maps body -> vehicle.
    // v_vehicle = mountRotation * v_body => v_body = mountRotation^T * v_vehicle
    const Mat3 R_b_v = m_cfg.mountRotation.toRotationMatrix().transpose();
    const Vec3 w_body = R_b_v * w_vehicle;
    const Vec3 f_body = R_b_v * f_vehicle;

    // Body to NED quaternion
    const Eigen::Quaternionf q_body = (m_q_v * m_cfg.mountRotation).normalized();

    // Raw accel in AttitudeMekf convention reads -f_body (so at rest it reads [0, 0, +9.81])
    const Vec3 rawAccelIdeal = -f_body;

    // Fill truth
    truth.time = m_simTime;
    truth.q = q_body;
    truth.v = m_v_ned;
    truth.p = m_p_ned;
    truth.w_b = w_body;
    truth.a_b = rawAccelIdeal;
    const Lla lla = m_ltp.nedToLla(m_p_ned);
    truth.lat = lla.latDeg;
    truth.lon = lla.lonDeg;
    truth.alt = static_cast<float>(lla.altM);
    truth.speed = m_speed;
    truth.courseDeg = m_yaw * RAD2DEG_F;
    if (truth.courseDeg < 0.0f) truth.courseDeg += 360.0f;

    // Synthesize noisy IMU measurements
    std::normal_distribution<float> nGyro(0.0f, m_cfg.gyroNoiseStd);
    std::normal_distribution<float> nAccel(0.0f, m_cfg.accelNoiseStd);

    gyroMeas = w_body + m_cfg.gyroBias + Vec3(nGyro(m_rng), nGyro(m_rng), nGyro(m_rng));
    accelMeas = rawAccelIdeal + m_cfg.accelBias + Vec3(nAccel(m_rng), nAccel(m_rng), nAccel(m_rng));

    // GPS updates
    m_gpsTimer += dt;
    const float gpsInterval = 1.0f / m_cfg.gpsRateHz;
    if (m_gpsTimer >= gpsInterval) {
        m_gpsTimer -= gpsInterval;

        const bool inOutage = (m_outageStart >= 0.0f) &&
                              (m_simTime >= m_outageStart) &&
                              (m_simTime < (m_outageStart + m_outageDuration));

        if (!inOutage) {
            const float timeOfDay = 43200.0f + m_simTime; // 12:00:00 start
            outNmea.push_back(generateGga(truth, timeOfDay));
            outNmea.push_back(generateRmc(truth, timeOfDay));
            if (m_cfg.emitVtg) {
                outNmea.push_back(generateVtg(truth));
            }
            if (m_cfg.emitGst) {
                outNmea.push_back(generateGst(truth, timeOfDay));
            }
        }
    }

    // Step time and segment
    m_simTime += dt;
    m_segElapsed += dt;
    if (m_segElapsed >= seg.durationS) {
        m_segElapsed = 0.0f;
        m_currentSeg++;
    }

    return true;
}

std::string VehicleSim::generateGga(const TruthState& truth, float timeOfDayS)
{
    std::normal_distribution<float> nPosH(0.0f, m_cfg.gpsPosNoiseH);
    std::normal_distribution<float> nPosV(0.0f, m_cfg.gpsPosNoiseV);

    const Vec3 noisyNed = truth.p + Vec3(nPosH(m_rng), nPosH(m_rng), nPosV(m_rng));
    const Lla noisyLla = m_ltp.nedToLla(noisyNed);

    std::ostringstream ss;
    ss << "GNGGA," << formatUtc(timeOfDayS) << ","
       << formatLatNmea(noisyLla.latDeg) << ","
       << formatLonNmea(noisyLla.lonDeg) << ","
       << "1,12,0.9," // Fix=1, 12 sats, HDOP=0.9
       << std::fixed << std::setprecision(1) << noisyLla.altM << ",M,0.0,M,,";
    return formatNmeaSentence(ss.str());
}

std::string VehicleSim::generateRmc(const TruthState& truth, float timeOfDayS)
{
    std::normal_distribution<float> nPosH(0.0f, m_cfg.gpsPosNoiseH);
    std::normal_distribution<float> nSpd(0.0f, m_cfg.gpsSpeedNoise);
    std::normal_distribution<float> nCrs(0.0f, m_cfg.gpsCourseNoiseDeg);

    const Vec3 noisyNed = truth.p + Vec3(nPosH(m_rng), nPosH(m_rng), 0.0f);
    const Lla noisyLla = m_ltp.nedToLla(noisyNed);

    const float knots = std::max(0.0f, (truth.speed + nSpd(m_rng)) * 1.94384f);
    float course = truth.courseDeg + nCrs(m_rng);
    while (course >= 360.0f) course -= 360.0f;
    while (course < 0.0f) course += 360.0f;

    std::ostringstream ss;
    ss << "GNRMC," << formatUtc(timeOfDayS) << ",A,"
       << formatLatNmea(noisyLla.latDeg) << ","
       << formatLonNmea(noisyLla.lonDeg) << ","
       << std::fixed << std::setprecision(2) << knots << ","
       << std::fixed << std::setprecision(1) << course << ","
       << "141026,,,A";
    return formatNmeaSentence(ss.str());
}

std::string VehicleSim::generateVtg(const TruthState& truth)
{
    std::normal_distribution<float> nSpd(0.0f, m_cfg.gpsSpeedNoise);
    std::normal_distribution<float> nCrs(0.0f, m_cfg.gpsCourseNoiseDeg);

    const float speedKmh = std::max(0.0f, (truth.speed + nSpd(m_rng)) * 3.6f);
    const float knots = speedKmh * 0.539957f;
    float course = truth.courseDeg + nCrs(m_rng);
    while (course >= 360.0f) course -= 360.0f;
    while (course < 0.0f) course += 360.0f;

    std::ostringstream ss;
    ss << "GNVTG," << std::fixed << std::setprecision(1) << course << ",T,,M,"
       << std::fixed << std::setprecision(2) << knots << ",N,"
       << std::fixed << std::setprecision(2) << speedKmh << ",K,A";
    return formatNmeaSentence(ss.str());
}

std::string VehicleSim::generateGst(const TruthState&, float timeOfDayS)
{
    std::ostringstream ss;
    ss << "GNGST," << formatUtc(timeOfDayS) << ",1.0,,,,1.2,1.1,2.5";
    return formatNmeaSentence(ss.str());
}

} // namespace gnc::ins::sim
