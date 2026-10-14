/**
 * @file ins_navigator.cpp
 * @brief Implementation of top-level automotive INS navigator.
 */

#include "ins_navigator.hpp"
#include <cmath>
#include <algorithm>

namespace gnc::ins {

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float DEG2RAD_F = PI_F / 180.0f;
constexpr float RAD2DEG_F = 180.0f / PI_F;

float wrapPi(float a) {
    while (a > PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

Eigen::Quaternionf eulerToQuat(float rollRad, float pitchRad, float yawRad) {
    return Eigen::AngleAxisf(yawRad, Eigen::Vector3f::UnitZ())
         * Eigen::AngleAxisf(pitchRad, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(rollRad, Eigen::Vector3f::UnitX());
}
} // namespace

InsNavigator::InsNavigator(const InsConfig& cfg)
    : m_cfg(cfg)
    , m_filter(cfg)
    , m_detector(cfg)
{
    reset();
}

void InsNavigator::reset()
{
    m_status = NavStatus::WaitStatic;
    m_alignTimer = 0.0f;
    m_alignTimeoutTimer = 0.0f;
    m_accelAccum.setZero();
    m_gyroAccum.setZero();
    m_alignSamples = 0;
    m_magRefNorm = 0.0f;
    m_lastMagUpdateTimer = 0.0f;
    m_nhcTimer = 0.0f;
    m_zuptTimer = 0.0f;
    m_gpsAgeTimer = 0.0f;
    m_lastGpsPosEpoch = 0;
    m_lastGpsVelEpoch = 0;
    m_consecutiveGpsRejects = 0;
    m_latestGpsSpeed = -1.0f;
    m_detector.reset();
}

void InsNavigator::onImu(float dt, const Vec3& gyroBody, const Vec3& accelBody)
{
    if (dt <= 0.0f) return;

    // Convert from AttitudeMekf "+g" convention ([0,0,+9.81] at rest)
    // to physical specific force f_b = -accelBody ([0,0,-9.81] at rest).
    const Vec3 specForce = -accelBody;

    // Update stationary detector
    m_detector.update(dt, gyroBody, specForce, m_latestGpsSpeed);

    if (m_status == NavStatus::WaitStatic) {
        stepAlignment(dt, gyroBody, accelBody);
        return;
    }

    // Prediction step
    m_filter.predict(dt, gyroBody, specForce);

    // Timers
    m_lastMagUpdateTimer += dt;
    m_gpsAgeTimer += dt;

    // Motion constraint updates
    if (m_detector.isStationary()) {
        handleStationaryUpdates(dt, gyroBody);
    } else {
        handleNhcUpdates(dt);
    }

    // GPS timeout check
    if (m_status == NavStatus::Navigating && m_gpsAgeTimer > m_cfg.gpsTimeoutS) {
        m_status = NavStatus::DeadReckoning;
    }

    checkDivergenceAndRecentering();
}

void InsNavigator::stepAlignment(float dt, const Vec3& gyroBody, const Vec3& rawAccel)
{
    m_alignTimeoutTimer += dt;

    const float aNorm = rawAccel.norm();
    const float gNorm = gyroBody.norm();
    const bool isQuiet = (std::abs(aNorm - m_cfg.gravity) < 1.5f) && (gNorm < 0.1f);

    if (isQuiet) {
        m_accelAccum += rawAccel;
        m_gyroAccum += gyroBody;
        m_alignSamples++;
        m_alignTimer += dt;

        if (m_alignTimer >= m_cfg.alignWindowS && m_alignSamples > 10) {
            const Vec3 accelMean = m_accelAccum / static_cast<float>(m_alignSamples);
            const Vec3 gyroMean = m_gyroAccum / static_cast<float>(m_alignSamples);

            // Compute Roll and Pitch from static gravity vector
            // In NED, gravity is +Z. When level, rawAccel reads [0, 0, +g].
            const float roll = std::atan2(accelMean.y(), accelMean.z());
            const float pitch = std::atan2(-accelMean.x(),
                                           std::sqrt(accelMean.y() * accelMean.y() + accelMean.z() * accelMean.z()));
            const float yaw = 0.0f;

            const Eigen::Quaternionf qInit = eulerToQuat(roll, pitch, yaw);

            ErrorMat P0 = ErrorMat::Zero();
            P0.block<2, 2>(IDX_ATT, IDX_ATT) = Eigen::Matrix2f::Identity() * (m_cfg.initLevelSigma * m_cfg.initLevelSigma);
            P0(IDX_ATT + 2, IDX_ATT + 2)     = m_cfg.initYawSigma * m_cfg.initYawSigma;
            P0.block<3, 3>(IDX_VEL, IDX_VEL) = Mat3::Identity() * (m_cfg.initVelSigma * m_cfg.initVelSigma);
            P0.block<3, 3>(IDX_POS, IDX_POS) = Mat3::Identity() * (m_cfg.initPosSigma * m_cfg.initPosSigma);
            P0.block<3, 3>(IDX_BG, IDX_BG)   = Mat3::Identity() * (m_cfg.initGyroBiasSigma * m_cfg.initGyroBiasSigma);
            P0.block<3, 3>(IDX_BA, IDX_BA)   = Mat3::Identity() * (m_cfg.initAccelBiasSigma * m_cfg.initAccelBiasSigma);

            m_filter.initialize(qInit, Vec3::Zero(), Vec3::Zero(), gyroMean, Vec3::Zero(), P0);

            m_status = NavStatus::Levelled;
        }
    } else {
        // Vehicle moved during alignment; reset accumulator
        m_accelAccum.setZero();
        m_gyroAccum.setZero();
        m_alignSamples = 0;
        m_alignTimer = 0.0f;
    }
}

void InsNavigator::handleStationaryUpdates(float dt, const Vec3& gyroBody)
{
    if (!m_cfg.enableZupt) return;

    m_zuptTimer += dt;
    const float zuptInterval = 1.0f / m_cfg.zuptRateHz;
    if (m_zuptTimer >= zuptInterval) {
        m_zuptTimer = 0.0f;
        m_filter.updateZeroVelocity(m_cfg.zuptSigma);
        m_filter.updateZeroRate(gyroBody, m_cfg.zaruSigma);
    }
}

void InsNavigator::handleNhcUpdates(float dt)
{
    if (!m_cfg.enableNhc) return;

    // Apply NHC only when moving above minimum threshold
    if (m_filter.getVelocity().norm() < m_cfg.nhcMinSpeed) return;

    m_nhcTimer += dt;
    const float nhcInterval = 1.0f / m_cfg.nhcRateHz;
    if (m_nhcTimer >= nhcInterval) {
        m_nhcTimer = 0.0f;
        m_filter.updateNonHolonomic(m_cfg.mountRotation, m_cfg.nhcSigmaLateral, m_cfg.nhcSigmaVertical);
    }
}

void InsNavigator::onMag(const Vec3& magBody)
{
    if (!m_cfg.enableMag) return;

    const float magNorm = magBody.norm();
    if (magNorm < 1e-3f) return;

    if (m_status == NavStatus::WaitStatic) {
        // Learn reference field magnitude during static period
        m_magRefNorm = magNorm;
        return;
    }

    // Field magnitude sanity check
    if (m_magRefNorm > 0.0f) {
        if (std::abs(magNorm - m_magRefNorm) / m_magRefNorm > m_cfg.magNormTolerance) {
            return; // Distorted magnetic field
        }
    } else {
        m_magRefNorm = magNorm;
    }

    // Compute tilt-compensated magnetic heading
    const Mat3 R = m_filter.getQuaternion().toRotationMatrix();
    // Rotate body magnetic measurement into current level horizontal plane
    const Vec3 euler = m_filter.getEulerAnglesDeg();
    const float roll = euler.x() * DEG2RAD_F;
    const float pitch = euler.y() * DEG2RAD_F;

    const float sR = std::sin(roll), cR = std::cos(roll);
    const float sP = std::sin(pitch), cP = std::cos(pitch);

    // Level-frame magnetic components
    const float mx = magBody.x() * cP + magBody.y() * (sP * sR) + magBody.z() * (sP * cR);
    const float my = magBody.y() * cR - magBody.z() * sR;

    float yawMag = std::atan2(-my, mx); // Magnetic North heading
    yawMag = wrapPi(yawMag + m_cfg.magDeclinationRad);

    if (m_status == NavStatus::Levelled) {
        // Set yaw and transition to Aligned
        const Eigen::Quaternionf qAligned = eulerToQuat(roll, pitch, yawMag);
        m_filter.initialize(qAligned, m_filter.getVelocity(), m_filter.getPosition(),
                            m_filter.getGyroBias(), m_filter.getAccelBias(), m_filter.getCovariance());
        m_status = NavStatus::Aligned;
    } else if (m_status == NavStatus::Navigating || m_status == NavStatus::DeadReckoning) {
        const float magInterval = 1.0f / m_cfg.magUpdateRateHz;
        if (m_lastMagUpdateTimer >= magInterval) {
            m_lastMagUpdateTimer = 0.0f;
            m_filter.updateHeading(yawMag, m_cfg.magHeadingSigmaRad);
        }
    }
}

void InsNavigator::onGps(const firmware::sensors::GpsData& gps)
{
    const bool fixValid = gps.valid && (gps.fixType >= 1) && (gps.satellites >= m_cfg.gpsMinSatellites);
    if (!fixValid) {
        return;
    }

    m_latestGpsSpeed = gps.speed;

    // Check course-over-ground heading alignment when Levelled
    if (m_status == NavStatus::Levelled && gps.hasVelocity && gps.speed >= m_cfg.courseInitMinSpeed) {
        const Vec3 euler = m_filter.getEulerAnglesDeg();
        const float roll = euler.x() * DEG2RAD_F;
        const float pitch = euler.y() * DEG2RAD_F;
        const float courseRad = gps.course * DEG2RAD_F;

        // Account for IMU mount yaw: course is vehicle heading, body yaw = course - mountYaw
        const Vec3 mountEuler = m_cfg.mountRotation.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
        const float mountYaw = mountEuler[0];
        const float bodyYaw = wrapPi(courseRad - mountYaw);

        const Eigen::Quaternionf qAligned = eulerToQuat(roll, pitch, bodyYaw);
        m_filter.initialize(qAligned, m_filter.getVelocity(), m_filter.getPosition(),
                            m_filter.getGyroBias(), m_filter.getAccelBias(), m_filter.getCovariance());
        m_status = NavStatus::Aligned;
    }

    // Transition from Aligned to Navigating on first GPS fix
    if (m_status == NavStatus::Aligned) {
        m_ltp.setOrigin(gps.latitude, gps.longitude, gps.altitude);

        // Initial velocity from GPS speed & course
        const float courseRad = gps.course * DEG2RAD_F;
        const Vec3 v0(gps.speed * std::cos(courseRad),
                      gps.speed * std::sin(courseRad),
                      0.0f);

        m_filter.initialize(m_filter.getQuaternion(), v0, Vec3::Zero(),
                            m_filter.getGyroBias(), m_filter.getAccelBias(), m_filter.getCovariance());

        m_status = NavStatus::Navigating;
        m_lastGpsPosEpoch = gps.posEpochId;
        m_lastGpsVelEpoch = gps.velEpochId;
        m_gpsAgeTimer = 0.0f;
        m_consecutiveGpsRejects = 0;
        return;
    }

    if (m_status == NavStatus::Navigating || m_status == NavStatus::DeadReckoning) {
        // Position update
        if (gps.posEpochId != m_lastGpsPosEpoch) {
            m_lastGpsPosEpoch = gps.posEpochId;

            const Vec3 posNed = m_ltp.llaToNed(gps.latitude, gps.longitude, gps.altitude);

            Vec3 sigmaNed;
            if (gps.hasGst && gps.sigmaLat > 0.0f && gps.sigmaLon > 0.0f) {
                sigmaNed.x() = std::max(gps.sigmaLat, m_cfg.gpsMinSigmaH);
                sigmaNed.y() = std::max(gps.sigmaLon, m_cfg.gpsMinSigmaH);
                sigmaNed.z() = std::max(gps.sigmaAlt, m_cfg.gpsMinSigmaV);
            } else {
                const float hSigma = std::max(gps.hdop * m_cfg.gpsUereH, m_cfg.gpsMinSigmaH);
                const float vSigma = std::max(gps.hdop * m_cfg.gpsUereV, m_cfg.gpsMinSigmaV);
                sigmaNed = Vec3(hSigma, hSigma, vSigma);
            }

            const UpdateResult ur = m_filter.updatePosition(posNed, sigmaNed);
            if (ur.accepted) {
                m_gpsAgeTimer = 0.0f;
                m_consecutiveGpsRejects = 0;
                if (m_status == NavStatus::DeadReckoning) {
                    m_status = NavStatus::Navigating;
                }
            } else {
                m_consecutiveGpsRejects++;
            }
        }

        // Velocity update
        if (gps.velEpochId != m_lastGpsVelEpoch && gps.hasVelocity) {
            m_lastGpsVelEpoch = gps.velEpochId;

            const float courseRad = gps.course * DEG2RAD_F;
            const Vec2 velNE(gps.speed * std::cos(courseRad),
                             gps.speed * std::sin(courseRad));

            m_filter.updateVelocityNE(velNE, m_cfg.gpsSpeedSigma);

            // Optional course heading aiding
            if (m_cfg.enableCourseHeadingAiding && gps.speed >= m_cfg.courseInitMinSpeed) {
                const Vec3 mountEuler = m_cfg.mountRotation.toRotationMatrix().eulerAngles(2, 1, 0);
                const float bodyYaw = wrapPi(courseRad - mountEuler[0]);
                m_filter.updateHeading(bodyYaw, m_cfg.courseHeadingSigmaRad);
            }
        }
    }
}

void InsNavigator::checkDivergenceAndRecentering()
{
    // Divergence check
    if (m_consecutiveGpsRejects >= m_cfg.maxConsecutiveGpsRejects) {
        m_status = NavStatus::Levelled;
        m_consecutiveGpsRejects = 0;
        return;
    }

    if (m_status == NavStatus::DeadReckoning) {
        const ErrorMat& P = m_filter.getCovariance();
        const float posSigmaH = std::sqrt(P(IDX_POS, IDX_POS) + P(IDX_POS + 1, IDX_POS + 1));
        if (posSigmaH > m_cfg.maxHorizontalSigmaDr) {
            m_status = NavStatus::Levelled;
            return;
        }
    }

    // LTP re-centring
    if (m_ltp.hasOrigin()) {
        const Vec3 pos = m_filter.getPosition();
        if (pos.head<2>().norm() > m_cfg.ltpRecentreDistance) {
            const Mat3 rNewFromOld = m_ltp.recentre(pos);
            m_filter.shiftOrigin(pos, rNewFromOld);
        }
    }
}

NavSolution InsNavigator::getSolution() const
{
    NavSolution sol;
    sol.status = m_status;
    sol.posNed = m_filter.getPosition();
    sol.velNed = m_filter.getVelocity();
    sol.speed = sol.velNed.head<2>().norm();
    sol.q = m_filter.getQuaternion();
    sol.eulerDeg = m_filter.getEulerAnglesDeg();
    sol.gyroBias = m_filter.getGyroBias();
    sol.accelBias = m_filter.getAccelBias();
    sol.stationary = m_detector.isStationary();
    sol.gpsAgeS = m_gpsAgeTimer;
    sol.covFaultCount = m_filter.getCovarianceFaultCount();

    if (m_ltp.hasOrigin()) {
        const Lla lla = m_ltp.nedToLla(sol.posNed);
        sol.latitudeDeg = lla.latDeg;
        sol.longitudeDeg = lla.lonDeg;
        sol.altitudeM = static_cast<float>(lla.altM);
    }

    const ErrorMat& P = m_filter.getCovariance();
    sol.posSigmaH = std::sqrt(std::max(0.0f, P(IDX_POS, IDX_POS) + P(IDX_POS + 1, IDX_POS + 1)));
    sol.posSigmaD = std::sqrt(std::max(0.0f, P(IDX_POS + 2, IDX_POS + 2)));
    sol.velSigmaH = std::sqrt(std::max(0.0f, P(IDX_VEL, IDX_VEL) + P(IDX_VEL + 1, IDX_VEL + 1)));
    sol.yawSigmaDeg = std::sqrt(std::max(0.0f, P(IDX_ATT + 2, IDX_ATT + 2))) * RAD2DEG_F;

    return sol;
}

} // namespace gnc::ins
