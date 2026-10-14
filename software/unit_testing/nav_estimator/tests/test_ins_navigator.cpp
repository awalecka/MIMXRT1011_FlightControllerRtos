/**
 * @file test_ins_navigator.cpp
 * @brief End-to-end integration and scenario tests for InsNavigator with VehicleSim and NmeaParser.
 */

#include <gtest/gtest.h>
#include "ins_navigator.hpp"
#include "sim/vehicle_sim.hpp"
#include "../../../nmea/src/sensors/nmea.h"
#include <cmath>
#include <vector>

using namespace gnc::ins;
using namespace gnc::ins::sim;
using namespace firmware::sensors;

namespace {
constexpr float DEG2RAD_F = 3.14159265358979323846f / 180.0f;
constexpr float RAD2DEG_F = 180.0f / 3.14159265358979323846f;

float wrap180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}
} // namespace

class InsNavigatorScenarioTest : public ::testing::Test {
protected:
    InsConfig m_cfg;
    SimConfig m_simCfg;

    void SetUp() override {
        m_cfg.alignWindowS = 1.0f;
        m_cfg.alignTimeoutS = 10.0f;
        m_cfg.courseInitMinSpeed = 4.0f;
        m_cfg.courseInitMaxYawRate = 0.1f;
        m_cfg.enableNhc = true;
        m_cfg.enableZupt = true;
        m_cfg.gpsTimeoutS = 2.0f;

        m_simCfg.imuRateHz = 208.0f;
        m_simCfg.gpsRateHz = 5.0f;
    }

    void feedNmeaToNavigator(InsNavigator& nav, NmeaParser& parser, const std::vector<std::string>& sentences) {
        for (const auto& sentence : sentences) {
            for (char c : sentence) {
                if (parser.processByte(static_cast<uint8_t>(c))) {
                    nav.onGps(parser.getLatestData());
                }
            }
        }
    }
};

TEST_F(InsNavigatorScenarioTest, StaticStartNoMag_ReachesLevelled) {
    InsNavigator nav(m_cfg);
    NmeaParser parser;

    VehicleSim sim(m_simCfg);
    sim.setTrajectory({ { 2.0f, 0.0f, 0.0f } }); // 2 seconds stationary

    TruthState truth;
    Vec3 w_m, a_m;
    std::vector<std::string> nmea;

    while (sim.step(truth, w_m, a_m, nmea)) {
        nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
        feedNmeaToNavigator(nav, parser, nmea);
    }

    EXPECT_EQ(nav.getStatus(), NavStatus::Levelled);
    const Vec3 euler = nav.getSolution().eulerDeg;
    EXPECT_NEAR(euler.x(), 0.0f, 0.5f);
    EXPECT_NEAR(euler.y(), 0.0f, 0.5f);
}

TEST_F(InsNavigatorScenarioTest, StaticStartWithMag_ReachesAligned) {
    m_cfg.enableMag = true;
    InsNavigator nav(m_cfg);
    NmeaParser parser;

    VehicleSim sim(m_simCfg);
    sim.setTrajectory({ { 2.5f, 0.0f, 0.0f } }); // 2.5 seconds stationary

    TruthState truth;
    Vec3 w_m, a_m;
    std::vector<std::string> nmea;

    // Simulate body magnetometer pointing towards Magnetic North
    // (In NED: mx ~ 25 uT, my ~ 0 uT, mz ~ 40 uT)
    const Vec3 magReading(25.0f, 0.0f, 40.0f);

    while (sim.step(truth, w_m, a_m, nmea)) {
        nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
        nav.onMag(magReading);
    }

    EXPECT_EQ(nav.getStatus(), NavStatus::Aligned);
    EXPECT_NEAR(nav.getSolution().eulerDeg.z(), 0.0f, 3.0f);
}

TEST_F(InsNavigatorScenarioTest, DriveOffStraight_CourseInitializesHeading) {
    InsNavigator nav(m_cfg);
    NmeaParser parser;

    VehicleSim sim(m_simCfg);
    // Trajectory: 1.5s stop (level), then accelerate at 2 m/s^2 forward (heading = 0 deg North)
    sim.setTrajectory({
        { 1.5f, 0.0f, 0.0f },
        { 4.0f, 2.0f, 0.0f }  // Reaches 8 m/s (> 4 m/s course init)
    });

    TruthState truth;
    Vec3 w_m, a_m;
    std::vector<std::string> nmea;

    while (sim.step(truth, w_m, a_m, nmea)) {
        nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
        feedNmeaToNavigator(nav, parser, nmea);
    }

    EXPECT_EQ(nav.getStatus(), NavStatus::Navigating);
    EXPECT_NEAR(nav.getSolution().eulerDeg.z(), 0.0f, 2.0f);
    EXPECT_NEAR(nav.getSolution().speed, truth.speed, 0.3f);
}

TEST_F(InsNavigatorScenarioTest, CityLoopScenario_TruthTracking) {
    // Run twice: once with HDOP only (standard NMEA), once with GST
    for (bool withGst : { false, true }) {
        m_simCfg.emitGst = withGst;
        InsNavigator nav(m_cfg);
        NmeaParser parser;

        VehicleSim sim(m_simCfg);
        // City loop:
        // 1. Static 1.5s (levelling)
        // 2. Accel to 10 m/s (5s, a=2.0)
        // 3. Cruise straight 5s
        // 4. 90-deg right turn (yawRate = 0.314 rad/s for 5s ~ 90 deg)
        // 5. Cruise 5s
        // 6. Decel to stop (5s, a=-2.0)
        // 7. Stationary at stop 2s
        sim.setTrajectory({
            { 1.5f,  0.0f, 0.0f },
            { 5.0f,  2.0f, 0.0f },
            { 5.0f,  0.0f, 0.0f },
            { 5.0f,  0.0f, 0.314159f },
            { 5.0f,  0.0f, 0.0f },
            { 5.0f, -2.0f, 0.0f },
            { 2.0f,  0.0f, 0.0f }
        });

        TruthState truth;
        Vec3 w_m, a_m;
        std::vector<std::string> nmea;

        float maxPosErr = 0.0f;
        float sumSqPosErr = 0.0f;
        size_t samples = 0;

        LocalTangentPlane simLtp;
        simLtp.setOrigin(m_simCfg.originLat, m_simCfg.originLon, m_simCfg.originAlt);

        while (sim.step(truth, w_m, a_m, nmea)) {
            nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
            feedNmeaToNavigator(nav, parser, nmea);

            if (nav.getStatus() == NavStatus::Navigating) {
                const NavSolution sol = nav.getSolution();
                const Vec3 navNedInSim = simLtp.llaToNed(sol.latitudeDeg, sol.longitudeDeg, sol.altitudeM);
                const Vec3 posErr = navNedInSim - truth.p;
                const float hErr = posErr.head<2>().norm();
                if (hErr > maxPosErr) maxPosErr = hErr;
                sumSqPosErr += hErr * hErr;
                samples++;
            }
        }

        ASSERT_GT(samples, 100);
        const float posRms = std::sqrt(sumSqPosErr / static_cast<float>(samples));

        // In 5 Hz GPS with 1m noise, RMS position tracking should be within 2.0 m
        EXPECT_LT(posRms, 2.0f) << "Failed with withGst=" << withGst;
        EXPECT_LT(maxPosErr, 4.5f) << "Failed with withGst=" << withGst;

        // Check vehicle stop detected
        EXPECT_TRUE(nav.isStationary());
        EXPECT_NEAR(nav.getSolution().speed, 0.0f, 0.05f);
        EXPECT_EQ(nav.getFilter().getCovarianceFaultCount(), 0u);
    }
}

TEST_F(InsNavigatorScenarioTest, GpsOutage_NhcReducesDrift) {
    auto runOutage = [this](bool enableNhc) -> float {
        m_cfg.enableNhc = enableNhc;
        InsNavigator nav(m_cfg);
        NmeaParser parser;

        VehicleSim sim(m_simCfg);
        // 1.5s stop, 5s accel to 10 m/s, 5s cruise with GPS, 10s curve in GPS OUTAGE, 5s post-outage
        sim.setTrajectory({
            { 1.5f, 0.0f, 0.0f },
            { 5.0f, 2.0f, 0.0f },
            { 5.0f, 0.0f, 0.0f },
            { 10.0f, 0.0f, 0.05f }, // Gentle curve in outage
            { 5.0f, 0.0f, 0.0f }
        });
        sim.setGpsOutage(11.5f, 10.0f); // 10s outage

        TruthState truth;
        Vec3 w_m, a_m;
        std::vector<std::string> nmea;

        LocalTangentPlane simLtp;
        simLtp.setOrigin(m_simCfg.originLat, m_simCfg.originLon, m_simCfg.originAlt);

        float maxDriftDuringOutage = 0.0f;

        while (sim.step(truth, w_m, a_m, nmea)) {
            nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
            feedNmeaToNavigator(nav, parser, nmea);

            if (truth.time >= 11.5f && truth.time <= 21.5f && nav.getStatus() == NavStatus::DeadReckoning) {
                const NavSolution sol = nav.getSolution();
                const Vec3 navNedInSim = simLtp.llaToNed(sol.latitudeDeg, sol.longitudeDeg, sol.altitudeM);
                const float hErr = (navNedInSim - truth.p).head<2>().norm();
                if (hErr > maxDriftDuringOutage) {
                    maxDriftDuringOutage = hErr;
                }
            }
        }
        return maxDriftDuringOutage;
    };

    const float driftWithNhc = runOutage(true);
    const float driftWithoutNhc = runOutage(false);

    // NHC should keep drift significantly lower by constraining lateral and vertical velocities
    EXPECT_LT(driftWithNhc, 12.0f);
    EXPECT_LE(driftWithNhc, driftWithoutNhc);
}

TEST_F(InsNavigatorScenarioTest, MountingMisalignmentCompensation) {
    // IMU mounted with 3 deg yaw and 2 deg pitch relative to vehicle chassis
    const Eigen::Quaternionf mountRot = Eigen::AngleAxisf(3.0f * DEG2RAD_F, Eigen::Vector3f::UnitZ())
                                      * Eigen::AngleAxisf(2.0f * DEG2RAD_F, Eigen::Vector3f::UnitY());
    m_simCfg.mountRotation = mountRot;
    m_cfg.mountRotation = mountRot; // Properly configured navigator

    InsNavigator nav(m_cfg);
    NmeaParser parser;

    VehicleSim sim(m_simCfg);
    sim.setTrajectory({
        { 1.5f, 0.0f, 0.0f },
        { 5.0f, 2.0f, 0.0f },
        { 5.0f, 0.0f, 0.0f }
    });

    TruthState truth;
    Vec3 w_m, a_m;
    std::vector<std::string> nmea;

    LocalTangentPlane simLtp;
    simLtp.setOrigin(m_simCfg.originLat, m_simCfg.originLon, m_simCfg.originAlt);

    while (sim.step(truth, w_m, a_m, nmea)) {
        nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
        feedNmeaToNavigator(nav, parser, nmea);
    }

    EXPECT_EQ(nav.getStatus(), NavStatus::Navigating);
    const NavSolution sol = nav.getSolution();
    const Vec3 navNedInSim = simLtp.llaToNed(sol.latitudeDeg, sol.longitudeDeg, sol.altitudeM);
    const float posErr = (navNedInSim - truth.p).head<2>().norm();
    EXPECT_LT(posErr, 2.0f);
}

TEST_F(InsNavigatorScenarioTest, GpsOutlierRejection) {
    InsNavigator nav(m_cfg);
    NmeaParser parser;

    VehicleSim sim(m_simCfg);
    sim.setTrajectory({
        { 1.5f, 0.0f, 0.0f },
        { 5.0f, 2.0f, 0.0f },
        { 5.0f, 0.0f, 0.0f }
    });

    TruthState truth;
    Vec3 w_m, a_m;
    std::vector<std::string> nmea;

    while (sim.step(truth, w_m, a_m, nmea)) {
        nav.onImu(1.0f / m_simCfg.imuRateHz, w_m, a_m);
        feedNmeaToNavigator(nav, parser, nmea);
    }

    ASSERT_EQ(nav.getStatus(), NavStatus::Navigating);
    const Vec3 posBefore = nav.getSolution().posNed;

    // Inject 100m outlier packet
    std::string outlierGga = VehicleSim::formatNmeaSentence(
        "GNGGA,120530.00,3747.000,N,12224.000,W,1,12,0.9,15.0,M,0.0,M,,"
    );
    for (char c : outlierGga) {
        if (parser.processByte(static_cast<uint8_t>(c))) {
            nav.onGps(parser.getLatestData());
        }
    }

    const Vec3 posAfter = nav.getSolution().posNed;
    // Outlier must be rejected by chi-square gate without corrupting the state
    EXPECT_NEAR((posAfter - posBefore).norm(), 0.0f, 0.5f);
}

TEST_F(InsNavigatorScenarioTest, MagDisturbanceRejection) {
    m_cfg.enableMag = true;
    m_cfg.magNormTolerance = 0.20f; // 20%
    InsNavigator nav(m_cfg);

    // Initial static window to calibrate mag reference
    for (int i = 0; i < 250; ++i) {
        nav.onImu(0.01f, Vec3::Zero(), Vec3(0.0f, 0.0f, 9.81f));
        nav.onMag(Vec3(25.0f, 0.0f, 40.0f)); // norm ~ 47.17 uT
    }

    EXPECT_EQ(nav.getStatus(), NavStatus::Aligned);
    const float yawBefore = nav.getSolution().eulerDeg.z();

    // Inject massive magnetic distortion (+100% field jump)
    const Vec3 distortedMag(80.0f, 50.0f, 40.0f); // norm ~ 102 uT
    for (int i = 0; i < 50; ++i) {
        nav.onMag(distortedMag);
    }

    // Yaw should remain unperturbed because norm gate rejected the distorted readings
    EXPECT_NEAR(nav.getSolution().eulerDeg.z(), yawBefore, 1.0f);
}
