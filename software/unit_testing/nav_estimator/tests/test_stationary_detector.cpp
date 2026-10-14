/**
 * @file test_stationary_detector.cpp
 * @brief Unit tests for automotive stationary detector.
 */

#include <gtest/gtest.h>
#include "stationary_detector.hpp"

using namespace gnc::ins;

class StationaryDetectorTest : public ::testing::Test {
protected:
    InsConfig m_cfg;

    void SetUp() override {
        m_cfg.stationaryWindowS = 0.5f;
        m_cfg.stationaryAccelStd = 0.35f;
        m_cfg.stationaryGyroMax = 0.05f;
        m_cfg.stationaryEnterS = 0.3f;
        m_cfg.stationaryExitS = 0.1f;
        m_cfg.stationaryGpsSpeed = 0.3f;
    }
};

TEST_F(StationaryDetectorTest, DeclaresStationaryAtRest) {
    StationaryDetector det(m_cfg);

    const float dt = 0.01f;
    const Vec3 gyro(0.001f, -0.002f, 0.001f);
    const Vec3 accel(0.0f, 0.0f, -9.81f);

    // Feed 1 second of static data
    bool stationary = false;
    for (int i = 0; i < 100; ++i) {
        stationary = det.update(dt, gyro, accel, 0.0f);
    }

    EXPECT_TRUE(stationary);
    EXPECT_TRUE(det.isStationary());
    EXPECT_LT(det.getAccelStd(), 0.05f);
}

TEST_F(StationaryDetectorTest, EngineVibrationTolerated) {
    StationaryDetector det(m_cfg);

    const float dt = 0.005f; // 200 Hz
    // 30 Hz engine vibration with 0.2 m/s^2 amplitude
    bool stationary = false;
    for (int i = 0; i < 200; ++i) {
        const float vib = 0.2f * std::sin(2.0f * 3.14159f * 30.0f * (i * dt));
        const Vec3 accel(vib, 0.0f, -9.81f);
        const Vec3 gyro(0.005f, 0.002f, -0.001f);
        stationary = det.update(dt, gyro, accel, 0.0f);
    }

    EXPECT_TRUE(stationary);
}

TEST_F(StationaryDetectorTest, MotionExitsStationary) {
    StationaryDetector det(m_cfg);

    const float dt = 0.01f;
    // Get into stationary state first
    for (int i = 0; i < 100; ++i) {
        det.update(dt, Vec3::Zero(), Vec3(0.0f, 0.0f, -9.81f), 0.0f);
    }
    ASSERT_TRUE(det.isStationary());

    // Introduce sharp turn / acceleration
    bool stationary = true;
    for (int i = 0; i < 30; ++i) { // 0.3 seconds (> 0.1s exit debounce)
        stationary = det.update(dt, Vec3(0.0f, 0.0f, 0.3f), Vec3(2.0f, 0.0f, -9.81f), 5.0f);
    }

    EXPECT_FALSE(stationary);
    EXPECT_FALSE(det.isStationary());
}

TEST_F(StationaryDetectorTest, GpsSpeedGating) {
    StationaryDetector det(m_cfg);

    const float dt = 0.01f;
    // Quiet IMU, but vehicle is rolling (GPS speed = 2.0 m/s > 0.3 m/s)
    bool stationary = false;
    for (int i = 0; i < 100; ++i) {
        stationary = det.update(dt, Vec3::Zero(), Vec3(0.0f, 0.0f, -9.81f), 2.0f);
    }

    EXPECT_FALSE(stationary);
}
