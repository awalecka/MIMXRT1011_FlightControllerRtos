/**
 * @file test_ins_mekf.cpp
 * @brief Unit tests and numerical verification for InsMekf (15-state error filter).
 */

#include <gtest/gtest.h>
#include "ins_mekf.hpp"

using namespace gnc::ins;

namespace {
constexpr float DEG2RAD_F = 3.14159265358979323846f / 180.0f;

Eigen::Quaternionf eulerToQuat(float rollDeg, float pitchDeg, float yawDeg) {
    return Eigen::AngleAxisf(yawDeg * DEG2RAD_F, Eigen::Vector3f::UnitZ())
         * Eigen::AngleAxisf(pitchDeg * DEG2RAD_F, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(rollDeg * DEG2RAD_F, Eigen::Vector3f::UnitX());
}
} // namespace

class InsMekfTest : public ::testing::Test {
protected:
    InsConfig m_cfg;

    void SetUp() override {
        m_cfg.gravity = 9.81f;
        m_cfg.gate1Dof = 10.83f;
        m_cfg.gate2Dof = 13.82f;
        m_cfg.gate3Dof = 16.27f;
    }
};

TEST_F(InsMekfTest, InitializationAndGetters) {
    InsMekf filter(m_cfg);

    const Eigen::Quaternionf q0 = eulerToQuat(5.0f, -3.0f, 45.0f);
    const Vec3 v0(10.0f, 2.0f, -0.5f);
    const Vec3 p0(100.0f, -50.0f, 12.0f);
    const Vec3 bg0(0.01f, -0.02f, 0.005f);
    const Vec3 ba0(-0.05f, 0.03f, 0.1f);

    ErrorMat P0 = ErrorMat::Identity() * 0.1f;
    filter.initialize(q0, v0, p0, bg0, ba0, P0);

    EXPECT_NEAR(filter.getQuaternion().dot(q0), 1.0f, 1e-4f);
    EXPECT_NEAR((filter.getVelocity() - v0).norm(), 0.0f, 1e-5f);
    EXPECT_NEAR((filter.getPosition() - p0).norm(), 0.0f, 1e-5f);
    EXPECT_NEAR((filter.getGyroBias() - bg0).norm(), 0.0f, 1e-5f);
    EXPECT_NEAR((filter.getAccelBias() - ba0).norm(), 0.0f, 1e-5f);

    const Vec3 euler = filter.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 5.0f, 1e-3f);
    EXPECT_NEAR(euler.y(), -3.0f, 1e-3f);
    EXPECT_NEAR(euler.z(), 45.0f, 1e-3f);
}

TEST_F(InsMekfTest, KinematicPredictionAtRest) {
    InsMekf filter(m_cfg);

    // Initialized level, at rest
    const Eigen::Quaternionf q0 = Eigen::Quaternionf::Identity();
    const Vec3 v0 = Vec3::Zero();
    const Vec3 p0 = Vec3::Zero();
    ErrorMat P0 = ErrorMat::Identity() * 0.01f;
    filter.initialize(q0, v0, p0, Vec3::Zero(), Vec3::Zero(), P0);

    // Feed specific force = [0, 0, -9.81] (exact specific force when stationary in level NED)
    // and gyro = [0, 0, 0]
    const float dt = 0.01f;
    for (int i = 0; i < 100; ++i) {
        filter.predict(dt, Vec3::Zero(), Vec3(0.0f, 0.0f, -9.81f));
    }

    // Velocity and position should remain near zero
    EXPECT_NEAR(filter.getVelocity().norm(), 0.0f, 1e-4f);
    EXPECT_NEAR(filter.getPosition().norm(), 0.0f, 1e-4f);
    EXPECT_NEAR(filter.getEulerAnglesDeg().norm(), 0.0f, 1e-4f);

    // Covariance should have grown monotonically due to process noise
    EXPECT_GT(filter.getCovariance()(IDX_POS, IDX_POS), P0(IDX_POS, IDX_POS));
    EXPECT_GT(filter.getCovariance()(IDX_VEL, IDX_VEL), P0(IDX_VEL, IDX_VEL));
}

TEST_F(InsMekfTest, ConstantAccelerationPrediction) {
    InsMekf filter(m_cfg);

    const Eigen::Quaternionf q0 = Eigen::Quaternionf::Identity();
    filter.initialize(q0, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), ErrorMat::Identity() * 0.01f);

    // Vehicle accelerates forward at 2.0 m/s^2 along body X
    // Specific force f_b = [a_x, 0, 0] - [0, 0, g] = [2.0, 0, -9.81]
    const float dt = 0.01f;
    const float totalTime = 2.0f; // 2 seconds
    const int steps = static_cast<int>(totalTime / dt);

    for (int i = 0; i < steps; ++i) {
        filter.predict(dt, Vec3::Zero(), Vec3(2.0f, 0.0f, -9.81f));
    }

    // Expected v_x = a * t = 4.0 m/s, p_x = 0.5 * a * t^2 = 4.0 m
    EXPECT_NEAR(filter.getVelocity().x(), 4.0f, 0.02f);
    EXPECT_NEAR(filter.getVelocity().y(), 0.0f, 1e-4f);
    EXPECT_NEAR(filter.getPosition().x(), 4.0f, 0.05f);
}

TEST_F(InsMekfTest, JacobianFiniteDifferencePosition) {
    InsMekf filter(m_cfg);

    // Initial position uncertainty
    ErrorMat P0 = ErrorMat::Identity() * 10.0f;
    filter.initialize(Eigen::Quaternionf::Identity(), Vec3::Zero(), Vec3(10.0f, 20.0f, 5.0f),
                      Vec3::Zero(), Vec3::Zero(), P0);

    // Measurement at [12.0, 19.0, 4.5]
    const Vec3 meas(12.0f, 19.0f, 4.5f);
    const Vec3 sigma(1.0f, 1.0f, 1.0f);
    const UpdateResult res = filter.updatePosition(meas, sigma);

    EXPECT_TRUE(res.accepted);
    // Position should be pulled towards measurement
    EXPECT_GT(filter.getPosition().x(), 10.0f);
    EXPECT_LT(filter.getPosition().y(), 20.0f);
    EXPECT_LT(filter.getPosition().z(), 5.0f);

    // Position covariance should have shrunk
    EXPECT_LT(filter.getCovariance()(IDX_POS, IDX_POS), P0(IDX_POS, IDX_POS));
}

TEST_F(InsMekfTest, PositionOutlierRejection) {
    InsMekf filter(m_cfg);
    ErrorMat P0 = ErrorMat::Identity() * 1.0f; // 1 m std dev
    filter.initialize(Eigen::Quaternionf::Identity(), Vec3::Zero(), Vec3::Zero(),
                      Vec3::Zero(), Vec3::Zero(), P0);

    // Outlier 50m away with 1m sigma: NIS = (50/sqrt(1 + 1))^2 ~ 1250 >> 16.27
    const UpdateResult res = filter.updatePosition(Vec3(50.0f, 0.0f, 0.0f), Vec3::Constant(1.0f));
    EXPECT_FALSE(res.accepted);
    EXPECT_GT(res.nis, m_cfg.gate3Dof);
    // State should not be affected
    EXPECT_NEAR(filter.getPosition().x(), 0.0f, 1e-4f);
}

TEST_F(InsMekfTest, HeadingUpdateWraparound) {
    InsMekf filter(m_cfg);
    // Heading currently at +175 deg
    const Eigen::Quaternionf q0 = eulerToQuat(0.0f, 0.0f, 175.0f);
    ErrorMat P0 = ErrorMat::Identity() * 0.1f;
    filter.initialize(q0, Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), Vec3::Zero(), P0);

    // Measurement at -175 deg (+185 deg). Innovation across boundary is +10 deg, not -350 deg!
    const float measYaw = -175.0f * DEG2RAD_F;
    const UpdateResult res = filter.updateHeading(measYaw, 0.05f);

    EXPECT_TRUE(res.accepted);
    // Estimated yaw should move towards +180/-180 deg
    const float newYaw = filter.getEulerAnglesDeg().z();
    EXPECT_GT(std::abs(newYaw), 175.0f);
}

TEST_F(InsMekfTest, ZuptAndZaruObservation) {
    InsMekf filter(m_cfg);
    // Non-zero velocity and bias
    const Vec3 vInit(1.0f, -0.5f, 0.2f);
    const Vec3 bgInit(0.02f, -0.01f, 0.03f);
    filter.initialize(Eigen::Quaternionf::Identity(), vInit, Vec3::Zero(),
                      bgInit, Vec3::Zero(), ErrorMat::Identity() * 0.1f);

    // Apply ZUPT and ZARU repeatedly
    for (int i = 0; i < 20; ++i) {
        filter.updateZeroVelocity(0.02f);
        filter.updateZeroRate(Vec3::Zero(), 0.005f); // Raw gyro measures 0 => true rate 0 => bg converges to 0
    }

    EXPECT_NEAR(filter.getVelocity().norm(), 0.0f, 0.05f);
    EXPECT_NEAR(filter.getGyroBias().norm(), 0.0f, 0.01f);
}

TEST_F(InsMekfTest, NonHolonomicConstraintReducesLateralVelocity) {
    InsMekf filter(m_cfg);

    // Vehicle moving at 10 m/s forward, but has a sideways drift of 2 m/s
    const Vec3 vInit(10.0f, 2.0f, 0.0f);
    ErrorMat P0 = ErrorMat::Identity() * 1.0f;
    P0.block<3, 3>(IDX_ATT, IDX_ATT) = Mat3::Identity() * (0.001f * 0.001f);
    filter.initialize(Eigen::Quaternionf::Identity(), vInit, Vec3::Zero(),
                      Vec3::Zero(), Vec3::Zero(), P0);

    // Apply NHC with identity mounting rotation
    for (int i = 0; i < 10; ++i) {
        filter.updateNonHolonomic(Eigen::Quaternionf::Identity(), 0.2f, 0.2f);
    }

    // Lateral velocity (Y in vehicle frame) should be driven towards 0
    EXPECT_LT(std::abs(filter.getVelocity().y()), 0.5f);
    // Forward velocity should be largely retained
    EXPECT_NEAR(filter.getVelocity().x(), 10.0f, 0.5f);
}
