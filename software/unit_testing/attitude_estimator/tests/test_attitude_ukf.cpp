/**
 * @file test_attitude_ukf.cpp
 * @brief Unit tests for AttitudeUkf class using GoogleTest.
 *
 * Implements a 6-state UKF error state.
 * Validates initialization, kinematic propagation, sensor fusion (Accel/Mag),
 * bias estimation, and full flight dynamics.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <random>
#include "attitude_ukf.hpp" 

 // Define PI if not available (Standard C++17/20 difference safety)
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

using namespace gnc;

// ----------------------------------------------------------------------------
// Test Fixture
// ----------------------------------------------------------------------------
class AttitudeUkfTest : public ::testing::Test {
protected:
    gnc::FilterConfig m_default_config;

    void SetUp() override {
        // Robust Tuning for Float Stability
        m_default_config.alpha = 0.1f;  // Prevent catastrophic cancellation
        m_default_config.beta = 2.0f;   // Optimal for Gaussian
        m_default_config.kappa = 0.0f;
        m_default_config.qGyro = 0.01f;
        m_default_config.qBias = 0.0001f;
        m_default_config.rAccel = 0.1f;
        m_default_config.rMag = 0.1f;   // Standard trust
    }

    // Helper to create a specific quaternion from Euler angles (Degrees)
    Eigen::Quaternionf createQuat(float rollDeg, float pitchDeg, float yawDeg) {
        float roll = rollDeg * M_PI / 180.0f;
        float pitch = pitchDeg * M_PI / 180.0f;
        float yaw = yawDeg * M_PI / 180.0f;

        return Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
    }
};

// ----------------------------------------------------------------------------
// Initialization Tests
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, InitializationStateIsIdentity) {
    AttitudeUkf ukf(m_default_config);

    // Default constructor should result in Identity [1, 0, 0, 0]
    Eigen::Quaternionf q = ukf.getQuaternion();

    EXPECT_NEAR(q.w(), 1.0f, 1e-5f);
    EXPECT_NEAR(q.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(q.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(q.z(), 0.0f, 1e-5f);

    // Biases should be zero
    const auto& state = ukf.getState();
    EXPECT_NEAR(state(3), 0.0f, 1e-5f); // Bx
    EXPECT_NEAR(state(4), 0.0f, 1e-5f); // By
    EXPECT_NEAR(state(5), 0.0f, 1e-5f); // Bz
}

// ----------------------------------------------------------------------------
// Prediction Tests (Kinematics)
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, PredictPureZRotation) {
    AttitudeUkf ukf(m_default_config);

    // Simulate a 90 degree/sec rotation around Z axis for 1 second.
    // Result should be 90 degree yaw.
    float dt = 0.01f; // 100 Hz
    int steps = 100;

    Eigen::Vector3f omegaMeas(0.0f, 0.0f, static_cast<float>(M_PI) / 2.0f); // 90 deg/s

    for (int i = 0; i < steps; ++i) {
        ukf.predict(dt, omegaMeas);
    }

    Eigen::Quaternionf q = ukf.getQuaternion();

    // Euler angles from quaternion
    // Yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));

    // Expect 90 degrees (approx 1.5707 rad)
    EXPECT_NEAR(yaw, static_cast<float>(M_PI) / 2.0f, 0.05f);
}

// ----------------------------------------------------------------------------
// Correction Tests
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, UpdateAccelCorrectsPitch) {
    AttitudeUkf ukf(m_default_config);

    // Scenario: Pitch Up +45 Degrees.
    // Frame: NED (Gravity Points +Z)
    // 
    // Gravity Vector (Inertial): [0, 0, 9.81]
    // Body Orientation: +45 deg pitch (Nose Up)
    //
    // Projection of Gravity onto Body Frame:
    // X_body points "Up and Forward". Gravity points "Down". 
    //   -> Component X is NEGATIVE (-sin(45)*g)
    // Z_body points "Down and Back". Gravity points "Down".
    //   -> Component Z is POSITIVE (cos(45)*g)

    float g = 9.81f;
    float angle = static_cast<float>(M_PI) / 4.0f; // 45 deg
    Eigen::Vector3f accelMeas(
        -std::sin(angle) * g,
        0.0f,
        std::cos(angle) * g   // Positive Z component for NED Pitch Up
    );

    Eigen::Vector3f omegaZero(0.0f, 0.0f, 0.0f);
    float dt = 0.01f;

    for (int i = 0; i < 500; ++i) {
        ukf.predict(dt, omegaZero);
        ukf.updateAccel(accelMeas);
    }

    Eigen::Quaternionf q = ukf.getQuaternion();
    float pitch = std::asin(2.0f * (q.w() * q.y() - q.z() * q.x()));

    EXPECT_NEAR(pitch, angle, 0.05f);
}

TEST_F(AttitudeUkfTest, UpdateMagCorrectsYaw) {
    // 1. Setup a "Strong Update" config
    // We lower rMag to make the filter trust the magnetometer more.
    // This overcomes the large initial error (90 degrees) faster.
    gnc::FilterConfig quickConvergeConfig = m_default_config;
    quickConvergeConfig.rMag = 0.001f; // High trust in measurement

    AttitudeUkf ukf(quickConvergeConfig);

    // 1. Setup Reference: Magnetic field points North (X-axis)
    Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);

    // 2. Scenario:
    //    - Filter initialized at Identity (Facing North).
    //    - Real world: We are facing EAST (+90 deg Yaw).
    //    - In NED, if Body X points East, North is to our Left (-Y).
    Eigen::Vector3f magMeas(0.0f, -1.0f, 0.0f);

    Eigen::Vector3f omegaZero(0.0f, 0.0f, 0.0f);
    float dt = 0.01f;

    // 3. Run Filter
    for (int i = 0; i < 1000; ++i) {
        ukf.predict(dt, omegaZero);
        ukf.updateMag(magMeas, magRef);
    }

    // 4. Check Yaw
    Eigen::Quaternionf q = ukf.getQuaternion();
    float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));

    // Expect +90 degrees (approx 1.57 rad)
    EXPECT_NEAR(yaw, static_cast<float>(M_PI) / 2.0f, 0.05f);
}

TEST_F(AttitudeUkfTest, UpdateFusedCorrectsCombinedOrientation) {
    // 1. Setup - Stronger trust for faster convergence in test
    gnc::FilterConfig simConfig = m_default_config;
    simConfig.rAccel = 0.5f;
    simConfig.rMag = 0.5f;

    AttitudeUkf ukf(simConfig);

    // Reference Vectors (NED)
    Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f); // North

    // 2. Scenario: Pitch 45 deg Up, Yaw 90 deg East.
    // We construct the "True" rotation.
    Eigen::Quaternionf qTrue = createQuat(0.0f, 45.0f, 90.0f);

    // 3. Generate Measurements
    // Accel = R_nb^T * g_n
    Eigen::Vector3f accelMeas = qTrue.conjugate() * g_ned;
    // Mag = R_nb^T * mag_n
    Eigen::Vector3f magMeas = qTrue.conjugate() * mag_ned;

    // 4. Run Filter
    // The MEKF correctly shrinks covariance without collapsing, so its gains drop off naturally.
    // It takes a little longer to completely eliminate a massive 90-degree starting error.
    Eigen::Vector3f omegaZero = Eigen::Vector3f::Zero();
    float dt = 0.01f;

    for (int i = 0; i < 5000; ++i) {
        ukf.predict(dt, omegaZero);
        ukf.updateAccel(accelMeas);
        ukf.updateMag(magMeas, mag_ned);
    }

    // 5. Verify
    Eigen::Quaternionf qEst = ukf.getQuaternion();

    // Check Dot Product (should be close to 1 for same orientation)
    // q and -q represent same rotation, so abs(dot)
    EXPECT_NEAR(std::abs(qEst.dot(qTrue)), 1.0f, 0.01f);

    // Double check Euler angles for clarity
    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(euler.y(), 45.0f, 1.0f); // Pitch
    EXPECT_NEAR(euler.z(), 90.0f, 1.0f); // Yaw
}

// ----------------------------------------------------------------------------
// Bias Estimation Tests
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, EstimateGyroBias) {
    AttitudeUkf ukf(m_default_config);

    // Scenario:
    // The device is STATIONARY (True rate = 0).
    // The Gyro reports a bias of 0.1 rad/s on X axis.
    // The Accel reports perfect gravity (Level).
    // The filter should detect that the rotation indicated by gyro is FAKE
    // because the accelerometer vector isn't changing. It should learn the bias.
    //
    // NOTE — Observability: a 3-axis accelerometer alone cannot observe yaw-axis
    // (Z) gyro bias because the gravity vector has no yaw sensitivity. Only Roll
    // and Pitch biases are observable from accelerometer updates. A magnetometer
    // update is required to make yaw-axis bias observable. This test therefore
    // only validates convergence of the X-axis bias; a Z-axis bias input would
    // not converge and must not be asserted here.

    Eigen::Vector3f trueBias(0.1f, 0.0f, 0.0f);
    Eigen::Vector3f omegaMeas = trueBias; // We read bias, not motion

    // Stationary Accel (NED Level): [0, 0, 9.81]
    Eigen::Vector3f accelMeas(0.0f, 0.0f, 9.81f);
    float dt = 0.01f;

    for (int i = 0; i < 1500; ++i) { // Give it time to converge
        ukf.predict(dt, omegaMeas);
        ukf.updateAccel(accelMeas);
    }

    const auto& state = ukf.getState();

    // Check if Bias X (state index 3) has converged to ~0.1
    EXPECT_NEAR(state(3), 0.1f, 0.02f);
}

// ----------------------------------------------------------------------------
// Filter Health / Covariance Tests
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, CovarianceSanityCheck) {
    // 1. Setup
    // Use alpha=1.0 to prevent negative weights (numerical stability)
    // Increase Bias Process Noise so the growth is obvious
    gnc::FilterConfig config = m_default_config;
    config.alpha = 1.0f;
    config.qBias = 0.1f;

    AttitudeUkf ukf(config);
    ukf.init(Eigen::Quaternionf::Identity(), Eigen::Vector3f::Zero());

    // Helper to get the Trace (sum of diagonals) of the Bias block (last 3 states)
    auto getBiasTrace = [&]() {
        return ukf.getCovariance().block<3, 3>(3, 3).trace();
        };

    // --- PREDICTION STEP ---
    float initBiasTrace = getBiasTrace();

    // Predict should ADD Process Noise to the Bias estimate.
    // Since Bias is a linear random walk, this MUST increase variance.
    ukf.predict(0.01f, Eigen::Vector3f::Zero());

    float predBiasTrace = getBiasTrace();

    EXPECT_GT(predBiasTrace, initBiasTrace)
        << "Bias uncertainty must grow during prediction (Process Noise).";

    // --- UPDATE STEP ---
    // We check the WHOLE matrix trace here.
    // The measurement update provides information about orientation, 
    // which should reduce the overall uncertainty of the system.
    float predTotalTrace = ukf.getCovariance().trace();

    // Perfect gravity measurement
    ukf.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));

    float updateTotalTrace = ukf.getCovariance().trace();

    EXPECT_LT(updateTotalTrace, predTotalTrace)
        << "Total uncertainty must shrink after a valid measurement update.";
}

// ----------------------------------------------------------------------------
// Full System Simulation
// ----------------------------------------------------------------------------
TEST_F(AttitudeUkfTest, CompleteFlightSimulation) {
    // 1. Setup - Use high mag trust for cleaner simulation tracking
    gnc::FilterConfig simConfig = m_default_config;
    simConfig.rMag = 0.01f;

    AttitudeUkf ukf(simConfig);

    // Truth State
    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();

    // Environmental Constants (NED)
    Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);   // Gravity Down
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);  // North Forward

    float dt = 0.01f; // 100 Hz

    // Helper Lambda: Run Simulation Steps
    auto run_segment = [&](Eigen::Vector3f body_rates, float duration_sec) {
        int steps = static_cast<int>(duration_sec / dt);

        for (int i = 0; i < steps; ++i) {
            // A. Propagate Truth (Kinematics)
            // Exact closed-form quaternion integration. Setting w=1 and normalising
            // is only valid for |omega|*dt << 1; at rates above ~0.5 rad/s the
            // approximation accumulates systematic under-rotation error every step.
            Eigen::Quaternionf dq;
            const float theta = body_rates.norm() * dt;
            if (theta > 1e-6f) {
                dq.w() = std::cos(theta * 0.5f);
                dq.vec() = (body_rates / body_rates.norm()) * std::sin(theta * 0.5f);
            }
            else {
                dq.w() = 1.0f;
                dq.vec() = body_rates * dt * 0.5f;
                dq.normalize();
            }

            qTrue = qTrue * dq;
            qTrue.normalize();

            // B. Generate Synthetic Measurements
            // Sensors measure the inertial vector rotated into the body frame
            // Accel = R_nb^T * g_n
            Eigen::Vector3f accel_body = qTrue.conjugate() * g_ned;
            Eigen::Vector3f mag_body = qTrue.conjugate() * mag_ned;
            Eigen::Vector3f gyro_meas = body_rates;

            // C. Run UKF
            ukf.predict(dt, gyro_meas);
            ukf.updateAccel(accel_body);
            ukf.updateMag(mag_body, mag_ned);
        }
        };

    // --- PHASE 1: Warmup (Stationary) ---
    // Duration: 1.0s
    run_segment(Eigen::Vector3f(0, 0, 0), 1.0f);

    // Check: Should remain at Identity
    EXPECT_NEAR(ukf.getQuaternion().w(), 1.0f, 0.01f);

    // --- PHASE 2: Pitch Up Maneuver ---
    // Duration: 1.0s, Rate: 45 deg/s
    float pitch_rate = static_cast<float>(M_PI) / 4.0f;
    run_segment(Eigen::Vector3f(0, pitch_rate, 0), 1.0f);

    // Check: Pitch should be approx 45 degrees
    // Truth Check
    Eigen::Quaternionf qEst = ukf.getQuaternion();
    // Dot product == 1.0 means identical orientation
    EXPECT_NEAR(std::abs(qEst.dot(qTrue)), 1.0f, 0.01f);

    // --- PHASE 3: Coordinated Turn (Yawing while Pitched) ---
    // Duration: 1.0s, Yaw Rate: 90 deg/s
    // Note: Yawing in body frame while pitched results in Roll+Yaw in Inertial frame
    float yaw_rate = static_cast<float>(M_PI) / 2.0f;
    run_segment(Eigen::Vector3f(0, 0, yaw_rate), 1.0f);

    // Check: Orientation should still match truth closely
    qEst = ukf.getQuaternion();
    EXPECT_NEAR(std::abs(qEst.dot(qTrue)), 1.0f, 0.02f);

    // --- PHASE 4: Return to Level ---
    // Duration: 1.0s, Negative Pitch Rate
    // We need to unwind the pitch. 
    run_segment(Eigen::Vector3f(0, -pitch_rate, 0), 1.0f);

    // Final check: Filter should track the return
    qEst = ukf.getQuaternion();
    EXPECT_NEAR(std::abs(qEst.dot(qTrue)), 1.0f, 0.02f);
}

// ============================================================================
// Test: getEulerAnglesDeg
// ============================================================================

TEST_F(AttitudeUkfTest, GetEulerAngles_Identity) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // Identity Quaternion -> 0, 0, 0
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    AttitudeUkf::Vector3 bias = AttitudeUkf::Vector3::Zero();
    ukf.init(q, bias);

    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();

    EXPECT_NEAR(euler.x(), 0.0f, 0.01f); // Roll
    EXPECT_NEAR(euler.y(), 0.0f, 0.01f); // Pitch
    EXPECT_NEAR(euler.z(), 0.0f, 0.01f); // Yaw
}

TEST_F(AttitudeUkfTest, GetEulerAngles_90DegRoll) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // 90 deg Roll
    Eigen::Quaternionf q = createQuat(90.0f, 0.0f, 0.0f);
    AttitudeUkf::Vector3 bias = AttitudeUkf::Vector3::Zero();
    ukf.init(q, bias);

    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();

    EXPECT_NEAR(euler.x(), 90.0f, 0.1f);
    EXPECT_NEAR(euler.y(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 0.0f, 0.1f);
}

TEST_F(AttitudeUkfTest, GetEulerAngles_NegativePitch) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // -45 deg Pitch
    Eigen::Quaternionf q = createQuat(0.0f, -45.0f, 0.0f);
    AttitudeUkf::Vector3 bias = AttitudeUkf::Vector3::Zero();
    ukf.init(q, bias);

    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();

    EXPECT_NEAR(euler.x(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.y(), -45.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 0.0f, 0.1f);
}

TEST_F(AttitudeUkfTest, GetEulerAngles_ComplexOrientation) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // R=10, P=20, Y=30
    Eigen::Quaternionf q = createQuat(10.0f, 20.0f, 30.0f);
    AttitudeUkf::Vector3 bias = AttitudeUkf::Vector3::Zero();
    ukf.init(q, bias);

    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();

    EXPECT_NEAR(euler.x(), 10.0f, 0.1f);
    EXPECT_NEAR(euler.y(), 20.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 30.0f, 0.1f);
}

// ============================================================================
// Test: align
// ============================================================================

TEST_F(AttitudeUkfTest, Align_LevelAndNorth) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // Case: Vehicle is perfectly level and facing North.
    // Accel measures Gravity (Down = +9.81)
    AttitudeUkf::Vector3 accelMean = { 0.0f, 0.0f, 9.81f };

    // Mag measures North (X) + Down (Z) (Northern Hemisphere Dip)
    // No Y component implies purely North.
    AttitudeUkf::Vector3 magMean = { 0.4f, 0.0f, 0.9f };

    AttitudeUkf::Vector3 magRefOut;
    ukf.align(accelMean, magMean, magRefOut);

    // 1. Verify Attitude (Should be 0, 0, 0)
    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 1.0f); // Roll
    EXPECT_NEAR(euler.y(), 0.0f, 1.0f); // Pitch
    EXPECT_NEAR(euler.z(), 0.0f, 1.0f); // Yaw

    // 2. Verify Mag Reference
    // Since we aligned to this measurement, the Ref vector should match the normalized input magnitude
    // but projected into the estimated frame. Since estimated frame is identity, it should match input normalized.
    AttitudeUkf::Vector3 magMeanNorm = magMean.normalized();
    EXPECT_NEAR(magRefOut.x(), magMeanNorm.x(), 0.01f);
    EXPECT_NEAR(magRefOut.y(), magMeanNorm.y(), 0.01f);
    EXPECT_NEAR(magRefOut.z(), magMeanNorm.z(), 0.01f);
}

TEST_F(AttitudeUkfTest, Align_Rolled90DegreesRight) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // Case: Vehicle is rolled 90 degrees to the Right.
    // Body Frame: X=North, Y=Down, Z=Left.

    // Gravity (Down) is now along Body Y axis (+9.81)
    AttitudeUkf::Vector3 accelMean = { 0.0f, 9.81f, 0.0f };

    // Mag (North) is still along Body X (since we only rolled)
    // Mag (Down) is along Body Y.
    // So Mag vector = [0.4, 0.9, 0.0]
    AttitudeUkf::Vector3 magMean = { 0.4f, 0.9f, 0.0f };

    AttitudeUkf::Vector3 magRefOut;
    ukf.align(accelMean, magMean, magRefOut);

    // 1. Verify Attitude
    AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 90.0f, 1.0f); // Roll should be 90
    EXPECT_NEAR(euler.y(), 0.0f, 1.0f);  // Pitch 0
    EXPECT_NEAR(euler.z(), 0.0f, 1.0f);  // Yaw 0 (Facing North)

    // 2. Verify Reference Vector (Inertial Frame)
    // The calculated reference vector should reconstruct the inertial dip.
    // Even though the body is rolled, the "World" magnetic field didn't change.
    // Ideally Ref = [North_Mag, 0, Down_Mag] (Normalized)
    // North = 0.4, Down = 0.9.
    Eigen::Vector3f expectedRef = Eigen::Vector3f(0.4f, 0.0f, 0.9f).normalized();

    EXPECT_NEAR(magRefOut.x(), expectedRef.x(), 0.05f);
    EXPECT_NEAR(magRefOut.y(), 0.0f, 0.05f); // Should always have 0 East component in NED ref
    EXPECT_NEAR(magRefOut.z(), expectedRef.z(), 0.05f);
}

TEST_F(AttitudeUkfTest, Align_Pitched90DegreesUp) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // Case: Vehicle is pitched 90 degrees Up (Nose pointing to Sky).
    // Body Frame: X=Up, Y=Right, Z=Back.

    // Gravity (Down) is along -X axis.
    AttitudeUkf::Vector3 accelMean = { -9.81f, 0.0f, 0.0f };

    // Mag (North) is now along -Z (Belly)
    // Mag (Down) is along -X.
    // Input Mag = [-0.9, 0.0, 0.4] => Z must be positive to represent North pointing towards the belly when Pitched 90 Up
    AttitudeUkf::Vector3 magMean = { -0.9f, 0.0f, 0.4f };

    AttitudeUkf::Vector3 magRefOut;
    ukf.align(accelMean, magMean, magRefOut);

    // 1. Verify Attitude
    // At Pitch=90, Euler angles suffer from Gimbal Lock. `getEulerAnglesDeg` may output {0, 90, 0} or {180, 90, 180},
    // which represent the EXACT same physical orientation. Therefore, we evaluate the quaternion directly.
    Eigen::Quaternionf q = ukf.getQuaternion();
    Eigen::Quaternionf expectedQ = createQuat(0.0f, 90.0f, 0.0f);
    EXPECT_NEAR(std::abs(q.dot(expectedQ)), 1.0f, 1e-3f);

    // 2. Verify Reference Vector (Inertial Frame)
    Eigen::Vector3f expectedRef = Eigen::Vector3f(0.4f, 0.0f, 0.9f).normalized();
    EXPECT_NEAR(magRefOut.x(), expectedRef.x(), 0.05f);
    EXPECT_NEAR(magRefOut.z(), expectedRef.z(), 0.05f);
}

// ============================================================================
// Test: Sign Convention (NED)
// ============================================================================

TEST_F(AttitudeUkfTest, SignConvention_NED) {
    // Setup
    gnc::FilterConfig simConfig = m_default_config;
    AttitudeUkf ukf(simConfig);

    // 1. Verify Positive Roll (Right Wing Down)
    // We create a quaternion representing a 30-degree positive roll
    Eigen::Quaternionf qRoll = createQuat(30.0f, 0.0f, 0.0f);
    ukf.init(qRoll, AttitudeUkf::Vector3::Zero());

    AttitudeUkf::Vector3 eulerRoll = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(eulerRoll.x(), 30.0f, 0.1f) << "Positive Roll (Right Wing Down) should be +30 deg";

    // 2. Verify Positive Pitch (Nose Up)
    // We create a quaternion representing a 30-degree positive pitch
    Eigen::Quaternionf qPitch = createQuat(0.0f, 30.0f, 0.0f);
    ukf.init(qPitch, AttitudeUkf::Vector3::Zero());

    AttitudeUkf::Vector3 eulerPitch = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(eulerPitch.y(), 30.0f, 0.1f) << "Positive Pitch (Nose Up) should be +30 deg";

    // 3. Verify Positive Yaw (Nose Right)
    // We create a quaternion representing a 30-degree positive yaw
    Eigen::Quaternionf qYaw = createQuat(0.0f, 0.0f, 30.0f);
    ukf.init(qYaw, AttitudeUkf::Vector3::Zero());

    AttitudeUkf::Vector3 eulerYaw = ukf.getEulerAnglesDeg();
    EXPECT_NEAR(eulerYaw.z(), 30.0f, 0.1f) << "Positive Yaw (Nose Right) should be +30 deg";
}

// ============================================================================
// Test: Accuracy Under Sensor Noise (Monte Carlo Simulation)
// ============================================================================

TEST_F(AttitudeUkfTest, AccuracyWithSensorNoise) {
    // 1. Setup with standard tuning
    gnc::FilterConfig config = m_default_config;
    // Typical low-cost MEMS noise parameters
    config.qGyro = 0.001f;
    config.rAccel = 0.1f;
    config.rMag = 0.1f;

    AttitudeUkf ukf(config);

    // 2. Random Number Generation
    std::default_random_engine gen(42); // Fixed seed for reproducibility
    std::normal_distribution<float> noiseGyro(0.0f, 0.01f); // ~0.5 deg/s noise
    std::normal_distribution<float> noiseAccel(0.0f, 0.1f); // ~0.1 m/s^2 noise
    std::normal_distribution<float> noiseMag(0.0f, 0.02f);  // Small mag noise

    // Truth State
    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();

    // Environment
    Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);

    float dt = 0.01f; // 100 Hz
    float totalTime = 100.0f; // 10 seconds simulation
    int steps = static_cast<int>(totalTime / dt);

    // Metrics
    float maxErrorDeg = 0.0f;
    float sumSquaredError = 0.0f;
    int evaluatedSteps = 0;

    for (int i = 0; i < steps; ++i) {
        float t = i * dt;

        // 3. True Dynamics: Sinusoidal Roll and Pitch (Coning motion)
        float freq = 1.0f; // 1 rad/s cycle
        float amp = 0.5f;  // ~30 degrees amplitude

        // Rates are the time derivative of the angle
        float rollRate = amp * freq * std::cos(freq * t);
        float pitchRate = amp * freq * std::sin(freq * t);
        Eigen::Vector3f trueRates(rollRate, pitchRate, 0.0f);

        // Propagate Truth State.
        // Exact closed-form integration prevents the truth model from sharing the
        // same first-order under-rotation error as the filter under test, which
        // would otherwise mask Bug 1 (both halves making the same approximation).
        Eigen::Quaternionf dq;
        const float theta = trueRates.norm() * dt;
        if (theta > 1e-6f) {
            dq.w() = std::cos(theta * 0.5f);
            dq.vec() = (trueRates / trueRates.norm()) * std::sin(theta * 0.5f);
        }
        else {
            dq.w() = 1.0f;
            dq.vec() = trueRates * dt * 0.5f;
            dq.normalize();
        }

        qTrue = qTrue * dq;
        qTrue.normalize();

        // 4. Generate Noisy Measurements
        // Gyro: Truth + Noise
        Eigen::Vector3f gyroMeas = trueRates + Eigen::Vector3f(noiseGyro(gen), noiseGyro(gen), noiseGyro(gen));

        // Accel: Rotate Gravity into Body Frame + Noise
        // Accel = R_nb^T * g_n
        Eigen::Vector3f accelMeas = qTrue.conjugate() * g_ned;
        accelMeas += Eigen::Vector3f(noiseAccel(gen), noiseAccel(gen), noiseAccel(gen));

        // Mag: Rotate North into Body Frame + Noise
        // Mag = R_nb^T * mag_n
        Eigen::Vector3f magMeas = qTrue.conjugate() * mag_ned;
        magMeas += Eigen::Vector3f(noiseMag(gen), noiseMag(gen), noiseMag(gen));

        // 5. Run Filter
        ukf.predict(dt, gyroMeas);
        ukf.updateAccel(accelMeas);
        ukf.updateMag(magMeas, mag_ned);

        // 6. Compute Error (Skip first 1.0s to allow convergence)
        if (t > 1.0f) {
            Eigen::Quaternionf qEst = ukf.getQuaternion();

            // Calculate geodesic angle distance between quaternions
            // angle = 2 * acos(|q1 . q2|)
            float dot = std::abs(qTrue.dot(qEst));
            if (dot > 1.0f) dot = 1.0f; // Clamp for safety

            float angleErrorRad = 2.0f * std::acos(dot);
            float angleErrorDeg = angleErrorRad * 180.0f / M_PI;

            if (angleErrorDeg > maxErrorDeg) maxErrorDeg = angleErrorDeg;
            sumSquaredError += angleErrorDeg * angleErrorDeg;
            evaluatedSteps++;
        }
    }

    // 7. Verify Results
    float rmsError = std::sqrt(sumSquaredError / evaluatedSteps);

    // Log results for debugging if needed
    std::cout << "Max Error: " << maxErrorDeg << " deg, RMS: " << rmsError << " deg" << std::endl;

    // Assertions:
    // With these noise levels, a well-tuned UKF should keep RMS error well below 2.0 degrees.
    EXPECT_LT(rmsError, 0.5f) << "RMS Error is too high (" << rmsError << " deg)";
    EXPECT_LT(maxErrorDeg, 1.0f) << "Max Peak Error is too high (" << maxErrorDeg << " deg)";
}

// ============================================================================
// Test: Rank Deficiency / Cholesky Failure Catch
// ============================================================================

TEST_F(AttitudeUkfTest, CholeskyRankDeficiencyTrigger) {
    // This test artificially accelerates the rank-deficiency issue inherently
    // present in full state UKF architectures.
    // It does this by continuously applying pure prediction (no measurement
    // corrections to anchor the covariance) under high dynamics and noise.

    gnc::FilterConfig config = m_default_config;
    config.qGyro = 0.5f; // High process noise to blow up the covariance quickly

    AttitudeUkf ukf(config);

    float dt = 0.01f;
    int steps = 2000; // 20 seconds of pure integration

    // We simulate a constant, aggressive rotation on all 3 axes
    Eigen::Vector3f aggressiveRates(1.0f, -0.5f, 2.0f);

    // Extract the initial covariance trace 
    float previousTrace = ukf.getCovariance().trace();
    bool fallbackTriggered = false;

    for (int i = 0; i < steps; ++i) {
        ukf.predict(dt, aggressiveRates);

        float currentTrace = ukf.getCovariance().trace();

        // During a pure PREDICTION step without measurement correction, 
        // the addition of Process Noise guarantees the covariance trace MUST increase.
        // If it ever shrinks, it's because the internal Cholesky decomposition 
        // failed and triggered a covariance reset.
        if (currentTrace < previousTrace) {
            fallbackTriggered = true;
        }
        previousTrace = currentTrace;
    }

    // The covariance trace must grow monotonically under pure prediction because
    // process noise is added unconditionally every step. A decrease means the
    // Cholesky decomposition failed and triggered an identity reset, which
    // destroys all accumulated uncertainty information and indicates a bug.
    EXPECT_FALSE(fallbackTriggered)
        << "Covariance trace decreased during pure prediction: Cholesky fallback "
        "triggered an identity reset.";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}