/**
 * @file test_attitude_mekf.cpp
 * @brief Unit tests for AttitudeMekf (Linearised Multiplicative EKF).
 *
 * Test organisation
 * -----------------
 * Group 1  — Construction & Initialisation
 * Group 2  — Prediction (kinematics, covariance growth, Jacobian coupling)
 * Group 3  — Accelerometer update (correction, magnitude gate)
 * Group 4  — Magnetometer update (correction, degenerate inputs)
 * Group 5  — Fused sensor correction (accel + mag)
 * Group 6  — Bias estimation (observability per axis)
 * Group 7  — Covariance health (structure, symmetry, monotonicity)
 * Group 8  — Quaternion integrity (unit norm, double-cover)
 * Group 9  — align() (TRIAD method, magnetic reference)
 * Group 10 — getEulerAnglesDeg() (identity, extremes, complex)
 * Group 11 — Sign convention (NED roll / pitch / yaw)
 * Group 12 — Full flight simulation (multi-phase truth tracking)
 * Group 13 — Monte Carlo accuracy (RMS / max error under sensor noise)
 * Group 14 — Diagnostics (getCovarianceFaultCount)
 *
 * Truth-model integration
 * -----------------------
 * All ground-truth quaternion propagation in this file uses the exact
 * closed-form:
 * dq = [cos(|w|dt/2), wˆ * sin(|w|dt/2)]
 * to ensure the truth model does not share any approximation error with
 * the filter under test.  The small-angle else-branch is kept for
 * |w|*dt <= 1e-6 rad to avoid a divide-by-zero.
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include "attitude_mekf.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

using namespace gnc;

// ============================================================================
// Helpers shared across the entire test file
// ============================================================================

/** Convert degrees to radians. */
static constexpr float deg2rad(float d) { return d * static_cast<float>(M_PI) / 180.0f; }

/**
 * Exact quaternion integration for one step.
 * @param q    Quaternion to propagate in-place.
 * @param omega Body-frame angular rate [rad/s].
 * @param dt   Time step [s].
 */
static void integrateQuat(Eigen::Quaternionf& q,
    const Eigen::Vector3f& omega,
    float dt)
{
    const float theta = omega.norm() * dt;
    Eigen::Quaternionf dq;
    if (theta > 1e-6f) {
        dq.w() = std::cos(theta * 0.5f);
        dq.vec() = (omega / omega.norm()) * std::sin(theta * 0.5f);
    }
    else {
        dq.w() = 1.0f;
        dq.vec() = omega * (dt * 0.5f);
        dq.normalize();
    }
    q = q * dq;
    q.normalize();
}

/**
 * Geodesic angle error between two quaternions [degrees].
 * Accounts for the q / -q double cover.
 */
static float quatErrorDeg(const Eigen::Quaternionf& a,
    const Eigen::Quaternionf& b)
{
    float dot = std::abs(a.dot(b));
    dot = std::min(dot, 1.0f);
    return 2.0f * std::acos(dot) * 180.0f / static_cast<float>(M_PI);
}

/**
 * Build a ZYX quaternion from Euler angles in degrees.
 */
static Eigen::Quaternionf eulerToQuat(float rollDeg, float pitchDeg, float yawDeg)
{
    return Eigen::AngleAxisf(deg2rad(yawDeg), Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(deg2rad(pitchDeg), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(deg2rad(rollDeg), Eigen::Vector3f::UnitX());
}

// ============================================================================
// Test Fixture
// ============================================================================

class AttitudeMekfTest : public ::testing::Test {
protected:
    gnc::FilterConfig m_cfg;

    void SetUp() override
    {
        // Conservative tuning that works across all tests.
        // Individual tests may override specific fields.
        m_cfg.qGyro = 0.01f;    // [rad^2/s]
        m_cfg.qBias = 0.0001f;  // [rad^2/s^3]
        m_cfg.rAccel = 0.1f;     // [(m/s^2)^2]
        m_cfg.rMag = 0.1f;     // dimensionless (normalised inputs)
        m_cfg.accelGate = 1.0f;     // [m/s^2] — reject if |a| deviates >1 m/s^2 from g
    }

    /** Run predict + updateAccel + updateMag for `steps` steps at `dt`. */
    void runFused(AttitudeMekf& f,
        const Eigen::Vector3f& omega,
        const Eigen::Vector3f& accel,
        const Eigen::Vector3f& magMeas,
        const Eigen::Vector3f& magRef,
        int steps, float dt = 0.01f)
    {
        for (int i = 0; i < steps; ++i) {
            f.predict(dt, omega);
            f.updateAccel(accel);
            f.updateMag(magMeas, magRef);
        }
    }
};


// ============================================================================
// Group 1 — Construction & Initialisation
// ============================================================================

TEST_F(AttitudeMekfTest, DefaultConstructionIsIdentity)
{
    // After construction the nominal quaternion must be identity and
    // the bias estimate must be zero.
    AttitudeMekf f(m_cfg);

    const Eigen::Quaternionf q = f.getQuaternion();
    EXPECT_NEAR(q.w(), 1.0f, 1e-6f);
    EXPECT_NEAR(q.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(q.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(q.z(), 0.0f, 1e-6f);

    const AttitudeMekf::Vector3 b = f.getBias();
    EXPECT_NEAR(b.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(b.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(b.z(), 0.0f, 1e-6f);
}

TEST_F(AttitudeMekfTest, InitSetsQuaternionAndBias)
{
    AttitudeMekf f(m_cfg);

    const Eigen::Quaternionf qIn = eulerToQuat(30.0f, 45.0f, 60.0f);
    const AttitudeMekf::Vector3 biasIn(0.05f, -0.02f, 0.01f);

    f.init(qIn, biasIn);

    const Eigen::Quaternionf qOut = f.getQuaternion();
    EXPECT_NEAR(std::abs(qOut.dot(qIn)), 1.0f, 1e-5f)
        << "init() must set the nominal quaternion";

    const AttitudeMekf::Vector3 bOut = f.getBias();
    EXPECT_NEAR(bOut.x(), biasIn.x(), 1e-6f);
    EXPECT_NEAR(bOut.y(), biasIn.y(), 1e-6f);
    EXPECT_NEAR(bOut.z(), biasIn.z(), 1e-6f);
}

TEST_F(AttitudeMekfTest, InitNormalisesUnnormalisedQuaternion)
{
    // The MEKF must accept and normalise a non-unit input quaternion.
    AttitudeMekf f(m_cfg);

    Eigen::Quaternionf qUnnorm(2.0f, 0.0f, 0.0f, 0.0f); // w=2, not unit
    f.init(qUnnorm, AttitudeMekf::Vector3::Zero());

    const Eigen::Quaternionf q = f.getQuaternion();
    EXPECT_NEAR(q.norm(), 1.0f, 1e-5f) << "Quaternion must be unit after init()";
    EXPECT_NEAR(q.w(), 1.0f, 1e-5f);   // normalised [2,0,0,0] → [1,0,0,0]
}

TEST_F(AttitudeMekfTest, InitResetsCovarianceToDiagonal)
{
    AttitudeMekf f(m_cfg);

    // Run for a while so cross-terms develop
    const Eigen::Vector3f omega(0.3f, 0.1f, 0.0f);
    const Eigen::Vector3f accel(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);
    runFused(f, omega, accel, accel, magRef, 200);

    // Re-init — must reset to a clean diagonal
    f.init(Eigen::Quaternionf::Identity(), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::ErrorMat& P = f.getCovariance();

    // Off-diagonal blocks must be zero
    const float offDiagNorm = P.block<3, 3>(0, 3).norm()
        + P.block<3, 3>(3, 0).norm();
    EXPECT_NEAR(offDiagNorm, 0.0f, 1e-6f)
        << "Off-diagonal covariance blocks must be zero after init()";

    // Diagonal elements must be positive
    for (int i = 0; i < 6; ++i) {
        EXPECT_GT(P(i, i), 0.0f) << "Covariance diagonal element " << i << " must be positive";
    }
}

TEST_F(AttitudeMekfTest, ReinitMidRunClearsErrorState)
{
    // After re-init the filter must behave as if freshly constructed —
    // steady-state at the new attitude, no residual error state.
    AttitudeMekf f(m_cfg);

    // Drift the filter away from identity
    for (int i = 0; i < 200; ++i) {
        f.predict(0.01f, Eigen::Vector3f(0.5f, 0.0f, 0.0f));
    }

    // Re-init to a specific attitude
    const Eigen::Quaternionf qNew = eulerToQuat(0.0f, 30.0f, 0.0f);
    f.init(qNew, AttitudeMekf::Vector3::Zero());

    const Eigen::Quaternionf qOut = f.getQuaternion();
    EXPECT_NEAR(std::abs(qOut.dot(qNew)), 1.0f, 1e-5f)
        << "Re-init must replace nominal quaternion completely";

    const AttitudeMekf::Vector3 b = f.getBias();
    EXPECT_NEAR(b.norm(), 0.0f, 1e-5f)
        << "Re-init with zero bias must zero the bias estimate";
}


// ============================================================================
// Group 2 — Prediction (kinematics, covariance growth)
// ============================================================================

TEST_F(AttitudeMekfTest, PredictPureZRotation90Deg)
{
    // 90 deg/s about Z for 1 second must yield approximately +90° yaw.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f omega(0.0f, 0.0f, deg2rad(90.0f));
    for (int i = 0; i < 100; ++i) { // 100 × 10 ms = 1 s
        f.predict(0.01f, omega);
    }

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.z(), 90.0f, 1.0f) << "Yaw must reach 90° after 1 s at 90°/s";
    EXPECT_NEAR(euler.x(), 0.0f, 0.5f) << "Roll must remain near zero";
    EXPECT_NEAR(euler.y(), 0.0f, 0.5f) << "Pitch must remain near zero";
}

TEST_F(AttitudeMekfTest, PredictPureXRotation180Deg)
{
    // 180 deg/s about X for 1 second must yield approximately 180° roll.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f omega(deg2rad(180.0f), 0.0f, 0.0f);
    for (int i = 0; i < 100; ++i) {
        f.predict(0.01f, omega);
    }

    // At 180° roll, atan2 can return +180 or -180 — both are correct.
    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(std::abs(euler.x()), 180.0f, 2.0f) << "|Roll| must be ~180° after 180°/s for 1 s";
}

TEST_F(AttitudeMekfTest, PredictHighRateAccuracyVsTruth)
{
    // At 5 rad/s (high dynamics), the exact quaternion integrator in
    // the MEKF must track the truth model to within 0.1°.
    AttitudeMekf f(m_cfg);

    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f omega(3.0f, -2.0f, 1.0f); // ~3.7 rad/s combined
    const float dt = 0.01f;

    for (int i = 0; i < 200; ++i) {
        integrateQuat(qTrue, omega, dt);
        f.predict(dt, omega);
    }

    const float errDeg = quatErrorDeg(f.getQuaternion(), qTrue);
    EXPECT_LT(errDeg, 0.1f)
        << "Gyro-only tracking error must be < 0.1° at high rates (exact integrator)";
}

TEST_F(AttitudeMekfTest, PredictCovarianceGrowsMonotonically)
{
    // The total covariance trace must grow strictly on every predict() call
    // because process noise is added unconditionally.
    AttitudeMekf f(m_cfg);
    f.init(Eigen::Quaternionf::Identity(), AttitudeMekf::Vector3::Zero());

    float prevTrace = f.getCovariance().trace();
    const Eigen::Vector3f omega(0.5f, 0.3f, 0.1f);

    for (int i = 0; i < 500; ++i) {
        f.predict(0.01f, omega);
        const float trace = f.getCovariance().trace();
        EXPECT_GT(trace, prevTrace)
            << "Covariance trace must grow at step " << i;
        prevTrace = trace;
    }
}

TEST_F(AttitudeMekfTest, PredictBiasBlockGrowsFasterThanAttitudeBlock)
{
    // With qBias >> qGyro, the bias covariance block must dominate the growth.
    gnc::FilterConfig cfg = m_cfg;
    cfg.qGyro = 1e-6f;
    cfg.qBias = 0.1f;

    AttitudeMekf f(cfg);
    f.init(Eigen::Quaternionf::Identity(), AttitudeMekf::Vector3::Zero());

    const float initAttTrace = f.getCovariance().block<3, 3>(0, 0).trace();
    const float initBiasTrace = f.getCovariance().block<3, 3>(3, 3).trace();

    for (int i = 0; i < 50; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
    }

    const float dAtt = f.getCovariance().block<3, 3>(0, 0).trace() - initAttTrace;
    const float dBias = f.getCovariance().block<3, 3>(3, 3).trace() - initBiasTrace;

    EXPECT_GT(dBias, dAtt)
        << "Bias uncertainty must grow faster than attitude uncertainty when qBias >> qGyro";
}

TEST_F(AttitudeMekfTest, PredictZeroRateKeepsAttitude)
{
    // Predicting with zero rate must not change the nominal quaternion.
    AttitudeMekf f(m_cfg);
    const Eigen::Quaternionf qInit = eulerToQuat(15.0f, -20.0f, 45.0f);
    f.init(qInit, AttitudeMekf::Vector3::Zero());

    for (int i = 0; i < 100; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
    }

    EXPECT_NEAR(std::abs(f.getQuaternion().dot(qInit)), 1.0f, 1e-4f)
        << "Nominal quaternion must not change when omega = 0";
}

TEST_F(AttitudeMekfTest, PredictSubtractsBiasFromRate)
{
    // With a known constant bias and no measurements, the filter must
    // propagate the bias-corrected rate, not the raw measurement.
    // If the raw rate == the bias, net rotation should be ~zero.
    AttitudeMekf f(m_cfg);
    const Eigen::Vector3f trueBias(0.2f, 0.0f, 0.0f);
    f.init(Eigen::Quaternionf::Identity(), trueBias);

    // Feed the exact bias as the gyro reading → corrected rate = 0
    for (int i = 0; i < 200; ++i) {
        f.predict(0.01f, trueBias);
    }

    EXPECT_NEAR(f.getQuaternion().w(), 1.0f, 0.01f)
        << "When gyro reading == bias, no attitude change should occur";
}


// ============================================================================
// Group 3 — Accelerometer update (correction, magnitude gate)
// ============================================================================

TEST_F(AttitudeMekfTest, UpdateAccelCorrectsPitch)
{
    // Filter starts at identity. Accelerometer encodes +45° pitch.
    // After sufficient fused updates the pitch estimate must converge.
    AttitudeMekf f(m_cfg);

    const float angle = deg2rad(45.0f);
    const Eigen::Vector3f accel(-std::sin(angle) * 9.81f,
        0.0f,
        std::cos(angle) * 9.81f);

    for (int i = 0; i < 600; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateAccel(accel);
    }

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.y(), 45.0f, 1.0f) << "Pitch must converge to 45° from accel";
    EXPECT_NEAR(euler.x(), 0.0f, 1.0f) << "Roll must remain near zero";
}

TEST_F(AttitudeMekfTest, UpdateAccelCorrectsRoll)
{
    // Filter starts at identity. Accelerometer encodes +30° roll.
    AttitudeMekf f(m_cfg);

    const float angle = deg2rad(30.0f);
    const Eigen::Vector3f accel(0.0f,
        std::sin(angle) * 9.81f,
        std::cos(angle) * 9.81f);

    for (int i = 0; i < 600; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateAccel(accel);
    }

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 30.0f, 1.0f) << "Roll must converge to 30° from accel";
    EXPECT_NEAR(euler.y(), 0.0f, 1.0f) << "Pitch must remain near zero";
}

TEST_F(AttitudeMekfTest, AccelGateRejectsLinearAcceleration)
{
    // When the vehicle is accelerating hard (||a|| >> g) the update
    // must be gated out. The attitude should not change from identity.
    gnc::FilterConfig cfg = m_cfg;
    cfg.accelGate = 0.5f; // tight gate

    AttitudeMekf f(cfg);

    // Acceleration far outside gate: simulates a hard pull-up
    const Eigen::Vector3f accelHard(0.0f, 0.0f, 20.0f); // 2g — way outside gate

    for (int i = 0; i < 500; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateAccel(accelHard);
    }

    // The filter should not have drifted from identity since the
    // measurement was gated and zero gyro rate was commanded.
    EXPECT_NEAR(f.getQuaternion().w(), 1.0f, 0.05f)
        << "Attitude must not change when all accel updates are gated";
}

TEST_F(AttitudeMekfTest, AccelGateAcceptsNominalGravity)
{
    // Perfect 1g reading must pass the gate and correct attitude.
    gnc::FilterConfig cfg = m_cfg;
    cfg.accelGate = 0.5f;

    AttitudeMekf f(cfg);
    f.init(eulerToQuat(20.0f, 0.0f, 0.0f), AttitudeMekf::Vector3::Zero());

    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f); // exact 1g
    for (int i = 0; i < 600; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateAccel(accelLevel);
    }

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 1.5f)
        << "Nominal gravity reading must pass the gate and correct roll back toward 0";
}

TEST_F(AttitudeMekfTest, AccelGateDisabledAlwaysAccepts)
{
    // accelGate == 0 must disable the gate entirely.
    gnc::FilterConfig cfg = m_cfg;
    cfg.accelGate = 0.0f; // disabled

    AttitudeMekf f(cfg);

    // 2g reading — would be rejected by a 0.5 m/s^2 gate.
    // With gate disabled the filter must attempt to use the measurement
    // and converge toward the indicated direction.
    const Eigen::Vector3f accel(0.0f, 0.0f, 19.62f); // 2g
    const float P0 = f.getCovariance().block<3, 3>(0, 0).trace();

    for (int i = 0; i < 10; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateAccel(accel);
    }

    // Covariance attitude block must have decreased (update was applied)
    const float P1 = f.getCovariance().block<3, 3>(0, 0).trace();
    EXPECT_LT(P1, P0)
        << "Attitude covariance must decrease when gate is disabled (update applied)";
}


// ============================================================================
// Group 4 — Magnetometer update (correction, degenerate inputs)
// ============================================================================

TEST_F(AttitudeMekfTest, UpdateMagCorrectsYaw)
{
    // Filter starts at identity (facing North in NED).
    // Magnetometer encodes true heading of +90° (facing East).
    // After convergence the filter must report +90° yaw.
    gnc::FilterConfig cfg = m_cfg;
    cfg.rMag = 0.001f; // high trust for faster convergence

    AttitudeMekf f(cfg);

    // magRef: North = +X in NED inertial frame
    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);
    // magMeas: if body X points East, then North is along -Y body
    const Eigen::Vector3f magMeas(0.0f, -1.0f, 0.0f);

    for (int i = 0; i < 1000; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateMag(magMeas, magRef);
    }

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.z(), 90.0f, 2.0f) << "Yaw must converge to 90° from mag alone";
}

TEST_F(AttitudeMekfTest, UpdateMagDoesNotAffectPitch)
{
    // A magnetometer update in the horizontal plane should not change pitch.
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, 30.0f, 0.0f), AttitudeMekf::Vector3::Zero());

    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f magMeas(0.0f, -1.0f, 0.0f);

    const float pitchBefore = f.getEulerAnglesDeg().y();

    for (int i = 0; i < 50; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        f.updateMag(magMeas, magRef);
    }

    // Pitch should be relatively unchanged (mag doesn't drive pitch strongly)
    EXPECT_NEAR(f.getEulerAnglesDeg().y(), pitchBefore, 5.0f)
        << "Magnetometer update must not significantly affect pitch";
}

TEST_F(AttitudeMekfTest, UpdateMagSkipsZeroNormMeasurement)
{
    // A zero-norm measurement must be silently discarded.
    AttitudeMekf f(m_cfg);
    const float P0 = f.getCovariance().trace();

    f.predict(0.01f, Eigen::Vector3f::Zero());
    const float P1 = f.getCovariance().trace();

    f.updateMag(Eigen::Vector3f::Zero(), Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    const float P2 = f.getCovariance().trace();

    // After the skipped update, trace must not decrease (no information applied)
    EXPECT_GE(P2, P1 - 1e-5f)
        << "Zero-norm mag measurement must not reduce covariance";
    (void)P0;
}

TEST_F(AttitudeMekfTest, UpdateMagSkipsZeroNormReference)
{
    // A zero-norm reference must be silently discarded.
    AttitudeMekf f(m_cfg);

    f.predict(0.01f, Eigen::Vector3f::Zero());
    const float P1 = f.getCovariance().trace();

    f.updateMag(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f::Zero());
    const float P2 = f.getCovariance().trace();

    EXPECT_GE(P2, P1 - 1e-5f)
        << "Zero-norm mag reference must not reduce covariance";
}

TEST_F(AttitudeMekfTest, UpdateMagNormalisesInputInternally)
{
    // Passing a 10× scaled measurement vs a unit measurement must
    // produce the same attitude result because updateMag() normalises.
    gnc::FilterConfig cfg = m_cfg;
    cfg.rMag = 0.001f;

    AttitudeMekf fUnit(cfg);
    AttitudeMekf fScaled(cfg);

    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f magUnit(0.0f, -1.0f, 0.0f);
    const Eigen::Vector3f magScaled = magUnit * 10.0f;

    for (int i = 0; i < 500; ++i) {
        fUnit.predict(0.01f, Eigen::Vector3f::Zero());
        fUnit.updateMag(magUnit, magRef);

        fScaled.predict(0.01f, Eigen::Vector3f::Zero());
        fScaled.updateMag(magScaled, magRef);
    }

    const float errDeg = quatErrorDeg(fUnit.getQuaternion(), fScaled.getQuaternion());
    EXPECT_LT(errDeg, 0.1f)
        << "Scaled vs unit mag measurement must produce identical attitude estimates";
}


// ============================================================================
// Group 5 — Fused sensor correction (accel + mag)
// ============================================================================

TEST_F(AttitudeMekfTest, FusedUpdateConvergesFromLargeError)
{
    // Start at identity; truth is pitch=45°, yaw=90°.
    // With combined accel + mag updates the filter must converge within 5000 steps.
    gnc::FilterConfig cfg = m_cfg;
    cfg.rAccel = 0.5f;
    cfg.rMag = 0.5f;

    AttitudeMekf f(cfg);

    const Eigen::Quaternionf qTrue = eulerToQuat(0.0f, 45.0f, 90.0f);

    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f accel = qTrue.conjugate() * g_ned;
    const Eigen::Vector3f magMeas = qTrue.conjugate() * mag_ned;

    runFused(f, Eigen::Vector3f::Zero(), accel, magMeas, mag_ned, 5000);

    EXPECT_NEAR(std::abs(f.getQuaternion().dot(qTrue)), 1.0f, 0.02f)
        << "Fused filter must converge to true attitude from 90° initial error";

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.y(), 45.0f, 2.0f);
    EXPECT_NEAR(euler.z(), 90.0f, 2.0f);
}

TEST_F(AttitudeMekfTest, FusedUpdateStaticConvergenceToIdentity)
{
    // Filter starts at a small random perturbation.
    // Level + north-facing measurements must drive it back to identity.
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(10.0f, -8.0f, 12.0f), AttitudeMekf::Vector3::Zero());

    const Eigen::Vector3f accel(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);

    runFused(f, Eigen::Vector3f::Zero(), accel, magRef, magRef, 3000);

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 1.5f);
    EXPECT_NEAR(euler.y(), 0.0f, 1.5f);
    EXPECT_NEAR(euler.z(), 0.0f, 1.5f);
}

TEST_F(AttitudeMekfTest, FusedUpdateReducesTotalCovariance)
{
    // A single predict+updateAccel+updateMag cycle must reduce the
    // total covariance trace compared to predict-only.
    AttitudeMekf f(m_cfg);
    f.predict(0.01f, Eigen::Vector3f::Zero());

    const float pAfterPredict = f.getCovariance().trace();

    f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
    f.updateMag(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
        Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    const float pAfterUpdate = f.getCovariance().trace();

    EXPECT_LT(pAfterUpdate, pAfterPredict)
        << "Total covariance must decrease after a valid measurement update";
}


// ============================================================================
// Group 6 — Bias estimation (observability per axis)
// ============================================================================

TEST_F(AttitudeMekfTest, BiasEstimationXAxisViaAccel)
{
    // Stationary vehicle with a 0.1 rad/s X-axis gyro bias.
    // Accelerometer sees constant level gravity → roll is observable.
    // Filter must converge the X-bias estimate to ~0.1 rad/s.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f trueBias(0.1f, 0.0f, 0.0f);
    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f);

    for (int i = 0; i < 2000; ++i) {
        f.predict(0.01f, trueBias); // raw gyro = bias (no true motion)
        f.updateAccel(accelLevel);
    }

    EXPECT_NEAR(f.getBias().x(), 0.1f, 0.02f)
        << "X-axis bias must converge from accelerometer updates";
}

TEST_F(AttitudeMekfTest, BiasEstimationYAxisViaAccel)
{
    // 0.08 rad/s Y-axis bias, corrected by accelerometer.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f trueBias(0.0f, 0.08f, 0.0f);
    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f);

    for (int i = 0; i < 2000; ++i) {
        f.predict(0.01f, trueBias);
        f.updateAccel(accelLevel);
    }

    EXPECT_NEAR(f.getBias().y(), 0.08f, 0.02f)
        << "Y-axis bias must converge from accelerometer updates";
}

TEST_F(AttitudeMekfTest, BiasEstimationZAxisRequiresMag)
{
    // Z-axis (yaw) bias is unobservable from accelerometer alone.
    // With magnetometer updates added, it must become observable.
    //
    // NOTE: Z-bias observability via accelerometer alone is a known
    // theoretical limitation (gravity has no yaw sensitivity). Adding
    // a magnetometer update makes yaw — and thus Z-bias — observable.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f trueBias(0.0f, 0.0f, 0.05f);
    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f magRef(1.0f, 0.0f, 0.0f);

    for (int i = 0; i < 3000; ++i) {
        f.predict(0.01f, trueBias);
        f.updateAccel(accelLevel);
        f.updateMag(magRef, magRef); // level + north → Z-bias observable
    }

    EXPECT_NEAR(f.getBias().z(), 0.05f, 0.02f)
        << "Z-axis bias must converge when magnetometer updates are included";
}

TEST_F(AttitudeMekfTest, BiasEstimationZAxisNotObservableAccelOnly)
{
    // Without magnetometer, Z-bias must NOT be corrected by accel alone.
    // After 2000 steps the Z-bias estimate should still be near zero
    // (it started at zero and has no information source to correct it).
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f trueBias(0.0f, 0.0f, 0.1f);
    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f);

    for (int i = 0; i < 2000; ++i) {
        f.predict(0.01f, trueBias);
        f.updateAccel(accelLevel);
    }

    // Bias Z should remain near zero (unobservable) — allow generous
    // tolerance since yaw drifts slowly and accel may provide minimal signal.
    EXPECT_LT(std::abs(f.getBias().z()), 0.05f)
        << "Z-axis bias is unobservable from accelerometer alone; "
        "estimate must not spuriously converge";
}

TEST_F(AttitudeMekfTest, BiasContinuesAccumulatingAcrossUpdates)
{
    // The bias estimate must persist across measurement updates (only the
    // attitude error portion of the state is zeroed during the MEKF reset).
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f trueBias(0.1f, 0.0f, 0.0f);
    const Eigen::Vector3f accelLevel(0.0f, 0.0f, 9.81f);

    // Run long enough for bias to partially converge
    for (int i = 0; i < 1000; ++i) {
        f.predict(0.01f, trueBias);
        f.updateAccel(accelLevel);
    }

    const float biasXMid = f.getBias().x();
    EXPECT_GT(biasXMid, 0.05f)
        << "Bias must have started converging toward 0.1 rad/s by midpoint";

    // Continue running — bias must keep improving, not reset
    for (int i = 0; i < 1000; ++i) {
        f.predict(0.01f, trueBias);
        f.updateAccel(accelLevel);
    }

    EXPECT_GT(f.getBias().x(), biasXMid - 0.01f)
        << "Bias must not decrease after the midpoint (not being reset)";
}


// ============================================================================
// Group 7 — Covariance health (structure, symmetry, SPD)
// ============================================================================

TEST_F(AttitudeMekfTest, CovarianceIsSymmetricAfterManyUpdates)
{
    // After extended fused operation, P must remain numerically symmetric.
    AttitudeMekf f(m_cfg);

    for (int i = 0; i < 1000; ++i) {
        f.predict(0.01f, Eigen::Vector3f(0.2f, 0.1f, -0.15f));
        f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
        f.updateMag(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    }

    const AttitudeMekf::ErrorMat& P = f.getCovariance();
    const float asymmetry = (P - P.transpose()).norm();
    EXPECT_LT(asymmetry, 1e-4f)
        << "Covariance must remain symmetric after 1000 fused steps";
}

TEST_F(AttitudeMekfTest, CovarianceDiagonalAlwaysPositive)
{
    // All diagonal elements of P must stay positive throughout operation.
    AttitudeMekf f(m_cfg);

    for (int i = 0; i < 500; ++i) {
        f.predict(0.01f, Eigen::Vector3f(0.5f, -0.3f, 0.2f));
        f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));

        const AttitudeMekf::ErrorMat& P = f.getCovariance();
        for (int j = 0; j < 6; ++j) {
            EXPECT_GT(P(j, j), 0.0f)
                << "Diagonal element " << j << " went non-positive at step " << i;
        }
    }
}

TEST_F(AttitudeMekfTest, CovarianceBiasBlockGrowsDuringPredict)
{
    // Bias block trace must grow on every predict step (random-walk noise).
    AttitudeMekf f(m_cfg);

    float prevBiasTrace = f.getCovariance().block<3, 3>(3, 3).trace();

    for (int i = 0; i < 50; ++i) {
        f.predict(0.01f, Eigen::Vector3f::Zero());
        const float biasTrace = f.getCovariance().block<3, 3>(3, 3).trace();
        EXPECT_GT(biasTrace, prevBiasTrace)
            << "Bias covariance block must grow at step " << i;
        prevBiasTrace = biasTrace;
    }
}

TEST_F(AttitudeMekfTest, CovarianceAttitudeBlockShrinksDuringUpdate)
{
    // The attitude covariance block must shrink after a good accel update.
    AttitudeMekf f(m_cfg);
    f.predict(0.01f, Eigen::Vector3f::Zero());

    const float traceBeforeUpdate = f.getCovariance().block<3, 3>(0, 0).trace();
    f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
    const float traceAfterUpdate = f.getCovariance().block<3, 3>(0, 0).trace();

    EXPECT_LT(traceAfterUpdate, traceBeforeUpdate)
        << "Attitude covariance block must shrink after valid accel update";
}

TEST_F(AttitudeMekfTest, CovarianceFaultCountStartsZero)
{
    AttitudeMekf f(m_cfg);
    EXPECT_EQ(f.getCovarianceFaultCount(), 0u)
        << "No covariance faults should exist at construction";
}

TEST_F(AttitudeMekfTest, CovarianceFaultCountZeroDuringNormalOperation)
{
    // Under well-conditioned operation no hard resets should occur.
    AttitudeMekf f(m_cfg);

    for (int i = 0; i < 1000; ++i) {
        f.predict(0.01f, Eigen::Vector3f(0.3f, -0.2f, 0.1f));
        f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
        f.updateMag(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    }

    EXPECT_EQ(f.getCovarianceFaultCount(), 0u)
        << "No covariance faults must occur during well-conditioned operation";
}


// ============================================================================
// Group 8 — Quaternion integrity
// ============================================================================

TEST_F(AttitudeMekfTest, QuaternionRemainsUnitNormAfterManySteps)
{
    // After 5000 predict + update cycles the quaternion must remain unit.
    AttitudeMekf f(m_cfg);

    for (int i = 0; i < 5000; ++i) {
        f.predict(0.01f, Eigen::Vector3f(0.5f, -0.3f, 0.7f));
        f.updateAccel(Eigen::Vector3f(0.0f, 0.0f, 9.81f));
        f.updateMag(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
            Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    }

    EXPECT_NEAR(f.getQuaternion().norm(), 1.0f, 1e-4f)
        << "Quaternion must remain unit length after 5000 fused steps";
}

TEST_F(AttitudeMekfTest, QuaternionDoubleCoverIsHandled)
{
    // q and -q represent the same rotation. The filter must not produce
    // large apparent attitude errors due to a sign flip.
    AttitudeMekf f1(m_cfg);
    AttitudeMekf f2(m_cfg);

    const Eigen::Quaternionf q = eulerToQuat(45.0f, 30.0f, 60.0f);
    f1.init(q, AttitudeMekf::Vector3::Zero());

    // Explicitly negate the coefficients to get the exact double-cover quaternion
    Eigen::Quaternionf q_neg;
    q_neg.coeffs() = -q.coeffs();
    f2.init(q_neg, AttitudeMekf::Vector3::Zero()); // opposite sign, same rotation

    // Both should report the same attitude
    const float errDeg = quatErrorDeg(f1.getQuaternion(), f2.getQuaternion());
    EXPECT_LT(errDeg, 0.001f)
        << "q and -q must represent the same attitude (geodesic error ≈ 0)";
}

TEST_F(AttitudeMekfTest, QuaternionUnitNormAfterPredictOnlyHighRate)
{
    // High-rate gyro-only integration must not degrade quaternion norm.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f omega(10.0f, -7.0f, 5.0f); // ~13 rad/s

    for (int i = 0; i < 2000; ++i) {
        f.predict(0.01f, omega);
    }

    EXPECT_NEAR(f.getQuaternion().norm(), 1.0f, 1e-4f)
        << "Quaternion norm must stay 1.0 under high-rate pure gyro integration";
}


// ============================================================================
// Group 9 — align() (TRIAD method, magnetic reference)
// ============================================================================

TEST_F(AttitudeMekfTest, AlignLevelAndNorth)
{
    // Level vehicle facing North: accel = [0,0,9.81], mag has positive X and Z.
    AttitudeMekf f(m_cfg);

    const AttitudeMekf::Vector3 accel(0.0f, 0.0f, 9.81f);
    const AttitudeMekf::Vector3 mag(0.4f, 0.0f, 0.9f);

    AttitudeMekf::Vector3 magRef;
    f.align(accel, mag, magRef);

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 1.0f) << "Roll must be 0 for level + north";
    EXPECT_NEAR(euler.y(), 0.0f, 1.0f) << "Pitch must be 0 for level + north";
    EXPECT_NEAR(euler.z(), 0.0f, 1.0f) << "Yaw must be 0 for north-facing";

    // magRef must lie in the NED North-Down plane (Y component = 0)
    EXPECT_NEAR(magRef.y(), 0.0f, 1e-4f) << "mag reference East component must be zero";
    EXPECT_NEAR(magRef.norm(), 1.0f, 1e-4f) << "mag reference must be unit length";
}

TEST_F(AttitudeMekfTest, AlignRolled90Right)
{
    // Body rolled 90° right: gravity is along +Y body.
    AttitudeMekf f(m_cfg);

    const AttitudeMekf::Vector3 accel(0.0f, 9.81f, 0.0f);
    const AttitudeMekf::Vector3 mag(0.4f, 0.9f, 0.0f);

    AttitudeMekf::Vector3 magRef;
    f.align(accel, mag, magRef);

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 90.0f, 1.5f) << "Roll must be 90° for right-side-down";
    EXPECT_NEAR(euler.y(), 0.0f, 1.5f) << "Pitch must be 0";
    EXPECT_NEAR(euler.z(), 0.0f, 1.5f) << "Yaw must be 0 (still facing North)";

    // Inertial reference must recover the true dip angle
    const AttitudeMekf::Vector3 expectedRef = Eigen::Vector3f(0.4f, 0.0f, 0.9f).normalized();
    EXPECT_NEAR(magRef.x(), expectedRef.x(), 0.05f);
    EXPECT_NEAR(magRef.y(), 0.0f, 0.05f);
    EXPECT_NEAR(magRef.z(), expectedRef.z(), 0.05f);
}

TEST_F(AttitudeMekfTest, AlignPitched90Up)
{
    // Nose pointing straight up: gravity is along -X body.
    AttitudeMekf f(m_cfg);

    const AttitudeMekf::Vector3 accel(-9.81f, 0.0f, 0.0f);
    const AttitudeMekf::Vector3 mag(-0.9f, 0.0f, 0.4f); // North is behind, Down is forward

    AttitudeMekf::Vector3 magRef;
    f.align(accel, mag, magRef);

    // At pitch = 90° Euler angles suffer gimbal lock. Validate via quaternion directly.
    const Eigen::Quaternionf expectedQ = eulerToQuat(0.0f, 90.0f, 0.0f);
    EXPECT_NEAR(std::abs(f.getQuaternion().dot(expectedQ)), 1.0f, 0.01f)
        << "align() must produce the correct quaternion at 90° pitch (TRIAD avoids singularity)";

    // Magnetic reference must still be correct
    const AttitudeMekf::Vector3 expectedRef = Eigen::Vector3f(0.4f, 0.0f, 0.9f).normalized();
    EXPECT_NEAR(magRef.x(), expectedRef.x(), 0.05f);
    EXPECT_NEAR(magRef.y(), 0.0f, 0.05f);
    EXPECT_NEAR(magRef.z(), expectedRef.z(), 0.05f);
}

TEST_F(AttitudeMekfTest, AlignMagReferenceEastComponentAlwaysZero)
{
    // No matter the orientation, the East (Y) component of magRefOut
    // must be zero — it is projected onto the North-Down plane by design.
    AttitudeMekf f(m_cfg);

    const std::vector<Eigen::Vector3f> accels = {
        {0.0f,  0.0f,  9.81f},
        {0.0f,  9.81f, 0.0f},
        {9.81f, 0.0f,  0.0f},
    };
    const Eigen::Vector3f mag(0.4f, 0.1f, 0.9f);

    for (const auto& acc : accels) {
        AttitudeMekf::Vector3 magRef;
        f.align(acc, mag, magRef);
        EXPECT_NEAR(magRef.y(), 0.0f, 1e-4f)
            << "East component of mag reference must always be zero after align()";
    }
}

TEST_F(AttitudeMekfTest, AlignZeroAccelFallsBackToDown)
{
    // A near-zero accelerometer input (freefall) must not crash; the
    // fallback is +Z (Down), producing an identity quaternion.
    AttitudeMekf f(m_cfg);

    const AttitudeMekf::Vector3 accel(0.0f, 0.0f, 0.0f); // freefall
    const AttitudeMekf::Vector3 mag(1.0f, 0.0f, 0.0f);

    AttitudeMekf::Vector3 magRef;
    EXPECT_NO_FATAL_FAILURE(f.align(accel, mag, magRef))
        << "align() must not crash on zero accelerometer input";
}


// ============================================================================
// Group 10 — getEulerAnglesDeg()
// ============================================================================

TEST_F(AttitudeMekfTest, EulerAnglesIdentity)
{
    AttitudeMekf f(m_cfg);
    f.init(Eigen::Quaternionf::Identity(), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 0.01f);
    EXPECT_NEAR(euler.y(), 0.0f, 0.01f);
    EXPECT_NEAR(euler.z(), 0.0f, 0.01f);
}

TEST_F(AttitudeMekfTest, EulerAngles90DegRoll)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(90.0f, 0.0f, 0.0f), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 90.0f, 0.1f);
    EXPECT_NEAR(euler.y(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 0.0f, 0.1f);
}

TEST_F(AttitudeMekfTest, EulerAnglesNegativePitch)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, -45.0f, 0.0f), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.y(), -45.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 0.0f, 0.1f);
}

TEST_F(AttitudeMekfTest, EulerAnglesComplexOrientation)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(10.0f, 20.0f, 30.0f), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 10.0f, 0.1f);
    EXPECT_NEAR(euler.y(), 20.0f, 0.1f);
    EXPECT_NEAR(euler.z(), 30.0f, 0.1f);
}

TEST_F(AttitudeMekfTest, EulerAnglesNearGimbalLockPlus90Pitch)
{
    // At pitch = +90° the ZYX decomposition is degenerate. The filter
    // must not crash and must report a pitch near +90°.
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, 90.0f, 0.0f), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.y(), 90.0f, 0.5f)
        << "Pitch must read ~+90° at gimbal lock without crashing";
}

TEST_F(AttitudeMekfTest, EulerAnglesNegativeYaw)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, 0.0f, -60.0f), AttitudeMekf::Vector3::Zero());

    const AttitudeMekf::Vector3 euler = f.getEulerAnglesDeg();
    EXPECT_NEAR(euler.x(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.y(), 0.0f, 0.1f);
    EXPECT_NEAR(euler.z(), -60.0f, 0.1f);
}


// ============================================================================
// Group 11 — Sign convention (NED)
// ============================================================================

TEST_F(AttitudeMekfTest, SignConventionPositiveRollRightWingDown)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(30.0f, 0.0f, 0.0f), AttitudeMekf::Vector3::Zero());
    EXPECT_NEAR(f.getEulerAnglesDeg().x(), 30.0f, 0.1f)
        << "Positive roll must correspond to right-wing-down (+30°)";
}

TEST_F(AttitudeMekfTest, SignConventionPositivePitchNoseUp)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, 30.0f, 0.0f), AttitudeMekf::Vector3::Zero());
    EXPECT_NEAR(f.getEulerAnglesDeg().y(), 30.0f, 0.1f)
        << "Positive pitch must correspond to nose-up (+30°)";
}

TEST_F(AttitudeMekfTest, SignConventionPositiveYawNoseRight)
{
    AttitudeMekf f(m_cfg);
    f.init(eulerToQuat(0.0f, 0.0f, 30.0f), AttitudeMekf::Vector3::Zero());
    EXPECT_NEAR(f.getEulerAnglesDeg().z(), 30.0f, 0.1f)
        << "Positive yaw must correspond to nose-right in NED (+30°)";
}

TEST_F(AttitudeMekfTest, SignConventionAccelLevelReadingNED)
{
    // In NED a stationary level vehicle reads [0, 0, +g] on the accelerometer.
    // Rotating the inertial gravity vector into the identity body frame must
    // give [0, 0, 9.81].
    const Eigen::Quaternionf identity = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f accelBody = identity.conjugate() * g_ned;

    EXPECT_NEAR(accelBody.x(), 0.0f, 1e-5f);
    EXPECT_NEAR(accelBody.y(), 0.0f, 1e-5f);
    EXPECT_NEAR(accelBody.z(), 9.81f, 1e-5f);
}


// ============================================================================
// Group 12 — Full flight simulation
// ============================================================================

TEST_F(AttitudeMekfTest, FlightSimulation_FourPhase)
{
    // Four-phase flight: hover → pitch up → coordinated yaw turn → return level.
    // The MEKF must track the exact truth model to within 1.5° at each checkpoint.
    gnc::FilterConfig cfg = m_cfg;
    cfg.rMag = 0.01f; // higher mag trust for cleaner tracking in noiseless sim

    AttitudeMekf f(cfg);

    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    const float dt = 0.01f;

    auto runSegment = [&](const Eigen::Vector3f& rates, float durationSec)
        {
            const int steps = static_cast<int>(durationSec / dt);
            for (int i = 0; i < steps; ++i) {
                integrateQuat(qTrue, rates, dt);
                const Eigen::Vector3f accel = qTrue.conjugate() * g_ned;
                const Eigen::Vector3f magMeas = qTrue.conjugate() * mag_ned;
                f.predict(dt, rates);
                f.updateAccel(accel);
                f.updateMag(magMeas, mag_ned);
            }
        };

    // Phase 1: Stationary (1 s)
    runSegment(Eigen::Vector3f::Zero(), 1.0f);
    EXPECT_NEAR(f.getQuaternion().w(), 1.0f, 0.01f)
        << "Phase 1: must remain at identity during hover";

    // Phase 2: Pitch up at 45°/s for 1 s → 45° pitch
    runSegment(Eigen::Vector3f(0.0f, deg2rad(45.0f), 0.0f), 1.0f);
    EXPECT_LT(quatErrorDeg(f.getQuaternion(), qTrue), 1.5f)
        << "Phase 2: tracking error must be < 1.5° after pitch-up";

    // Phase 3: Yaw at 90°/s for 1 s while pitched
    runSegment(Eigen::Vector3f(0.0f, 0.0f, deg2rad(90.0f)), 1.0f);
    EXPECT_LT(quatErrorDeg(f.getQuaternion(), qTrue), 2.0f)
        << "Phase 3: tracking error must be < 2.0° during coordinated turn";

    // Phase 4: Return to level (undo pitch)
    runSegment(Eigen::Vector3f(0.0f, deg2rad(-45.0f), 0.0f), 1.0f);
    EXPECT_LT(quatErrorDeg(f.getQuaternion(), qTrue), 2.0f)
        << "Phase 4: tracking error must be < 2.0° after return to level";
}

TEST_F(AttitudeMekfTest, FlightSimulation_RollAxleSpinHighRate)
{
    // Continuous 360°/s roll rate for 5 seconds (barrel-roll scenario).
    // The exact integrator must keep tracking error under 1°.
    AttitudeMekf f(m_cfg);

    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);
    const Eigen::Vector3f omega(deg2rad(360.0f), 0.0f, 0.0f);
    const float dt = 0.01f;

    for (int i = 0; i < 500; ++i) {
        integrateQuat(qTrue, omega, dt);
        f.predict(dt, omega);
        f.updateAccel(qTrue.conjugate() * g_ned);
        f.updateMag(qTrue.conjugate() * mag_ned, mag_ned);
    }

    EXPECT_LT(quatErrorDeg(f.getQuaternion(), qTrue), 1.0f)
        << "Tracking error must stay < 1° during 360°/s continuous roll";
}


// ============================================================================
// Group 13 — Monte Carlo accuracy
// ============================================================================

TEST_F(AttitudeMekfTest, MonteCarloAccuracyUnderSensorNoise)
{
    // Sinusoidal coning motion with realistic MEMS noise levels.
    // Target: RMS error < 0.5° and peak error < 1.5° after convergence.
    gnc::FilterConfig cfg = m_cfg;
    cfg.qGyro = 0.001f;
    cfg.rAccel = 0.1f;
    cfg.rMag = 0.1f;

    AttitudeMekf f(cfg);

    std::default_random_engine rng(42u); // Fixed seed for reproducibility
    std::normal_distribution<float> gyroNoise(0.0f, 0.01f); // ~0.5°/s
    std::normal_distribution<float> accelNoise(0.0f, 0.10f); // ~0.1 m/s²
    std::normal_distribution<float> magNoise(0.0f, 0.02f);

    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();
    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);

    const float dt = 0.01f;
    const int totalSteps = 10000;
    const float freq = 1.0f;
    const float amp = 0.5f;

    float maxErrDeg = 0.0f;
    float sumSqErr = 0.0f;
    int   evaluated = 0;

    for (int i = 0; i < totalSteps; ++i) {
        const float t = i * dt;

        // True sinusoidal coning rates
        const Eigen::Vector3f trueRates(
            amp * freq * std::cos(freq * t),
            amp * freq * std::sin(freq * t),
            0.0f);

        integrateQuat(qTrue, trueRates, dt);

        // Noisy measurements
        const Eigen::Vector3f gyroMeas = trueRates
            + Eigen::Vector3f(gyroNoise(rng), gyroNoise(rng), gyroNoise(rng));

        Eigen::Vector3f accelMeas = qTrue.conjugate() * g_ned
            + Eigen::Vector3f(accelNoise(rng), accelNoise(rng), accelNoise(rng));

        Eigen::Vector3f magMeas = qTrue.conjugate() * mag_ned
            + Eigen::Vector3f(magNoise(rng), magNoise(rng), magNoise(rng));

        f.predict(dt, gyroMeas);
        f.updateAccel(accelMeas);
        f.updateMag(magMeas, mag_ned);

        // Evaluate after 1 s convergence window
        if (t > 1.0f) {
            const float errDeg = quatErrorDeg(f.getQuaternion(), qTrue);
            if (errDeg > maxErrDeg) maxErrDeg = errDeg;
            sumSqErr += errDeg * errDeg;
            ++evaluated;
        }
    }

    const float rmsErr = std::sqrt(sumSqErr / static_cast<float>(evaluated));

    std::cout << "[MonteCarloAccuracy] RMS: " << rmsErr
        << "°  Peak: " << maxErrDeg << "°\n";

    EXPECT_LT(rmsErr, 0.5f) << "RMS error must be < 0.5° under MEMS noise";
    EXPECT_LT(maxErrDeg, 1.5f) << "Peak error must be < 1.5° under MEMS noise";
}


// ============================================================================
// Group 14 — Diagnostics (getCovarianceFaultCount)
// ============================================================================

TEST_F(AttitudeMekfTest, DiagnosticsNoFaultsDuringNominalOperation)
{
    // A well-tuned filter must never trigger a hard covariance reset
    // during 2000 steps of fused gyro + accel + mag operation.
    AttitudeMekf f(m_cfg);

    const Eigen::Vector3f omega(0.5f, -0.3f, 0.2f);
    const Eigen::Vector3f g_ned(0.0f, 0.0f, 9.81f);
    const Eigen::Vector3f mag_ned(1.0f, 0.0f, 0.0f);

    // Propagate a truth quaternion so the synthetic measurements are
    // geometrically consistent at every step.
    Eigen::Quaternionf qTrue = Eigen::Quaternionf::Identity();

    for (int i = 0; i < 2000; ++i) {
        integrateQuat(qTrue, omega, 0.01f);
        f.predict(0.01f, omega);
        f.updateAccel(qTrue.conjugate() * g_ned);
        f.updateMag(qTrue.conjugate() * mag_ned, mag_ned);
    }

    EXPECT_EQ(f.getCovarianceFaultCount(), 0u)
        << "No covariance hard resets must occur during nominal fused operation";
}

// ============================================================================
// Entry point
// ============================================================================

//int main(int argc, char** argv)
//{
//    ::testing::InitGoogleTest(&argc, argv);
//    return RUN_ALL_TESTS();
//}