/**
 * @file test_attitude_controller.cpp
 * @brief Unit tests for AttitudeController using GoogleTest.
 *
 * Validates the cascaded fixed-wing attitude controller across the following
 * areas:
 *   - Default construction / initial state
 *   - Setpoint clamping (flight-envelope limits)
 *   - dt / sensor-input guard paths (fault hold)
 *   - Outer-loop proportional response (angle → rate demand)
 *   - Coordinated-turn yaw rate generation
 *   - Airspeed scaling (dynamic-pressure inverse scaler)
 *   - Anti-windup clamp inversion guard
 *   - Actuator output saturation to [-1, 1]
 *   - Integrator convergence to zero steady-state error
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "attitude_controller.h"

// Standard loop period used throughout the suite [s]
static constexpr float DT = 0.01f;

// Gravity constant — must match the value in the controller
static constexpr float GRAVITY_MS2 = 9.80665f;

// Nominal tuning airspeed [m/s] — must match NOMINAL_AIRSPEED_FOR_TUNING
static constexpr float NOMINAL_AIRSPEED = 20.0f;

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

/** Returns a fully populated FullSensorData representing straight-and-level
 *  flight at the supplied airspeed with all angles and rates at zero. */
static FullSensorData makeLevelFlight(float airspeedMs = NOMINAL_AIRSPEED)
{
    FullSensorData s{};
    s.rollDeg         = 0.0f;
    s.pitchDeg        = 0.0f;
    s.yawDeg          = 0.0f;
    s.rollRateDps     = 0.0f;
    s.pitchRateDps    = 0.0f;
    s.yawRateDps      = 0.0f;
    s.trueAirspeedMs  = airspeedMs;
    return s;
}

// ----------------------------------------------------------------------------
// Test Fixture
// ----------------------------------------------------------------------------
class AttitudeControllerTest : public ::testing::Test {
protected:
    AttitudeController controller;

    void SetUp() override
    {
        // Controller is default-constructed; setpoints start at 0, 0.
        // Each test configures the setpoints it needs explicitly.
    }
};

// ============================================================================
// Construction / Initial State
// ============================================================================

TEST_F(AttitudeControllerTest, DefaultSetpointsAreZero)
{
    EXPECT_FLOAT_EQ(controller.getTargetRollDeg(),  0.0f);
    EXPECT_FLOAT_EQ(controller.getTargetPitchDeg(), 0.0f);
}

TEST_F(AttitudeControllerTest, LevelFlightZeroSetpointProducesZeroOutput)
{
    // With setpoints and measured attitude both at zero the controller should
    // demand no deflection on any surface.
    controller.setSetpoints(0.0f, 0.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);

    EXPECT_NEAR(out.aileron,  0.0f, 1e-4f);
    EXPECT_NEAR(out.elevator, 0.0f, 1e-4f);
    EXPECT_NEAR(out.rudder,   0.0f, 1e-4f);
}

// ============================================================================
// Setpoint Clamping
// ============================================================================

TEST_F(AttitudeControllerTest, RollSetpointClampedToMaxRoll)
{
    // Commands beyond ±70° must be silently saturated to protect the
    // coordinated-turn tan(φ) singularity near ±90°.
    controller.setSetpoints(200.0f, 0.0f);
    EXPECT_NEAR(controller.getTargetRollDeg(), 70.0f, 1e-4f);

    controller.setSetpoints(-200.0f, 0.0f);
    EXPECT_NEAR(controller.getTargetRollDeg(), -70.0f, 1e-4f);
}

TEST_F(AttitudeControllerTest, PitchSetpointClampedToMaxPitch)
{
    // Commands beyond ±45° must be clamped to keep the kinematic transform
    // well-conditioned (cos(θ) > ~0.7).
    controller.setSetpoints(0.0f, 90.0f);
    EXPECT_NEAR(controller.getTargetPitchDeg(), 45.0f, 1e-4f);

    controller.setSetpoints(0.0f, -90.0f);
    EXPECT_NEAR(controller.getTargetPitchDeg(), -45.0f, 1e-4f);
}

TEST_F(AttitudeControllerTest, SetpointsWithinEnvelopeAreStoredExactly)
{
    controller.setSetpoints(30.0f, -20.0f);
    EXPECT_FLOAT_EQ(controller.getTargetRollDeg(),  30.0f);
    EXPECT_FLOAT_EQ(controller.getTargetPitchDeg(), -20.0f);
}

// ============================================================================
// dt Guard — Fault Hold
// ============================================================================

TEST_F(AttitudeControllerTest, ZeroDtReturnsPreviousOutput)
{
    // A good cycle establishes a known output; a subsequent zero-dt call must
    // return that cached value unchanged rather than producing NaN.
    controller.setSetpoints(10.0f, 0.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput good = controller.update(s, DT);

    // Zero dt — scheduler fault simulation
    ActuatorOutput held = controller.update(s, 0.0f);

    EXPECT_FLOAT_EQ(held.aileron,  good.aileron);
    EXPECT_FLOAT_EQ(held.elevator, good.elevator);
    EXPECT_FLOAT_EQ(held.rudder,   good.rudder);
}

TEST_F(AttitudeControllerTest, NegativeDtReturnsPreviousOutput)
{
    controller.setSetpoints(0.0f, 5.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput good = controller.update(s, DT);
    ActuatorOutput held = controller.update(s, -0.1f);

    EXPECT_FLOAT_EQ(held.aileron,  good.aileron);
    EXPECT_FLOAT_EQ(held.elevator, good.elevator);
    EXPECT_FLOAT_EQ(held.rudder,   good.rudder);
}

TEST_F(AttitudeControllerTest, ExcessiveDtReturnsPreviousOutput)
{
    // dt > MAX_DT_S (0.5 s) is considered a scheduler overrun.
    controller.setSetpoints(0.0f, 0.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput good = controller.update(s, DT);
    ActuatorOutput held = controller.update(s, 1.0f);  // 1 s >> MAX_DT_S

    EXPECT_FLOAT_EQ(held.aileron,  good.aileron);
    EXPECT_FLOAT_EQ(held.elevator, good.elevator);
    EXPECT_FLOAT_EQ(held.rudder,   good.rudder);
}

// ============================================================================
// Sensor Guard — Fault Hold on Non-Finite Input
// ============================================================================

TEST_F(AttitudeControllerTest, NanRollAngleReturnsPreviousOutput)
{
    controller.setSetpoints(0.0f, 0.0f);
    FullSensorData good = makeLevelFlight();
    ActuatorOutput goodOut = controller.update(good, DT);

    FullSensorData bad = good;
    bad.rollDeg = std::numeric_limits<float>::quiet_NaN();
    ActuatorOutput held = controller.update(bad, DT);

    EXPECT_FLOAT_EQ(held.aileron,  goodOut.aileron);
    EXPECT_FLOAT_EQ(held.elevator, goodOut.elevator);
    EXPECT_FLOAT_EQ(held.rudder,   goodOut.rudder);
}

TEST_F(AttitudeControllerTest, InfAirspeedReturnsPreviousOutput)
{
    controller.setSetpoints(0.0f, 0.0f);
    FullSensorData good = makeLevelFlight();
    ActuatorOutput goodOut = controller.update(good, DT);

    FullSensorData bad = good;
    bad.trueAirspeedMs = std::numeric_limits<float>::infinity();
    ActuatorOutput held = controller.update(bad, DT);

    EXPECT_FLOAT_EQ(held.aileron,  goodOut.aileron);
    EXPECT_FLOAT_EQ(held.elevator, goodOut.elevator);
    EXPECT_FLOAT_EQ(held.rudder,   goodOut.rudder);
}

// ============================================================================
// Outer-Loop Proportional Response
// ============================================================================

TEST_F(AttitudeControllerTest, PositiveRollErrorProducesPositiveAileronDemand)
{
    // Setpoint > current angle → positive roll rate demand → positive aileron.
    controller.setSetpoints(20.0f, 0.0f);
    FullSensorData s = makeLevelFlight();  // current roll = 0

    ActuatorOutput out = controller.update(s, DT);

    EXPECT_GT(out.aileron, 0.0f);
}

TEST_F(AttitudeControllerTest, NegativeRollErrorProducesNegativeAileronDemand)
{
    controller.setSetpoints(-20.0f, 0.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);

    EXPECT_LT(out.aileron, 0.0f);
}

TEST_F(AttitudeControllerTest, PositivePitchErrorProducesPositiveElevatorDemand)
{
    controller.setSetpoints(0.0f, 15.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);

    EXPECT_GT(out.elevator, 0.0f);
}

TEST_F(AttitudeControllerTest, NegativePitchErrorProducesNegativeElevatorDemand)
{
    controller.setSetpoints(0.0f, -15.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);

    EXPECT_LT(out.elevator, 0.0f);
}

TEST_F(AttitudeControllerTest, LargerAngleErrorProducesLargerDemand)
{
    // The outer loop is purely proportional so a larger error must produce a
    // proportionally larger command.
    FullSensorData s = makeLevelFlight();

    controller.setSetpoints(10.0f, 0.0f);
    float smallCmd = controller.update(s, DT).aileron;

    // Reset controller state for a clean comparison
    AttitudeController fresh;
    fresh.setSetpoints(30.0f, 0.0f);
    float largeCmd = fresh.update(s, DT).aileron;

    EXPECT_GT(largeCmd, smallCmd);
}

// ============================================================================
// Coordinated Turn
// ============================================================================

TEST_F(AttitudeControllerTest, BankedFlightProducesNonZeroRudderDemand)
{
    // A non-zero bank angle at cruise speed should generate a coordinated-turn
    // yaw rate setpoint, which in turn demands rudder.
    controller.setSetpoints(30.0f, 0.0f);
    FullSensorData s = makeLevelFlight();
    s.rollDeg = 30.0f;  // Aircraft already at the commanded bank angle
                         // (no roll rate error) so aileron demand ≈ 0

    ActuatorOutput out = controller.update(s, DT);

    // The yaw rate setpoint = (g/V)*tan(30°) ≠ 0, so rudder must be non-zero.
    EXPECT_NE(out.rudder, 0.0f);
}

TEST_F(AttitudeControllerTest, ZeroAirspeedDisablesCoordinatedTurnYawRate)
{
    // Below the minimum airspeed threshold the coordinated-turn term is gated
    // off to avoid division by near-zero.
    controller.setSetpoints(30.0f, 0.0f);
    FullSensorData s = makeLevelFlight(0.0f);  // zero airspeed
    s.rollDeg = 30.0f;

    ActuatorOutput out = controller.update(s, DT);

    // With no coordinated-turn feed-forward and roll error = 0, rudder ≈ 0.
    EXPECT_NEAR(out.rudder, 0.0f, 1e-4f);
}

// ============================================================================
// Airspeed Scaling
// ============================================================================

TEST_F(AttitudeControllerTest, HigherAirspeedReducesActuatorDemand)
{
    // At airspeeds above the tuning point the dynamic-pressure scaler < 1,
    // so the same angle error must produce a smaller actuator demand.
    controller.setSetpoints(20.0f, 0.0f);

    FullSensorData sNominal = makeLevelFlight(NOMINAL_AIRSPEED);
    FullSensorData sFast    = makeLevelFlight(NOMINAL_AIRSPEED * 2.0f);

    ActuatorOutput outNominal = controller.update(sNominal, DT);

    AttitudeController fresh;
    fresh.setSetpoints(20.0f, 0.0f);
    ActuatorOutput outFast = fresh.update(sFast, DT);

    EXPECT_LT(std::abs(outFast.aileron), std::abs(outNominal.aileron));
}

TEST_F(AttitudeControllerTest, LowerAirspeedIncreasesActuatorDemand)
{
    // At airspeeds below the tuning point the dynamic-pressure scaler > 1,
    // so the same angle error must produce a larger actuator demand.
    controller.setSetpoints(20.0f, 0.0f);

    FullSensorData sNominal = makeLevelFlight(NOMINAL_AIRSPEED);
    FullSensorData sSlow    = makeLevelFlight(NOMINAL_AIRSPEED * 0.5f);

    ActuatorOutput outNominal = controller.update(sNominal, DT);

    AttitudeController fresh;
    fresh.setSetpoints(20.0f, 0.0f);
    ActuatorOutput outSlow = fresh.update(sSlow, DT);

    EXPECT_GT(std::abs(outSlow.aileron), std::abs(outNominal.aileron));
}

// ============================================================================
// Output Saturation
// ============================================================================

TEST_F(AttitudeControllerTest, ActuatorOutputNeverExceedsUnity)
{
    // Even a commanding angle error that would otherwise overflow the actuator
    // range must be clamped to [-1, 1] on every axis.
    controller.setSetpoints(70.0f, 45.0f);  // Maximum clamped setpoints

    FullSensorData s = makeLevelFlight();

    // Run for multiple cycles so the integrator has time to wind up
    ActuatorOutput out{};
    for (int i = 0; i < 200; ++i) {
        out = controller.update(s, DT);
    }

    EXPECT_LE(out.aileron,   1.0f);
    EXPECT_GE(out.aileron,  -1.0f);
    EXPECT_LE(out.elevator,  1.0f);
    EXPECT_GE(out.elevator, -1.0f);
    EXPECT_LE(out.rudder,    1.0f);
    EXPECT_GE(out.rudder,   -1.0f);
}

// ============================================================================
// Steady-State Convergence
// ============================================================================

TEST_F(AttitudeControllerTest, RollErrorDrivesToZeroSteadyState)
{
    // Simulate a closed-loop system: apply the aileron demand as a body roll
    // rate each cycle and integrate.  The integral term of the inner PID
    // should drive the steady-state aileron demand to zero once the angle
    // error is eliminated.

    const float targetRollDeg = 20.0f;
    controller.setSetpoints(targetRollDeg, 0.0f);

    FullSensorData s = makeLevelFlight();

    const int   maxSteps   = 2000;
    const float tolerance  = 0.5f;  // degrees
    bool        converged  = false;

    for (int i = 0; i < maxSteps; ++i) {
        ActuatorOutput out = controller.update(s, DT);

        // Simple plant model: aileron demand drives roll rate at 60 deg/s per
        // unit deflection.  This is intentionally approximate; the test is
        // only checking that the integral removes steady-state error, not that
        // the transient matches a particular aircraft model.
        const float rollRateDps = out.aileron * 60.0f;
        s.rollDeg     += rollRateDps * DT;
        s.rollRateDps  = rollRateDps;

        if (std::abs(s.rollDeg - targetRollDeg) < tolerance) {
            converged = true;
            break;
        }
    }

    EXPECT_TRUE(converged)
        << "Roll did not converge to " << targetRollDeg
        << " deg within " << maxSteps << " steps. "
        << "Final roll: " << s.rollDeg << " deg";
}

TEST_F(AttitudeControllerTest, PitchErrorDrivesToZeroSteadyState)
{
    const float targetPitchDeg = 10.0f;
    controller.setSetpoints(0.0f, targetPitchDeg);

    FullSensorData s = makeLevelFlight();

    const int   maxSteps  = 2000;
    const float tolerance = 0.5f;
    bool        converged = false;

    for (int i = 0; i < maxSteps; ++i) {
        ActuatorOutput out = controller.update(s, DT);

        const float pitchRateDps = out.elevator * 60.0f;
        s.pitchDeg     += pitchRateDps * DT;
        s.pitchRateDps  = pitchRateDps;

        if (std::abs(s.pitchDeg - targetPitchDeg) < tolerance) {
            converged = true;
            break;
        }
    }

    EXPECT_TRUE(converged)
        << "Pitch did not converge to " << targetPitchDeg
        << " deg within " << maxSteps << " steps. "
        << "Final pitch: " << s.pitchDeg << " deg";
}

// ============================================================================
// Sign Convention
// ============================================================================

TEST_F(AttitudeControllerTest, SignConvention_PositiveRollSetpointPositiveAileron)
{
    // Positive roll setpoint (right wing down) must demand positive aileron.
    controller.setSetpoints(30.0f, 0.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);
    EXPECT_GT(out.aileron, 0.0f) << "Positive roll setpoint must produce positive aileron";
}

TEST_F(AttitudeControllerTest, SignConvention_PositivePitchSetpointPositiveElevator)
{
    // Positive pitch setpoint (nose up) must demand positive elevator.
    controller.setSetpoints(0.0f, 20.0f);
    FullSensorData s = makeLevelFlight();

    ActuatorOutput out = controller.update(s, DT);
    EXPECT_GT(out.elevator, 0.0f) << "Positive pitch setpoint must produce positive elevator";
}

// ============================================================================
// Edge Cases and Dynamic Flight Constraints
// ============================================================================

TEST_F(AttitudeControllerTest, EulerSingularityGuard_90DegreeRoll) {
    // Tests the division-by-zero guard in the coordinated turn calculation
    // when the aircraft reaches exactly 90 degrees of bank (gimbal lock state).
    controller.setSetpoints(90.0f, 0.0f);
    FullSensorData s = makeLevelFlight();
    s.rollDeg = 90.0f;
    s.trueAirspeedMs = NOMINAL_AIRSPEED;

    ActuatorOutput out = controller.update(s, DT);

    // The tangent of 90 degrees is undefined. The controller must intercept 
    // the near-zero cosine and force the yaw demand to zero rather than NaN.
    EXPECT_FALSE(std::isnan(out.rudder)) << "Rudder output must not be NaN at 90 degrees roll";
    EXPECT_FALSE(std::isnan(out.aileron));
    EXPECT_FALSE(std::isnan(out.elevator));
}

TEST_F(AttitudeControllerTest, StaticGroundState_ZeroAirspeedDivByZeroGuard) {
    // Ensures the controller safely bypasses airspeed-dependent calculations
    // (dynamic pressure scaling and coordinated turn generation) while stationary.
    controller.setSetpoints(30.0f, 0.0f);
    FullSensorData s = makeLevelFlight();
    s.trueAirspeedMs = 0.0f;
    s.rollDeg = 30.0f; // Simulate sitting banked on uneven ground

    ActuatorOutput out = controller.update(s, DT);

    // Bypassing the coordinated turn logic should prevent the rudder from 
    // slamming hard-over while the aircraft is sitting on the runway.
    EXPECT_FALSE(std::isnan(out.rudder)) << "Output must not be NaN at zero airspeed";
    EXPECT_FALSE(std::isinf(out.aileron)) << "Output must not be infinite at zero airspeed";
}

TEST_F(AttitudeControllerTest, DynamicPressureScaler_BoundsAreClamped) {
    // Proves the dynamic loop gain does not scale infinitely towards zero 
    // or infinity at the absolute extremes of the flight envelope.
    AttitudeController ctrlLow;
    ctrlLow.setSetpoints(1.0f, 0.0f);
    FullSensorData sLow = makeLevelFlight();
    sLow.trueAirspeedMs = 2.0f; // Deep stall, forces upper clamp limit
    float lowSpeedAileron = ctrlLow.update(sLow, DT).aileron;

    AttitudeController ctrlHigh;
    ctrlHigh.setSetpoints(1.0f, 0.0f);
    FullSensorData sHigh = makeLevelFlight();
    sHigh.trueAirspeedMs = 100.0f; // Vertical dive, forces lower clamp limit
    float highSpeedAileron = ctrlHigh.update(sHigh, DT).aileron;

    // The airspeed scaling factor clamps at 10.0 maximum and 0.1 minimum.
    // Given an identical small error, the ratio between the deepest stall 
    // actuator deflection and the fastest dive deflection must be exactly 100.
    EXPECT_NEAR(lowSpeedAileron / highSpeedAileron, 100.0f, 0.01f);
}

TEST_F(AttitudeControllerTest, AntiWindup_DynamicPressureShrinksIntegratorLimits) {
    // Verifies that the internal PID integrator bounds shrink proportionally 
    // when the dynamic pressure scaler is large, guaranteeing the final 
    // output never exceeds 1.0.
    AttitudeController ctrl;
    FullSensorData s = makeLevelFlight();
    s.trueAirspeedMs = 2.0f; // Triggers maximum 10.0x dynamic multiplier
    ctrl.setSetpoints(45.0f, 0.0f); // Induce massive error to force windup

    ActuatorOutput out;
    for (int i = 0; i < 500; ++i) {
        out = ctrl.update(s, DT);
    }

    // Output must be heavily saturated but perfectly clamped to 1.0.
    // The internal integrator should have clamped at 0.1 to account for 
    // the subsequent 10.0 multiplier.
    EXPECT_FLOAT_EQ(out.aileron, 1.0f);

    // Reverse the error slightly. If the internal integrator wound past 0.1, 
    // the output would stay pegged at 1.0. If the dynamic anti-windup succeeded, 
    // the output will detach and drop immediately.
    ctrl.setSetpoints(44.0f, 0.0f);
    s.rollDeg = 45.0f;
    ActuatorOutput outReversed = ctrl.update(s, DT);

    EXPECT_LT(outReversed.aileron, 1.0f) << "Anti-windup failed to dynamically restrict integrator limits";
}

TEST_F(AttitudeControllerTest, KinematicCrossCoupling_ClimbingTurn) {
    // Validates the Euler-to-Body-Rate transformation matrix by inducing a 
    // condition where pure yaw (coordinated turn) leaks into the pitch and roll axes.
    AttitudeController ctrl;

    // Set targets exactly equal to current state to nullify all proportional angle error
    ctrl.setSetpoints(45.0f, 30.0f);
    FullSensorData s = makeLevelFlight();
    s.rollDeg = 45.0f;
    s.pitchDeg = 30.0f;
    s.trueAirspeedMs = GRAVITY_MS2; // Forces the g/V factor to exactly 1.0

    ActuatorOutput out = ctrl.update(s, DT);

    // Even with zero angle error, the physics of a coordinated climbing turn 
    // demand a body-roll and body-pitch rate to maintain the steady 30 degree pitch attitude.
    // According to r = -thetaDot*sin(phi) + psiDot*cos(theta)*cos(phi)
    EXPECT_LT(out.aileron, 0.0f) << "Climbing right turn must demand negative body roll rate";
    EXPECT_GT(out.elevator, 0.0f) << "Climbing right turn must demand positive body pitch rate";
    EXPECT_GT(out.rudder, 0.0f) << "Climbing right turn must demand positive body yaw rate";
}

// ============================================================================
// Entry Point
// ============================================================================

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}