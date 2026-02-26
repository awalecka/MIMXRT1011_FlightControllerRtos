/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for PIDController class using GoogleTest.
 *
 * Validates proportional, integral, and derivative responses, exponential 
 * moving average (EMA) filtering, saturation clamping, and conditional 
 * integration (anti-windup) logic.
 */

#include <gtest/gtest.h>
#include <cmath>
#include "pid_controller.h"

// ----------------------------------------------------------------------------
// Test Fixture
// ----------------------------------------------------------------------------
class PidControllerTest : public ::testing::Test {
protected:
    // Standard limits used for most tests
    const float LIMIT_MIN = -1.0f;
    const float LIMIT_MAX = 1.0f;
    const float DT_DEFAULT = 0.01f; // 100 Hz

    void SetUp() override {
        // No complex setup required for standard tests
    }
    
    // Helper to evaluate a step response over a given number of iterations
    float runSteps(PIDController& pid, float setpoint, float current, float dt, int steps) {
        float out = 0.0f;
        for (int i = 0; i < steps; ++i) {
            out = pid.calculate(setpoint, current, dt, LIMIT_MIN, LIMIT_MAX);
        }
        return out;
    }
};

// ----------------------------------------------------------------------------
// Initialization & Basic Response Tests
// ----------------------------------------------------------------------------

TEST_F(PidControllerTest, ZeroErrorYieldsZeroOutput) {
    PIDController pid(1.0f, 1.0f, 1.0f, 1.0f);
    
    float out = pid.calculate(0.0f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

TEST_F(PidControllerTest, ZeroDtReturnsZeroSafely) {
    PIDController pid(1.0f, 0.0f, 1.0f, 1.0f);
    
    // A dt of 0.0f would normally cause a divide-by-zero in the derivative term.
    // The controller should trap this and return 0.0f safely.
    float out = pid.calculate(1.0f, 0.0f, 0.0f, LIMIT_MIN, LIMIT_MAX);
    EXPECT_FLOAT_EQ(out, 0.0f);
}

// ----------------------------------------------------------------------------
// Proportional, Integral, Derivative (PID) Isolation Tests
// ----------------------------------------------------------------------------

TEST_F(PidControllerTest, PureProportionalResponse) {
    // Kp = 2.0, Ki = 0, Kd = 0
    PIDController pid(2.0f, 0.0f, 0.0f, 1.0f);
    
    // Error = Setpoint(5.0) - Current(2.0) = 3.0
    // Expected Out = 3.0 * 2.0 = 6.0
    float out = pid.calculate(5.0f, 2.0f, DT_DEFAULT, -10.0f, 10.0f);
    EXPECT_FLOAT_EQ(out, 6.0f);
}

TEST_F(PidControllerTest, PureIntegralAccumulation) {
    // Kp = 0, Ki = 2.0, Kd = 0
    PIDController pid(0.0f, 2.0f, 0.0f, 1.0f);
    
    // Error = 1.0. dt = 0.01. 
    // Integrator adds (1.0 * 0.01) = 0.01 per step.
    // Over 50 steps, integral = 0.5.
    // Output = Ki * integral = 2.0 * 0.5 = 1.0.
    float out = runSteps(pid, 1.0f, 0.0f, DT_DEFAULT, 50);
    EXPECT_NEAR(out, 1.0f, 1e-5f);
}

TEST_F(PidControllerTest, PureDerivativeUnfiltered) {
    // Kp = 0, Ki = 0, Kd = 0.5, Alpha = 1.0 (No filtering)
    PIDController pid(0.0f, 0.0f, 0.5f, 1.0f);
    
    // Step 1: Initialize prevError to 0 internally. Output will jump due to step input.
    pid.calculate(0.0f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX); 
    
    // Step 2: Introduce an error of 1.0 over dt = 0.1s. 
    // Rate of change (de/dt) = (1.0 - 0.0) / 0.1 = 10.0
    // Expected Out = Kd * de/dt = 0.5 * 10.0 = 5.0
    float out = pid.calculate(1.0f, 0.0f, 0.1f, -10.0f, 10.0f);
    EXPECT_FLOAT_EQ(out, 5.0f);
}

// ----------------------------------------------------------------------------
// Filter & Math Validation Tests
// ----------------------------------------------------------------------------

TEST_F(PidControllerTest, DerivativeEmaFiltering) {
    // Kd = 1.0, Alpha = 0.5 (Heavy filtering)
    PIDController pid(0.0f, 0.0f, 1.0f, 0.5f);
    
    // Step 1: Error goes from 0 to 1 over dt=1.0. Raw derivative = 1.0.
    // Filtered = (0.5 * 1.0) + (0.5 * 0.0) = 0.5
    float out1 = pid.calculate(1.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    EXPECT_FLOAT_EQ(out1, 0.5f);
    
    // Step 2: Error goes from 1 to 2 over dt=1.0. Raw derivative = 1.0.
    // Filtered = (0.5 * 1.0) + (0.5 * 0.5) = 0.75
    float out2 = pid.calculate(2.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    EXPECT_FLOAT_EQ(out2, 0.75f);
}

// ----------------------------------------------------------------------------
// Saturation and Anti-Windup Tests
// ----------------------------------------------------------------------------

TEST_F(PidControllerTest, OutputIsClampedToLimits) {
    PIDController pid(10.0f, 0.0f, 0.0f, 1.0f); // High P gain
    
    // Error = 5.0. P_out = 50.0. Should clamp to LIMIT_MAX (1.0).
    float outMax = pid.calculate(5.0f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX);
    EXPECT_FLOAT_EQ(outMax, LIMIT_MAX);
    
    // Error = -5.0. P_out = -50.0. Should clamp to LIMIT_MIN (-1.0).
    float outMin = pid.calculate(-5.0f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX);
    EXPECT_FLOAT_EQ(outMin, LIMIT_MIN);
}

TEST_F(PidControllerTest, AntiWindupPreventsDeepSaturation) {
    // Kp = 1.0, Ki = 10.0. High integral gain to induce windup quickly.
    PIDController pid(1.0f, 10.0f, 0.0f, 1.0f);
    
    // Drive the system hard into MAX saturation for 2 seconds (200 steps).
    // If anti-windup is missing, the integral term would accumulate to 20.0.
    runSteps(pid, 1.0f, 0.0f, DT_DEFAULT, 200);
    
    // Output should be clamped.
    float saturatedOut = pid.calculate(1.0f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX);
    EXPECT_FLOAT_EQ(saturatedOut, LIMIT_MAX);
    
    // Because of conditional integration (anti-windup), the integral stopped
    // growing the moment total output breached LIMIT_MAX.
    // If we instantly reverse the error slightly (e.g. error becomes negative),
    // the output should drop immediately, rather than waiting for a massive 
    // wound-up accumulator to drain.
    float reversedOut = pid.calculate(-0.1f, 0.0f, DT_DEFAULT, LIMIT_MIN, LIMIT_MAX);
    
    // If it wound up, reversedOut would still be pegged at 1.0.
    // Since it didn't, the negative P-term (-0.1) and the immediate reduction
    // in the integral should pull it below the max limit instantly.
    EXPECT_LT(reversedOut, LIMIT_MAX) 
        << "Anti-windup failed: Integrator accumulated during saturation.";
}

TEST_F(PidControllerTest, ConditionalIntegrationAllowsUnwinding) {
    PIDController pid(0.0f, 1.0f, 0.0f, 1.0f); // Pure I controller
    
    // 1. Push slightly into saturation
    pid.calculate(2.0f, 0.0f, 1.0f, -1.0f, 1.0f); // Int = 2.0 (Output clamped to 1.0)
    
    // 2. Apply positive error again. The conditional logic should PREVENT integration.
    pid.calculate(2.0f, 0.0f, 1.0f, -1.0f, 1.0f); 
    
    // 3. Apply negative error. The conditional logic `(isSaturatedMax && error < 0.0f)` 
    // MUST allow integration to pull the system out of saturation.
    float recoveringOut = pid.calculate(-1.0f, 0.0f, 1.0f, -1.0f, 1.0f);
    
    // Previous valid integral was 2.0. We just added (-1.0 * 1.0). 
    // New integral should be 1.0.
    EXPECT_FLOAT_EQ(recoveringOut, 1.0f);
    
    // One more negative step should drop it below saturation completely.
    float recoveredOut = pid.calculate(-0.5f, 0.0f, 1.0f, -1.0f, 1.0f);
    EXPECT_FLOAT_EQ(recoveredOut, 0.5f);
}

// ----------------------------------------------------------------------------
// State Management
// ----------------------------------------------------------------------------

TEST_F(PidControllerTest, ResetClearsHistory) {
    PIDController pid(1.0f, 1.0f, 1.0f, 1.0f);

    // Build up integral and derivative states WITHOUT hitting saturation limits.
    // By using limits of +/- 200.0f, the massive initial derivative spike (100.0)
    // won't trigger the anti-windup clamping, allowing the integrator to run.
    for (int i = 0; i < 100; ++i) {
        pid.calculate(1.0f, 0.0f, DT_DEFAULT, -200.0f, 200.0f);
    }

    // After 100 steps:
    // P = 1.0
    // I = 1.0 (0.01 added per step for 100 steps)
    // D = 0.0 (error is steady)
    // Next calculation adds one more integration step (0.01). Total expected: 2.01.
    float preResetOut = pid.calculate(1.0f, 0.0f, DT_DEFAULT, -200.0f, 200.0f);
    EXPECT_GT(preResetOut, 1.5f);

    // Reset the controller history
    pid.reset();

    // With history cleared, the exact same inputs should now yield the 
    // Proportional response, a fresh D spike, AND the first I step.
    // P_out = 1.0
    // D_out = (1.0 - 0) / 0.01 = 100.0
    // Integral = 0 + (1.0 * 0.01) = 0.01
    float postResetOut = pid.calculate(1.0f, 0.0f, DT_DEFAULT, -200.0f, 200.0f);

    EXPECT_FLOAT_EQ(postResetOut, 101.01f);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}