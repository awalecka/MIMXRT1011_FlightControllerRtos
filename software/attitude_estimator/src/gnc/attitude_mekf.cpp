/**
 * @file attitude_mekf.cpp
 * @brief Linearised MEKF implementation for attitude estimation.
 *
 * Design notes
 * ------------
 * The filter operates on a 6D error state [delta_r, delta_b] where delta_r
 * is a 3D rotation vector error and delta_b is a 3D gyro bias error. A
 * separate nominal quaternion q_nom is maintained and is always unit-length.
 *
 * Prediction
 * ----------
 * The nominal quaternion is propagated with the exact closed-form integral:
 *   q_nom[k+1] = q_nom[k] * exp(0.5 * omega_corrected * dt)
 * where omega_corrected = omegaMeas - nominalBias.
 *
 * The error covariance is propagated via the linearised process Jacobian:
 *   P[k+1] = F * P[k] * F^T + Q
 *
 * The process Jacobian F for the error state kinematics is:
 *   F = [ I - [omega_corrected]x * dt,  -I * dt ]
 *       [ 0_{3x3},                       I      ]
 * where [omega_corrected]x is the 3x3 skew-symmetric matrix of omega_corrected.
 * This comes from linearising d(delta_r)/dt = -omega x delta_r - delta_b
 * around the nominal trajectory.
 *
 * Measurement update
 * ------------------
 * For a vector observation h(q) = R(q)^T * r_inertial (the inertial reference
 * vector rotated into the body frame), the measurement Jacobian is:
 *   H = [ [R(q_nom) * r_inertial]x,  0_{3x3} ]
 *         (3x3 skew of predicted meas, padded with zeros for bias cols)
 *
 * This is the first-order expansion of h(q_nom * delta_q) around delta_q = 0.
 * The expected measurement is predicted_meas = q_nom.conjugate() * r_inertial.
 *
 * Both measurement and reference are normalised before the update so that
 * the noise parameter rNoise is scale-independent.
 *
 * Covariance update form
 * ----------------------
 * Two forms are available, selected at compile time via MEKF_USE_JOSEPH_FORM:
 *
 *   Standard form (default, MEKF_USE_JOSEPH_FORM = 0):
 *     P+ = P- - K*S*K^T  followed by (P + P^T)/2 re-symmetrisation.
 *     Cost: ~324 FP multiplies per update on a 6-state / 3-meas system.
 *     Rationale: sufficient for well-conditioned MEMS operation at 500 Hz.
 *     repairCovariance() acts as a telemetry-visible safety net for the
 *     rare adversarial-input case; it is not needed in nominal flight.
 *
 *   Joseph stabilised form (MEKF_USE_JOSEPH_FORM = 1):
 *     P+ = (I-KH)*P*(I-KH)^T + K*R*K^T
 *     Cost: ~2× the standard form (~800 FP multiplies per update).
 *     Rationale: stronger SPD guarantee in exact arithmetic. In float32
 *     the improvement is marginal — round-off still accumulates — but
 *     useful when characterising filter behaviour under stressed inputs.
 *     Does NOT eliminate the need for repairCovariance().
 *
 * MEKF reset
 * ----------
 * After each update the attitude error delta_r is absorbed multiplicatively:
 *   q_nom[k+1] = q_nom[k] * rotVecToQuat(delta_r)
 * and the bias nominal is updated additively:
 *   nominalBias[k+1] = nominalBias[k] + delta_b
 * The error state is then zeroed. The bias sub-state (indices 3-5) is zeroed
 * as part of the reset because it represents deviation from the nominal bias,
 * which was just updated.
 *
 * Adheres to: GNU C++23, no dynamic allocation, FreeRTOS-safe.
 */

#include "attitude_mekf.hpp"

// Set to 1 to use the Joseph stabilised covariance update instead of the
// cheaper standard form. See file-level comment above for the tradeoff.
#ifndef MEKF_USE_JOSEPH_FORM
#define MEKF_USE_JOSEPH_FORM 0
#endif

namespace gnc {

    // ----------------------------------------------------------------------------
    // Physics constants
    // ----------------------------------------------------------------------------
    static constexpr float GRAVITY_MPS2 = 9.81f;
    static constexpr float PI = 3.14159265358979f;
    static constexpr float RAD_TO_DEG = 180.0f / PI;

    // Initial covariance scalars — diagonal seeds for attitude and bias blocks
    static constexpr float INIT_ATT_VAR = 0.1f;   ///< [rad^2] initial attitude uncertainty
    static constexpr float INIT_BIAS_VAR = 0.01f;  ///< [(rad/s)^2] initial bias uncertainty

    // Repair constants for repairCovariance()
    static constexpr float COV_RIDGE = 1e-6f;  ///< Ridge added during repair attempt
    static constexpr float COV_RESET_VAL = 0.001f; ///< Diagonal value on hard reset


    // ============================================================================
    // Construction
    // ============================================================================

    AttitudeMekf::AttitudeMekf(const Config& config)
        : nominalQuat(Eigen::Quaternionf::Identity())
        , nominalBias(Vector3::Zero())
        , errorState(ErrorVec::Zero())
        , config(config)
        , covFaultCount(0u)
    {
        // Seed covariance: attitude block and bias block on the diagonal,
        // all cross terms zero.
        errorCov.setZero();
        errorCov.block<3, 3>(0, 0) = Matrix3::Identity() * INIT_ATT_VAR;
        errorCov.block<3, 3>(3, 3) = Matrix3::Identity() * INIT_BIAS_VAR;
    }


    // ============================================================================
    // Public API
    // ============================================================================

    void AttitudeMekf::init(const Eigen::Quaternionf& initialQuat, const Vector3& initialBias)
    {
        nominalQuat = initialQuat;
        nominalQuat.normalize();

        nominalBias = initialBias;

        // Reset the error state — it represents deviation from the nominal,
        // which is now exactly initialQuat / initialBias by definition.
        errorState.setZero();

        // Reseed covariance conservatively. Cross terms are zero: at
        // initialisation the attitude and bias errors are independent.
        errorCov.setZero();
        errorCov.block<3, 3>(0, 0) = Matrix3::Identity() * INIT_ATT_VAR;
        errorCov.block<3, 3>(3, 3) = Matrix3::Identity() * INIT_BIAS_VAR;
    }

    void AttitudeMekf::align(const Vector3& accelMean, const Vector3& magMean, Vector3& magRefOut)
    {
        // ------------------------------------------------------------------
        // TRIAD method — avoids Euler-angle singularities at +-90 deg pitch.
        //
        // The TRIAD vectors express each NED axis *as measured in the body
        // frame*, because the inputs (accel, mag) are body-frame readings.
        // Stacking them as columns of a matrix gives R_i2b (inertial-to-body).
        // Transposing gives R_b2i (body-to-inertial) for the quaternion.
        // ------------------------------------------------------------------

        // NED Down axis: gravity points Down, so the accelerometer reading
        // (in NED where Down is +Z) directly gives the Down direction in body.
        Vector3 zBody = accelMean;
        if (zBody.norm() < 1e-3f) {
            zBody = Vector3(0.0f, 0.0f, 1.0f); // freefall fallback
        }
        else {
            zBody.normalize();
        }

        // NED East axis: East = Down x MagNorth. Cross product of the Down
        // body-vector with the magnetometer reading gives the East body-vector,
        // perpendicular to both gravity and magnetic north.
        Vector3 yBody = zBody.cross(magMean);
        if (yBody.norm() < 1e-3f) {
            // Degenerate case: mag field is parallel to gravity (magnetic pole)
            yBody = Vector3(0.0f, 1.0f, 0.0f);
        }
        else {
            yBody.normalize();
        }

        // NED North axis: North = East x Down (completes right-hand set)
        Vector3 xBody = yBody.cross(zBody);
        xBody.normalize();

        // R_i2b: columns are the NED axes expressed in the body frame.
        // R_i2b * v_inertial = v_body
        Eigen::Matrix3f R_i2b;
        R_i2b.col(0) = xBody; // NED North expressed in body frame
        R_i2b.col(1) = yBody; // NED East  expressed in body frame
        R_i2b.col(2) = zBody; // NED Down  expressed in body frame

        // R_b2i: body-to-inertial, used to construct the quaternion.
        // R_b2i * v_body = v_inertial
        const Eigen::Matrix3f R_b2i = R_i2b.transpose();

        Eigen::Quaternionf qInit(R_b2i);
        qInit.normalize();

        init(qInit, Vector3::Zero());

        // ------------------------------------------------------------------
        // Magnetic reference vector in the inertial frame.
        // Project the body-frame mag reading into the inertial frame, then
        // zero the East (Y) component so the reference lies in the NED
        // North-Down plane. This captures the local magnetic dip angle.
        // ------------------------------------------------------------------
        const Vector3 magInertial = R_b2i * magMean;
        magRefOut << magInertial.x(), 0.0f, magInertial.z();

        if (magRefOut.norm() > 1e-3f) {
            magRefOut.normalize();
        }
        else {
            magRefOut = Vector3(1.0f, 0.0f, 0.0f); // fallback: flat North
        }
    }

    void AttitudeMekf::predict(float dt, const Vector3& omegaMeas)
    {
        // ------------------------------------------------------------------
        // Compute the bias-corrected angular rate using the current nominal
        // bias estimate.
        // ------------------------------------------------------------------
        const Vector3 omegaCorrected = omegaMeas - nominalBias;

        // ------------------------------------------------------------------
        // Nominal quaternion propagation — exact closed-form integration.
        //
        // q[k+1] = q[k] * dq,  where dq = [cos(theta/2), axis*sin(theta/2)]
        // and theta = |omega_corrected| * dt.
        //
        // The small-angle approximation (w=1, vec=omega*dt/2) is only valid
        // for |omega|*dt << 1 rad and accumulates systematic under-rotation
        // above ~0.5 rad/s. The exact formula is used unconditionally.
        // ------------------------------------------------------------------
        const float theta = omegaCorrected.norm() * dt;
        Eigen::Quaternionf dq;
        if (theta > 1e-4f) {
            // Exact closed-form: no approximation error for any representable theta.
            dq.w() = std::cos(theta * 0.5f);
            dq.vec() = (omegaCorrected / omegaCorrected.norm()) * std::sin(theta * 0.5f);
        }
        else {
            // Taylor series: sin(theta/2)/theta ≈ 1/2 - theta^2/48.
            // At theta = 1e-4 rad, truncation error ≈ 2e-14 — five orders of
            // magnitude below float32 epsilon (~1.2e-7). Avoids dividing by a
            // near-zero theta that would force the FPU to the edge of precision.
            dq.w() = 1.0f;
            dq.vec() = omegaCorrected * (dt * 0.5f);
            dq.normalize();
        }

        nominalQuat = nominalQuat * dq;
        nominalQuat.normalize();

        // ------------------------------------------------------------------
        // Error covariance propagation via the linearised process Jacobian.
        //
        // The continuous-time error kinematics are:
        //   d(delta_r)/dt = -[omega_corrected]x * delta_r - delta_b
        //   d(delta_b)/dt = 0   (bias is modelled as a random walk)
        //
        // Discretising to first order gives the 6x6 Jacobian F:
        //   F = [ I - [omega_corrected]x * dt,  -I * dt ]
        //       [ 0_{3x3},                       I      ]
        // ------------------------------------------------------------------
        const Matrix3 Sk = skew(omegaCorrected);

        ErrorMat F = ErrorMat::Identity();
        F.block<3, 3>(0, 0) -= Sk * dt;      // attitude block: I - [omega]x*dt
        F.block<3, 3>(0, 3) = -Matrix3::Identity() * dt; // attitude-bias coupling: -I*dt

        // P[k+1|k] = F * P[k|k] * F^T + Q
        errorCov = F * errorCov * F.transpose();

        // Discrete process noise Q = diag(qGyro*dt * I_3, qBias*dt * I_3)
        errorCov.block<3, 3>(0, 0) += Matrix3::Identity() * (config.qGyro * dt);
        errorCov.block<3, 3>(3, 3) += Matrix3::Identity() * (config.qBias * dt);
    }

    void AttitudeMekf::updateAccel(const Vector3& accelMeas)
    {
        const float accelNorm = accelMeas.norm();

        // Guard against a zero or near-zero measurement (e.g. freefall with the
        // magnitude gate disabled). Dividing by accelNorm would inject NaN into
        // the state and covariance matrices, which cannot be recovered from.
        if (accelNorm < 1e-3f) {
            return;
        }

        // Gate: reject the update when the accelerometer is not measuring
        // predominantly gravity (i.e. the vehicle has significant linear
        // acceleration). config.accelGate == 0 disables the gate.
        if (config.accelGate > 0.0f) {
            if (std::abs(accelNorm - GRAVITY_MPS2) > config.accelGate) {
                return; // measurement contaminated by linear acceleration
            }
        }

        // Normalise both measurement and reference so rAccel is scale-independent
        const Vector3 measNorm = accelMeas / accelNorm;
        const Vector3 refNorm = Vector3(0.0f, 0.0f, 1.0f); // NED: gravity is +Z

        updateVectorObservation(measNorm, refNorm, config.rAccel);
    }

    void AttitudeMekf::updateMag(const Vector3& magMeas, const Vector3& magRef)
    {
        const float measNormVal = magMeas.norm();
        const float refNormVal = magRef.norm();

        // Guard against degenerate inputs
        if (measNormVal < 1e-3f || refNormVal < 1e-3f) {
            return;
        }

        const Vector3 measNorm = magMeas / measNormVal;
        const Vector3 refNorm = magRef / refNormVal;

        updateVectorObservation(measNorm, refNorm, config.rMag);
    }

    Eigen::Quaternionf AttitudeMekf::getQuaternion() const
    {
        return nominalQuat;
    }

    AttitudeMekf::Vector3 AttitudeMekf::getBias() const
    {
        return nominalBias;
    }

    const AttitudeMekf::ErrorMat& AttitudeMekf::getCovariance() const
    {
        return errorCov;
    }

    AttitudeMekf::Vector3 AttitudeMekf::getEulerAnglesDeg() const
    {
        const Eigen::Quaternionf q = nominalQuat;
        const float qw = q.w();
        const float qx = q.x();
        const float qy = q.y();
        const float qz = q.z();

        const float roll = std::atan2(2.0f * (qw * qx + qy * qz),
            1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;

        // Clamp sinp to [-1, 1] to guard the asin domain at +-90 deg pitch
        const float sinp = 2.0f * (qw * qy - qz * qx);
        float pitch;
        if (std::abs(sinp) >= 1.0f) {
            pitch = std::copysign(90.0f, sinp);
        }
        else {
            pitch = std::asin(sinp) * RAD_TO_DEG;
        }

        const float yaw = std::atan2(2.0f * (qw * qz + qx * qy),
            1.0f - 2.0f * (qy * qy + qz * qz)) * RAD_TO_DEG;

        return Vector3(roll, pitch, yaw);
    }

    uint32_t AttitudeMekf::getCovarianceFaultCount() const
    {
        return covFaultCount;
    }


    // ============================================================================
    // Private helpers
    // ============================================================================

    void AttitudeMekf::updateVectorObservation(const Vector3& measNorm,
        const Vector3& refNorm,
        float rNoise)
    {
        // ------------------------------------------------------------------
        // Predicted measurement: rotate the inertial reference into the body
        // frame using the current nominal quaternion.
        //   z_pred = R(q_nom)^T * r_inertial = q_nom.conjugate() * r_inertial
        // ------------------------------------------------------------------
        const Vector3 zPred = nominalQuat.conjugate() * refNorm;

        // ------------------------------------------------------------------
        // Measurement Jacobian H [3x6].
        //
        // The observation model is h(q) = R(q)^T * r_inertial.
        // Linearising around q_nom with a small rotation error delta_r:
        //   h(q_nom * delta_q) ≈ h(q_nom) + dh/d(delta_r) * delta_r
        //
        // The Jacobian of R(q)^T * r with respect to a rotation vector
        // perturbation is [R(q)*r]x, the skew-symmetric matrix of the
        // rotated reference:
        //   dh/d(delta_r) = [R(q_nom) * r_inertial]x = [z_pred]x
        //
        // The bias columns are zero because h does not depend on bias.
        //   H = [ [z_pred]x, 0_{3x3} ]
        // ------------------------------------------------------------------
        Jacobian H = Jacobian::Zero();
        H.block<3, 3>(0, 0) = skew(zPred);

        // ------------------------------------------------------------------
        // Innovation covariance S = H * P * H^T + R
        // ------------------------------------------------------------------
        const MeasMat S = H * errorCov * H.transpose()
            + Matrix3::Identity() * rNoise;

        // ------------------------------------------------------------------
        // Kalman gain K [6x3].
        //
        // We need K = P * H^T * S^-1.  Rather than forming S^-1 explicitly,
        // we solve S * K^T = H * P for K^T and transpose:
        //   K^T = S^-1 * (H * P)   =>   K = (S^-1 * H * P)^T
        //
        // S is 3x3 symmetric positive-definite, so LDLT is both cheaper and
        // more stable than LLT and avoids the explicit inverse entirely.
        // ------------------------------------------------------------------
        const KalmanGain K = (S.ldlt().solve(H * errorCov)).transpose();

        // ------------------------------------------------------------------
        // Error state update.
        //
        // The error state is always zero on entry (zeroed by the MEKF reset
        // at the end of every previous call to this function). The correction
        // is therefore fully determined by the current innovation alone — not
        // accumulated across multiple steps — so we use = rather than +=.
        // ------------------------------------------------------------------
        const Vector3 innovation = measNorm - zPred;
        errorState = K * innovation;

        // ------------------------------------------------------------------
        // Covariance update.
        //
        // Two forms, selected by MEKF_USE_JOSEPH_FORM (see file top comment):
        //
        //   Standard:  P+ = P- - K*S*K^T  + symmetrisation
        //   Joseph:    P+ = (I-KH)*P*(I-KH)^T + K*R*K^T
        //
        // The standard form is the default. repairCovariance() remains as a
        // telemetry-visible safety net either way.
        // ------------------------------------------------------------------
#if MEKF_USE_JOSEPH_FORM
        {
            const ErrorMat I_KH = ErrorMat::Identity() - K * H;
            const MeasMat  R_mat = MeasMat::Identity() * rNoise;
            errorCov = I_KH * errorCov * I_KH.transpose()
                + K * R_mat * K.transpose();
        }
#else
        errorCov -= K * S * K.transpose();
        errorCov = (errorCov + errorCov.transpose()) * 0.5f;
#endif

        // ------------------------------------------------------------------
        // MEKF reset: absorb the error state into the nominal state so the
        // error state returns to zero and linearisation remains valid.
        //
        // Attitude: q_nom[+] = q_nom * rotVecToQuat(delta_r)
        // Bias:     b_nom[+] = b_nom + delta_b
        //
        // Both error sub-states are zeroed after absorption.
        // ------------------------------------------------------------------
        const Vector3 deltaR = errorState.segment<3>(0);
        const Vector3 deltaB = errorState.segment<3>(3);

        nominalQuat = nominalQuat * rotVecToQuat(deltaR);
        nominalQuat.normalize();

        nominalBias += deltaB;

        errorState.setZero();

        // Repair the covariance if round-off has degraded it
        repairCovariance();
    }

    void AttitudeMekf::repairCovariance()
    {
        // Attempt soft repair first: re-symmetrise and add a small ridge.
        // This corrects asymmetry from float round-off without discarding
        // the covariance shape.
        Eigen::LLT<ErrorMat> llt(errorCov);
        if (llt.info() != Eigen::Success) {
            errorCov = (errorCov + errorCov.transpose()) * 0.5f;
            errorCov += ErrorMat::Identity() * COV_RIDGE;
            llt.compute(errorCov);

            if (llt.info() != Eigen::Success) {
                // Soft repair failed: the matrix is too corrupted to salvage.
                // Reset to a conservative diagonal and count the fault so
                // it is visible via getCovarianceFaultCount() / telemetry.
                ++covFaultCount;
                errorCov.setZero();
                errorCov.block<3, 3>(0, 0) = Matrix3::Identity() * COV_RESET_VAL;
                errorCov.block<3, 3>(3, 3) = Matrix3::Identity() * COV_RESET_VAL;
            }
        }
    }

    AttitudeMekf::Matrix3 AttitudeMekf::skew(const Vector3& v)
    {
        Matrix3 S;
        S << 0.0f, -v.z(), v.y(),
            v.z(), 0.0f, -v.x(),
            -v.y(), v.x(), 0.0f;
        return S;
    }

    Eigen::Quaternionf AttitudeMekf::rotVecToQuat(const Vector3& rv)
    {
        Eigen::Quaternionf q;
        const float theta = rv.norm();
        if (theta > 1e-4f) {
            // Exact formula: no approximation error for any representable theta.
            q.w() = std::cos(theta * 0.5f);
            q.vec() = (rv / theta) * std::sin(theta * 0.5f);
        }
        else {
            // Taylor series: sin(theta/2)/theta ≈ 1/2 - theta^2/48.
            // At theta = 1e-4, truncation error ≈ 2e-14 << float32 epsilon.
            // Cheaper and safer than dividing by a near-zero theta.
            q.w() = 1.0f;
            q.vec() = rv * 0.5f;
            q.normalize();
        }
        return q;
    }

} // namespace gnc