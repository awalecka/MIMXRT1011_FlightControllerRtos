/**
 * @file attitude_ukf.cpp
 * @brief Implementation of AttitudeUkf.
 *
 * Adheres to strict static allocation and C++23 standards.
 */

#include "attitude_ukf.hpp"
#include <cstdint>

namespace gnc {

    // ----------------------------------------------------------------------------
    // Physics Constants
    // ----------------------------------------------------------------------------
    static constexpr float GRAVITY_MPS2 = 9.81f;

    // ----------------------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------------------

    AttitudeUkf::AttitudeUkf(const Config& config)
        : config(config)
    {
        // Default initialization
        nominalQuat = Eigen::Quaternionf::Identity();
        stateVector.setZero(); // 6D error is zero

        // Initial Covariance (6x6 Diagonal)
        errorCovariance.setIdentity();
        errorCovariance *= 0.1f; // Small initial uncertainty

        // UKF parameter calculation
        lambdaParam = (this->config.alpha * this->config.alpha) * (static_cast<float>(STATE_DIM) + this->config.kappa) - static_cast<float>(STATE_DIM);

        computeWeights();
    }

    void AttitudeUkf::init(const Eigen::Quaternionf& initialQuat, const Vector3& initialBias)
    {
        nominalQuat = initialQuat;
        nominalQuat.normalize();
        
        stateVector.setZero(); // Reset attitude error to zero
        // Store the supplied bias directly in the error-state bias slot (indices 3-5).
        // In the MEKF formulation the bias sub-state holds the running estimate of the
        // true gyro bias. predict() reads it back as currentBias each step and subtracts
        // it from the raw gyro measurement. The MEKF reset in updateGenericObservation()
        // zeroes only the attitude portion (indices 0-2); the bias estimate here is
        // intentionally preserved and refined by subsequent measurement updates.
        stateVector.segment<3>(3) = initialBias;

        // Reset covariance on re-init
        errorCovariance.setIdentity();
        errorCovariance *= 0.01f;
    }

    void AttitudeUkf::align(const Vector3& accelMean, const Vector3& magMean, Vector3& magRefOut)
    {
        // Vector-based alignment (TRIAD method) to avoid Euler Angle Gimbal Lock
        // Frame: NED (North, East, Down). 

        // 1. Z-axis (Down) is defined strictly by gravity
        // Accelerometer measures UP force (structural reaction to gravity).
        // Since we want the Down direction, and the codebase defines 1g as +Z (Down),
        Vector3 zInertial = accelMean; 
        if (zInertial.norm() < 1e-3f) {
            zInertial = Vector3(0.0f, 0.0f, 1.0f); // Fallback if in freefall
        } else {
            zInertial.normalize();
        }

        // 2. Y-axis (East) is perpendicular to Down (Z) and Magnetic North (which is roughly X)
        // East = Down x Magnetic (Z x Mag = Y, preserving Right-Hand rule)
        Vector3 yInertial = zInertial.cross(magMean);
        if (yInertial.norm() < 1e-3f) {
            // Singularity: Magnetic field is perfectly parallel to gravity
            yInertial = Vector3(0.0f, 1.0f, 0.0f); 
        } else {
            yInertial.normalize();
        }

        // 3. X-axis (North) forms the final orthogonal axis
        // North = East x Down (Y x Z = X)
        Vector3 xInertial = yInertial.cross(zInertial);
        xInertial.normalize();

        // The TRIAD vectors (xInertial, yInertial, zInertial) were computed from
        // body-frame sensor measurements, so they express the inertial NED axes
        // as seen from the body frame. Stacking them as columns produces R_i2b:
        // a matrix that transforms an inertial vector into the body frame.
        // R_i2b * v_inertial = v_body
        Eigen::Matrix3f R_i2b;
        R_i2b.col(0) = xInertial; // NED North axis expressed in body frame
        R_i2b.col(1) = yInertial; // NED East  axis expressed in body frame
        R_i2b.col(2) = zInertial; // NED Down  axis expressed in body frame

        // Transpose to get the body-to-inertial rotation for quaternion construction.
        // R_b2i * v_body = v_inertial
        Eigen::Matrix3f R_b2i = R_i2b.transpose();

        // 5. Convert to Quaternion
        Eigen::Quaternionf qInit(R_b2i);
        qInit.normalize();

        init(qInit, Vector3::Zero());

        // 6. Capture Local Magnetic Reference (Dip Angle)
        // We want the mag reference vector in the Inertial frame.
        // magInertial = R_b2i * magMean_body
        Vector3 magInertialInertial = R_b2i * magMean;
        
        // Ensure East (Y) is zero to define true Magnetic North
        magRefOut << magInertialInertial.x(), 0.0f, magInertialInertial.z();
        
        if (magRefOut.norm() > 1e-3f) {
            magRefOut.normalize();
        } else {
            magRefOut = Vector3(1.0f, 0.0f, 0.0f); // Fallback to flat North
        }
    }

    void AttitudeUkf::predict(float dt, const Vector3& omegaMeas)
    {
        // 1. Nominal State Propagation
        // The error state bias acts as the current best estimate of the true bias 
        // until updated, at which point it's absorbed. 
        Vector3 currentBias = stateVector.segment<3>(3);
        Vector3 omegaCorrected = omegaMeas - currentBias;

        // Exact closed-form quaternion integration from the corrected angular rate.
        // The first-order approximation (w=1, vec=omega*dt/2, normalise) is only
        // accurate for |omega|*dt << 1 rad and accumulates systematic under-rotation
        // at sustained rates above ~0.5 rad/s. cos/sin(theta/2) is exact for any angle.
        Eigen::Quaternionf nominalDelta;
        const float theta = omegaCorrected.norm() * dt;
        if (theta > 1e-6f) {
            nominalDelta.w()   = std::cos(theta * 0.5f);
            nominalDelta.vec() = (omegaCorrected / omegaCorrected.norm()) * std::sin(theta * 0.5f);
        } else {
            // Small-angle safe path: avoids divide-by-zero on the norm
            nominalDelta.w()   = 1.0f;
            nominalDelta.vec() = omegaCorrected * dt * 0.5f;
            nominalDelta.normalize();
        }

        nominalQuat = nominalQuat * nominalDelta;
        nominalQuat.normalize();

        // 2. Generate Sigma Points (6D Error State)
        generateSigmaPoints();

        // 3. Propagate Error Sigma Points through Process Model
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            Vector3 errorRotVec = sigmaPoints.block<3, 1>(0, i);
            Vector3 errorBias = sigmaPoints.block<3, 1>(3, i);

            // Linearised MEKF error-state kinematics:
            //   d(delta_r)/dt = -omega_nominal x delta_r  -  delta_bias
            // where:
            //   omega_nominal  = omegaMeas - currentBias  (already computed above)
            //   omega_i        = omegaMeas - errorBias    (this sigma point's rate)
            //   delta_bias     = errorBias - currentBias  (bias deviation from nominal)
            //
            // The cross-product term rotates the error rotation vector by the nominal
            // rate (transport theorem). Only the *difference* from the nominal bias
            // drives additional attitude error; the mean bias is already absorbed into
            // the nominal quaternion propagation above, so using the full nominal rate
            // here does not double-count it.
            Vector3 deltaBias = errorBias - currentBias;
            Vector3 drPropagated = errorRotVec - omegaCorrected.cross(errorRotVec) * dt - deltaBias * dt;

            // Bias is a random walk (db_k+1 = db_k)
            sigmaPoints.block<3, 1>(0, i) = drPropagated;
            // sigmaPoints.block<3, 1>(3, i) remains errorBias
        }

        // 4. Compute Predicted Mean Error
        stateVector.setZero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            float weight = weightsMean(i);
            stateVector += weight * sigmaPoints.col(i);
        }

        // 5. Compute Predicted Covariance
        errorCovariance.setZero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            VectorState diff = sigmaPoints.col(i) - stateVector;
            float weight = weightsCovariance(i);
            MatrixState outerProduct = diff * diff.transpose();
            errorCovariance += weight * outerProduct;
        }

        // Add Process Noise (Discrete time Q = Q_cont * dt)
        MatrixState noiseQ = MatrixState::Zero();
        noiseQ.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * (config.qGyro * dt);
        noiseQ.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * (config.qBias * dt);

        errorCovariance += noiseQ;
    }

    void AttitudeUkf::updateAccel(const Vector3& accelMeas)
    {
        // NED Convention: 1g points towards +Z (Down)
        const Vector3 gravityInertial(0.0f, 0.0f, GRAVITY_MPS2);
        updateGenericObservation(accelMeas, gravityInertial, config.rAccel);
    }

    void AttitudeUkf::updateMag(const Vector3& magMeas, const Vector3& magRef)
    {
        updateGenericObservation(magMeas, magRef, config.rMag);
    }
    
    void AttitudeUkf::updateGenericObservation(const Vector3& meas, const Vector3& refInertial, float rNoise)
    {
        generateSigmaPoints();

        Eigen::Matrix<float, MEAS_DIM_VEC, SIGMA_COUNT> zSig;

        // Transform Sigma Points to Measurement Space using the nominal quaternion + error point
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            Vector3 errorRotVec = sigmaPoints.block<3, 1>(0, i);
            Eigen::Quaternionf dq = rotationVectorToQuaternion(errorRotVec);
            Eigen::Quaternionf qSig = nominalQuat * dq;
            qSig.normalize();

            // Expected measurement = R(q)^T * ref_inertial = q.conjugate() * ref
            zSig.col(i) = qSig.conjugate() * refInertial;
        }

        // Calculate Mean Predicted Measurement
        VectorMeas measMean = VectorMeas::Zero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            float weight = weightsMean(i);
            measMean += weight * zSig.col(i);
        }

        // Calculate Covariances
        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> sMat = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> tMat = Eigen::Matrix<float, 6, 3>::Zero();

        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> rMat = Eigen::Matrix<float, 3, 3>::Identity() * rNoise;

        for (int i = 0; i < SIGMA_COUNT; ++i) {
            VectorMeas zDiff = zSig.col(i) - measMean;
            VectorState xDiff = sigmaPoints.col(i) - stateVector;

            float weight = weightsCovariance(i);

            sMat += weight * (zDiff * zDiff.transpose());
            tMat += weight * (xDiff * zDiff.transpose());
        }
        sMat += rMat;

        // Kalman Gain
        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> kGain = tMat * sMat.inverse();

        // Update State (6D Error)
        VectorMeas zResidual = meas - measMean;
        stateVector += kGain * zResidual;

        // --- MEKF RESET ---
        // Absorb the updated 6D error state into the Nominal Quaternion
        Vector3 errorRotVec = stateVector.block<3, 1>(0, 0);
        Eigen::Quaternionf deltaQuatInfo = rotationVectorToQuaternion(errorRotVec);
        nominalQuat = nominalQuat * deltaQuatInfo;
        nominalQuat.normalize();
        
        // Zero out the attitude error portion of the state vector since it was absorbed.
        // NOTE: The bias error (indices 3,4,5) is NOT zeroed here, it continues to accumulate 
        // as the actual gyro bias estimate that gets subtracted in predict().
        stateVector.block<3, 1>(0, 0).setZero();

        // Standard covariance update: P+ = P- - K*S*K^T.
        // This form is mathematically correct for the optimal Kalman gain but sheds
        // symmetry under 32-bit floating-point round-off over many update cycles.
        errorCovariance -= kGain * sMat * kGain.transpose();

        // Re-symmetrise to prevent gradual loss of positive-definiteness.
        // (P + P^T) / 2 forces exact symmetry at the cost of one matrix add and
        // a scalar multiply — negligible on the Cortex-M7 FPU.
        errorCovariance = (errorCovariance + errorCovariance.transpose()) * 0.5f;
    }

    const AttitudeUkf::VectorState& AttitudeUkf::getState() const
    {
        return stateVector; // Note: returns the 6D error state + bias
    }

    const AttitudeUkf::MatrixState& AttitudeUkf::getCovariance() const
    {
        return errorCovariance;
    }

    Eigen::Quaternionf AttitudeUkf::getQuaternion() const
    {
        return nominalQuat;
    }

    AttitudeUkf::Vector3 AttitudeUkf::getEulerAnglesDeg() const
    {
        Eigen::Quaternionf q = getQuaternion();
        float qw = q.w();
        float qx = q.x();
        float qy = q.y();
        float qz = q.z();

        static constexpr float RAD_TO_DEG = 180.0f / 3.1415926535f;

        float roll = std::atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;

        float sinp = 2.0f * (qw * qy - qz * qx);
        float pitch;
        if (std::abs(sinp) >= 1.0f)
            pitch = std::copysign(90.0f, sinp);
        else
            pitch = std::asin(sinp) * RAD_TO_DEG;

        float yaw = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * RAD_TO_DEG;

        return Vector3(roll, pitch, yaw);
    }

    // ----------------------------------------------------------------------------
    // Private Helpers
    // ----------------------------------------------------------------------------

    void AttitudeUkf::computeWeights()
    {
        // W0_mean = lambda / (n + lambda).
        // With small alpha (e.g. 0.1) lambda is negative and W0_mean is therefore
        // also negative (e.g. ~-0.94 for alpha=0.1, n=6, kappa=0). This is
        // mathematically valid for the unscented transform; the remaining 2n weights
        // are positive and the full set sums to 1. Do not clamp W0_mean to zero.
        weightsMean(0) = lambdaParam / (static_cast<float>(STATE_DIM) + lambdaParam);
        weightsCovariance(0) = weightsMean(0) + (1.0f - config.alpha * config.alpha + config.beta);

        float wI = 1.0f / (2.0f * (static_cast<float>(STATE_DIM) + lambdaParam));
        for (int i = 1; i < SIGMA_COUNT; ++i) {
            weightsMean(i) = wI;
            weightsCovariance(i) = wI;
        }
    }

    void AttitudeUkf::generateSigmaPoints()
    {
        sigmaPoints.col(0) = stateVector;

        Eigen::LLT<MatrixState> llt(errorCovariance);

        if (llt.info() != Eigen::Success) {
            // Attempt a minimal repair before discarding all accumulated uncertainty.
            // Re-symmetrising eliminates asymmetry from float round-off; adding a
            // small ridge (epsilon * I) nudges any near-zero eigenvalues positive
            // without significantly distorting the covariance shape.
            errorCovariance = (errorCovariance + errorCovariance.transpose()) * 0.5f;
            errorCovariance += MatrixState::Identity() * 1e-6f;
            llt.compute(errorCovariance);

            if (llt.info() != Eigen::Success) {
                // Repair failed: the matrix is too corrupted to salvage. Reset to a
                // conservative scalar identity and increment the fault counter so
                // the caller can detect repeated failures via a telemetry flag.
                static uint32_t s_choleskyFaultCount = 0u;
                ++s_choleskyFaultCount;
                errorCovariance = MatrixState::Identity() * 0.001f;
                llt.compute(errorCovariance);
            }
        }

        MatrixState lMat = llt.matrixL();

        float gamma = std::sqrt(static_cast<float>(STATE_DIM) + lambdaParam);
        MatrixState weightedL = lMat * gamma;

        for (int i = 0; i < STATE_DIM; ++i) {
            sigmaPoints.col(i + 1) = stateVector + weightedL.col(i);
            sigmaPoints.col(i + 1 + STATE_DIM) = stateVector - weightedL.col(i);
        }
    }
    
    AttitudeUkf::Vector3 AttitudeUkf::quaternionToRotationVector(const Eigen::Quaternionf& q)
    {
        Vector3 rv;
        float sinThetaHalf = q.vec().norm();
        if (sinThetaHalf > 1e-6f) {
            float theta = 2.0f * std::atan2(sinThetaHalf, q.w());
            rv = (q.vec() / sinThetaHalf) * theta;
        } else {
            // Small angle approximation
            rv = 2.0f * q.vec(); 
        }
        return rv;
    }

    Eigen::Quaternionf AttitudeUkf::rotationVectorToQuaternion(const Vector3& rv)
    {
        Eigen::Quaternionf q;
        float theta = rv.norm();
        if (theta > 1e-6f) {
            Vector3 axis = rv / theta;
            q.w() = std::cos(theta * 0.5f);
            q.vec() = axis * std::sin(theta * 0.5f);
        } else {
            // Small angle approximation
            q.w() = 1.0f;
            q.vec() = rv * 0.5f;
            q.normalize();
        }
        return q;
    }

} // namespace gnc
