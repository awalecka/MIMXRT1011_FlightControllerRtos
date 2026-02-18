/**
 * @file attitude_ukf.cpp
 * @brief Implementation of AttitudeUkf.
 *
 * Adheres to strict static allocation and C++23 standards.
 */

#include "attitude_ukf.hpp"

namespace gnc {

    // ----------------------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------------------

    AttitudeUkf::AttitudeUkf(const Config& config)
        : config(config)
    {
        // Default initialization
        stateVector.setZero();
        stateVector(0) = 1.0f; // Identity quaternion [1, 0, 0, 0]

        // Initial Covariance (Diagonal)
        errorCovariance.setIdentity();
        errorCovariance *= 0.1f; // Small initial uncertainty

        // UKF parameter calculation
        lambdaParam = (this->config.alpha * this->config.alpha) * (static_cast<float>(STATE_DIM) + this->config.kappa) - static_cast<float>(STATE_DIM);

        computeWeights();
    }

    void AttitudeUkf::init(const Eigen::Quaternionf& initialQuat, const Vector3& initialBias)
    {
        stateVector(0) = initialQuat.w();
        stateVector(1) = initialQuat.x();
        stateVector(2) = initialQuat.y();
        stateVector(3) = initialQuat.z();

        stateVector.segment<3>(4) = initialBias;

        // Reset covariance on re-init
        errorCovariance.setIdentity();
        errorCovariance *= 0.01f;
    }

    void AttitudeUkf::align(const Vector3& accelMean, const Vector3& magMean, Vector3& magRefOut)
    {
        // Calculate Initial Roll/Pitch from Gravity (Accel)
        // Expects Gravity Vector (Down = +g)
        // Roll (Phi) = atan2(ay, az)
        float initRoll = std::atan2(accelMean.y(), accelMean.z());

        // Pitch (Theta) = atan2(-ax, sqrt(ay^2 + az^2))
        float initPitch = std::atan2(-accelMean.x(), std::sqrt(accelMean.y()*accelMean.y() + accelMean.z()*accelMean.z()));

        // Tilt-Compensate Magnetometer to find Yaw
        float cosPhi = std::cos(initRoll);
        float sinPhi = std::sin(initRoll);
        float cosTheta = std::cos(initPitch);
        float sinTheta = std::sin(initPitch);

        // Rotate Mag vector into horizontal plane (Earth Frame components X and Y)
        // Hy = My*cosPhi - Mz*sinPhi
        float Hy = magMean.y() * cosPhi - magMean.z() * sinPhi;
        // Hx = Mx*cosTheta + My*sinTheta*sinPhi + Mz*sinTheta*cosPhi
        float Hx = magMean.x() * cosTheta + magMean.y() * sinTheta * sinPhi + magMean.z() * sinTheta * cosPhi;

        // Initial Yaw (Psi) = atan2(-Hy, Hx)
        float initYaw = std::atan2(-Hy, Hx);

        // Initialize UKF Quaternion
        // Create quaternion from Euler angles (ZYX sequence)
        Eigen::Quaternionf qInit = Eigen::AngleAxisf(initYaw, Eigen::Vector3f::UnitZ()) *
                                   Eigen::AngleAxisf(initPitch, Eigen::Vector3f::UnitY()) *
                                   Eigen::AngleAxisf(initRoll, Eigen::Vector3f::UnitX());

        // Initialize state (Assume zero gyro bias at startup)
        init(qInit, Vector3::Zero());

        // 4. Capture Local Magnetic Reference (Dip Angle)
        // Bx (North) = sqrt(Hx^2 + Hy^2)
        // Bz (Down)  = -Mx*sinTheta + My*cosTheta*sinPhi + Mz*cosTheta*cosPhi
        float Bx = std::sqrt(Hx*Hx + Hy*Hy);
        float Bz = -magMean.x() * sinTheta + magMean.y() * cosTheta * sinPhi + magMean.z() * cosTheta * cosPhi;

        magRefOut << Bx, 0.0f, Bz;
        magRefOut.normalize();
    }

    void AttitudeUkf::predict(float dt, const Vector3& omegaMeas)
    {
        // 1. Generate Sigma Points based on current P
        generateSigmaPoints();

        // 2. Propagate Sigma Points through Process Model
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            Eigen::Quaternionf quatK(
                sigmaPoints(0, i),
                sigmaPoints(1, i),
                sigmaPoints(2, i),
                sigmaPoints(3, i)
            );
            quatK.normalize();

            Vector3 biasK = sigmaPoints.block<3, 1>(4, i);
            Vector3 omegaCorrected = omegaMeas - biasK;

            // Quaternion integration using small angle approximation
            Eigen::Quaternionf deltaQuat;
            Vector3 vec = omegaCorrected * dt * 0.5f;
            deltaQuat.w() = 1.0f;
            deltaQuat.vec() = vec;
            deltaQuat.normalize();

            Eigen::Quaternionf quatPred = quatK * deltaQuat;
            quatPred.normalize();

            // Write back predicted state
            sigmaPoints(0, i) = quatPred.w();
            sigmaPoints(1, i) = quatPred.x();
            sigmaPoints(2, i) = quatPred.y();
            sigmaPoints(3, i) = quatPred.z();
            sigmaPoints.block<3, 1>(4, i) = biasK;
        }

        // 3. Compute Predicted Mean
        stateVector.setZero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            float weight = weightsMean(i);
            stateVector += weight * sigmaPoints.col(i);
        }

        // Force Quaternion Normalization on the Mean
        Eigen::Map<Eigen::Vector4f> quatMean(stateVector.data());
        quatMean.normalize();

        // 4. Compute Predicted Covariance
        errorCovariance.setZero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            VectorState diff = sigmaPoints.col(i) - stateVector;
            float weight = weightsCovariance(i);
            MatrixState outerProduct = diff * diff.transpose().template cast<float>();
            errorCovariance += weight * outerProduct;
        }

        // Add Process Noise
        MatrixState noiseQ = MatrixState::Zero();
        noiseQ.block<4, 4>(0, 0) = MatrixState::Identity().block<4, 4>(0, 0) * (config.qGyro * dt);
        noiseQ.block<3, 3>(4, 4) = MatrixState::Identity().block<3, 3>(0, 0) * (config.qBias * dt);

        errorCovariance += noiseQ;
    }

    void AttitudeUkf::updateAccel(const Vector3& accelMeas)
    {
        // NED Convention: 1g points towards +Z (Down)
        const Vector3 gravityInertial(0.0f, 0.0f, 9.81f);

        generateSigmaPoints();

        Eigen::Matrix<float, MEAS_DIM_VEC, SIGMA_COUNT> zSig;

        // Transform Sigma Points to Measurement Space
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            Eigen::Quaternionf quatK(
                sigmaPoints(0, i),
                sigmaPoints(1, i),
                sigmaPoints(2, i),
                sigmaPoints(3, i)
            );

            // Expected accel = R(q)^T * g_inertial = q.conjugate * g
            zSig.col(i) = quatK.conjugate() * gravityInertial;
        }

        // Calculate Mean Predicted Measurement
        VectorMeas measMean = VectorMeas::Zero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            float weight = weightsMean(i);
            measMean += weight * zSig.col(i);
        }

        // Calculate Covariances
        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> sMat;
        sMat.setZero();
        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> tMat;
        tMat.setZero();

        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> rMat;
        rMat = Eigen::Matrix<float, 3, 3>::Identity() * config.rAccel;

        for (int i = 0; i < SIGMA_COUNT; ++i) {
            VectorMeas zDiff = zSig.col(i) - measMean;
            VectorState xDiff = sigmaPoints.col(i) - stateVector;

            float weight = weightsCovariance(i);

            // Explicit cast to float used to resolve expression template ambiguities
            Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> sUpdate = zDiff.template cast<float>() * zDiff.transpose();
            Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> tUpdate = xDiff.template cast<float>() * zDiff.transpose();

            sMat += weight * sUpdate;
            tMat += weight * tUpdate;
        }
        sMat += rMat;

        // Kalman Gain
        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> kGain;
        kGain = tMat.template cast<float>() * sMat.inverse();

        // Update State
        VectorMeas zActual = accelMeas;
        VectorMeas zResidual = zActual - measMean;
        stateVector += kGain.template cast<float>() * zResidual;

        // Normalize Quaternion Post-Update
        Eigen::Map<Eigen::Vector4f> quatFinal(stateVector.data());
        quatFinal.normalize();

        // Update Covariance
        errorCovariance -= kGain.template cast<float>() * sMat * kGain.transpose();
    }

    void AttitudeUkf::updateMag(const Vector3& magMeas, const Vector3& magRef)
    {
        generateSigmaPoints();

        Eigen::Matrix<float, MEAS_DIM_VEC, SIGMA_COUNT> zSig;

        for (int i = 0; i < SIGMA_COUNT; ++i) {
            Eigen::Quaternionf quatK(
                sigmaPoints(0, i),
                sigmaPoints(1, i),
                sigmaPoints(2, i),
                sigmaPoints(3, i)
            );
            zSig.col(i) = quatK.conjugate() * magRef;
        }

        VectorMeas measMean = VectorMeas::Zero();
        for (int i = 0; i < SIGMA_COUNT; ++i) {
            float weight = weightsMean(i);
            measMean += weight * zSig.col(i);
        }

        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> sMat = Eigen::Matrix<float, 3, 3>::Zero();
        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> tMat = Eigen::Matrix<float, 7, 3>::Zero();

        Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> rMat;
        rMat = Eigen::Matrix<float, 3, 3>::Identity() * config.rMag;

        for (int i = 0; i < SIGMA_COUNT; ++i) {
            VectorMeas zDiff = zSig.col(i) - measMean;
            VectorState xDiff = sigmaPoints.col(i) - stateVector;

            float weight = weightsCovariance(i);

            Eigen::Matrix<float, MEAS_DIM_VEC, MEAS_DIM_VEC> sUpdate = zDiff.template cast<float>() * zDiff.transpose();
            Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> tUpdate = xDiff.template cast<float>() * zDiff.transpose();

            sMat += weight * sUpdate;
            tMat += weight * tUpdate;
        }
        sMat += rMat;

        Eigen::Matrix<float, STATE_DIM, MEAS_DIM_VEC> kGain = tMat.template cast<float>() * sMat.inverse();

        stateVector += kGain * (magMeas - measMean);

        Eigen::Map<Eigen::Vector4f> quatFinal(stateVector.data());
        quatFinal.normalize();

        errorCovariance -= kGain.template cast<float>() * sMat * kGain.transpose();
    }

    const AttitudeUkf::VectorState& AttitudeUkf::getState() const
    {
        return stateVector;
    }

    const AttitudeUkf::MatrixState& AttitudeUkf::getCovariance() const
    {
        return errorCovariance;
    }

    Eigen::Quaternionf AttitudeUkf::getQuaternion() const
    {
        return Eigen::Quaternionf(stateVector(0), stateVector(1), stateVector(2), stateVector(3));
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

        // Robustness: Check if Cholesky failed (e.g., non-positive definite matrix).
        if (llt.info() != Eigen::Success) {
            // Numerical instability detected (likely due to float precision).
            // Reset covariance to a small identity matrix to recover.
            errorCovariance.setIdentity();
            errorCovariance *= 0.001f;
            llt.compute(errorCovariance);
        }

        MatrixState lMat = llt.matrixL();

        float gamma = std::sqrt(static_cast<float>(STATE_DIM) + lambdaParam);
        MatrixState weightedL = lMat * gamma;

        for (int i = 0; i < STATE_DIM; ++i) {
            sigmaPoints.col(i + 1) = stateVector + weightedL.col(i);
            sigmaPoints.col(i + 1 + STATE_DIM) = stateVector - weightedL.col(i);
        }
    }

} // namespace gnc
