#ifndef RLS_MAGNETOMETER_CALIBRATION_H
#define RLS_MAGNETOMETER_CALIBRATION_H

#include "Eigen.h"
#include <random>
#include <cmath>
#include <limits>

template<typename T>
class RlsMagnetometerCalibration {
public:
    // Eigen type definitions based on the template type T
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
    using Vector9 = Eigen::Matrix<T, 9, 1>;
    using Matrix9 = Eigen::Matrix<T, 9, 9>;
    using RowVector9 = Eigen::Matrix<T, 1, 9>;

    RlsMagnetometerCalibration(T forgettingFactor = T(0.98), T initialCovariance = T(1000.0), T smoothing = T(0.01));

    void update(const Vector3& rawMagData);

    void setInitialCalibration(const Matrix3& softIronCorrection, const Vector3& hardIronOffset);

    Vector3 getHardIronOffset() const;
    Matrix3 getSoftIronCorrection() const;

    Vector3 getCalibratedData(const Vector3& rawMagData) const;
    T getEstimatedFieldStrength() const;

    template<typename TContainer>
    static void generateEllipsoidalSamples(TContainer& samples, const Matrix3& softIron, const Vector3& hardIron, T noiseStdDev);

private:
    // RLS filter parameters
    T lambda;
    Vector9 theta;
    Matrix9 P;

    // Field strength estimation
    T fieldStrengthSmoothing;
    T estimatedFieldStrengthSquared;

    // Stored calibration parameters
    Vector3 hardIronOffset_;
    Matrix3 softIronCorrection_;

    // Private method to update calibration parameters
    void updateCalibrationParameters();
};

// --- Template Function Definitions ---

template<typename T>
RlsMagnetometerCalibration<T>::RlsMagnetometerCalibration(T forgettingFactor, T initialCovariance, T smoothing)
    : lambda(forgettingFactor),
      fieldStrengthSmoothing(smoothing),
      estimatedFieldStrengthSquared(T(625.0)) { // Assuming initial field strength of 25 uT -> 25^2 = 625
    theta.setZero();
    theta(3) = T(1.0);
    theta(6) = T(1.0);
    theta(8) = T(1.0);

    P = Matrix9::Identity() * initialCovariance;

    hardIronOffset_.setZero();
    softIronCorrection_.setIdentity();
}

template<typename T>
void RlsMagnetometerCalibration<T>::update(const Vector3& rawMagData) {
    RowVector9 H;
    H << rawMagData.x(), rawMagData.y(), rawMagData.z(),
         -rawMagData.x() * rawMagData.x(), T(-2.0) * rawMagData.x() * rawMagData.y(), T(-2.0) * rawMagData.x() * rawMagData.z(),
         -rawMagData.y() * rawMagData.y(), T(-2.0) * rawMagData.y() * rawMagData.z(), -rawMagData.z() * rawMagData.z();

    T yHat = H * theta;
    T error = estimatedFieldStrengthSquared - yHat;

    Eigen::Matrix<T, 9, 1> K = (P * H.transpose()) / (lambda + H * P * H.transpose());

    theta = theta + K * error;
    P = (P - K * H * P) / lambda;

    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::setInitialCalibration(const Matrix3& softIronCorrection, const Vector3& hardIronOffset) {
    T B = getEstimatedFieldStrength();
    Matrix3 S = softIronCorrection * B;
    Matrix3 W = S.transpose() * S;
    Vector3 V = T(2.0) * W * hardIronOffset;

    theta.template head<3>() = V;
    theta(3) = W(0, 0); theta(4) = W(0, 1); theta(5) = W(0, 2);
    theta(6) = W(1, 1); theta(7) = W(1, 2); theta(8) = W(2, 2);

    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::updateCalibrationParameters() {
    Matrix3 W;
    W << theta(3), theta(4), theta(5),
         theta(4), theta(6), theta(7),
         theta(5), theta(7), theta(8);

    Vector3 V;
    V << theta(0), theta(1), theta(2);

    if (std::abs(W.determinant()) < std::numeric_limits<T>::epsilon()) {
        return;
    }
    hardIronOffset_ = W.inverse() * (V / T(2.0));

    Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(W);
    if (eigensolver.info() != Eigen::Success) {
        softIronCorrection_.setIdentity();
        return;
    }

    const T EPS = std::numeric_limits<T>::epsilon() * T(100.0);
    const auto& eigenvalues = eigensolver.eigenvalues();

    if (eigenvalues.minCoeff() < EPS) {
        softIronCorrection_.setIdentity();
        return;
    }

    Matrix3 V_eigen = eigensolver.eigenvectors();
    Matrix3 D_sqrt = eigenvalues.cwiseSqrt().asDiagonal();
    Matrix3 S = V_eigen * D_sqrt * V_eigen.transpose();

    T detS = S.determinant();
    if (std::abs(detS) < EPS) {
        softIronCorrection_.setIdentity();
        return;
    }

    T normFactor = cbrt(detS);
    softIronCorrection_ = S / normFactor;
}

template<typename T>
typename RlsMagnetometerCalibration<T>::Vector3 RlsMagnetometerCalibration<T>::getHardIronOffset() const {
    return hardIronOffset_;
}

template<typename T>
typename RlsMagnetometerCalibration<T>::Matrix3 RlsMagnetometerCalibration<T>::getSoftIronCorrection() const {
    return softIronCorrection_;
}

template<typename T>
typename RlsMagnetometerCalibration<T>::Vector3 RlsMagnetometerCalibration<T>::getCalibratedData(const Vector3& rawMagData) const {
    return softIronCorrection_ * (rawMagData - hardIronOffset_);
}

template<typename T>
T RlsMagnetometerCalibration<T>::getEstimatedFieldStrength() const {
    return sqrt(estimatedFieldStrengthSquared);
}

template<typename T>
template<typename TContainer>
void RlsMagnetometerCalibration<T>::generateEllipsoidalSamples(TContainer& samples, const Matrix3& softIron, const Vector3& hardIron, T noiseStdDev) {
    std::default_random_engine rng;
    std::uniform_real_distribution<T> distUnitSphere(T(0.0), T(1.0));
    std::normal_distribution<T> distNoise(T(0.0), noiseStdDev);

    for (size_t i = 0; i < samples.size(); ++i) {
        T u = distUnitSphere(rng);
        T v = distUnitSphere(rng);
        T thetaAzimuth = T(2.0 * M_PI) * u;
        T phiPolar = acos(T(2.0) * v - T(1.0));

        T xUnit = sin(phiPolar) * cos(thetaAzimuth);
        T yUnit = sin(phiPolar) * sin(thetaAzimuth);
        T zUnit = cos(phiPolar);
        Vector3 unitSpherePoint(xUnit, yUnit, zUnit);

        Vector3 distortedMag = softIron * unitSpherePoint + hardIron;

        if (noiseStdDev > 0.0) {
            distortedMag.x() += distNoise(rng);
            distortedMag.y() += distNoise(rng);
            distortedMag.z() += distNoise(rng);
        }
        samples[i] = distortedMag;
    }
}

using RlsMagnetometerCalibratorF = RlsMagnetometerCalibration<float>;
using RlsMagnetometerCalibratorD = RlsMagnetometerCalibration<double>;

#endif // RLS_MAGNETOMETER_CALIBRATION_H
