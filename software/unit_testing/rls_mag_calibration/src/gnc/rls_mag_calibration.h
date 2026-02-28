#ifndef RLS_MAGNETOMETER_CALIBRATION_H
#define RLS_MAGNETOMETER_CALIBRATION_H

#include <Eigen>
#include <random>
#include <cmath>
#include <limits>
#include <numbers>

/**
 * @brief Recursive Least Squares (RLS) Magnetometer Calibration.
 *
 * This class implements a real-time, online calibration algorithm for 3-axis magnetometers.
 * It estimates Hard Iron (offset) and Soft Iron (scaling/skew) distortions by fitting
 * incoming measurement data to an ellipsoid model using a Recursive Least Squares filter.
 *
 * The algorithm continuously updates the calibration parameters as new data arrives,
 * allowing it to adapt to changing magnetic environments over time.
 *
 * @tparam T Floating-point type (float or double).
 */
template<typename T>
class RlsMagnetometerCalibration {
public:
    // Eigen type definitions based on the template type T
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
    using Vector9 = Eigen::Matrix<T, 9, 1>;
    using Matrix9 = Eigen::Matrix<T, 9, 9>;
    using RowVector9 = Eigen::Matrix<T, 1, 9>;

    /**
     * @brief Construct a new Rls Magnetometer Calibration object.
     *
     * @param forgettingFactor Controls the rate of adaptation (lambda).
     * Values closer to 1.0 (e.g., 0.98-0.999) provide more stability and
     * noise rejection but slower convergence. Lower values track changes faster.
     * @param initialCovariance Initial value for the P matrix (inverse correlation matrix).
     * High values indicate high uncertainty in the initial state.
     * @param smoothing Smoothing factor for the estimated field strength B.
     */
    RlsMagnetometerCalibration(T forgettingFactor = T(0.98), T initialCovariance = T(1000.0), T smoothing = T(0.01));

    /**
     * @brief Updates the calibration model with a new magnetometer measurement.
     *
     * Performs a single step of the RLS algorithm:
     * Constructs the measurement vector H based on the quadric surface equation.
     * Calculates the prediction error against the current model.
     * Updates the gain vector K and the parameter vector theta.
     * Updates the covariance matrix P for the next step.
     * Re-calculates the physical hard/soft iron parameters from the updated theta.
     *
     * @param rawMagData Raw (uncalibrated) magnetometer reading [x, y, z].
     */
    void update(const Vector3& rawMagData);

    /**
     * @brief Sets the initial calibration parameters.
     *
     * Useful for initializing the filter with a known good calibration
     * stored in non-volatile memory to speed up convergence at startup.
     *
     * @param softIronCorrection The 3x3 Soft Iron correction matrix.
     * @param hardIronOffset The Hard Iron offset vector.
     */
    void setInitialCalibration(const Matrix3& softIronCorrection, const Vector3& hardIronOffset);

    /**
     * @brief Gets the current estimated Hard Iron offset.
     *
     * Hard Iron distortion is caused by permanent magnets or magnetized iron
     * fixed relative to the sensor, acting as an additive offset.
     *
     * @return Vector3 The offset vector [x, y, z].
     */
    Vector3 getHardIronOffset() const;

    /**
     * @brief Gets the current estimated Soft Iron correction matrix.
     *
     * Soft Iron distortion is caused by magnetically soft materials that
     * distort the magnetic field, resulting in scaling and skewing of the measurement sphere.
     *
     * @return Matrix3 The 3x3 correction matrix.
     */
    Matrix3 getSoftIronCorrection() const;

    /**
     * @brief Applies the current calibration to a raw measurement.
     *
     * Formula: Calibrated = SoftIronMatrix * (Raw - HardIronOffset)
     *
     * @param rawMagData Raw magnetometer reading.
     * @return Vector3 Calibrated magnetometer reading.
     */
    Vector3 getCalibratedData(const Vector3& rawMagData) const;

    /**
     * @brief Gets the estimated magnitude of the local magnetic field.
     *
     * @return T Field strength (radius of the fitted sphere).
     */
    T getEstimatedFieldStrength() const;

    /**
     * @brief Generates synthetic ellipsoidal data for testing/simulation.
     *
     * @tparam TContainer Container type (e.g., std::vector<Vector3>).
     * @param samples Container to fill with samples.
     * @param softIron Soft iron matrix to apply.
     * @param hardIron Hard iron offset to apply.
     * @param noiseStdDev Standard deviation of Gaussian noise to add.
     */
    template<typename TContainer>
    static void generateEllipsoidalSamples(TContainer& samples, const Matrix3& softIron, const Vector3& hardIron, T noiseStdDev);

private:
    // RLS filter parameters
    T lambda;      ///< Forgetting factor (0 < lambda <= 1)
    Vector9 theta; ///< Parameter vector representing the quadric surface coefficients
    Matrix9 P;     ///< Inverse correlation matrix

    // Field strength estimation
    T fieldStrengthSmoothing;
    T estimatedFieldStrengthSquared;

    // Stored calibration parameters (derived from theta)
    Vector3 hardIronOffset_;
    Matrix3 softIronCorrection_;

    /**
     * @brief Extracts physical calibration parameters from the RLS state vector theta.
     *
     * Solves the linear algebra problem to convert the algebraic parameters of the
     * fitted ellipsoid (theta) into geometric parameters (center and shape matrix).
     * Performs eigen-decomposition to ensure the resulting soft iron matrix is valid.
     */
    void updateCalibrationParameters();
};

// --- Template Function Definitions ---

template<typename T>
RlsMagnetometerCalibration<T>::RlsMagnetometerCalibration(T forgettingFactor, T initialCovariance, T smoothing)
    : lambda(forgettingFactor),
    fieldStrengthSmoothing(smoothing),
    estimatedFieldStrengthSquared(T(625.0)) {

    theta.setZero();
    theta(0) = T(1.0); // W11
    theta(1) = T(1.0); // W22
    theta(8) = -estimatedFieldStrengthSquared; // D

    // Initialize P matrix with large values to indicate high initial uncertainty
    P = Matrix9::Identity() * initialCovariance;

    hardIronOffset_.setZero();
    softIronCorrection_.setIdentity();
}

template<typename T>
void RlsMagnetometerCalibration<T>::update(const Vector3& rawMagData) {
    T x = rawMagData.x();
    T y = rawMagData.y();
    T z = rawMagData.z();

    // Target is -x^2. Anchors W00 = 1.0 to eliminate the scale ambiguity null space.
    T yHatTarget = -x * x;

    // H = [y^2, z^2, 2xy, 2xz, 2yz, x, y, z, 1] 
    // Sign convention for x, y, z fixed to positive
    RowVector9 H;
    H << y * y, z* z,
        T(2.0)* x* y, T(2.0)* x* z, T(2.0)* y* z,
        x, y, z,
        T(1.0);

    T yHat = H * theta;
    T error = yHatTarget - yHat;

    Eigen::Matrix<T, 9, 1> K = (P * H.transpose()) / (lambda + H * P * H.transpose());
    theta = theta + K * error;

    // Symmetric Joseph form update for improved numerical stability
    Matrix9 I = Matrix9::Identity();
    Matrix9 I_KH = I - K * H;
    P = (I_KH * P * I_KH.transpose() + K * K.transpose()) / lambda;

    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::setInitialCalibration(const Matrix3& softIronCorrection, const Vector3& hardIronOffset) {
    T B = getEstimatedFieldStrength();

    Matrix3 W_raw = softIronCorrection.transpose() * softIronCorrection;

    T scale = T(1.0) / W_raw(0, 0);
    Matrix3 W = W_raw * scale;

    // Fixed sign matching the algebraic quadric equation derivation
    Vector3 V = T(-2.0) * W * hardIronOffset;

    T D = hardIronOffset.dot(W * hardIronOffset) - (B * B * scale);

    theta(0) = W(1, 1);
    theta(1) = W(2, 2);
    theta(2) = W(0, 1);
    theta(3) = W(0, 2);
    theta(4) = W(1, 2);
    theta(5) = V.x();
    theta(6) = V.y();
    theta(7) = V.z();
    theta(8) = D;

    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::updateCalibrationParameters() {
    Matrix3 W;
    W << T(1.0), theta(2), theta(3),
        theta(2), theta(0), theta(4),
        theta(3), theta(4), theta(1);

    Vector3 V;
    V << theta(5), theta(6), theta(7);

    T D = theta(8);

    if (std::abs(W.determinant()) < std::numeric_limits<T>::epsilon()) {
        return;
    }

    // 1. Calculate Hard Iron Offset (Fixed sign extraction)
    hardIronOffset_ = W.inverse() * (-V / T(2.0));

    // 2. Dynamically estimate the True Field Strength
    T newFieldStrengthSq = hardIronOffset_.dot(W * hardIronOffset_) - D;
    if (newFieldStrengthSq > T(0.0)) {
        estimatedFieldStrengthSquared = newFieldStrengthSq;
    }

    // Calculate the Soft Iron Correction using Eigen Decomposition of W
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
    // S = W^(1/2), the matrix square root of W (encodes the forward distortion shape)
    Matrix3 D_sqrt = eigenvalues.cwiseSqrt().asDiagonal();
    Matrix3 S = V_eigen * D_sqrt * V_eigen.transpose();

    T detS = S.determinant();
    if (std::abs(detS) < EPS) {
        softIronCorrection_.setIdentity();
        return;
    }

    // 3. Invert S to get the correction matrix that undoes the distortion.
    //    S_inv = W^(-1/2) = V * D^(-1/2) * V^T (safe since all eigenvalues > EPS)
    Matrix3 D_sqrt_inv = eigenvalues.cwiseInverse().cwiseSqrt().asDiagonal();
    Matrix3 S_inv = V_eigen * D_sqrt_inv * V_eigen.transpose();

    // 4. Normalize by cbrt(det(S_inv)) to make the correction volume-preserving (det = 1).
    T detS_inv = T(1.0) / detS; // det(S_inv) = 1/det(S)
    T normFactor = cbrt(detS_inv);
    softIronCorrection_ = S_inv / normFactor;
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
    T safeStdDev = (noiseStdDev > T(0.0)) ? noiseStdDev : T(1e-6);
    std::normal_distribution<T> distNoise(T(0.0), safeStdDev);

    for (size_t i = 0; i < samples.size(); ++i) {
        // Generate random points on a unit sphere using spherical coordinates
        T u = distUnitSphere(rng);
        T v = distUnitSphere(rng);
        T thetaAzimuth = T(2.0 * static_cast<float>(std::numbers::pi_v<double>)) * u;
        T phiPolar = acos(T(2.0) * v - T(1.0));

        T xUnit = sin(phiPolar) * cos(thetaAzimuth);
        T yUnit = sin(phiPolar) * sin(thetaAzimuth);
        T zUnit = cos(phiPolar);
        Vector3 unitSpherePoint(xUnit, yUnit, zUnit);

        // Apply distortions to simulate raw sensor data
        Vector3 distortedMag = softIron * unitSpherePoint + hardIron;

        // Add Gaussian noise if requested
        if (noiseStdDev > 0.0) {
            distortedMag.x() += distNoise(rng);
            distortedMag.y() += distNoise(rng);
            distortedMag.z() += distNoise(rng);
        }
        samples[i] = distortedMag;
    }
}

// Instantiate common template specializations
using RlsMagnetometerCalibratorF = RlsMagnetometerCalibration<float>;
using RlsMagnetometerCalibratorD = RlsMagnetometerCalibration<double>;

#endif // RLS_MAGNETOMETER_CALIBRATION_H