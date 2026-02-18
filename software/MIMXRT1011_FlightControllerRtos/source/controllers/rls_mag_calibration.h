#ifndef RLS_MAGNETOMETER_CALIBRATION_H
#define RLS_MAGNETOMETER_CALIBRATION_H

#include <Eigen>
#include <random>
#include <cmath>
#include <limits>

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
      estimatedFieldStrengthSquared(T(625.0)) { // Assuming initial field strength of 25 uT -> 25^2 = 625
    // Initialize theta to represent a unit sphere initially
    theta.setZero();
    theta(3) = T(1.0);
    theta(6) = T(1.0);
    theta(8) = T(1.0);

    // Initialize P matrix with large values to indicate high initial uncertainty
    P = Matrix9::Identity() * initialCovariance;

    hardIronOffset_.setZero();
    softIronCorrection_.setIdentity();
}

template<typename T>
void RlsMagnetometerCalibration<T>::update(const Vector3& rawMagData) {
    // Construct the measurement vector H based on the ellipsoid equation components
    // H = [x, y, z, -x^2, -2xy, -2xz, -y^2, -2yz, -z^2]
    RowVector9 H;
    H << rawMagData.x(), rawMagData.y(), rawMagData.z(),
         -rawMagData.x() * rawMagData.x(), T(-2.0) * rawMagData.x() * rawMagData.y(), T(-2.0) * rawMagData.x() * rawMagData.z(),
         -rawMagData.y() * rawMagData.y(), T(-2.0) * rawMagData.y() * rawMagData.z(), -rawMagData.z() * rawMagData.z();

    // Calculate the estimated output and the prediction error
    T yHat = H * theta;
    T error = estimatedFieldStrengthSquared - yHat;

    // Calculate gain vector K using the standard RLS formula
    Eigen::Matrix<T, 9, 1> K = (P * H.transpose()) / (lambda + H * P * H.transpose());

    // Update the state vector theta and the covariance matrix P
    theta = theta + K * error;
    P = (P - K * H * P) / lambda;

    // Convert the algebraic ellipsoid parameters (theta) into geometric calibration data
    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::setInitialCalibration(const Matrix3& softIronCorrection, const Vector3& hardIronOffset) {
    // Reverse engineer the algebraic parameters (theta) from the known geometric ones
    T B = getEstimatedFieldStrength();
    Matrix3 S = softIronCorrection * B;
    Matrix3 W = S.transpose() * S;
    Vector3 V = T(2.0) * W * hardIronOffset;

    // Populate theta vector
    theta.template head<3>() = V;
    theta(3) = W(0, 0); theta(4) = W(0, 1); theta(5) = W(0, 2);
    theta(6) = W(1, 1); theta(7) = W(1, 2); theta(8) = W(2, 2);

    updateCalibrationParameters();
}

template<typename T>
void RlsMagnetometerCalibration<T>::updateCalibrationParameters() {
    // Extract the shape matrix W and the center vector V from theta
    Matrix3 W;
    W << theta(3), theta(4), theta(5),
         theta(4), theta(6), theta(7),
         theta(5), theta(7), theta(8);

    Vector3 V;
    V << theta(0), theta(1), theta(2);

    // Check for singularity before inversion
    if (std::abs(W.determinant()) < std::numeric_limits<T>::epsilon()) {
        return;
    }

    // Calculate the Hard Iron Offset (center of the ellipsoid)
    hardIronOffset_ = W.inverse() * (V / T(2.0));

    // Calculate the Soft Iron Correction using Eigen Decomposition of W
    Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(W);
    if (eigensolver.info() != Eigen::Success) {
        softIronCorrection_.setIdentity();
        return;
    }

    const T EPS = std::numeric_limits<T>::epsilon() * T(100.0);
    const auto& eigenvalues = eigensolver.eigenvalues();

    // Ensure all eigenvalues are positive (ellipsoid constraint)
    if (eigenvalues.minCoeff() < EPS) {
        softIronCorrection_.setIdentity();
        return;
    }

    // Compute the correction matrix
    Matrix3 V_eigen = eigensolver.eigenvectors();
    Matrix3 D_sqrt = eigenvalues.cwiseSqrt().asDiagonal();
    Matrix3 S = V_eigen * D_sqrt * V_eigen.transpose();

    // Normalize the matrix to preserve field magnitude
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
        // Generate random points on a unit sphere using spherical coordinates
        T u = distUnitSphere(rng);
        T v = distUnitSphere(rng);
        T thetaAzimuth = T(2.0 * M_PI) * u;
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
