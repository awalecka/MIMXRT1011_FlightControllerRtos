#ifndef RLS_MAGNETOMETER_CALIBRATION_H
#define RLS_MAGNETOMETER_CALIBRATION_H

#include "Eigen.h"
#include <random>
#include <cmath>
#include <limits>

template<typename T>
class RLS_MagnetometerCalibration {
public:
    // Eigen type definitions based on the template type T
    using Vector3 = Eigen::Matrix<T, 3, 1>;
    using Matrix3 = Eigen::Matrix<T, 3, 3>;
    using Vector9 = Eigen::Matrix<T, 9, 1>;
    using Matrix9 = Eigen::Matrix<T, 9, 9>;
    using RowVector9 = Eigen::Matrix<T, 1, 9>;

    RLS_MagnetometerCalibration(T forgetting_factor = T(0.98), T initial_covariance = T(1000.0), T smoothing = T(0.01));

    void update(const Vector3& raw_mag_data);

    void set_initial_calibration(const Matrix3& soft_iron_correction, const Vector3& hard_iron_offset);

    Vector3 get_hard_iron_offset() const;
    Matrix3 get_soft_iron_correction() const;

    Vector3 get_calibrated_data(const Vector3& raw_mag_data) const;
    T get_estimated_field_strength() const;

    template<typename TContainer>
    static void generateEllipsoidalSamples(TContainer& samples, const Matrix3& softIron, const Vector3& hardIron, T noise_std_dev);

private:
    // RLS filter parameters
    T lambda;
    Vector9 theta;
    Matrix9 P;

    // Field strength estimation
    T field_strength_smoothing;
    T estimated_field_strength_squared;

    // Stored calibration parameters
    Vector3 hard_iron_offset_;
    Matrix3 soft_iron_correction_;

    // Private method to update calibration parameters
    void update_calibration_parameters();
};


// --- Template Function Definitions ---
// Must be in the header file

template<typename T>
RLS_MagnetometerCalibration<T>::RLS_MagnetometerCalibration(T forgetting_factor, T initial_covariance, T smoothing)
    : lambda(forgetting_factor),
      field_strength_smoothing(smoothing),
      estimated_field_strength_squared(T(625.0)) { // Assuming initial field strength of 25 uT -> 25^2 = 625
    theta.setZero();
    theta(3) = T(1.0);
    theta(6) = T(1.0);
    theta(8) = T(1.0);

    P = Matrix9::Identity() * initial_covariance;

    hard_iron_offset_.setZero();
    soft_iron_correction_.setIdentity();
}

template<typename T>
void RLS_MagnetometerCalibration<T>::update(const Vector3& raw_mag_data) {
    RowVector9 H;
    H << raw_mag_data.x(), raw_mag_data.y(), raw_mag_data.z(),
         -raw_mag_data.x() * raw_mag_data.x(), T(-2.0) * raw_mag_data.x() * raw_mag_data.y(), T(-2.0) * raw_mag_data.x() * raw_mag_data.z(),
         -raw_mag_data.y() * raw_mag_data.y(), T(-2.0) * raw_mag_data.y() * raw_mag_data.z(), -raw_mag_data.z() * raw_mag_data.z();

    T y_hat = H * theta;
    T error = estimated_field_strength_squared - y_hat;

    Eigen::Matrix<T, 9, 1> K = (P * H.transpose()) / (lambda + H * P * H.transpose());

    theta = theta + K * error;
    P = (P - K * H * P) / lambda;

    update_calibration_parameters();
}

template<typename T>
void RLS_MagnetometerCalibration<T>::set_initial_calibration(const Matrix3& soft_iron_correction, const Vector3& hard_iron_offset) {
    // This function reverse-engineers the internal state vector 'theta'
    // from the desired calibration matrices.

    // 1. Reconstruct the un-normalized soft-iron matrix 'S'
    T B = get_estimated_field_strength(); // Assumes field strength is the scaling factor
    Matrix3 S = soft_iron_correction * B;

    // 2. Reconstruct the ellipsoid matrix 'W' from S (W = S' * S)
    Matrix3 W = S.transpose() * S;

    // 3. Reconstruct the ellipsoid vector 'V' from W and the hard iron offset
    // V = 2 * W * hard_iron
    Vector3 V = T(2.0) * W * hard_iron_offset;

    // 4. Populate the internal state vector 'theta'
    // V part
    theta.template head<3>() = V;
    // W part
    theta(3) = W(0, 0);
    theta(4) = W(0, 1);
    theta(5) = W(0, 2);
    theta(6) = W(1, 1);
    theta(7) = W(1, 2);
    theta(8) = W(2, 2);

    // 5. Update the stored member variables to match
    update_calibration_parameters();
}


template<typename T>
void RLS_MagnetometerCalibration<T>::update_calibration_parameters() {
    Matrix3 W;
    W << theta(3), theta(4), theta(5),
         theta(4), theta(6), theta(7),
         theta(5), theta(7), theta(8);

    Vector3 V;
    V << theta(0), theta(1), theta(2);

    // Check for invertibility
    if (std::abs(W.determinant()) < std::numeric_limits<T>::epsilon()) {
        // If W is singular, cannot compute hard iron offset, return early
        return;
    }
    hard_iron_offset_ = W.inverse() * (V / T(2.0));

    Eigen::SelfAdjointEigenSolver<Matrix3> eigensolver(W);
    if (eigensolver.info() != Eigen::Success) {
        soft_iron_correction_.setIdentity();
        return;
    }

    const T EPS = std::numeric_limits<T>::epsilon() * T(100.0);
    const auto& eigenvalues = eigensolver.eigenvalues();

    if (eigenvalues.minCoeff() < EPS) {
        soft_iron_correction_.setIdentity();
        return;
    }

    Matrix3 V_eigen = eigensolver.eigenvectors();
    Matrix3 D_sqrt = eigenvalues.cwiseSqrt().asDiagonal();
    Matrix3 S = V_eigen * D_sqrt * V_eigen.transpose();

    T det_S = S.determinant();
    if (std::abs(det_S) < EPS) {
        soft_iron_correction_.setIdentity();
        return;
    }

    T norm_factor = cbrt(det_S);
    soft_iron_correction_ = S / norm_factor;
}

template<typename T>
typename RLS_MagnetometerCalibration<T>::Vector3 RLS_MagnetometerCalibration<T>::get_hard_iron_offset() const {
    return hard_iron_offset_;
}

template<typename T>
typename RLS_MagnetometerCalibration<T>::Matrix3 RLS_MagnetometerCalibration<T>::get_soft_iron_correction() const {
    return soft_iron_correction_;
}

template<typename T>
typename RLS_MagnetometerCalibration<T>::Vector3 RLS_MagnetometerCalibration<T>::get_calibrated_data(const Vector3& raw_mag_data) const {
    return soft_iron_correction_ * (raw_mag_data - hard_iron_offset_);
}

template<typename T>
T RLS_MagnetometerCalibration<T>::get_estimated_field_strength() const {
    return sqrt(estimated_field_strength_squared);
}

template<typename T>
template<typename TContainer>
void RLS_MagnetometerCalibration<T>::generateEllipsoidalSamples(TContainer& samples, const Matrix3& softIron, const Vector3& hardIron, T noise_std_dev) {
    std::default_random_engine rng;
    std::uniform_real_distribution<T> dist_unit_sphere(T(0.0), T(1.0));
    std::normal_distribution<T> dist_noise(T(0.0), noise_std_dev);

    for (size_t i = 0; i < samples.size(); ++i) {
        T u = dist_unit_sphere(rng);
        T v = dist_unit_sphere(rng);
        T theta_azimuth = T(2.0 * M_PI) * u;
        T phi_polar = acos(T(2.0) * v - T(1.0));

        T x_unit = sin(phi_polar) * cos(theta_azimuth);
        T y_unit = sin(phi_polar) * sin(theta_azimuth);
        T z_unit = cos(phi_polar);
        Vector3 unit_sphere_point(x_unit, y_unit, z_unit);

        Vector3 distorted_mag = softIron * unit_sphere_point + hardIron;

        if (noise_std_dev > 0.0) {
            distorted_mag.x() += dist_noise(rng);
            distorted_mag.y() += dist_noise(rng);
            distorted_mag.z() += dist_noise(rng);
        }
        samples[i] = distorted_mag;
    }
}

// --- Convenience Type Aliases ---
using RlsMagnetometerCalibratorF = RLS_MagnetometerCalibration<float>;
using RlsMagnetometerCalibratorD = RLS_MagnetometerCalibration<double>;

#endif // RLS_MAGNETOMETER_CALIBRATION_H
