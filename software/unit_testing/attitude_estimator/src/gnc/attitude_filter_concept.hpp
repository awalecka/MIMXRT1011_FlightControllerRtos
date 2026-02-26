/**
 * @file attitude_filter_concept.hpp
 * @brief C++20 concept defining the structural contract for attitude filters.
 *
 * Design rationale
 * ----------------
 * FlightController is templated on a filter policy type (see flight_controller.h).
 * Any class that satisfies AttitudeFilter can be used as the policy without
 * modifying the controller. This is static (compile-time) polymorphism:
 *
 * - Zero virtual dispatch overhead — the compiler sees the concrete type
 * and can inline all filter calls at the call site.
 * - Zero heap allocation — the filter object lives as a direct member of
 * FlightController, sized and aligned at compile time.
 * - No unused code in the final binary — only the instantiated filter is
 * compiled and linked.
 * - Extensible without touching flight_controller.h — a new filter that
 * satisfies AttitudeFilter drops in without any changes to the controller.
 *
 * Interface requirements
 * ----------------------
 * Every filter must expose:
 *
 * void init(Quaternionf, Vector3)
 * Set nominal quaternion and bias, reset error state and covariance.
 *
 * void align(Vector3 accel, Vector3 mag, Vector3& magRefOut)
 * Static alignment via TRIAD. Computes initial attitude and the inertial
 * magnetic reference vector for subsequent updateMag() calls.
 *
 * void predict(float dt, Vector3 omega)
 * Propagate nominal quaternion and error covariance by dt seconds.
 *
 * void updateAccel(Vector3 accel)
 * Measurement update from accelerometer [m/s²].
 *
 * void updateMag(Vector3 mag, Vector3 ref)
 * Measurement update from magnetometer with inertial reference.
 *
 * Quaternionf getQuaternion()
 * Current attitude estimate.
 *
 * Vector3 getEulerAnglesDeg()
 * Current attitude as [roll, pitch, yaw] in degrees, ZYX convention.
 *
 * Vector3 getBias()
 * Current gyro bias estimate [rad/s].
 *
 * uint32_t getCovarianceFaultCount()
 * Number of hard covariance resets since construction. Telemetry-visible
 * diagnostic.
 */

#ifndef ATTITUDE_FILTER_CONCEPT_HPP
#define ATTITUDE_FILTER_CONCEPT_HPP

#include <Eigen>
#include <concepts>
#include <cstdint>

namespace gnc {

/**
 * @brief Canonical tuning parameter bundle for all attitude filters.
 * Fields unused by a specific filter implementation are safely ignored.
 */
struct FilterConfig {
    // --- shared ---
    float qGyro     = 0.1f;    ///< Gyroscope rate noise PSD [rad²/s]
    float qBias     = 0.001f;  ///< Gyro bias random-walk PSD [rad²/s³]
    float rAccel    = 0.5f;    ///< Accelerometer noise variance [(m/s²)²]
    float rMag      = 1.0f;    ///< Magnetometer noise variance (normalised)

    // --- MEKF-specific ---
    float accelGate = 2.0f;    ///< Accel magnitude gate half-width [m/s²]. Set 0 to disable. Ignored by UKF.

    // --- UKF-specific ---
    float alpha     = 1.0f;    ///< Sigma-point spread factor. Ignored by MEKF.
    float beta      = 2.0f;    ///< Distribution prior. Ignored by MEKF.
    float kappa     = 0.0f;    ///< Secondary scaling. Ignored by MEKF.
};

/**
 * @brief Structural concept for attitude estimator policies.
 *
 * A type T satisfies AttitudeFilter if and only if it provides all of the
 * operations listed below with the correct signatures. The concept is checked
 * at the point of template instantiation in FlightController, so a violation
 * produces a clear compile-time diagnostic rather than a linker error.
 */
template <typename T>
concept AttitudeFilter = requires(
    T                       filter,
    Eigen::Quaternionf      q,
    Eigen::Matrix<float,3,1> v3,
    float                   f,
    uint32_t                u32)
{
    // ----- initialisation -----

    /** Set nominal state directly; called during re-init or after alignment. */
    { filter.init(q, v3) } -> std::same_as<void>;

    /** TRIAD static alignment; sets attitude and writes magnetic reference. */
    { filter.align(v3, v3, v3) } -> std::same_as<void>;

    // ----- main filter cycle -----

    /** Gyro-driven prediction step. */
    { filter.predict(f, v3) } -> std::same_as<void>;

    /** Accelerometer measurement update. */
    { filter.updateAccel(v3) } -> std::same_as<void>;

    /** Magnetometer measurement update with inertial reference. */
    { filter.updateMag(v3, v3) } -> std::same_as<void>;

    // ----- outputs -----

    /** Current attitude quaternion (always unit length). */
    { filter.getQuaternion() } -> std::same_as<Eigen::Quaternionf>;

    /** [Roll, Pitch, Yaw] in degrees, ZYX aerospace convention. */
    { filter.getEulerAnglesDeg() } -> std::same_as<Eigen::Matrix<float,3,1>>;

    /** Gyro bias estimate [rad/s]. */
    { filter.getBias() } -> std::same_as<Eigen::Matrix<float,3,1>>;

    /** Hard covariance reset counter for telemetry health monitoring. */
    { filter.getCovarianceFaultCount() } -> std::same_as<uint32_t>;
};

} // namespace gnc

#endif // ATTITUDE_FILTER_CONCEPT_HPP
