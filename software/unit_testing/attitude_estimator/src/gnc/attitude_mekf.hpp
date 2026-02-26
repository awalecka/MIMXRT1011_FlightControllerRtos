/**
 * @file attitude_mekf.hpp
 * @brief Multiplicative Extended Kalman Filter (MEKF) for Attitude Estimation.
 *
 * Implements a linearised MEKF over a 6D error state:
 * [delta_r (3D rotation vector error), delta_b (3D gyro bias error)]
 *
 * A nominal quaternion is maintained separately and updated via a
 * multiplicative reset after each measurement correction. This avoids
 * the quaternion unit-constraint violation that occurs in additive filters
 * and eliminates the 7x7 rank-deficiency present in full-state UKF approaches.
 *
 * Compared to the MEKF-UKF hybrid this replaces, the sigma-point machinery
 * is removed entirely. The nonlinear process and measurement functions are
 * linearised analytically, producing Jacobians F and H that are cheap to
 * compute on the Cortex-M7 FPU and exact to first order for the small error
 * state that the MEKF formulation guarantees.
 *
 * Frame convention: NED (North-East-Down). Gravity is +Z inertial.
 * Target: NXP RT1011 (Cortex-M7, 500 MHz, single-precision FPU).
 *
 * References:
 * Crassidis & Markley, "Unscented Filtering for Spacecraft Attitude
 * Estimation", AIAA Journal of Guidance, 2003.
 * Trawny & Roumeliotis, "Indirect Kalman Filter for 3D Attitude
 * Estimation", UMN TR-2005-002, 2005.
 */

#ifndef ATTITUDE_MEKF_HPP
#define ATTITUDE_MEKF_HPP

#include "attitude_filter_concept.hpp"
#include <Eigen>
#include <cmath>
#include <cstdint>

namespace gnc {

/**
 * @brief Linearised Multiplicative Extended Kalman Filter for attitude
 * estimation from gyroscope, accelerometer, and magnetometer.
 *
 * Error State x_err [6x1]: [delta_rx, delta_ry, delta_rz,
 * delta_bx, delta_by, delta_bz]
 *
 * The attitude error (delta_r) is a 3D rotation vector representing the
 * small rotation from the nominal quaternion to the true quaternion.
 * The bias error (delta_b) is the deviation of the gyro bias from the
 * current nominal bias estimate.
 *
 * After each update, delta_r is absorbed into the nominal quaternion via
 * the MEKF reset, zeroing the attitude error sub-state while the bias
 * sub-state continues to accumulate corrections.
 */
class AttitudeMekf {
public:
    // Dimension constants
    static constexpr int ERROR_DIM = 6; ///< Error state dimension [delta_r, delta_b]
    static constexpr int MEAS_DIM  = 3; ///< Measurement dimension (vector observation)

    // Convenience type aliases — all fixed-size for zero heap allocation
    using Vector3      = Eigen::Matrix<float, 3, 1>;
    using Matrix3      = Eigen::Matrix<float, 3, 3>;
    using ErrorVec     = Eigen::Matrix<float, ERROR_DIM, 1>;
    using ErrorMat     = Eigen::Matrix<float, ERROR_DIM, ERROR_DIM>;
    using MeasVec      = Eigen::Matrix<float, MEAS_DIM, 1>;
    using MeasMat      = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>;
    using KalmanGain   = Eigen::Matrix<float, ERROR_DIM, MEAS_DIM>;
    using Jacobian     = Eigen::Matrix<float, MEAS_DIM, ERROR_DIM>;

    /**
     * @brief Construct the filter with the given tuning configuration.
     * @param config Unified filter configuration parameters.
     */
    explicit AttitudeMekf(const FilterConfig& config);

    /**
     * @brief Set the nominal state directly.
     *
     * Resets the error state to zero and resets the covariance to a
     * conservative diagonal. Use align() for sensor-derived initialisation.
     *
     * @param initialQuat Initial attitude quaternion [w, x, y, z], need not
     * be normalised (will be normalised internally).
     * @param initialBias Initial gyro bias estimate [rad/s].
     */
    void init(const Eigen::Quaternionf& initialQuat, const Vector3& initialBias);

    /**
     * @brief Derive initial attitude and magnetic reference from static
     * sensor averages (TRIAD method).
     *
     * Computes the initial Roll, Pitch, and tilt-compensated Yaw from the
     * averaged accelerometer and magnetometer readings. Sets the nominal
     * quaternion and zeroes the bias estimate. Also computes the inertial
     * magnetic reference vector (dip angle) for use in subsequent
     * updateMag() calls.
     *
     * Avoids Euler-angle singularities by using the TRIAD vector method
     * rather than atan2-based decomposition.
     *
     * @param accelMean  Static-averaged accelerometer reading [m/s^2].
     * NED convention: gravity is along +Z, so a level
     * stationary vehicle reads approximately [0, 0, 9.81].
     * @param magMean    Static-averaged magnetometer reading [uT or Gauss].
     * @param magRefOut  [out] Normalised inertial magnetic reference vector
     * [North, 0, Down] in the NED frame. Store this and
     * pass it to every subsequent updateMag() call.
     */
    void align(const Vector3& accelMean, const Vector3& magMean, Vector3& magRefOut);

    /**
     * @brief Prediction step — propagate nominal quaternion and grow P.
     *
     * Integrates the nominal quaternion using the exact closed-form
     * rotation (not a small-angle approximation). Propagates the error
     * covariance using the analytically derived process Jacobian F.
     *
     * @param dt        Time step [s].
     * @param omegaMeas Raw gyroscope measurement [rad/s] in body frame.
     */
    void predict(float dt, const Vector3& omegaMeas);

    /**
     * @brief Measurement update — accelerometer.
     *
     * Treats the normalised accelerometer reading as a gravity vector
     * observation. The update is gated on the accelerometer magnitude:
     * readings too far from 1g are rejected as contaminated by linear
     * acceleration. Configure the gate width via FilterConfig::accelGate.
     *
     * Both the measurement and the predicted gravity are normalised before
     * computing the residual to make the update scale-invariant.
     *
     * @param accelMeas Raw accelerometer measurement [m/s^2].
     */
    void updateAccel(const Vector3& accelMeas);

    /**
     * @brief Measurement update — magnetometer.
     *
     * Treats the normalised magnetometer reading as a magnetic field vector
     * observation. Both measurement and reference are normalised internally.
     *
     * @param magMeas  Raw magnetometer measurement [uT or Gauss].
     * @param magRef   Inertial magnetic reference vector (output of align()).
     * Need not be unit-length; will be normalised internally.
     */
    void updateMag(const Vector3& magMeas, const Vector3& magRef);

    /**
     * @brief Return the current nominal quaternion (attitude estimate).
     * @return Normalised Eigen::Quaternionf [w, x, y, z].
     */
    [[nodiscard]] Eigen::Quaternionf getQuaternion() const;

    /**
     * @brief Return the current gyro bias estimate [rad/s].
     * @return Vector3 [bx, by, bz].
     */
    [[nodiscard]] Vector3 getBias() const;

    /**
     * @brief Return the 6x6 error covariance matrix.
     * @return Const reference — upper-left 3x3 is attitude uncertainty,
     * lower-right 3x3 is bias uncertainty.
     */
    [[nodiscard]] const ErrorMat& getCovariance() const;

    /**
     * @brief Derive Euler angles from the current nominal quaternion.
     *
     * ZYX convention (yaw-pitch-roll, aerospace standard).
     * Pitch is clamped to +-90 degrees at the gimbal-lock singularity.
     *
     * @return Vector3 [roll, pitch, yaw] in degrees.
     */
    [[nodiscard]] Vector3 getEulerAnglesDeg() const;

    /**
     * @brief Return the number of hard covariance resets since construction.
     *
     * A non-zero value indicates the filter has recovered from numerical
     * instability. Expose via telemetry to detect degraded operation.
     *
     * @return Reset count.
     */
    [[nodiscard]] uint32_t getCovarianceFaultCount() const;

private:
    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    Eigen::Quaternionf nominalQuat; ///< Nominal attitude (always unit length)
    Vector3            nominalBias; ///< Nominal gyro bias estimate [rad/s]
    ErrorVec           errorState;  ///< 6D error state [delta_r, delta_b]
    ErrorMat           errorCov;    ///< 6x6 error covariance

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------
    FilterConfig config;

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------
    uint32_t covFaultCount; ///< Count of hard covariance identity resets

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /**
     * @brief Generic vector-observation update (shared by accel and mag).
     *
     * Given a body-frame measurement and its inertial reference, computes
     * the analytically linearised measurement Jacobian H, forms the
     * innovation covariance S, computes the Kalman gain K, updates the
     * 6D error state, performs the MEKF reset (absorbs attitude error into
     * the nominal quaternion and bias error into the nominal bias), and
     * re-symmetrises the covariance.
     *
     * @param measNorm   Unit-vector measurement in the body frame.
     * @param refNorm    Unit-vector reference in the inertial frame.
     * @param rNoise     Measurement noise variance scalar.
     */
    void updateVectorObservation(const Vector3& measNorm,
                                 const Vector3& refNorm,
                                 float rNoise);

    /**
     * @brief Repair or reset the covariance matrix if it is no longer
     * symmetric positive-definite.
     *
     * Attempts re-symmetrisation and a small ridge addition first.
     * Falls back to a conservative diagonal identity reset only if the
     * repaired matrix still fails an LLT decomposition. Increments
     * covFaultCount on every hard reset.
     */
    void repairCovariance();

    /**
     * @brief Compute the 3x3 skew-symmetric (cross-product) matrix of v.
     * @param v Input vector.
     * @return 3x3 skew-symmetric matrix [v]x such that [v]x * u = v x u.
     */
    static Matrix3 skew(const Vector3& v);

    /**
     * @brief Convert a small rotation vector to a quaternion.
     *
     * Uses the exact formula for |rv| > 1e-6 rad and the first-order
     * approximation for smaller angles to avoid divide-by-zero.
     *
     * @param rv Rotation vector [rad]. Magnitude is the rotation angle.
     * @return Unit quaternion representing the rotation.
     */
    static Eigen::Quaternionf rotVecToQuat(const Vector3& rv);
};

} // namespace gnc

#endif // ATTITUDE_MEKF_HPP
