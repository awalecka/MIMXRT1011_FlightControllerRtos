/**
 * @file attitude_ukf.hpp
 * @brief Unscented Kalman Filter for Attitude Estimation (Quaternions).
 *
 * Implements a 7-state UKF (4 Quaternion + 3 Gyro Bias).
 * Target: NXP RT1011 (Cortex-M7).
 */

#ifndef ATTITUDE_UKF_HPP
#define ATTITUDE_UKF_HPP

#include <Eigen/Dense>
#include <cmath>

namespace gnc {

/**
 * @brief Unscented Kalman Filter for Attitude Estimation (MEKF formulation).
 *
 * State Vector Error x [6x1]: [dx, dy, dz, dbx, dby, dbz]
 * (3D Rotation Vector Error + 3D Gyro Bias Error)
 * 
 * Uses static allocation via Eigen::Matrix fixed sizes.
 */
class AttitudeUkf {
public:
    // Constants
    static constexpr int STATE_DIM = 6;
    static constexpr int MEAS_DIM_VEC = 3; // For Accel/Mag (3-axis)
    static constexpr int SIGMA_COUNT = 2 * STATE_DIM + 1;

    // Type Definitions for Readability and Safety
    using VectorState   = Eigen::Matrix<float, STATE_DIM, 1>;
    using MatrixState   = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
    using Vector3       = Eigen::Matrix<float, 3, 1>;
    using VectorMeas    = Eigen::Matrix<float, MEAS_DIM_VEC, 1>;
    using VectorWeights = Eigen::Matrix<float, SIGMA_COUNT, 1>;

    /**
     * @brief Configuration struct for process and measurement noise.
     */
    struct Config {
        float alpha;  ///< Sigma-point spread factor (typical range: 1e-3 to 1).
        float beta;   ///< Distribution prior; 2 is optimal for a Gaussian.
        float kappa;  ///< Secondary scaling parameter (typically 0).

        /// @brief Gyroscope angle-rate noise power spectral density [rad^2/s].
        /// Pass the variance per unit time (sigma_gyro^2), NOT the standard
        /// deviation. The filter adds (qGyro * dt) to the attitude error
        /// variance each prediction step.
        float qGyro;

        /// @brief Gyro bias random-walk power spectral density [rad^2/s^3].
        /// Pass the variance per unit time (sigma_bias^2), NOT the standard
        /// deviation. The filter adds (qBias * dt) to the bias variance each
        /// prediction step.
        float qBias;

        /// @brief Accelerometer measurement noise variance [(m/s^2)^2].
        /// Pass sigma_accel^2. Applied as a scalar multiple of the 3x3
        /// identity matrix in the innovation covariance.
        float rAccel;

        /// @brief Magnetometer measurement noise variance [uT^2] (or
        /// dimensionless if the measurement is normalised before calling
        /// updateMag). Pass sigma_mag^2.
        float rMag;
    };

    /**
     * @brief Constructor.
     * @param config Initial filter configuration.
     */
    explicit AttitudeUkf(const Config& config);

    /**
     * @brief Initialize the state.
     * @param initialQuat Initial quaternion [w, x, y, z].
     * @param initialBias Initial gyro bias [x, y, z].
     */
    void init(const Eigen::Quaternionf& initialQuat, const Vector3& initialBias);

    /**
     * @brief Performs initial alignment using static sensor data.
     * * Calculates the initial Roll and Pitch from the gravity vector (accel),
     * and the initial Yaw from the magnetometer (tilt-compensated).
     * Initializes the internal state vector and calculates the inertial
     * magnetic field reference (Dip Angle).
     *
     * @param accelMean Averaged Accelerometer vector [m/s^2] (Gravity Vector, Down=+).
     * @param magMean   Averaged Magnetometer vector [Gauss/uT].
     * @param magRefOut [Out] Calculated inertial magnetic reference vector.
     */
    void align(const Vector3& accelMean, const Vector3& magMean, Vector3& magRefOut);

    /**
     * @brief Prediction Step.
     * Propagates the state using Gyroscope data.
     *
     * @param dt Time step in seconds.
     * @param omegaMeas Measured angular velocity (rad/s).
     */
    void predict(float dt, const Vector3& omegaMeas);

    /**
     * @brief Update Step: Accelerometer.
     * Corrects attitude using gravity vector.
     *
     * @param accelMeas Measured acceleration (m/s^2).
     */
    void updateAccel(const Vector3& accelMeas);

    /**
     * @brief Update Step: Magnetometer.
     * Corrects attitude using magnetic field vector.
     *
     * @param magMeas Measured magnetic field (normalized or uT).
     * @param magRef Inertial reference magnetic field vector.
     */
    void updateMag(const Vector3& magMeas, const Vector3& magRef);

    /**
     * @brief Get the current State Estimate.
     * @return Const reference to current state vector.
     */
    [[nodiscard]] const VectorState& getState() const;

    /**
     * @brief Get the current Error Covariance Matrix.
     * @return Const reference to covariance matrix.
     */
    [[nodiscard]] const MatrixState& getCovariance() const;

    /**
     * @brief Get the current Rotation as a Quaternion.
     * @return Eigen::Quaternionf
     */
    [[nodiscard]] Eigen::Quaternionf getQuaternion() const;

    /**
     * @brief Get the current attitude as Euler Angles in degrees.
     * @return Vector3 [Roll, Pitch, Yaw] in degrees.
     */
    [[nodiscard]] Vector3 getEulerAnglesDeg() const;

private:
    // Filter State
    Eigen::Quaternionf nominalQuat; // Global Reference Attitude
    VectorState stateVector;       // State Estimate Error (6D)
    MatrixState errorCovariance;   // Error Covariance (6x6)

    // UKF Parameters
    Config config;
    float lambdaParam;

    // FIX: Use fixed-size members
    VectorWeights weightsMean;
    VectorWeights weightsCovariance;

    // Sigma Points Buffer (pre-allocated)
    Eigen::Matrix<float, STATE_DIM, SIGMA_COUNT> sigmaPoints;

    // Helper Methods
    void generateSigmaPoints();
    void computeWeights();
    
    // MEKF Quaternion Math Helpers
    static Vector3 quaternionToRotationVector(const Eigen::Quaternionf& q);
    static Eigen::Quaternionf rotationVectorToQuaternion(const Vector3& rv);
    
    // Generic Observation Update
    void updateGenericObservation(const Vector3& meas, const Vector3& refInertial, float rNoise);
};

} // namespace gnc

#endif // ATTITUDE_UKF_HPP
