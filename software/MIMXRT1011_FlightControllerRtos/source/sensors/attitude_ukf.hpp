/**
 * @file attitude_ukf.hpp
 * @brief Unscented Kalman Filter for Attitude Estimation (Quaternions).
 *
 * Implements a 7-state UKF (4 Quaternion + 3 Gyro Bias).
 * Target: NXP RT1011 (Cortex-M7).
 */

#ifndef ATTITUDE_UKF_HPP
#define ATTITUDE_UKF_HPP

#include <Eigen>
#include <cmath>

namespace gnc {

/**
 * @brief Unscented Kalman Filter for Attitude Estimation.
 *
 * State Vector x [7x1]: [qw, qx, qy, qz, bx, by, bz]
 *
 * Uses static allocation via Eigen::Matrix fixed sizes.
 */
class AttitudeUkf {
public:
    // Constants
    static constexpr int STATE_DIM = 7;
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
        float alpha;        // Spread of sigma points (usually 1e-3 to 1)
        float beta;         // Prior knowledge of distribution (2 for Gaussian)
        float kappa;        // Secondary scaling parameter (usually 0)
        float qGyro;        // Process noise: Gyroscope (rad/s)
        float qBias;        // Process noise: Bias random walk (rad/s^2)
        float rAccel;       // Measurement noise: Accelerometer (m/s^2)
        float rMag;         // Measurement noise: Magnetometer (uT)
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
    VectorState stateVector;       // State Estimate
    MatrixState errorCovariance;   // Error Covariance

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
};

} // namespace gnc

#endif // ATTITUDE_UKF_HPP
