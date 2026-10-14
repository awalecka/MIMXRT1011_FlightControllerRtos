/**
 * @file ins_mekf.hpp
 * @brief 15-state Error-State Multiplicative Extended Kalman Filter (MEKF) for INS.
 *
 * State definition:
 *   Nominal state:
 *     - q: Attitude quaternion (body to NED)
 *     - v: Velocity in NED [m/s]
 *     - p: Position in NED relative to local tangent plane origin [m]
 *     - bg: Gyroscope bias [rad/s]
 *     - ba: Accelerometer bias [m/s^2]
 *
 *   Error state (15x1):
 *     [delta_theta(3), delta_v(3), delta_p(3), delta_bg(3), delta_ba(3)]
 *
 * Attitude error convention:
 *   Right multiplicative perturbation in the body frame:
 *   q_true = q_nom * dq(delta_theta)
 *   where dq(delta_theta) = [1, 0.5 * delta_theta]^T (first-order).
 *
 * Mechanization:
 *   Specific force f_b is the true specific force (reading [0, 0, -g] at rest in level NED).
 *   Acceleration in NED: a_ned = R(q_nom) * (f_b - ba) + [0, 0, g]^T
 */

#ifndef INS_MEKF_HPP
#define INS_MEKF_HPP

#include "ins_types.hpp"
#include <Eigen>
#include <cmath>

namespace gnc::ins {

class InsMekf {
public:
    explicit InsMekf(const InsConfig& cfg);

    /**
     * @brief Initialize filter nominal state and error covariance.
     */
    void initialize(const Eigen::Quaternionf& q0,
                    const Vec3& v0,
                    const Vec3& p0,
                    const Vec3& bg0,
                    const Vec3& ba0,
                    const ErrorMat& P0);

    /**
     * @brief Predict nominal state and propagate error covariance using IMU measurements.
     * @param dt Time step [s]
     * @param gyroBody Raw gyro rate in body frame [rad/s]
     * @param accelBody True specific force in body frame [m/s^2] (reading [0,0,-g] when static level)
     */
    void predict(float dt, const Vec3& gyroBody, const Vec3& accelBody);

    /**
     * @brief Measurement update for 3D position in NED frame.
     */
    UpdateResult updatePosition(const Vec3& posNed, const Vec3& sigma);

    /**
     * @brief Measurement update for horizontal velocity (North, East) in NED frame.
     */
    UpdateResult updateVelocityNE(const Vec2& velNE, float sigma);

    /**
     * @brief Measurement update for full 3D velocity in NED frame.
     */
    UpdateResult updateVelocity3D(const Vec3& velNed, const Vec3& sigma);

    /**
     * @brief Measurement update for heading / yaw angle [rad], where 0 = North, clockwise.
     */
    UpdateResult updateHeading(float yawRad, float sigma);

    /**
     * @brief Non-holonomic constraint (NHC) update.
     * Assumes zero lateral and vertical velocity in the vehicle frame.
     * @param mountRotation Fixed rotation from body to vehicle frame (q_v_b).
     * @param sigmaLat Lateral velocity constraint uncertainty [m/s]
     * @param sigmaVert Vertical velocity constraint uncertainty [m/s]
     */
    UpdateResult updateNonHolonomic(const Eigen::Quaternionf& mountRotation, float sigmaLat, float sigmaVert);

    /**
     * @brief Zero Velocity Update (ZUPT): updates 3D velocity to zero.
     */
    UpdateResult updateZeroVelocity(float sigma);

    /**
     * @brief Zero Angular Rate Update (ZARU): directly observes gyro bias bg.
     * @param gyroBody Current measured angular rate [rad/s]
     * @param sigma Uncertainty in zero rate [rad/s]
     */
    UpdateResult updateZeroRate(const Vec3& gyroBody, float sigma);

    /**
     * @brief Re-centre position and coordinate frame when LTP origin shifts.
     * @param deltaNed Position offset in old NED frame.
     * @param rNewFromOld Rotation mapping old NED to new NED.
     */
    void shiftOrigin(const Vec3& deltaNed, const Mat3& rNewFromOld);

    // Getters
    [[nodiscard]] const NominalState& getNominalState() const { return m_nominal; }
    [[nodiscard]] Eigen::Quaternionf getQuaternion() const { return m_nominal.q; }
    [[nodiscard]] Vec3 getVelocity() const { return m_nominal.v; }
    [[nodiscard]] Vec3 getPosition() const { return m_nominal.p; }
    [[nodiscard]] Vec3 getGyroBias() const { return m_nominal.bg; }
    [[nodiscard]] Vec3 getAccelBias() const { return m_nominal.ba; }
    [[nodiscard]] const ErrorMat& getCovariance() const { return m_cov; }
    [[nodiscard]] Vec3 getEulerAnglesDeg() const;
    [[nodiscard]] uint32_t getCovarianceFaultCount() const { return m_covFaultCount; }

    static Mat3 skew(const Vec3& v);
    static Eigen::Quaternionf rotVecToQuat(const Vec3& rv);

private:
    InsConfig m_cfg;
    NominalState m_nominal;
    ErrorMat m_cov;
    uint32_t m_covFaultCount = 0;

    /**
     * @brief Generic sequential/scalar or batch Joseph-form update.
     */
    template <int M>
    UpdateResult applyUpdate(const Eigen::Matrix<float, M, 1>& innov,
                             const Eigen::Matrix<float, M, ERROR_DIM>& H,
                             const Eigen::Matrix<float, M, 1>& rDiag,
                             float chi2Gate);

    void injectAndReset(const ErrorVec& dx);
    void repairCovariance();
};

} // namespace gnc::ins

#endif // INS_MEKF_HPP
