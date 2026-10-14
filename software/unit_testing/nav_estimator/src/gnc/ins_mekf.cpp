/**
 * @file ins_mekf.cpp
 * @brief Implementation of 15-state Error-State MEKF for INS.
 */

#include "ins_mekf.hpp"
#include <algorithm>

namespace gnc::ins {

namespace {
constexpr float PI_F = 3.14159265358979323846f;
constexpr float RAD2DEG_F = 180.0f / PI_F;
constexpr float COV_RIDGE = 1e-6f;
constexpr float COV_RESET_VAL = 0.01f;

float wrapPi(float angleRad) {
    while (angleRad > PI_F) angleRad -= 2.0f * PI_F;
    while (angleRad < -PI_F) angleRad += 2.0f * PI_F;
    return angleRad;
}
} // namespace

InsMekf::InsMekf(const InsConfig& cfg)
    : m_cfg(cfg)
{
    m_cov.setZero();
    m_cov.block<3, 3>(IDX_ATT, IDX_ATT) = Mat3::Identity() * (m_cfg.initLevelSigma * m_cfg.initLevelSigma);
    m_cov(IDX_ATT + 2, IDX_ATT + 2) = m_cfg.initYawSigma * m_cfg.initYawSigma;
    m_cov.block<3, 3>(IDX_VEL, IDX_VEL) = Mat3::Identity() * (m_cfg.initVelSigma * m_cfg.initVelSigma);
    m_cov.block<3, 3>(IDX_POS, IDX_POS) = Mat3::Identity() * (m_cfg.initPosSigma * m_cfg.initPosSigma);
    m_cov.block<3, 3>(IDX_BG, IDX_BG) = Mat3::Identity() * (m_cfg.initGyroBiasSigma * m_cfg.initGyroBiasSigma);
    m_cov.block<3, 3>(IDX_BA, IDX_BA) = Mat3::Identity() * (m_cfg.initAccelBiasSigma * m_cfg.initAccelBiasSigma);
}

void InsMekf::initialize(const Eigen::Quaternionf& q0,
                         const Vec3& v0,
                         const Vec3& p0,
                         const Vec3& bg0,
                         const Vec3& ba0,
                         const ErrorMat& P0)
{
    m_nominal.q = q0.normalized();
    m_nominal.v = v0;
    m_nominal.p = p0;
    m_nominal.bg = bg0;
    m_nominal.ba = ba0;
    m_cov = P0;
    repairCovariance();
}

Mat3 InsMekf::skew(const Vec3& v)
{
    Mat3 S;
    S <<  0.0f, -v.z(),  v.y(),
          v.z(),  0.0f, -v.x(),
         -v.y(),  v.x(),  0.0f;
    return S;
}

Eigen::Quaternionf InsMekf::rotVecToQuat(const Vec3& rv)
{
    Eigen::Quaternionf q;
    const float theta = rv.norm();
    if (theta > 1e-4f) {
        q.w() = std::cos(theta * 0.5f);
        q.vec() = (rv / theta) * std::sin(theta * 0.5f);
    } else {
        q.w() = 1.0f;
        q.vec() = rv * 0.5f;
        q.normalize();
    }
    return q;
}

Vec3 InsMekf::getEulerAnglesDeg() const
{
    const Eigen::Quaternionf& q = m_nominal.q;
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();

    const float roll = std::atan2(2.0f * (qw * qx + qy * qz),
                                  1.0f - 2.0f * (qx * qx + qy * qy)) * RAD2DEG_F;
    const float sinp = 2.0f * (qw * qy - qz * qx);
    float pitch;
    if (std::abs(sinp) >= 1.0f) {
        pitch = std::copysign(90.0f, sinp);
    } else {
        pitch = std::asin(sinp) * RAD2DEG_F;
    }
    const float yaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                 1.0f - 2.0f * (qy * qy + qz * qz)) * RAD2DEG_F;
    return Vec3(roll, pitch, yaw);
}

void InsMekf::predict(float dt, const Vec3& gyroBody, const Vec3& accelBody)
{
    if (dt <= 0.0f) return;

    // Corrected sensor signals
    const Vec3 omegaHat = gyroBody - m_nominal.bg;
    const Vec3 fHat = accelBody - m_nominal.ba;

    // Nominal attitude propagation (exact closed form)
    const Eigen::Quaternionf dq = rotVecToQuat(omegaHat * dt);
    m_nominal.q = (m_nominal.q * dq).normalized();

    // Nominal velocity and position propagation
    const Mat3 R = m_nominal.q.toRotationMatrix();
    const Vec3 gravityNed(0.0f, 0.0f, m_cfg.gravity);
    const Vec3 aNed = R * fHat + gravityNed;

    m_nominal.p += m_nominal.v * dt + 0.5f * aNed * dt * dt;
    m_nominal.v += aNed * dt;

    // Error state Jacobian F (15x15)
    // Continuous time error dynamics:
    //   d/dt(delta_theta) = -[omegaHat]x * delta_theta - delta_bg
    //   d/dt(delta_v)     = -R * [fHat]x * delta_theta - R * delta_ba
    //   d/dt(delta_p)     = delta_v
    //   d/dt(delta_bg)    = 0
    //   d/dt(delta_ba)    = 0
    ErrorMat F = ErrorMat::Identity();

    const Mat3 omegaSkew = skew(omegaHat);
    const Mat3 fSkew = skew(fHat);

    F.block<3, 3>(IDX_ATT, IDX_ATT) -= omegaSkew * dt;
    F.block<3, 3>(IDX_ATT, IDX_BG)  -= Mat3::Identity() * dt;

    F.block<3, 3>(IDX_VEL, IDX_ATT) -= R * fSkew * dt;
    F.block<3, 3>(IDX_VEL, IDX_BA)  -= R * dt;

    F.block<3, 3>(IDX_POS, IDX_VEL) += Mat3::Identity() * dt;
    // Second-order pos coupling
    F.block<3, 3>(IDX_POS, IDX_ATT) -= 0.5f * R * fSkew * (dt * dt);
    F.block<3, 3>(IDX_POS, IDX_BA)  -= 0.5f * R * (dt * dt);

    // Discrete process noise Qd
    ErrorMat Qd = ErrorMat::Zero();
    const float effGyroDensity = m_cfg.gyroNoiseDensity * m_cfg.gyroVibrationFactor;
    const float effAccelDensity = m_cfg.accelNoiseDensity * m_cfg.accelVibrationFactor;

    const float qAtt = effGyroDensity * effGyroDensity * dt;
    const float qVel = effAccelDensity * effAccelDensity * dt;
    const float qBg = m_cfg.gyroBiasRandomWalk * m_cfg.gyroBiasRandomWalk * dt;
    const float qBa = m_cfg.accelBiasRandomWalk * m_cfg.accelBiasRandomWalk * dt;

    Qd.block<3, 3>(IDX_ATT, IDX_ATT) = Mat3::Identity() * qAtt;
    Qd.block<3, 3>(IDX_VEL, IDX_VEL) = Mat3::Identity() * qVel;
    Qd.block<3, 3>(IDX_BG, IDX_BG)   = Mat3::Identity() * qBg;
    Qd.block<3, 3>(IDX_BA, IDX_BA)   = Mat3::Identity() * qBa;

    // Propagate covariance
    m_cov = F * m_cov * F.transpose() + Qd;
    repairCovariance();
}

template <int M>
UpdateResult InsMekf::applyUpdate(const Eigen::Matrix<float, M, 1>& innov,
                                  const Eigen::Matrix<float, M, ERROR_DIM>& H,
                                  const Eigen::Matrix<float, M, 1>& rDiag,
                                  float chi2Gate)
{
    UpdateResult res;
    // Innovation covariance: S = H * P * H^T + R
    const Eigen::Matrix<float, M, M> R = rDiag.asDiagonal();
    const Eigen::Matrix<float, M, M> S = H * m_cov * H.transpose() + R;

    // Solve for S^-1 * innov to evaluate NIS
    Eigen::LDLT<Eigen::Matrix<float, M, M>> ldltS(S);
    if (ldltS.info() != Eigen::Success) {
        repairCovariance();
        return res;
    }

    const Eigen::Matrix<float, M, 1> sInvInnov = ldltS.solve(innov);
    res.nis = innov.dot(sInvInnov);

    // Chi-square gating
    if (chi2Gate > 0.0f && res.nis > chi2Gate) {
        return res; // Gated out
    }

    // Kalman gain: K = P * H^T * S^-1 => K^T = S^-1 * (H * P)
    const Eigen::Matrix<float, ERROR_DIM, M> K = (ldltS.solve(H * m_cov)).transpose();

    // Error state estimate: dx = K * innov
    const ErrorVec dx = K * innov;

    // Joseph form covariance update:
    // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T
    const ErrorMat I_KH = ErrorMat::Identity() - K * H;
    m_cov = I_KH * m_cov * I_KH.transpose() + K * R * K.transpose();

    // Inject error state and perform MEKF reset
    injectAndReset(dx);
    res.accepted = true;
    return res;
}

void InsMekf::injectAndReset(const ErrorVec& dx)
{
    // Extract error states
    const Vec3 deltaTheta = dx.segment<3>(IDX_ATT);
    const Vec3 deltaV     = dx.segment<3>(IDX_VEL);
    const Vec3 deltaP     = dx.segment<3>(IDX_POS);
    const Vec3 deltaBg    = dx.segment<3>(IDX_BG);
    const Vec3 deltaBa    = dx.segment<3>(IDX_BA);

    // Multiplicative attitude reset: q_nom = q_nom * dq(delta_theta)
    const Eigen::Quaternionf dq = rotVecToQuat(deltaTheta);
    m_nominal.q = (m_nominal.q * dq).normalized();

    // Additive update for vector states
    m_nominal.v  += deltaV;
    m_nominal.p  += deltaP;
    m_nominal.bg += deltaBg;
    m_nominal.ba += deltaBa;

    // Covariance reset for attitude error:
    // For body-frame right error, after reset q_new = q_old * dq:
    // delta_theta_new ~ (I - 0.5 * [deltaTheta]x) * delta_theta_old
    // G = blkdiag(I - 0.5 * [deltaTheta]x, I_12)
    const Mat3 G_att = Mat3::Identity() - 0.5f * skew(deltaTheta);
    m_cov.block<3, ERROR_DIM>(IDX_ATT, 0) = G_att * m_cov.block<3, ERROR_DIM>(IDX_ATT, 0);
    m_cov.block<ERROR_DIM, 3>(0, IDX_ATT) = m_cov.block<ERROR_DIM, 3>(0, IDX_ATT) * G_att.transpose();

    repairCovariance();
}

void InsMekf::repairCovariance()
{
    // Re-symmetrize
    m_cov = (m_cov + m_cov.transpose()) * 0.5f;

    // Check positive definiteness via LLT
    Eigen::LLT<ErrorMat> llt(m_cov);
    if (llt.info() != Eigen::Success) {
        m_cov += ErrorMat::Identity() * COV_RIDGE;
        llt.compute(m_cov);

        if (llt.info() != Eigen::Success) {
            // Hard reset
            ++m_covFaultCount;
            m_cov.setZero();
            m_cov.block<3, 3>(IDX_ATT, IDX_ATT) = Mat3::Identity() * (m_cfg.initLevelSigma * m_cfg.initLevelSigma);
            m_cov(IDX_ATT + 2, IDX_ATT + 2) = m_cfg.initYawSigma * m_cfg.initYawSigma;
            m_cov.block<3, 3>(IDX_VEL, IDX_VEL) = Mat3::Identity() * (m_cfg.initVelSigma * m_cfg.initVelSigma);
            m_cov.block<3, 3>(IDX_POS, IDX_POS) = Mat3::Identity() * (m_cfg.initPosSigma * m_cfg.initPosSigma);
            m_cov.block<3, 3>(IDX_BG, IDX_BG)   = Mat3::Identity() * (m_cfg.initGyroBiasSigma * m_cfg.initGyroBiasSigma);
            m_cov.block<3, 3>(IDX_BA, IDX_BA)   = Mat3::Identity() * (m_cfg.initAccelBiasSigma * m_cfg.initAccelBiasSigma);
        }
    }
}

UpdateResult InsMekf::updatePosition(const Vec3& posNed, const Vec3& sigma)
{
    // Measurement model: z = p + v_meas
    // Innovation: innov = posNed - p_nom
    const Vec3 innov = posNed - m_nominal.p;

    Eigen::Matrix<float, 3, ERROR_DIM> H = Eigen::Matrix<float, 3, ERROR_DIM>::Zero();
    H.block<3, 3>(0, IDX_POS) = Mat3::Identity();

    const Vec3 rDiag = sigma.cwiseProduct(sigma);
    return applyUpdate<3>(innov, H, rDiag, m_cfg.gate3Dof);
}

UpdateResult InsMekf::updateVelocityNE(const Vec2& velNE, float sigma)
{
    // Measurement model: z = [v_N, v_E]^T + v_meas
    const Vec2 innov = velNE - m_nominal.v.head<2>();

    Eigen::Matrix<float, 2, ERROR_DIM> H = Eigen::Matrix<float, 2, ERROR_DIM>::Zero();
    H.block<2, 2>(0, IDX_VEL) = Eigen::Matrix2f::Identity();

    const Vec2 rDiag(sigma * sigma, sigma * sigma);
    return applyUpdate<2>(innov, H, rDiag, m_cfg.gate2Dof);
}

UpdateResult InsMekf::updateVelocity3D(const Vec3& velNed, const Vec3& sigma)
{
    const Vec3 innov = velNed - m_nominal.v;

    Eigen::Matrix<float, 3, ERROR_DIM> H = Eigen::Matrix<float, 3, ERROR_DIM>::Zero();
    H.block<3, 3>(0, IDX_VEL) = Mat3::Identity();

    const Vec3 rDiag = sigma.cwiseProduct(sigma);
    return applyUpdate<3>(innov, H, rDiag, m_cfg.gate3Dof);
}

UpdateResult InsMekf::updateHeading(float yawRad, float sigma)
{
    // Current estimated yaw from quaternion
    const Eigen::Quaternionf& q = m_nominal.q;
    const float qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
    const float estYaw = std::atan2(2.0f * (qw * qz + qx * qy),
                                    1.0f - 2.0f * (qy * qy + qz * qz));

    const float innovYaw = wrapPi(yawRad - estYaw);
    Eigen::Matrix<float, 1, 1> innov;
    innov(0, 0) = innovYaw;

    // Jacobian d(yaw)/d(delta_theta) for body-frame perturbation:
    // With R = R_nom * (I - [delta_theta]x):
    // In Euler ZYX: yaw = atan2(R_10, R_00).
    // Perturbation: d(yaw) = J_att * delta_theta
    // Specifically:
    //   d(yaw)/d(R_10) = R_00 / (R_00^2 + R_10^2)
    //   d(yaw)/d(R_00) = -R_10 / (R_00^2 + R_10^2)
    // and R = R_nom * (I - [delta_theta]x) gives:
    //   dR = - R_nom * [delta_theta]x
    // Thus dR_ij = - sum_k R_nom_ik * [delta_theta]x_kj
    const Mat3 R = q.toRotationMatrix();
    const float den = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
    if (den < 1e-4f) {
        // Gimbal lock / pitch near +-90 deg
        return UpdateResult{};
    }

    const float c0 = -R(1, 0) / den; // d(yaw)/d(R_00)
    const float c1 =  R(0, 0) / den; // d(yaw)/d(R_10)

    // For a vector w = delta_theta:
    // [w]x = [  0, -w2,  w1 ]
    //        [ w2,   0, -w0 ]
    //        [-w1,  w0,   0 ]
    // dR.col(0) = -R_nom * [w]x.col(0) = -R_nom * [0, w2, -w1]^T
    //           = -w2 * R_nom.col(1) + w1 * R_nom.col(2)
    // d(R_00) = -w2 * R(0, 1) + w1 * R(0, 2)
    // d(R_10) = -w2 * R(1, 1) + w1 * R(1, 2)
    // Therefore:
    // d(yaw) = c0 * (-w2 * R(0, 1) + w1 * R(0, 2)) + c1 * (-w2 * R(1, 1) + w1 * R(1, 2))
    //        = w0 * (0)
    //        + w1 * (c0 * R(0, 2) + c1 * R(1, 2))
    //        + w2 * (-c0 * R(0, 1) - c1 * R(1, 1))
    Eigen::Matrix<float, 1, ERROR_DIM> H = Eigen::Matrix<float, 1, ERROR_DIM>::Zero();
    H(0, IDX_ATT + 0) = 0.0f;
    H(0, IDX_ATT + 1) = -c0 * R(0, 2) - c1 * R(1, 2);
    H(0, IDX_ATT + 2) =  c0 * R(0, 1) + c1 * R(1, 1);

    Eigen::Matrix<float, 1, 1> rDiag;
    rDiag(0, 0) = sigma * sigma;

    return applyUpdate<1>(innov, H, rDiag, m_cfg.gate1Dof);
}

UpdateResult InsMekf::updateNonHolonomic(const Eigen::Quaternionf& mountRotation,
                                        float sigmaLat,
                                        float sigmaVert)
{
    // Non-holonomic constraints:
    // In vehicle frame (v): v_v = R_v_b * v_b = R_v_b * R^T * v_ned
    // We assume v_v(1) = 0 (lateral) and v_v(2) = 0 (vertical).
    const Mat3 R_v_b = mountRotation.toRotationMatrix();
    const Mat3 R_b_n = m_nominal.q.toRotationMatrix().transpose(); // R^T (NED to body)
    const Mat3 R_v_n = R_v_b * R_b_n;                              // NED to vehicle

    const Vec3 v_vehicle = R_v_n * m_nominal.v;

    // Measurement innovation: z = [0, 0]^T
    // innov = z - [v_vehicle(1), v_vehicle(2)]^T = -v_vehicle.tail<2>()
    const Vec2 innov = -v_vehicle.tail<2>();

    // Jacobian derivation:
    // v_vehicle = R_v_b * R(q_true)^T * v_true
    // With q_true = q_nom * dq(delta_theta):
    // R(q_true)^T = (R_nom * (I - [delta_theta]x))^T = (I + [delta_theta]x) * R_nom^T
    // v_vehicle ~ R_v_b * (I + [delta_theta]x) * R_nom^T * (v_nom + delta_v)
    //           ~ R_v_b * (R_nom^T * v_nom + R_nom^T * delta_v + [delta_theta]x * R_nom^T * v_nom)
    // Note [delta_theta]x * v_b = - [v_b]x * delta_theta
    // So: d(v_vehicle) = - R_v_b * [v_b]x * delta_theta + R_v_n * delta_v
    const Vec3 v_b = R_b_n * m_nominal.v;
    const Mat3 vbSkew = skew(v_b);

    Eigen::Matrix<float, 2, ERROR_DIM> H = Eigen::Matrix<float, 2, ERROR_DIM>::Zero();
    const Mat3 dV_dTheta = R_v_b * vbSkew;
    H.block<2, 3>(0, IDX_ATT) = dV_dTheta.bottomRows<2>();
    H.block<2, 3>(0, IDX_VEL) = R_v_n.bottomRows<2>();

    const Vec2 rDiag(sigmaLat * sigmaLat, sigmaVert * sigmaVert);
    return applyUpdate<2>(innov, H, rDiag, m_cfg.gate2Dof);
}

UpdateResult InsMekf::updateZeroVelocity(float sigma)
{
    // Observes v = 0 in NED frame
    const Vec3 innov = -m_nominal.v;

    Eigen::Matrix<float, 3, ERROR_DIM> H = Eigen::Matrix<float, 3, ERROR_DIM>::Zero();
    H.block<3, 3>(0, IDX_VEL) = Mat3::Identity();

    const Vec3 rDiag = Vec3::Constant(sigma * sigma);
    return applyUpdate<3>(innov, H, rDiag, m_cfg.gate3Dof);
}

UpdateResult InsMekf::updateZeroRate(const Vec3& gyroBody, float sigma)
{
    // Observes true angular rate is zero:
    // 0 = gyroBody - bg - delta_bg + noise
    // => bg = gyroBody
    // Innov = gyroBody - bg_nom
    const Vec3 innov = gyroBody - m_nominal.bg;

    Eigen::Matrix<float, 3, ERROR_DIM> H = Eigen::Matrix<float, 3, ERROR_DIM>::Zero();
    H.block<3, 3>(0, IDX_BG) = Mat3::Identity();

    const Vec3 rDiag = Vec3::Constant(sigma * sigma);
    return applyUpdate<3>(innov, H, rDiag, m_cfg.gate3Dof);
}

void InsMekf::shiftOrigin(const Vec3& deltaNed, const Mat3& rNewFromOld)
{
    // Position shift: p_new = rNewFromOld * (p_old - deltaNed)
    m_nominal.p = rNewFromOld * (m_nominal.p - deltaNed);
    // Velocity shift: v_new = rNewFromOld * v_old
    m_nominal.v = rNewFromOld * m_nominal.v;
    // Attitude shift: R_new_body = rNewFromOld * R_old_body
    const Mat3 R_new = rNewFromOld * m_nominal.q.toRotationMatrix();
    m_nominal.q = Eigen::Quaternionf(R_new).normalized();

    // Covariance rotation: delta_v and delta_p are in NED frame, so they rotate by rNewFromOld.
    // delta_theta, delta_bg, delta_ba are in body frame, so they are unchanged!
    m_cov.block<3, ERROR_DIM>(IDX_VEL, 0) = rNewFromOld * m_cov.block<3, ERROR_DIM>(IDX_VEL, 0);
    m_cov.block<ERROR_DIM, 3>(0, IDX_VEL) = m_cov.block<ERROR_DIM, 3>(0, IDX_VEL) * rNewFromOld.transpose();

    m_cov.block<3, ERROR_DIM>(IDX_POS, 0) = rNewFromOld * m_cov.block<3, ERROR_DIM>(IDX_POS, 0);
    m_cov.block<ERROR_DIM, 3>(0, IDX_POS) = m_cov.block<ERROR_DIM, 3>(0, IDX_POS) * rNewFromOld.transpose();

    repairCovariance();
}

} // namespace gnc::ins
