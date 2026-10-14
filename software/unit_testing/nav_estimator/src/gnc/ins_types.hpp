/**
 * @file ins_types.hpp
 * @brief Shared types and configuration for the automotive INS (15-state MEKF).
 *
 * Conventions (identical to AttitudeMekf)
 * ---------------------------------------
 * - NED navigation frame (North-East-Down), gravity = [0, 0, +g].
 * - Body frame = IMU frame, FRD-style (X forward, Y right, Z down when level).
 * - Quaternion q rotates body -> NED:   v_ned = q * v_body.
 * - Attitude error is a RIGHT (body-frame) perturbation:
 *       q_true = q_nom (x) dq(delta_theta)
 * - InsMekf takes TRUE specific force (reads [0,0,-g] when level and at rest).
 *   InsNavigator accepts accelerometer data in the AttitudeMekf "+g" convention
 *   (reads [0,0,+g] when level) and negates it exactly once at its input.
 * - Vehicle frame: X forward, Y right, Z down. `mountRotation` is the fixed
 *   rotation body -> vehicle (identity if the IMU axes are aligned with the car).
 *
 * Error state (15): [d_theta(3), d_vel(3), d_pos(3), d_bg(3), d_ba(3)]
 *
 * Target: NXP RT1011 (Cortex-M7, single-precision FPU). All filter maths is
 * float32 with fixed-size Eigen types (no heap). Double precision is used only
 * in LocalTangentPlane for the once-per-fix LLA <-> local-NED conversion.
 */

#ifndef INS_TYPES_HPP
#define INS_TYPES_HPP

#include <Eigen>
#include <cstdint>

namespace gnc::ins {

// ----------------------------------------------------------------------------
// Dimensions and indices
// ----------------------------------------------------------------------------
constexpr int ERROR_DIM = 15;
constexpr int IDX_ATT = 0;   ///< delta-theta (body frame rotation vector)
constexpr int IDX_VEL = 3;   ///< delta-velocity (NED)
constexpr int IDX_POS = 6;   ///< delta-position (NED)
constexpr int IDX_BG = 9;    ///< delta gyro bias
constexpr int IDX_BA = 12;   ///< delta accel bias

using Vec2 = Eigen::Matrix<float, 2, 1>;
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Mat3 = Eigen::Matrix<float, 3, 3>;
using ErrorVec = Eigen::Matrix<float, ERROR_DIM, 1>;
using ErrorMat = Eigen::Matrix<float, ERROR_DIM, ERROR_DIM>;

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

/**
 * @brief All tuning for the INS. Defaults are derived from the LSM6DSOX,
 * LIS3MDL and SAM-M8Q datasheets (see the implementation plan), inflated for
 * in-vehicle vibration.
 */
struct InsConfig {
    // ---- Physical constants ----
    float gravity = 9.81f;               ///< [m/s^2] (matches AttitudeMekf)

    // ---- IMU noise (LSM6DSOX datasheet, then inflated for vibration) ----
    float gyroNoiseDensity = 6.6e-5f;    ///< [rad/s/sqrt(Hz)]  (3.8 mdps/sqrt(Hz))
    float accelNoiseDensity = 6.9e-4f;   ///< [m/s^2/sqrt(Hz)]  (70 ug/sqrt(Hz))
    float gyroVibrationFactor = 5.0f;    ///< multiplies gyroNoiseDensity
    float accelVibrationFactor = 20.0f;  ///< multiplies accelNoiseDensity
    float gyroBiasRandomWalk = 1.0e-5f;  ///< [rad/s^2/sqrt(Hz)]
    float accelBiasRandomWalk = 2.0e-4f; ///< [m/s^3/sqrt(Hz)]

    // ---- Initial uncertainty ----
    float initGyroBiasSigma = 0.0087f;   ///< [rad/s] (0.5 dps)
    float initAccelBiasSigma = 0.1f;     ///< [m/s^2]
    float initLevelSigma = 0.02f;        ///< [rad] roll/pitch after static levelling
    float initYawSigma = 0.2f;           ///< [rad] yaw once aligned (mag or course)
    float initVelSigma = 0.3f;           ///< [m/s]
    float initPosSigma = 3.0f;           ///< [m]

    // ---- GPS (SAM-M8Q) ----
    float gpsUereH = 2.0f;               ///< [m] horizontal, multiplied by HDOP when no GST
    float gpsUereV = 5.0f;               ///< [m] vertical, multiplied by HDOP when no GST
    float gpsSpeedSigma = 0.1f;          ///< [m/s]
    float gpsMinSigmaH = 0.5f;           ///< [m] floor on any horizontal sigma
    float gpsMinSigmaV = 1.0f;           ///< [m] floor on vertical sigma
    float gpsTimeoutS = 2.0f;            ///< [s] no fresh fix for this long -> DeadReckoning
    uint8_t gpsMinSatellites = 4;        ///< minimum satellites to accept a fix

    // ---- Magnetometer (LIS3MDL) ----
    bool enableMag = false;              ///< use mag for heading
    float magHeadingSigmaRad = 0.1745f;  ///< [rad] (10 deg)
    float magDeclinationRad = 0.0f;      ///< [rad] east-positive
    float magNormTolerance = 0.25f;      ///< relative field-norm gate (+-25 %)
    float magUpdateRateHz = 5.0f;        ///< max mag heading update rate

    // ---- Mounting ----
    Eigen::Quaternionf mountRotation = Eigen::Quaternionf::Identity(); ///< body -> vehicle

    // ---- Vehicle-motion aids ----
    bool enableNhc = true;               ///< non-holonomic constraint
    float nhcSigmaLateral = 0.3f;        ///< [m/s]
    float nhcSigmaVertical = 0.3f;       ///< [m/s]
    float nhcRateHz = 10.0f;             ///< update rate (decimated)
    float nhcMinSpeed = 1.0f;            ///< [m/s] apply only above this speed

    bool enableZupt = true;              ///< zero-velocity + zero-rate updates when stationary
    float zuptSigma = 0.02f;             ///< [m/s]
    float zaruSigma = 0.005f;            ///< [rad/s]
    float zuptRateHz = 10.0f;            ///< update rate (decimated)

    bool enableCourseHeadingAiding = false; ///< continuous GPS-course yaw measurement
    float courseHeadingSigmaRad = 0.0524f;  ///< [rad] (3 deg)
    float courseInitMinSpeed = 5.0f;        ///< [m/s] for yaw initialisation from course
    float courseInitMaxYawRate = 0.0873f;   ///< [rad/s] (5 deg/s)

    // ---- Gating (chi-square, 99.9 %) ----
    float gate1Dof = 10.83f;
    float gate2Dof = 13.82f;
    float gate3Dof = 16.27f;

    // ---- Stationary detector ----
    float stationaryWindowS = 0.5f;      ///< [s]
    float stationaryAccelStd = 0.35f;    ///< [m/s^2] max std of |f| over window
    float stationaryGyroMax = 0.035f;    ///< [rad/s] max |w - mean| ... see detector
    float stationaryEnterS = 0.5f;       ///< [s] hold before declaring stationary
    float stationaryExitS = 0.1f;        ///< [s] hold before declaring moving
    float stationaryGpsSpeed = 0.3f;     ///< [m/s] GPS speed ceiling for stationary

    // ---- Alignment ----
    float alignWindowS = 2.0f;           ///< [s] static averaging window for levelling
    float alignTimeoutS = 600.0f;        ///< [s] give up waiting for a static window (unused if 0)

    // ---- Divergence / navigation ----
    uint8_t maxConsecutiveGpsRejects = 8;///< -> re-align
    float ltpRecentreDistance = 20000.0f;///< [m]
    float maxHorizontalSigmaDr = 500.0f; ///< [m] DeadReckoning -> Levelled when exceeded
};

// ----------------------------------------------------------------------------
// State / results
// ----------------------------------------------------------------------------

/** @brief Nominal (large-signal) state. */
struct NominalState {
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); ///< body -> NED
    Vec3 v = Vec3::Zero();   ///< NED velocity [m/s]
    Vec3 p = Vec3::Zero();   ///< NED position relative to LTP origin [m]
    Vec3 bg = Vec3::Zero();  ///< gyro bias [rad/s]
    Vec3 ba = Vec3::Zero();  ///< accel bias [m/s^2]
};

/** @brief Result of one measurement update attempt. */
struct UpdateResult {
    bool accepted = false;   ///< false if gated, degenerate, or numerically rejected
    float nis = 0.0f;        ///< normalised innovation squared
};

/** @brief Navigation mode / health. */
enum class NavStatus : uint8_t {
    WaitStatic = 0,    ///< waiting for a static window to level the IMU
    Levelled = 1,      ///< roll/pitch + gyro bias known, yaw not yet known
    Aligned = 2,       ///< yaw known, waiting for first GPS fix
    Navigating = 3,    ///< full aiding
    DeadReckoning = 4  ///< GPS lost, inertial + NHC only
};

/**
 * @brief One GPS measurement in the filter's terms. This is the decoupling
 * point between the NMEA parser and the filter; a UBX front end could fill it
 * identically.
 */
struct GpsMeasurement {
    bool hasPosition = false;
    Vec3 posNed = Vec3::Zero();      ///< relative to LTP origin [m]
    Vec3 posSigma = Vec3::Constant(5.0f); ///< N, E, D 1-sigma [m]

    bool hasVelocity = false;
    Vec2 velNE = Vec2::Zero();       ///< [m/s]
    float velSigma = 0.1f;           ///< [m/s]
    float speed = 0.0f;              ///< [m/s]
    float courseRad = 0.0f;          ///< [rad] course over ground, 0 = North, clockwise
};

/** @brief Navigation solution for consumers (control, telemetry). */
struct NavSolution {
    NavStatus status = NavStatus::WaitStatic;

    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    float altitudeM = 0.0f;          ///< above the LTP origin altitude + d_pos (ellipsoidal as reported by GPS)

    Vec3 posNed = Vec3::Zero();      ///< relative to LTP origin [m]
    Vec3 velNed = Vec3::Zero();      ///< [m/s]
    float speed = 0.0f;              ///< horizontal speed [m/s]
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Vec3 eulerDeg = Vec3::Zero();    ///< [roll, pitch, yaw] of the IMU body, ZYX
    Vec3 gyroBias = Vec3::Zero();
    Vec3 accelBias = Vec3::Zero();

    float posSigmaH = 0.0f;          ///< horizontal 1-sigma [m]
    float posSigmaD = 0.0f;
    float velSigmaH = 0.0f;
    float yawSigmaDeg = 0.0f;

    bool stationary = false;
    float gpsAgeS = 0.0f;            ///< seconds since last accepted GPS position
    uint32_t covFaultCount = 0;
};

} // namespace gnc::ins

#endif // INS_TYPES_HPP
