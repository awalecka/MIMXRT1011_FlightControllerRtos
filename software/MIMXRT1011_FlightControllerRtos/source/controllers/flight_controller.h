/**
 * @file flight_controller.h
 * @brief Coordinator for flight subsystems.
 *
 * Attitude filter selection
 * -------------------------
 * FlightController is templated on a filter policy type that satisfies the
 * gnc::AttitudeFilter concept (see attitude_filter_concept.hpp). The concrete
 * filter is selected at compile time by defining exactly one of:
 *
 * GNC_FILTER_MEKF   — Linearised Multiplicative EKF (default; recommended
 * for production; ~3-5× faster than UKF on Cortex-M7)
 * GNC_FILTER_UKF    — Unscented Kalman Filter (higher fidelity; useful for
 * development, characterisation, and regression testing)
 *
 * If neither flag is defined, GNC_FILTER_MEKF is used and a diagnostic is
 * emitted so the selection is always explicit in build logs.
 *
 * Design notes
 * ------------
 * The template parameter avoids virtual dispatch and heap allocation — both
 * of which are unacceptable on the RT1011 at 500 Hz. The compiler instantiates
 * exactly one concrete FlightController<Filter> version; the unused filter
 * class is not linked into the binary at all.
 *
 * Adding a third filter
 * ---------------------
 * Write or obtain the filter class, satisfying gnc::AttitudeFilter.
 * Add one #elif branch here mapping GNC_FILTER_<NEW> to a type alias.
 * FlightController.cpp requires no changes.
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <optional>

#include "common_types.h"

// ============================================================================
// Compile-time filter selection
// ============================================================================
//
// Exactly one GNC_FILTER_* macro must be defined. If none is specified the
// MEKF is used and a #pragma message is emitted so the choice is visible.
#define GNC_FILTER_MEKF

#if defined(GNC_FILTER_MEKF)
#   include "attitude_mekf.hpp"
    using ActiveFilter = gnc::AttitudeMekf;

#elif defined(GNC_FILTER_UKF)
#   include "attitude_ukf.hpp"
    using ActiveFilter = gnc::AttitudeUkf;

#else
    // Default to MEKF and make the implicit selection visible in the build log.
#   pragma message("GNC_FILTER_* not defined — defaulting to GNC_FILTER_MEKF. " \
                   "Pass -DGNC_FILTER_MEKF or -DGNC_FILTER_UKF to be explicit.")
#   define GNC_FILTER_MEKF
#   include "attitude_mekf.hpp"
    using ActiveFilter = gnc::AttitudeMekf;
#endif

// Validate the selected filter satisfies the structural contract.
// A violation here means the filter class is missing a required method.
#include "attitude_filter_concept.hpp"
static_assert(gnc::AttitudeFilter<ActiveFilter>,
    "ActiveFilter does not satisfy gnc::AttitudeFilter concept. "
    "Check attitude_filter_concept.hpp for required methods.");

// ============================================================================
// Subsystem headers
// ============================================================================

#include "attitude_controller.h"
#include "receiver.h"
#include "actuators.h"
#include "sensor_system.h"
#include "telemetry_manager.h"
#include "settings.h"
#include "nmea.h"

#include "lsm6dsox_adapter.hpp"
#include "lis3mdl_adapter.hpp"

// ============================================================================
// FlightController — templated on filter policy
// ============================================================================

/**
 * @brief Main flight controller, templated on an attitude filter policy.
 *
 * @tparam FilterPolicy  Any type satisfying gnc::AttitudeFilter. Select via
 * the GNC_FILTER_* compile definition. Do not pass a
 * type directly — use the ActiveFilter alias resolved
 * above so the selection is centrally controlled.
 */
template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
class FlightControllerT {
public:
    /**
     * @brief Injects external dependencies safely into the controller.
     */
    void injectDependencies(QueueHandle_t gpsQueue,
                            QueueHandle_t imuQueue,
                            QueueHandle_t magQueue,
                            QueueHandle_t commandQueue,
                            QueueHandle_t controlsQueue,
                            volatile FlightState_t* flightState);

    enum class ControlMode {
        STABILIZED,
        PASS_THROUGH
    };

    // --- Loop Timing Configuration ---
    static constexpr float    LOOP_RATE_HZ = 100.0f;
    static constexpr float    LOOP_DT_S    = 1.0f / LOOP_RATE_HZ;
    static constexpr uint32_t LOOP_DT_MS   = static_cast<uint32_t>(1000.0f / LOOP_RATE_HZ);

    using ImuData = SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::ImuData;
    using MagData = SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::MagData;

    /**
     * @brief Tuning parameters for the active filter.
     */
    static constexpr gnc::FilterConfig kFilterConfig {
        .qGyro     = 0.1f,
        .qBias     = 0.001f,
        .rAccel    = 0.5f,
        .rMag      = 1.0f,
        .accelGate = 2.0f,   // MEKF: reject updates when |a| deviates >2 m/s² from g
        .alpha     = 1.0f,   // UKF: sigma-point spread
        .beta      = 2.0f,   // UKF: optimal for Gaussian
        .kappa     = 0.0f,   // UKF: secondary scaling
    };

    /**
     * @brief Construct the flight controller with the compile-time filter tuning.
     * @param loopTime Nominal execution period of the flight loop [s].
     */
    explicit FlightControllerT(float loopTime);

    /**
     * @brief Initialise all subsystems and perform static alignment.
     * @return 0 on success, -1 on sensor initialisation failure.
     */
    int init();

    /** @brief Main update loop — sensors → estimator → controller → actuators. */
    void update();

    /** @brief Trigger gyroscope bias calibration. */
    void calibrateSensors();

    /** @brief Set the flight control mode (Stabilized / Pass-Through). */
    void setControlMode(ControlMode mode);

    /** @brief Retrieve the latest RC channel data. */
    RC_Channels_t getRcData() const;

    /** @brief Retrieve the latest GPS fix. */
    const firmware::sensors::GpsData& getGpsData() const { return m_latestGps; }

    /** @brief Perform one step of the interactive magnetometer calibration. */
    bool calibrateMagnetometerStep();

    /** @brief Save the current magnetometer calibration to non-volatile settings. */
    void saveCalibration();

    /** @brief Retrieve the normalised stick input. */
    void getStickInput(Receiver::StickInput& input);

    /** @brief Read the latest IMU data into the provided structure. */
    int readImu(ImuData& data);

    /** @brief Read the latest Magnetometer data into the provided structure. */
    int readMag(MagData& data);

    /**
	 * @brief Performs a smooth full-range sweep of the control surfaces.
	 * Keeps throttle at zero to ensure safety while sweeping aileron,
	 * elevator, and rudder from center, to extremes, and back to center.
	 */
	void performPreflightSweep();

    /** @brief Queries the receiver for any active sustained stick gestures. */
    Receiver::CommandGesture getActiveGesture();

    /**
     * @brief Return the number of hard covariance resets since boot.
     *
     * Non-zero values indicate the filter has recovered from numerical
     * instability. Expose via telemetry for health monitoring.
     */
    uint32_t getFilterFaultCount() const
    {
        return filter_.getCovarianceFaultCount();
    }

    /**
     * @brief Return the current gyro bias estimate [rad/s].
     */
    typename FilterPolicy::Vector3 getFilterBias() const
    {
        return filter_.getBias();
    }

private:
    /**
     * @brief Run one predict + update cycle of the attitude filter.
     * @param imuData High-rate IMU data.
     * @param magData Optional low-rate magnetometer data.
     */
    void estimateAttitude(const ImuData& imuData, const std::optional<MagData>& magData);

    QueueHandle_t m_gpsQueue = nullptr;
    QueueHandle_t m_imuQueue = nullptr;
    QueueHandle_t m_magQueue = nullptr;
    volatile FlightState_t* m_flightState = nullptr;

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter> sensorSystem;
    Receiver          receiver;
    Actuators         actuators;
    AttitudeController attitudeController;
    TelemetryManager  telemetry;

    // -------------------------------------------------------------------------
    // Attitude filter — concrete type is resolved at compile time.
    // -------------------------------------------------------------------------
    FilterPolicy                    filter_;
    typename FilterPolicy::Vector3  magRefVector_;

    // -------------------------------------------------------------------------
    // Loop state
    // -------------------------------------------------------------------------
    float       loopDt;
    float       currentRollDeg;
    float       currentPitchDeg;
    float       currentYawDeg;
    ControlMode currentControlMode;
    uint32_t    lastSensorTimestamp;
    int         staleDataCounter;
    uint32_t    lastUpdateTick;

    firmware::sensors::GpsData m_latestGps;
};

// ============================================================================
// Concrete type alias and global declaration
// ============================================================================

/**
 * @brief The concrete FlightController type for this build.
 *
 * All translation units that #include flight_controller.h see the same
 * resolved type. No code uses FlightControllerT<...> directly — always use
 * FlightController so that changing the filter selection here propagates
 * automatically everywhere.
 */
using FlightController = FlightControllerT<ActiveFilter>;

#endif // FLIGHT_CONTROLLER_H
