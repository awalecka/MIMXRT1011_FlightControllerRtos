/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 *
 * This file is filter-agnostic. The concrete filter type is resolved by
 * flight_controller.h based on the GNC_FILTER_* compile definition.
 */

#include "flight_controller.h"
#include "math.h"
#include <cmath>

using namespace firmware::drivers;

// ============================================================================
// Global instance — type is FlightController = FlightControllerT<ActiveFilter>
// ============================================================================

FlightController g_flightController(FlightController::LOOP_DT_S);

// ============================================================================
// File-scope constants
// ============================================================================

static constexpr float DEG_TO_RAD  = 3.1415926535f / 180.0f;
static constexpr float RAD_TO_DEG  = 180.0f / 3.1415926535f;
static constexpr float GRAVITY_MSS = 9.80665f;

// ============================================================================
// FlightControllerT method definitions
// ============================================================================
//
// Because FlightControllerT is a class template, definitions that depend on
// the template parameter must either live in the header or be explicitly
// instantiated in this translation unit. We use explicit instantiation below
// to keep the implementation in a .cpp file while still producing a single
// concrete type per binary.

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
FlightControllerT<FilterPolicy>::FlightControllerT(float loopTime)
    : filter_(kFilterConfig)
    , magRefVector_(FilterPolicy::Vector3::Zero())
    , loopDt(loopTime)
    , currentRollDeg(0.0f)
    , currentPitchDeg(0.0f)
    , currentYawDeg(0.0f)
    , currentControlMode(ControlMode::STABILIZED)
    , lastSensorTimestamp(0)
    , staleDataCounter(0)
    , lastUpdateTick(0)
{
}

// ----------------------------------------------------------------------------
// injectDependencies
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::injectDependencies(
                            QueueHandle_t gpsQueue,
                            QueueHandle_t imuQueue,
                            QueueHandle_t magQueue,
                            QueueHandle_t commandQueue,
                            QueueHandle_t controlsQueue,
                            volatile FlightState_t* flightState)
{
    m_gpsQueue = gpsQueue;
    m_imuQueue = imuQueue;
    m_magQueue = magQueue;
    m_flightState = flightState;

    receiver.injectQueue(commandQueue);
    telemetry.injectQueue(controlsQueue);
}

// ----------------------------------------------------------------------------
// init
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
int FlightControllerT<FilterPolicy>::init()
{
    if (sensorSystem.init() != 0) {
        return -1;
    }

    MagCalibrationParams calParams;
    if (Settings::loadMagCal(calParams)) {
        sensorSystem.setCalibration(calParams.hardIron, calParams.softIron);
    }

    receiver.init();
    actuators.init();

    // -------------------------------------------------------------------------
    // Static alignment — collect averages from stationary sensors and run TRIAD
    // -------------------------------------------------------------------------
    ImuData imuRaw;
    MagData magRaw;

    float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
    float sumMx = 0.0f, sumMy = 0.0f, sumMz = 0.0f;
    constexpr int kInitSamples = 50;

    for (int i = 0; i < kInitSamples; ++i) {
        while (sensorSystem.readImu(imuRaw) != 0) {}
        while (sensorSystem.readMag(magRaw) != 0) {}
        sumAx += imuRaw.accelXG;
        sumAy += imuRaw.accelYG;
        sumAz += imuRaw.accelZG;
        sumMx += magRaw.magXGauss;
        sumMy += magRaw.magYGauss;
        sumMz += magRaw.magZGauss;

        for (int k = 0; k < 10000; ++k) {
			__asm volatile("nop");
		}
    }

    typename FilterPolicy::Vector3 avgAccel;
    avgAccel << -(sumAx / kInitSamples) * GRAVITY_MSS,
                -(sumAy / kInitSamples) * GRAVITY_MSS,
                -(sumAz / kInitSamples) * GRAVITY_MSS;

    typename FilterPolicy::Vector3 avgMag;
    avgMag << sumMx / kInitSamples,
              sumMy / kInitSamples,
              sumMz / kInitSamples;

    // align() sets the initial quaternion and writes the inertial magnetic
    // reference vector used by every subsequent updateMag() call.
    filter_.align(avgAccel, avgMag, magRefVector_);

    return 0;
}

// ----------------------------------------------------------------------------
// calibrateSensors
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::calibrateSensors()
{
    sensorSystem.calibrateGyro();
}

// ----------------------------------------------------------------------------
// readImu
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
int FlightControllerT<FilterPolicy>::readImu(ImuData& data)
{
    return sensorSystem.readImu(data);
}

// ----------------------------------------------------------------------------
// readMag
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
int FlightControllerT<FilterPolicy>::readMag(MagData& data)
{
    return sensorSystem.readMag(data);
}

// ----------------------------------------------------------------------------
// setControlMode
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::setControlMode(ControlMode mode)
{
    currentControlMode = mode;
}

// ----------------------------------------------------------------------------
// getStickInput
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::getStickInput(Receiver::StickInput& input)
{
    receiver.getStickInput(input);
}

// ----------------------------------------------------------------------------
// getRcData
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
RC_Channels_t FlightControllerT<FilterPolicy>::getRcData() const
{
    return receiver.getCachedData();
}

// ----------------------------------------------------------------------------
// calibrateMagnetometerStep
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
bool FlightControllerT<FilterPolicy>::calibrateMagnetometerStep()
{
    MagData rawData;
    if (sensorSystem.feedMagCalibration(rawData) != 0) {
        return false;
    }
    telemetry.sendMagRaw(rawData.magXGauss, rawData.magYGauss, rawData.magZGauss);
    return true;
}

// ----------------------------------------------------------------------------
// saveCalibration
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::saveCalibration()
{
    MagCalibrationParams params;
    sensorSystem.getCalibration(params.hardIron, params.softIron);
    Settings::saveMagCal(params);
    telemetry.sendCalStatus(true);
}

// ----------------------------------------------------------------------------
// getActiveGesture
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
Receiver::CommandGesture FlightControllerT<FilterPolicy>::getActiveGesture()
{
    return receiver.getActiveGesture();
}

// ----------------------------------------------------------------------------
// update
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::update()
{
    receiver.update();

    // Mode switch on AUX1
    currentControlMode = (receiver.getChannel(RC_CH_AUX1) > 1500)
                       ? ControlMode::PASS_THROUGH
                       : ControlMode::STABILIZED;

    // Consume latest GPS fix from queue
    firmware::sensors::GpsData newGpsData;
    if (m_gpsQueue && xQueueReceive(m_gpsQueue, &newGpsData, 0) == pdPASS) {
        m_latestGps = newGpsData;
    }

    ImuData imuData;
    MagData magData;

    const bool hasNewImu = (m_imuQueue && xQueueReceive(m_imuQueue, &imuData, 0) == pdTRUE);
    const bool hasNewMag = (m_magQueue && xQueueReceive(m_magQueue, &magData, 0) == pdTRUE);

    std::optional<MagData> optMagData = std::nullopt;
    if (hasNewMag) {
        optMagData = magData;
    }

    ActuatorOutput surfaceCommands = {0.0f, 0.0f, 0.0f};
    float throttleOutput = 0.0f;

    if (hasNewImu) {
        // Dynamic dt: measure the actual tick delta rather than assuming
        // the nominal loop rate was met exactly.
        const uint32_t now = xTaskGetTickCount();
        if (lastUpdateTick > 0u) {
            const float dt = static_cast<float>(now - lastUpdateTick)
                           * (1.0f / static_cast<float>(configTICK_RATE_HZ));
            if (dt > 0.001f && dt < 0.1f) {
                loopDt = dt;
            }
        }
        lastUpdateTick = now;

        estimateAttitude(imuData, optMagData);

        Receiver::Setpoint setpoint;
        receiver.getSetpoint(setpoint);
        attitudeController.setSetpoints(setpoint.rollDeg, setpoint.pitchDeg);

        const FullSensorData controllerInput = {
            .rollDeg       = currentRollDeg,
            .pitchDeg      = currentPitchDeg,
            .yawDeg        = currentYawDeg,
            .rollRateDps   = imuData.gyroXDps,
            .pitchRateDps  = imuData.gyroYDps,
            .yawRateDps    = imuData.gyroZDps,
            .trueAirspeedMs = imuData.airspeedMs
        };

        surfaceCommands = attitudeController.update(controllerInput, loopDt);
        throttleOutput  = setpoint.throttle;
    }

    if (m_flightState && *m_flightState == STATE_IDLE) {
        actuators.setRawOutputs(
            Actuators::CENTER_PULSE_US,
            Actuators::CENTER_PULSE_US,
            Actuators::CENTER_PULSE_US,
            Actuators::THROTTLE_MIN_PULSE_US,
            receiver.getChannel(RC_CH_AUX2)
        );
    } else if (currentControlMode == ControlMode::PASS_THROUGH) {
        actuators.setRawOutputs(
            receiver.getChannel(RC_CH_ROLL),
            receiver.getChannel(RC_CH_PITCH),
            receiver.getChannel(RC_CH_YAW),
            receiver.getChannel(RC_CH_THROTTLE),
            receiver.getChannel(RC_CH_AUX2)
        );
    } else if (hasNewImu) {
        Receiver::StickInput stickInput;
        receiver.getStickInput(stickInput);

        actuators.setOutputs(
            surfaceCommands.aileron,
            surfaceCommands.elevator,
            surfaceCommands.rudder,
            throttleOutput,
            stickInput.gear
        );
    }

    if (hasNewImu && m_flightState) {
        FullSensorData teleData;
        teleData.rollDeg  = currentRollDeg;
        teleData.pitchDeg = currentPitchDeg;
        teleData.yawDeg   = currentYawDeg;
        telemetry.update(teleData, receiver.getCachedData(), *m_flightState);
    }
}

// ----------------------------------------------------------------------------
// estimateAttitude
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::estimateAttitude(
    const ImuData& imuData, const std::optional<MagData>& magData)
{
    // -------------------------------------------------------------------------
    // Prediction — gyroscope (convert deg/s → rad/s)
    // -------------------------------------------------------------------------
    typename FilterPolicy::Vector3 omega;
    omega << imuData.gyroXDps * DEG_TO_RAD,
             imuData.gyroYDps * DEG_TO_RAD,
             imuData.gyroZDps * DEG_TO_RAD;
    filter_.predict(loopDt, omega);

    // -------------------------------------------------------------------------
    // Measurement update — accelerometer
    //
    // Sign convention: raw data is in g (positive = structural reaction to
    // gravity). We convert to m/s² and negate to obtain the body-frame gravity
    // vector that the MEKF/UKF model expects (gravity is +Z in NED inertial).
    // -------------------------------------------------------------------------
    typename FilterPolicy::Vector3 accel;
    accel << -imuData.accelXG * GRAVITY_MSS,
             -imuData.accelYG * GRAVITY_MSS,
             -imuData.accelZG * GRAVITY_MSS;
    filter_.updateAccel(accel);

    // -------------------------------------------------------------------------
    // Measurement update — magnetometer
    //
    // Only update if the field vector is valid (non-zero magnitude).
    // The MEKF normalises internally; the UKF expects a normalised input.
    // We normalise here to satisfy both.
    // -------------------------------------------------------------------------
    if (magData.has_value()) {
        typename FilterPolicy::Vector3 mag;
        mag << magData->magXGauss,
               magData->magYGauss,
               magData->magZGauss;

        if (mag.norm() > 1e-6f) {
            mag.normalize();
            filter_.updateMag(mag, magRefVector_);
        }
    }

    // -------------------------------------------------------------------------
    // Output: quaternion → Euler angles
    // -------------------------------------------------------------------------
    const typename FilterPolicy::Vector3 euler = filter_.getEulerAnglesDeg();
    currentRollDeg  = euler.x();
    currentPitchDeg = euler.y();
    currentYawDeg   = euler.z();
}

// ----------------------------------------------------------------------------
// performPreflightSweep
// ----------------------------------------------------------------------------

template <typename FilterPolicy>
    requires gnc::AttitudeFilter<FilterPolicy>
void FlightControllerT<FilterPolicy>::performPreflightSweep()
{
    Receiver::StickInput stickInput;
    receiver.getStickInput(stickInput);

    // 300 steps at 10ms per step = 3.0 second sweep per control surface
    constexpr int SWEEP_STEPS = 300;
    constexpr TickType_t SWEEP_DELAY = pdMS_TO_TICKS(10);

    float aileronPos = 0.0f;
    float elevatorPos = 0.0f;
    float rudderPos = 0.0f;

    // Helper lambda to sweep a single targeted surface using a full sine wave period
    auto sweepSurface = [&](float& activeSurface) {
        for (int i = 0; i <= SWEEP_STEPS; ++i) {
            // Map the current step [0, SWEEP_STEPS] to a phase angle [0, 2π]
            float phase = (static_cast<float>(i) / static_cast<float>(SWEEP_STEPS)) * M_TWOPI;

            // sin(phase) smoothly maps exactly to our -1.0f to 1.0f output requirement
            activeSurface = std::sin(phase);

            actuators.setOutputs(aileronPos, elevatorPos, rudderPos, 0.0f, stickInput.gear);
            vTaskDelay(SWEEP_DELAY);
        }

        // Ensure the surface is explicitly clamped to dead-center before moving to the next
        activeSurface = 0.0f;
        actuators.setOutputs(aileronPos, elevatorPos, rudderPos, 0.0f, stickInput.gear);
    };

    // Execute the sweep sequence
    sweepSurface(aileronPos);
    sweepSurface(elevatorPos);
    sweepSurface(rudderPos);
}

// ============================================================================
// Explicit template instantiation
//
// This tells the linker to emit the FlightControllerT<ActiveFilter> object
// code from this translation unit. Without this, each TU that includes
// flight_controller.h would need the full method bodies — or the bodies would
// need to live in the header. Explicit instantiation is the correct approach
// for a template class whose definition and use are in separate TUs.
// ============================================================================

template class FlightControllerT<ActiveFilter>;
