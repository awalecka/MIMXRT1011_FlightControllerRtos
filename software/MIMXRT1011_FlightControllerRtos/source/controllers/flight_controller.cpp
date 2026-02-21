/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 */
#include "flight_controller.h"
#include <cmath>

using namespace firmware::drivers;

FlightController g_flightController(FlightController::LOOP_DT_S);

static constexpr float DEG_TO_RAD = 3.1415926535f / 180.0f;
static constexpr float RAD_TO_DEG = 180.0f / 3.1415926535f;
static constexpr float GRAVITY_MSS = 9.80665f;

FlightController::FlightController(float loopTime)
    : ukf({
          .alpha = 1.0f,
          .beta  = 2.0f,
          .kappa = 0.0f,
          .qGyro = 0.1f,
          .qBias = 0.001f,
          .rAccel = 0.5f,
          .rMag   = 1.0f
      }),
      loopDt(loopTime), currentRollDeg(0.0f), currentPitchDeg(0.0f), currentYawDeg(0.0f),
      currentControlMode(ControlMode::STABILIZED), lastSensorTimestamp(0), staleDataCounter(0), lastUpdateTick(0) {
}

int FlightController::init() {
    if (sensorSystem.init() != 0) {
        return -1;
    }

    MagCalibrationParams calParams;
    if (Settings::loadMagCal(calParams)) {
        sensorSystem.setCalibration(calParams.hardIron, calParams.softIron);
    }

    receiver.init();
    actuators.init();

    // --- Initial Alignment Sequence (UKF) ---
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData raw;

    float sumAx = 0, sumAy = 0, sumAz = 0;
    float sumMx = 0, sumMy = 0, sumMz = 0;
    const int initSamples = 50;

    // Collect samples to determine initial orientation
    for (int i = 0; i < initSamples; i++) {
        while (sensorSystem.readData(raw) != 0);
        sumAx += raw.accelXG;
        sumAy += raw.accelYG;
        sumAz += raw.accelZG;
        sumMx += raw.magXGauss;
        sumMy += raw.magYGauss;
        sumMz += raw.magZGauss;

        // Simple delay to allow sensor ODR to catch up
        for (volatile int k = 0; k < 10000; k++);
    }

    // Calculate averages
    gnc::AttitudeUkf::Vector3 avgAccel;
    avgAccel << sumAx / initSamples, sumAy / initSamples, sumAz / initSamples;

    gnc::AttitudeUkf::Vector3 avgMag;
    avgMag << sumMx / initSamples, sumMy / initSamples, sumMz / initSamples;

    // This sets the initial Quaternion and computes the local magnetic reference (Dip Angle)
    ukf.align(avgAccel, avgMag, magRefVector);

    return 0;
}

void FlightController::calibrateSensors() {
    sensorSystem.calibrateGyro();
}

int FlightController::readSensors(SensorData& data) {
    return sensorSystem.readData(data);
}

void FlightController::setControlMode(ControlMode mode) {
    currentControlMode = mode;
}

void FlightController::getStickInput(Receiver::StickInput& input) {
    receiver.getStickInput(input);
}

RC_Channels_t FlightController::getRcData() const {
    return receiver.getCachedData();
}

bool FlightController::calibrateMagnetometerStep() {
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData rawData;
    // Feed raw data into the RLS calibrator
    if (sensorSystem.feedMagCalibration(rawData) != 0) {
        return false;
    }
    telemetry.sendMagRaw(rawData.magXGauss, rawData.magYGauss, rawData.magZGauss);
    return true;
}

void FlightController::saveCalibration() {
    MagCalibrationParams params;
    sensorSystem.getCalibration(params.hardIron, params.softIron);
    Settings::saveMagCal(params);
    telemetry.sendCalStatus(true);
}

void FlightController::update() {
    receiver.update();

    // Check for mode switch on AUX1
    if (receiver.getChannel(RC_CH_AUX1) > 1500) {
        currentControlMode = ControlMode::PASS_THROUGH;
    } else {
        currentControlMode = ControlMode::STABILIZED;
    }

    // Sensor data is now read via Queue
    SensorData rawSensorData;
    bool hasNewSensorData = (xQueuePeek(g_sensor_data_queue, &rawSensorData, 0) == pdTRUE);

    if (currentControlMode == ControlMode::PASS_THROUGH) {
        actuators.setRawOutputs(
            receiver.getChannel(RC_CH_ROLL),
            receiver.getChannel(RC_CH_PITCH),
            receiver.getChannel(RC_CH_YAW),
            receiver.getChannel(RC_CH_THROTTLE)
        );
    } else {
        ActuatorOutput surfaceCommands = {0.0f, 0.0f, 0.0f};
        float throttleOutput = 0.0f;
    
        if (hasNewSensorData) {
            // --- Dynamic DT Calculation ---
            uint32_t now = xTaskGetTickCount();
            if (lastUpdateTick > 0) {
                // Convert ticks to seconds
                float dt = (now - lastUpdateTick) * (1.0f / configTICK_RATE_HZ);
                
                // Simple sanity check/clamping (0.001s to 0.1s)
                if (dt > 0.001f && dt < 0.1f) {
                    loopDt = dt; 
                }
            }
            lastUpdateTick = now;

            // Staleness Check
            estimateAttitude(rawSensorData);
            
            Receiver::Setpoint setpoint;
            receiver.getSetpoint(setpoint);
            attitudeController.setSetpoints(setpoint.rollDeg, setpoint.pitchDeg);

            FullSensorData controllerInput = {
                .rollDeg = currentRollDeg,
                .pitchDeg = currentPitchDeg,
                .yawDeg = currentYawDeg,
                .rollRateDps = rawSensorData.gyroXDps,
                .pitchRateDps = rawSensorData.gyroYDps,
                .yawRateDps = rawSensorData.gyroZDps,
                .trueAirspeedMs = rawSensorData.airspeedMs
            };

            surfaceCommands = attitudeController.update(controllerInput, loopDt);
            throttleOutput = setpoint.throttle;
        }
        actuators.setOutputs(surfaceCommands.aileron, surfaceCommands.elevator, surfaceCommands.rudder, throttleOutput);
    }

    if (hasNewSensorData) {
        FullSensorData teleData;
        teleData.rollDeg = currentRollDeg;
        teleData.pitchDeg = currentPitchDeg;
        teleData.yawDeg = currentYawDeg;

        telemetry.update(teleData, receiver.getCachedData(), g_flight_state);
    }
}

void FlightController::estimateAttitude(const SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData) {
    // --- UKF Prediction Step ---
    gnc::AttitudeUkf::Vector3 omega;
    omega << rawData.gyroXDps * DEG_TO_RAD,
             rawData.gyroYDps * DEG_TO_RAD,
             rawData.gyroZDps * DEG_TO_RAD;
    ukf.predict(loopDt, omega);

    // --- UKF Update Step (Accelerometer) ---
    gnc::AttitudeUkf::Vector3 accel;
    accel << -rawData.accelXG * GRAVITY_MSS,
             -rawData.accelYG * GRAVITY_MSS,
             -rawData.accelZG * GRAVITY_MSS;
    ukf.updateAccel(accel);

    // --- UKF Update Step (Magnetometer) ---
    gnc::AttitudeUkf::Vector3 mag;
    mag << rawData.magXGauss, rawData.magYGauss, rawData.magZGauss;

    // Only update mag if field vector is valid (non-zero)
    if (mag.norm() > 1e-6f) {
        mag.normalize();
        ukf.updateMag(mag, magRefVector);
    }

    // --- Output Conversion (Quaternion -> Euler Degrees) ---
    gnc::AttitudeUkf::Vector3 euler = ukf.getEulerAnglesDeg();
    currentRollDeg  = euler.x();
    currentPitchDeg = euler.y();
    currentYawDeg   = euler.z();
}
