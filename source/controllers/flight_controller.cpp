/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 */
#include "flight_controller.h"

using namespace firmware::drivers;

FlightController g_flightController(0.01f);

FlightController::FlightController(float loopTime)
    : loopDt(loopTime), currentRollDeg(0.0f), currentPitchDeg(0.0f), currentYawDeg(0.0f),
      currentControlMode(ControlMode::STABILIZED) {
}

int FlightController::init() {
    if (sensorSystem.init() != 0) {
        return -1;
    }

    // Load Calibration
    MagCalibrationParams calParams;
    if (Settings::loadMagCal(calParams)) {
        sensorSystem.setCalibration(calParams.hardIron, calParams.softIron);
    }

    receiver.init();
    actuators.init();
    FusionAhrsInitialise(&ahrs);
    return 0;
}

void FlightController::calibrateSensors() {
    sensorSystem.calibrateGyro();
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
    // Feeds internal RLS and returns raw data
    if (sensorSystem.feedMagCalibration(rawData) != 0) return false;

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
    // 1. Update Receiver
    receiver.update();

    // Check for AUX switch to toggle mode
    if (receiver.getChannel(RC_CH_AUX1) > 1500) {
        currentControlMode = ControlMode::PASS_THROUGH;
    } else {
        currentControlMode = ControlMode::STABILIZED;
    }

    // 2. Read Sensors & Estimate Attitude
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData rawSensorData;
    int sensorStatus = sensorSystem.readData(rawSensorData);

    if (sensorStatus == 0) {
        estimateAttitude(rawSensorData);
    } else if (sensorStatus != 0 && currentControlMode == ControlMode::STABILIZED) {
        currentControlMode = ControlMode::PASS_THROUGH; // Failsafe
    }

    // 3. Control Loop
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

        if (sensorStatus == 0) {
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

    // 4. Telemetry
    // Construct local FullSensorData for telemetry (reusing computed attitude)
    FullSensorData teleData;
    teleData.rollDeg = currentRollDeg;
    teleData.pitchDeg = currentPitchDeg;
    teleData.yawDeg = currentYawDeg;
    // We pass 0 for rates in teleData if not explicitly needed by current tele manager logic,
    // or we can pass the full raw struct if we expand telemetry later.

    telemetry.update(teleData, receiver.getCachedData(), g_flight_state);
}

void FlightController::estimateAttitude(const SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData) {
    const FusionVector gyroscope = {rawData.gyroXDps, rawData.gyroYDps, rawData.gyroZDps};
    const FusionVector accelerometer = {rawData.accelXG, rawData.accelYG, rawData.accelZG};

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, loopDt);
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    currentRollDeg = euler.angle.roll;
    currentPitchDeg = euler.angle.pitch;
    currentYawDeg = euler.angle.yaw;
}
