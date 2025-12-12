#include "flight_controller.h"

// --- PIDController Implementation ---

PIDController::PIDController(float p, float i, float d, float limit)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f), integralLimit(std::abs(limit)) {}

float PIDController::calculate(float setpoint, float currentValue, float dt) {
    if (dt <= 0) return 0.0f;
    float error = setpoint - currentValue;

    integral += error * dt;

    // Anti-windup
    if (integralLimit > 0.0f) {
        integral = std::clamp(integral, -integralLimit, integralLimit);
    }

    float derivative = (dt > 1e-6) ? (error - prevError) / dt : 0.0f;
    prevError = error;

    return (kp * error) + (ki * integral) + (kd * derivative);
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}

// --- AttitudeController Implementation ---

AttitudeController::AttitudeController()
    : kpRollAngle(6.0f), kpPitchAngle(6.0f),
      rollRateController(0.1f, 0.05f, 0.0f, 1.0f),
      pitchRateController(0.1f, 0.05f, 0.0f, 1.0f),
      yawRateController(0.15f, 0.08f, 0.0f, 1.0f),
      kFfRoll(0.1f), kFfPitch(0.1f), kFfYaw(0.1f),
      targetRollDeg(0.0f), targetPitchDeg(0.0f) {}

void AttitudeController::setSetpoints(float roll, float pitch) {
    targetRollDeg = roll;
    targetPitchDeg = pitch;
}

ActuatorOutput AttitudeController::update(const FullSensorData& sensorData, float dt) {
    float currentRollRad = degToRad(sensorData.rollDeg);
    float currentPitchRad = degToRad(sensorData.pitchDeg);
    float targetRollRad = degToRad(targetRollDeg);
    float targetPitchRad = degToRad(targetPitchDeg);

    // Coordinated turn calculation
    float coordinatedYawRateRadS = 0.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        coordinatedYawRateRadS = (GRAVITY_MS2 / sensorData.trueAirspeedMs) * std::tan(targetRollRad);
    }

    // Outer-loop (Angle)
    float phiDotSpRadS = kpRollAngle * (targetRollRad - currentRollRad);
    float thetaDotSpRadS = kpPitchAngle * (targetPitchRad - currentPitchRad);
    float psiDotSpRadS = coordinatedYawRateRadS;

    // Transform desired body rates
    float pSpRadS = phiDotSpRadS - psiDotSpRadS * std::sin(currentPitchRad);
    float qSpRadS = thetaDotSpRadS * std::cos(currentRollRad) + psiDotSpRadS * std::cos(currentPitchRad) * std::sin(currentRollRad);
    float rSpRadS = -thetaDotSpRadS * std::sin(currentRollRad) + psiDotSpRadS * std::cos(currentPitchRad) * std::cos(currentRollRad);

    // Feed-forward moments
    float ffRollMoment = kFfRoll * pSpRadS;
    float ffPitchMoment = kFfPitch * qSpRadS;
    float ffYawMoment = kFfYaw * rSpRadS;

    // Current rates
    float pRadS = degToRad(sensorData.rollRateDps);
    float qRadS = degToRad(sensorData.pitchRateDps);
    float rRadS = degToRad(sensorData.yawRateDps);

    // Feedback (PID)
    float fbRollMoment = rollRateController.calculate(pSpRadS, pRadS, dt);
    float fbPitchMoment = pitchRateController.calculate(qSpRadS, qRadS, dt);
    float fbYawMoment = yawRateController.calculate(rSpRadS, rRadS, dt);

    // Airspeed Scaling
    float airspeedScaler = 1.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        airspeedScaler = (NOMINAL_AIRSPEED_FOR_TUNING * NOMINAL_AIRSPEED_FOR_TUNING) /
                         (sensorData.trueAirspeedMs * sensorData.trueAirspeedMs);
    }

    float totalRollMoment = (fbRollMoment + ffRollMoment) * airspeedScaler;
    float totalPitchMoment = (fbPitchMoment + ffPitchMoment) * airspeedScaler;
    float totalYawMoment = (fbYawMoment + ffYawMoment) * airspeedScaler;

    ActuatorOutput controls;
    controls.aileron = std::clamp(totalRollMoment, -1.0f, 1.0f);
    controls.elevator = std::clamp(totalPitchMoment, -1.0f, 1.0f);
    controls.rudder = std::clamp(totalYawMoment, -1.0f, 1.0f);

    return controls;
}

// --- Hardware Stubs Implementation ---

void IMU::init() {
    PRINTF("IMU Initialized. \r\n");
}

int IMU::readData(IMU::RawData& rawData) {
    sensor_data_raw_t currentSensorDataRaw;
    sensor_data_vehicle_t currentSensorDataVehicle;

    if (LSM6DSOX_ReadAcc(&g_sensor_handle, &currentSensorDataRaw.acc_data) != 0 ||
        LSM6DSOX_ReadGyro(&g_sensor_handle, &currentSensorDataRaw.gyro_data) != 0 ||
        LIS3MDL_ReadMagnetometerRaw(&g_mag_handle, &currentSensorDataRaw.mag_data) != 0) {
        return -1;
    }

    LSM6DSOX_RemapData(&currentSensorDataRaw.acc_data, &vehicleMapping, &currentSensorDataVehicle.acc_data);
    LSM6DSOX_RemapData(&currentSensorDataRaw.gyro_data, &vehicleMapping, &currentSensorDataVehicle.gyro_data);

    rawData.accelXG = currentSensorDataVehicle.acc_data.x;
    rawData.accelYG = currentSensorDataVehicle.acc_data.y;
    rawData.accelZG = currentSensorDataVehicle.acc_data.z;
    rawData.gyroXDps = currentSensorDataVehicle.gyro_data.x;
    rawData.gyroYDps = currentSensorDataVehicle.gyro_data.y;
    rawData.gyroZDps = currentSensorDataVehicle.gyro_data.z;
    rawData.airspeedMs = 0.0f;

    return 0;
}

void Receiver::init() { PRINTF("Receiver Initialized. \r\n"); }

void Receiver::getSetpoint(Receiver::Setpoint& setpoint) {
    RC_Channels_t receivedChannels;
    // Implementation placeholder for queue receive logic
    setpoint.rollDeg = 0.0f;
    setpoint.pitchDeg = 0.0f;
    setpoint.throttle = 0.0f;
}

void Actuators::init() { PRINTF("Actuators Initialized. \r\n"); }

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {
    // Usage of mapFloat
    PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_0, kPWM_PwmA, kPWM_EdgeAligned, static_cast<uint8_t>(mapFloat(aileron, -30, 30, 50, 100)));
    PWM_SetPwmLdok(PWM1_PERIPHERAL, (kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3), true);
}

// --- FlightController Implementation ---

FlightController::FlightController(float loopTime)
    : loopDt(loopTime), currentRollDeg(0.0f), currentPitchDeg(0.0f), currentYawDeg(0.0f), teleUpdate(20), teleCounter(0) {
    imu.init();
    receiver.init();
    actuators.init();
    FusionAhrsInitialise(&ahrs);
}

void FlightController::update() {
    IMU::RawData rawSensorData;

    imu.readData(rawSensorData);
    estimateAttitude(rawSensorData);

    Receiver::Setpoint setpoint = {
        .rollDeg = attitudeController.getTargetPitchDeg(),
        .pitchDeg = attitudeController.getTargetRollDeg(),
        .throttle = 0.0f
    };

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

    ActuatorOutput surfaceCommands = attitudeController.update(controllerInput, loopDt);
    actuators.setOutputs(surfaceCommands.aileron, surfaceCommands.elevator, surfaceCommands.rudder, setpoint.throttle);

    teleCounter++;
    if (teleCounter >= teleUpdate) {
        xQueueSend(g_controls_data_queue, &surfaceCommands, (TickType_t)0);
        teleCounter = 0;
    }
}

void FlightController::estimateAttitude(const IMU::RawData& rawData) {
    const FusionVector gyroscope = {rawData.gyroXDps, rawData.gyroYDps, rawData.gyroZDps};
    const FusionVector accelerometer = {rawData.accelXG, rawData.accelYG, rawData.accelZG};

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, loopDt);
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    currentRollDeg = euler.angle.roll;
    currentPitchDeg = euler.angle.pitch;
    currentYawDeg = 0.0f; // Placeholder until magnetometer integration
}
