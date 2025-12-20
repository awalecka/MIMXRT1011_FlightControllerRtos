/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 */
#include "flight_controller.h"

using namespace firmware::drivers;

// Define the global FlightController instance
FlightController g_flightController(0.01f);

// Extern the I2C driver defined in board/peripherals
extern ARM_DRIVER_I2C LPI2C1_SENSORS_CMSIS_DRIVER;

// --- PIDController Implementation ---
PIDController::PIDController(float p, float i, float d, float alpha)
    : kp(p), ki(i), kd(d), integral(0.0f), prevError(0.0f), alpha(alpha), dTermFiltered(0.0f) {}

float PIDController::calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - currentValue;
    float pOut = kp * error;
    float derivativeRaw = (error - prevError) / dt;
    dTermFiltered = (alpha * derivativeRaw) + ((1.0f - alpha) * dTermFiltered);
    float dOut = kd * dTermFiltered;
    float currentTotal = pOut + (ki * integral) + dOut;

    bool isSaturatedMax = (currentTotal >= maxLimit);
    bool isSaturatedMin = (currentTotal <= minLimit);

    if (!isSaturatedMax && !isSaturatedMin) {
        integral += error * dt;
    } else if (isSaturatedMax && error < 0.0f) {
        integral += error * dt;
    } else if (isSaturatedMin && error > 0.0f) {
        integral += error * dt;
    }

    prevError = error;
    float finalOutput = pOut + (ki * integral) + dOut;
    return std::clamp(finalOutput, minLimit, maxLimit);
}

void PIDController::reset() {
    integral = 0.0f;
    prevError = 0.0f;
    dTermFiltered = 0.0f;
}

// --- AttitudeController Implementation ---
AttitudeController::AttitudeController()
    : kpRollAngle(6.0f), kpPitchAngle(6.0f),
      rollRateController(0.1f, 0.01f, 0.0f, 0.5569f),
      pitchRateController(0.1f, 0.01f, 0.0f, 0.5569f),
      yawRateController(0.15f, 0.01f, 0.0f, 0.5569f),
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

    float sinRoll = arm_sin_f32(currentRollRad);
    float cosRoll = arm_cos_f32(currentRollRad);
    float sinPitch = arm_sin_f32(currentPitchRad);
    float cosPitch = arm_cos_f32(currentPitchRad);
    float tanRoll = (std::abs(cosRoll) > 1e-4f) ? (sinRoll / cosRoll) : 0.0f;

    float coordinatedYawRateRadS = 0.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        coordinatedYawRateRadS = (GRAVITY_MS2 / sensorData.trueAirspeedMs) * tanRoll;
    }

    float phiDotSpRadS = kpRollAngle * (targetRollRad - currentRollRad);
    float thetaDotSpRadS = kpPitchAngle * (targetPitchRad - currentPitchRad);
    float psiDotSpRadS = coordinatedYawRateRadS;

    float pSpRadS = phiDotSpRadS - psiDotSpRadS * sinPitch;
    float qSpRadS = thetaDotSpRadS * cosRoll + psiDotSpRadS * cosPitch * sinRoll;
    float rSpRadS = -thetaDotSpRadS * sinRoll + psiDotSpRadS * cosPitch * cosRoll;

    float ffRollMoment = kFfRoll * pSpRadS;
    float ffPitchMoment = kFfPitch * qSpRadS;
    float ffYawMoment = kFfYaw * rSpRadS;

    float pRadS = degToRad(sensorData.rollRateDps);
    float qRadS = degToRad(sensorData.pitchRateDps);
    float rRadS = degToRad(sensorData.yawRateDps);

    float airspeedScaler = 1.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        airspeedScaler = (NOMINAL_AIRSPEED_FOR_TUNING * NOMINAL_AIRSPEED_FOR_TUNING) /
                         (sensorData.trueAirspeedMs * sensorData.trueAirspeedMs);
    }
    airspeedScaler = std::clamp(airspeedScaler, 0.1f, 10.0f);

    float scalerInv = 1.0f / airspeedScaler;
    float maxRollPID = (1.0f * scalerInv) - ffRollMoment;
    float minRollPID = (-1.0f * scalerInv) - ffRollMoment;
    float maxPitchPID = (1.0f * scalerInv) - ffPitchMoment;
    float minPitchPID = (-1.0f * scalerInv) - ffPitchMoment;
    float maxYawPID = (1.0f * scalerInv) - ffYawMoment;
    float minYawPID = (-1.0f * scalerInv) - ffYawMoment;

    float fbRollMoment = rollRateController.calculate(pSpRadS, pRadS, dt, minRollPID, maxRollPID);
    float fbPitchMoment = pitchRateController.calculate(qSpRadS, qRadS, dt, minPitchPID, maxPitchPID);
    float fbYawMoment = yawRateController.calculate(rSpRadS, rRadS, dt, minYawPID, maxYawPID);

    float totalRollMoment = (fbRollMoment + ffRollMoment) * airspeedScaler;
    float totalPitchMoment = (fbPitchMoment + ffPitchMoment) * airspeedScaler;
    float totalYawMoment = (fbYawMoment + ffYawMoment) * airspeedScaler;

    ActuatorOutput controls;
    controls.aileron = std::clamp(totalRollMoment, -1.0f, 1.0f);
    controls.elevator = std::clamp(totalPitchMoment, -1.0f, 1.0f);
    controls.rudder = std::clamp(totalYawMoment, -1.0f, 1.0f);

    return controls;
}

// --- IMU Implementation ---
IMU::IMU() : gyroBiasX(0.0f), gyroBiasY(0.0f), gyroBiasZ(0.0f),
             magCalibrator(0.99f, 500.0f, 0.01f) {
}

int IMU::init() {
    i2c_sync_global_init();
    if (LSM6DSOX_Init(&g_sensor_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LSM6DSOX_I2C_ADDR) != 0 ||
        LSM6DSOX_SetAccConfig(&g_sensor_handle, LSM6DSOX_ACC_ODR_52Hz, LSM6DSOX_ACC_FS_2G) != 0 ||
        LSM6DSOX_SetGyroConfig(&g_sensor_handle, LSM6DSOX_GYRO_ODR_104Hz, LSM6DSOX_GYRO_FS_250DPS) != 0) {
        return -1;
    }
    if (LIS3MDL_Init(&g_mag_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LIS3MDL_I2C_ADDR) != 0 ||
        LIS3MDL_SetConfig(&g_mag_handle, LIS3MDL_ODR_80_HZ_HP, LIS3MDL_FS_4_GAUSS) != 0) {
        return -1;
    }
    return 0;
}

void IMU::calibrateGyro() {
    const int sampleCount = 200;
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    sensor_data_raw_t raw;
    sensor_data_vehicle_t veh;

    PRINTF("Starting Gyro Calibration... Keep vehicle still.\r\n");

    for (int i = 0; i < sampleCount; i++) {
        if (LSM6DSOX_ReadGyro(&g_sensor_handle, &raw.gyro_data) == 0) {
            LSM6DSOX_RemapData(&raw.gyro_data, &vehicleMapping, &veh.gyro_data);
            sumX += veh.gyro_data.x;
            sumY += veh.gyro_data.y;
            sumZ += veh.gyro_data.z;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    gyroBiasX = sumX / (float)sampleCount;
    gyroBiasY = sumY / (float)sampleCount;
    gyroBiasZ = sumZ / (float)sampleCount;

    PRINTF("Gyro Calibration Complete. New Bias: [%.3f, %.3f, %.3f]\r\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

int IMU::readData(IMU::RawData& rawData) {
    sensor_data_raw_t currentSensorDataRaw;
    sensor_data_vehicle_t currentSensorDataVehicle;
    lis3mdl_3axis_data_t magDataGauss;

    if (LSM6DSOX_ReadAcc(&g_sensor_handle, &currentSensorDataRaw.acc_data) != 0 ||
        LSM6DSOX_ReadGyro(&g_sensor_handle, &currentSensorDataRaw.gyro_data) != 0 ||
        LIS3MDL_ReadMagnetometer(&g_mag_handle, &magDataGauss) != 0) {
        return -1;
    }

    LSM6DSOX_RemapData(&currentSensorDataRaw.acc_data, &vehicleMapping, &currentSensorDataVehicle.acc_data);
    LSM6DSOX_RemapData(&currentSensorDataRaw.gyro_data, &vehicleMapping, &currentSensorDataVehicle.gyro_data);

    rawData.gyroXDps = currentSensorDataVehicle.gyro_data.x - gyroBiasX;
    rawData.gyroYDps = currentSensorDataVehicle.gyro_data.y - gyroBiasY;
    rawData.gyroZDps = currentSensorDataVehicle.gyro_data.z - gyroBiasZ;
    rawData.accelXG = currentSensorDataVehicle.acc_data.x;
    rawData.accelYG = currentSensorDataVehicle.acc_data.y;
    rawData.accelZG = currentSensorDataVehicle.acc_data.z;
    rawData.airspeedMs = 0.0f;

    return 0;
}

// --- Receiver Implementation ---
void Receiver::init() {
    std::fill(std::begin(m_cachedRcData.channels), std::end(m_cachedRcData.channels), 1500);
    m_cachedRcData.channels[RC_CH_THROTTLE] = 1000;
    PRINTF("Receiver Initialized. \r\n");
}

void Receiver::update() {
    RC_Channels_t tempBuffer;
    if (xQueueReceive(g_command_data_queue, &tempBuffer, 0) == pdPASS) {
        m_cachedRcData = tempBuffer;
    }
}

void Receiver::getSetpoint(Receiver::Setpoint& setpoint) {
    setpoint.rollDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.pitchDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

void Receiver::getStickInput(Receiver::StickInput& input) {
    input.roll = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.pitch = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.yaw = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_YAW]), 1000.0f, 2000.0f, -1.0f, 1.0f);
    input.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

uint16_t Receiver::getChannel(uint8_t channel) const {
    if (channel < IBUS_MAX_CHANNELS) {
        return m_cachedRcData.channels[channel];
    }
    return 0;
}

const RC_Channels_t& Receiver::getCachedData() const {
    return m_cachedRcData;
}

// --- Actuators Implementation ---

// Static configuration for servo channels.
// Mapping is based on the provided .mex configuration.
// Index 0: Aileron  (PWM1 Mod 0 Ch A)
// Index 1: Elevator (PWM1 Mod 0 Ch B)
// Index 2: Throttle (PWM1 Mod 2 Ch A)
// Index 3: Rudder   (PWM1 Mod 2 Ch B)
static const ServoChannelConfig s_servoConfig[] = {
    { PWM1, kPWM_Module_0, kPWM_PwmA }, // Aileron
    { PWM1, kPWM_Module_0, kPWM_PwmB }, // Elevator
    { PWM1, kPWM_Module_2, kPWM_PwmA }, // Throttle
    { PWM1, kPWM_Module_2, kPWM_PwmB }, // Rudder
};

void Actuators::init() {
    // Initialize the ServoDriver with the static configuration
    m_servoDriver.init(s_servoConfig);
    PRINTF("Actuators Initialized with ServoDriver. \r\n");
}

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {
    // Set Control Surfaces (Normalized -1.0 to 1.0)
    m_servoDriver.setNormalizedOutput(0, aileron);  // Aileron
    m_servoDriver.setNormalizedOutput(1, elevator); // Elevator
    m_servoDriver.setNormalizedOutput(3, rudder);   // Rudder

    // Set Throttle
    // Map input (0.0 - 100.0) to Normalized (-1.0 to 1.0) for ServoDriver
    // 0% throttle -> -1.0 (1000us)
    // 100% throttle -> 1.0 (2000us)
    float throttleNormalized = mapFloat(throttle, 0.0f, 100.0f, -1.0f, 1.0f);
    m_servoDriver.setNormalizedOutput(2, throttleNormalized);
}

void Actuators::setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs) {
    m_servoDriver.setPulseWidthUs(0, aileronUs);  // Aileron
    m_servoDriver.setPulseWidthUs(1, elevatorUs); // Elevator
    m_servoDriver.setPulseWidthUs(2, throttleUs); // Throttle (Note: Index 2 is throttle in config)
    m_servoDriver.setPulseWidthUs(3, rudderUs);   // Rudder
}

// --- FlightController Implementation ---
FlightController::FlightController(float loopTime)
    : loopDt(loopTime), currentRollDeg(0.0f), currentPitchDeg(0.0f), currentYawDeg(0.0f),
      teleUpdate(20), teleCounter(0), currentControlMode(ControlMode::STABILIZED) {
}

int FlightController::init() {
    if (imu.init() != 0) {
        return -1;
    }
    receiver.init();
    actuators.init();
    FusionAhrsInitialise(&ahrs);
    return 0;
}

void FlightController::calibrateSensors() {
    imu.calibrateGyro();
}

void FlightController::setControlMode(ControlMode mode) {
    currentControlMode = mode;
}

void FlightController::update() {
    // Update Receiver Cache
    receiver.update();

    // Read Sensors
    IMU::RawData rawSensorData;
    int sensorStatus = imu.readData(rawSensorData);

    // 3. Determine Control Mode
    uint16_t aux1Value = receiver.getChannel(RC_CH_AUX1);
    if (aux1Value > 1500) {
        currentControlMode = ControlMode::PASS_THROUGH;
    } else {
        currentControlMode = ControlMode::STABILIZED;
    }

    // Failsafe: Force Pass-Through if Sensors Failed
    if (sensorStatus != 0 && currentControlMode == ControlMode::STABILIZED) {
        currentControlMode = ControlMode::PASS_THROUGH;
    }

    // Control Logic
    if (currentControlMode == ControlMode::PASS_THROUGH) {
        // --- RAW PASS THROUGH ---
        // Read raw channel values (microseconds) directly from Receiver
        uint16_t rollRaw     = receiver.getChannel(RC_CH_ROLL);
        uint16_t pitchRaw    = receiver.getChannel(RC_CH_PITCH);
        uint16_t yawRaw      = receiver.getChannel(RC_CH_YAW);
        uint16_t throttleRaw = receiver.getChannel(RC_CH_THROTTLE);

        // Send raw values directly to Actuators (Bypassing normalization)
        actuators.setRawOutputs(rollRaw, pitchRaw, yawRaw, throttleRaw);

        // For telemetry logging, we still populate surfaceCommands with normalized values
        // so the log remains readable, but these aren't used for control.
        Receiver::StickInput sticks;
        receiver.getStickInput(sticks);
        ActuatorOutput telemetryStr = { sticks.roll, sticks.pitch, sticks.yaw };

        teleCounter++;
        if (teleCounter >= teleUpdate) {
             xQueueSend(g_controls_data_queue, &telemetryStr, (TickType_t)0);
             teleCounter = 0;
        }

    } else {
        // --- STABILIZED MODE ---
        ActuatorOutput surfaceCommands = {0.0f, 0.0f, 0.0f};
        float throttleOutput = 0.0f;

        if (sensorStatus == 0) {
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

        // Apply Normalized Outputs
        actuators.setOutputs(surfaceCommands.aileron, surfaceCommands.elevator, surfaceCommands.rudder, throttleOutput);

        teleCounter++;
        if (teleCounter >= teleUpdate) {
             xQueueSend(g_controls_data_queue, &surfaceCommands, (TickType_t)0);
             teleCounter = 0;
        }
    }
}

void FlightController::estimateAttitude(const IMU::RawData& rawData) {
    const FusionVector gyroscope = {rawData.gyroXDps, rawData.gyroYDps, rawData.gyroZDps};
    const FusionVector accelerometer = {rawData.accelXG, rawData.accelYG, rawData.accelZG};

    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, loopDt);

    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    currentRollDeg = euler.angle.roll;
    currentPitchDeg = euler.angle.pitch;
    currentYawDeg = euler.angle.yaw;
}

RC_Channels_t FlightController::getRcData() const {
    return receiver.getCachedData();
}
