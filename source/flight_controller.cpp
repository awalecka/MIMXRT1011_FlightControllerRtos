/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 */
#include "flight_controller.h"

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

    // Calculate P term
    float pOut = kp * error;

    // Calculate D term with Low Pass Filter
    // 1. Compute raw derivative
    float derivativeRaw = (error - prevError) / dt;

    // 2. Apply EMA (Exponential Moving Average) filter using precomputed alpha
    dTermFiltered = (alpha * derivativeRaw) + ((1.0f - alpha) * dTermFiltered);

    float dOut = kd * dTermFiltered;

    // Tentative output without new integral
    float currentTotal = pOut + (ki * integral) + dOut;

    // --- Conditional Integration (Anti-Windup) ---
    // Only accumulate error if:
    // 1. We are NOT saturated
    //    OR
    // 2. We ARE saturated, but the error helps unwinding (sign opposes saturation)
    bool isSaturatedMax = (currentTotal >= maxLimit);
    bool isSaturatedMin = (currentTotal <= minLimit);

    if (!isSaturatedMax && !isSaturatedMin) {
        integral += error * dt;
    } else if (isSaturatedMax && error < 0.0f) {
        integral += error * dt; // Allow unwinding from max
    } else if (isSaturatedMin && error > 0.0f) {
        integral += error * dt; // Allow unwinding from min
    }

    // Save error for next D term
    prevError = error;

    // Final Calculation with (potentially) updated integral
    float finalOutput = pOut + (ki * integral) + dOut;

    // Hard clamp to ensure physical safety
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
      // Initialize PIDs with precomputed alpha for 20Hz cutoff at 100Hz loop.
      // tau = 1/(2*pi*20) ~= 0.007958.
      // alpha = dt / (tau + dt) = 0.01 / (0.007958 + 0.01) ~= 0.5569.
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

    // --- Optimized Trigonometry (CMSIS-DSP) ---
    // Calculate sin/cos once per update to save cycles
    float sinRoll = arm_sin_f32(currentRollRad);
    float cosRoll = arm_cos_f32(currentRollRad);
    float sinPitch = arm_sin_f32(currentPitchRad);
    float cosPitch = arm_cos_f32(currentPitchRad);

    // Calculate tan(roll) = sin/cos. Protect against div/0.
    float tanRoll = (std::abs(cosRoll) > 1e-4f) ? (sinRoll / cosRoll) : 0.0f;

    // Coordinated turn calculation
    float coordinatedYawRateRadS = 0.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        coordinatedYawRateRadS = (GRAVITY_MS2 / sensorData.trueAirspeedMs) * tanRoll;
    }

    float phiDotSpRadS = kpRollAngle * (targetRollRad - currentRollRad);
    float thetaDotSpRadS = kpPitchAngle * (targetPitchRad - currentPitchRad);
    float psiDotSpRadS = coordinatedYawRateRadS;

    // Mixing Matrix using pre-calculated trig values
    float pSpRadS = phiDotSpRadS - psiDotSpRadS * sinPitch;
    float qSpRadS = thetaDotSpRadS * cosRoll + psiDotSpRadS * cosPitch * sinRoll;
    float rSpRadS = -thetaDotSpRadS * sinRoll + psiDotSpRadS * cosPitch * cosRoll;

    // Feed-Forward Terms
    float ffRollMoment = kFfRoll * pSpRadS;
    float ffPitchMoment = kFfPitch * qSpRadS;
    float ffYawMoment = kFfYaw * rSpRadS;

    float pRadS = degToRad(sensorData.rollRateDps);
    float qRadS = degToRad(sensorData.pitchRateDps);
    float rRadS = degToRad(sensorData.yawRateDps);

    // Dynamic Scaling based on Airspeed
    float airspeedScaler = 1.0f;
    if (std::abs(sensorData.trueAirspeedMs) > 1.0f) {
        airspeedScaler = (NOMINAL_AIRSPEED_FOR_TUNING * NOMINAL_AIRSPEED_FOR_TUNING) /
                         (sensorData.trueAirspeedMs * sensorData.trueAirspeedMs);
    }
    // Prevent scaler from becoming infinite or zero
    airspeedScaler = std::clamp(airspeedScaler, 0.1f, 10.0f);


    // --- Dynamic Limit Calculation for PID ---
    // The actuator limits are [-1.0, 1.0].
    // Total Output = (PID + FF) * Scaler
    // Therefore: PID Limit = (ActuatorLimit / Scaler) - FF
    float scalerInv = 1.0f / airspeedScaler;

    float maxRollPID = (1.0f * scalerInv) - ffRollMoment;
    float minRollPID = (-1.0f * scalerInv) - ffRollMoment;

    float maxPitchPID = (1.0f * scalerInv) - ffPitchMoment;
    float minPitchPID = (-1.0f * scalerInv) - ffPitchMoment;

    float maxYawPID = (1.0f * scalerInv) - ffYawMoment;
    float minYawPID = (-1.0f * scalerInv) - ffYawMoment;

    // Calculate PID with Dynamic Limits (Anti-Windup active here)
    float fbRollMoment = rollRateController.calculate(pSpRadS, pRadS, dt, minRollPID, maxRollPID);
    float fbPitchMoment = pitchRateController.calculate(qSpRadS, qRadS, dt, minPitchPID, maxPitchPID);
    float fbYawMoment = yawRateController.calculate(rSpRadS, rRadS, dt, minYawPID, maxYawPID);

    // Summation (Safety clamp at the end just in case)
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

    // Initialize LSM6DSOX (Accel/Gyro)
    // NOTE: Gyro ODR should be >= 104Hz for 100Hz Loop
    if (LSM6DSOX_Init(&g_sensor_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LSM6DSOX_I2C_ADDR) != 0 ||
        LSM6DSOX_SetAccConfig(&g_sensor_handle, LSM6DSOX_ACC_ODR_52Hz, LSM6DSOX_ACC_FS_2G) != 0 ||
        LSM6DSOX_SetGyroConfig(&g_sensor_handle, LSM6DSOX_GYRO_ODR_104Hz, LSM6DSOX_GYRO_FS_250DPS) != 0) {
        return -1;
    }

    // Initialize LIS3MDL (Magnetometer)
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
        // Read directly from driver to get raw uncorrected values
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

    // Remap Accel and Gyro to Vehicle Frame
    LSM6DSOX_RemapData(&currentSensorDataRaw.acc_data, &vehicleMapping, &currentSensorDataVehicle.acc_data);
    LSM6DSOX_RemapData(&currentSensorDataRaw.gyro_data, &vehicleMapping, &currentSensorDataVehicle.gyro_data);

    // Apply Gyro Bias Correction
    rawData.gyroXDps = currentSensorDataVehicle.gyro_data.x - gyroBiasX;
    rawData.gyroYDps = currentSensorDataVehicle.gyro_data.y - gyroBiasY;
    rawData.gyroZDps = currentSensorDataVehicle.gyro_data.z - gyroBiasZ;

    rawData.accelXG = currentSensorDataVehicle.acc_data.x;
    rawData.accelYG = currentSensorDataVehicle.acc_data.y;
    rawData.accelZG = currentSensorDataVehicle.acc_data.z;
    rawData.airspeedMs = 0.0f;

    return 0;
}

void Receiver::init() {
    // Initialize cache to center/safe values
    std::fill(std::begin(m_cachedRcData.channels), std::end(m_cachedRcData.channels), 1500);
    // Ensure throttle is at minimum for safety
    m_cachedRcData.channels[RC_CH_THROTTLE] = 1000;
    PRINTF("Receiver Initialized. \r\n");
}

void Receiver::update() {
    RC_Channels_t tempBuffer;
    // Check if new data is available in the queue
    if (xQueueReceive(g_command_data_queue, &tempBuffer, 0) == pdPASS) {
        // Atomically update the cache (structure copy)
        m_cachedRcData = tempBuffer;
    }
}

void Receiver::getSetpoint(Receiver::Setpoint& setpoint) {
    // Read from the cached data structure
    setpoint.rollDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_ROLL]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.pitchDeg = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_PITCH]), 1000.0f, 2000.0f, -20.0f, 20.0f);
    setpoint.throttle = mapFloat(static_cast<float>(m_cachedRcData.channels[RC_CH_THROTTLE]), 1000.0f, 2000.0f, 0.0f, 100.0f);
}

void Receiver::getStickInput(Receiver::StickInput& input) {
    // Map standard 1000-2000us range to -1.0 to 1.0 for control surfaces
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

void Actuators::init() { PRINTF("Actuators Initialized. \r\n"); }

void Actuators::setOutputs(float aileron, float elevator, float rudder, float throttle) {

    PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, kPWM_Module_0, kPWM_PwmA, kPWM_EdgeAligned, static_cast<uint8_t>(mapFloat(aileron, -1.0f, 1.0f, 51.0f, 99.0f)));

    PWM_SetPwmLdok(PWM1_PERIPHERAL, (kPWM_Control_Module_0 | kPWM_Control_Module_2 | kPWM_Control_Module_3), true);
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
    // Update Receiver Cache (Always allowed)
    receiver.update();

    // Read Sensors (Attempt read)
    IMU::RawData rawSensorData;
    int sensorStatus = imu.readData(rawSensorData);

    // Determine Mode (Independent of sensor status)
    uint16_t aux1Value = receiver.getChannel(RC_CH_AUX1);
    if (aux1Value > 1500) {
        currentControlMode = ControlMode::PASS_THROUGH;
    } else {
        currentControlMode = ControlMode::STABILIZED;
    }

    // Force Pass-Through if Sensors Failed (Failsafe fallback)
    if (sensorStatus != 0 && currentControlMode == ControlMode::STABILIZED) {
        // Fallback to manual control to allow landing
        currentControlMode = ControlMode::PASS_THROUGH;
    }

    ActuatorOutput surfaceCommands = {0.0f, 0.0f, 0.0f}; // Default neutral
    float throttleOutput = 0.0f;

    // Control Logic
    if (currentControlMode == ControlMode::PASS_THROUGH) {
        Receiver::StickInput sticks;
        receiver.getStickInput(sticks);

        surfaceCommands.aileron  = sticks.roll;
        surfaceCommands.elevator = sticks.pitch;
        surfaceCommands.rudder   = sticks.yaw;
        throttleOutput           = sticks.throttle;

    } else if (sensorStatus == 0) {
        // Only run PID if sensors are healthy
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
    else {
        // Implicit else: Stabilized mode requested BUT sensors failed.
        // We fell back to PASS_THROUGH above, or we set Neutral output.
    }

    // Actuator Output (Always runs)
    actuators.setOutputs(surfaceCommands.aileron, surfaceCommands.elevator, surfaceCommands.rudder, throttleOutput);

    // 7. Telemetry
    teleCounter++;
    if (teleCounter >= teleUpdate) {
        xQueueSend(g_controls_data_queue, &surfaceCommands, (TickType_t)0);
        teleCounter = 0;
    }
}

void FlightController::estimateAttitude(const IMU::RawData& rawData) {
    const FusionVector gyroscope = {rawData.gyroXDps, rawData.gyroYDps, rawData.gyroZDps};
    const FusionVector accelerometer = {rawData.accelXG, rawData.accelYG, rawData.accelZG};
    //const FusionVector magnetometer = {rawData.magXGauss, rawData.magYGauss, rawData.magZGauss};

    // Update AHRS with Magnetometer
    //FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, loopDt);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, loopDt);

    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    currentRollDeg = euler.angle.roll;
    currentPitchDeg = euler.angle.pitch;
    currentYawDeg = euler.angle.yaw;
}

RC_Channels_t FlightController::getRcData() const {
    return receiver.getCachedData();
}
