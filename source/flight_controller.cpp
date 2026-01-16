/**
 * @file flight_controller.cpp
 * @brief Implementation of the main flight controller logic.
 */
#include "flight_controller.h"

using namespace firmware::drivers;

// Define the global FlightController instance
FlightController g_flightController(0.01f);

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

// --- Receiver Implementation ---
void Receiver::init() {
    std::fill(std::begin(m_cachedRcData.channels), std::end(m_cachedRcData.channels), 1500);
    m_cachedRcData.channels[RC_CH_THROTTLE] = 1000;
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
    uint16_t rollRaw     = receiver.getChannel(RC_CH_ROLL);
    uint16_t pitchRaw    = receiver.getChannel(RC_CH_PITCH);
    uint16_t yawRaw      = receiver.getChannel(RC_CH_YAW);
    uint16_t throttleRaw = receiver.getChannel(RC_CH_THROTTLE);
    uint16_t aux1Value   = receiver.getChannel(RC_CH_AUX1);

    // Determine Control Mode
    if (aux1Value > 1500) {
        currentControlMode = ControlMode::PASS_THROUGH;
    } else {
        currentControlMode = ControlMode::STABILIZED;
    }

// Read Sensors
    IMU<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData rawSensorData;
    int sensorStatus = imu.readData(rawSensorData);

    // Estimate Attitude (ALWAYS RUN if sensors are okay)
    if (sensorStatus == 0) {
        estimateAttitude(rawSensorData);
    } else if(sensorStatus != 0 && currentControlMode == ControlMode::STABILIZED) {
    	// Failsafe: Force Pass-Through if Sensors Failed
    	currentControlMode = ControlMode::PASS_THROUGH;
    }

    // Prepare Log Messages
    LogMessage_t attMsg;
    attMsg.type = LOG_TYPE_ATTITUDE;
    attMsg.data.attitude.roll = currentRollDeg;
    attMsg.data.attitude.pitch = currentPitchDeg;
    attMsg.data.attitude.yaw = currentYawDeg;

    // Log Raw Inputs
    LogMessage_t cmdMsg;
    cmdMsg.type = LOG_TYPE_COMMANDS;
    cmdMsg.data.commands.aileron = rollRaw;
    cmdMsg.data.commands.elevator = pitchRaw;
    cmdMsg.data.commands.rudder = yawRaw;
    cmdMsg.data.commands.throttle = throttleRaw;

    // Control Logic
    if (currentControlMode == ControlMode::PASS_THROUGH) {
        // --- RAW PASS THROUGH ---
        actuators.setRawOutputs(rollRaw, pitchRaw, yawRaw, throttleRaw);

    } else {
        // --- STABILIZED MODE ---
        ActuatorOutput surfaceCommands = {0.0f, 0.0f, 0.0f};
        float throttleOutput = 0.0f;

        if (sensorStatus == 0) {
             // Attitude estimation already done above

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

    // Send Telemetry
    teleCounter++;
    if (teleCounter >= teleUpdate) {
        // Send both packets
        xQueueSend(g_controls_data_queue, &attMsg, (TickType_t)0);
        xQueueSend(g_controls_data_queue, &cmdMsg, (TickType_t)0);
        teleCounter = 0;
    }
}

void FlightController::estimateAttitude(const IMU<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData) {
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
