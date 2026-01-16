/**
 * @file flight_controller.h
 * @brief Defines the main flight controller class and subsystems.
 */
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include "fusion.h"
#include "utils.h"

// NXP/CMSIS DSP Library for optimized math on Cortex-M7
#include "arm_math.h"
#include "board.h"
#include "peripherals.h"
#include "rls_mag_calibration.h"
#include "servo_driver.h"

// --- Include Adapters & Concepts ---
#include "drivers/imu_concepts.h"
#include "drivers/lsm6dsox_adapter.hpp"
#include "drivers/lis3mdl_adapter.hpp"

// --- Utility Functions ---
constexpr float degToRad(float deg) { return deg * PI / 180.0f; }
constexpr float radToDeg(float rad) { return rad * 180.0f / PI; }

// Channel Mapping
#define RC_CH_ROLL     0
#define RC_CH_PITCH    1
#define RC_CH_THROTTLE 2
#define RC_CH_YAW      3
#define RC_CH_AUX1     4
#define IBUS_MAX_CHANNELS 14

// IBUS Configuration Constants
#define IBUS_DMA_BUFFER_SIZE    128U
#define IBUS_LPUART_INSTANCE    LPUART4
#define IBUS_LPUART_IRQn        LPUART4_IRQn
#define IBUS_DMA_BASE           DMA0
#define IBUS_DMAMUX_BASE        DMAMUX
#define IBUS_DMA_CHANNEL        0U
#define IBUS_DMA_SOURCE         kDmaRequestMuxLPUART4Rx

// --- Logging Data Structures ---

// Packet Types
typedef enum {
    LOG_TYPE_ATTITUDE = 0x01,
    LOG_TYPE_COMMANDS = 0x02
} LogType_t;

// Payload for Attitude (Packet 1)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} LogAttitude_t;

// Payload for Raw Commands (Packet 2)
typedef struct {
    uint16_t aileron;
    uint16_t elevator;
    uint16_t rudder;
    uint16_t throttle;
} LogCommands_t;

// Unified Queue Item
typedef struct {
    LogType_t type;
    union {
        LogAttitude_t attitude;
        LogCommands_t commands;
    } data;
} LogMessage_t;

typedef struct {
    unsigned short channels[IBUS_MAX_CHANNELS];
} RC_Channels_t;

typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_FLIGHT,
    STATE_FAILSAFE,
    STATE_CALIBRATE
} FlightState_t;

struct FullSensorData {
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    float rollRateDps;
    float pitchRateDps;
    float yawRateDps;
    float trueAirspeedMs;
};

struct ActuatorOutput {
    float aileron;
    float elevator;
    float rudder;
};

// --- Task Prototypes ---
void stateManagerTask(void *pvParameters);
void commandHandlerTask(void *pvParameters);
void idleTask(void *pvParameters);
void flightTask(void *pvParameters);
void calibrateTask(void *pvParameters);
void loggingTask(void *pvParameters);
void heartbeatTask(void *pvParameters);

// --- Controller Components ---

/**
 * @brief PID Controller with Derivative Filtering and Anti-Windup.
 */
class PIDController {
public:
    PIDController(float p, float i, float d, float alpha);
    float calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit);
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float alpha;
    float dTermFiltered;
};

class AttitudeController {
public:
    AttitudeController();
    void setSetpoints(float roll, float pitch);
    ActuatorOutput update(const FullSensorData& sensorData, float dt);
    float getTargetPitchDeg() const { return targetPitchDeg; }
    float getTargetRollDeg() const { return targetRollDeg; }

private:
    float kpRollAngle, kpPitchAngle;
    PIDController rollRateController;
    PIDController pitchRateController;
    PIDController yawRateController;
    float kFfRoll, kFfPitch, kFfYaw;
    float targetRollDeg, targetPitchDeg;
    const float GRAVITY_MS2 = 9.80665f;
    const float NOMINAL_AIRSPEED_FOR_TUNING = 20.0f;
};

/**
 * @brief Policy-Based IMU HAL.
 * Templates resolve at compile time for zero runtime overhead.
 */
template <SixAxisSensor AccelPolicy, MagnetometerSensor MagPolicy>
class IMU {
public:
    struct RawData {
        float gyroXDps, gyroYDps, gyroZDps;
        float accelXG, accelYG, accelZG;
        float magXGauss, magYGauss, magZGauss;
        float airspeedMs;
    };

    IMU() : gyroBiasX(0.0f), gyroBiasY(0.0f), gyroBiasZ(0.0f),
            magCalibrator(0.99f, 500.0f, 0.01f) {
        // Vehicle mapping definition
        vehicleMapping.map_x = LSM6DSOX_AXIS_X_POSITIVE;
        vehicleMapping.map_y = LSM6DSOX_AXIS_Y_NEGATIVE;
        vehicleMapping.map_z = LSM6DSOX_AXIS_Z_NEGATIVE;
    }

    int init() {
        i2c_sync_global_init();
        if (accelDriver.init() != 0) return -1;
        if (magDriver.init() != 0) return -1;
        return 0;
    }

    void calibrateGyro() {
        const int sampleCount = 200;
        float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
        Axis3f rawGyro;
        Axis3f mappedGyro;

        for (int i = 0; i < sampleCount; i++) {
            if (accelDriver.readGyro(rawGyro) == 0) {
                // Manually remap here since we abstracted the driver
                remapAxis(rawGyro, mappedGyro);
                sumX += mappedGyro.x;
                sumY += mappedGyro.y;
                sumZ += mappedGyro.z;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        gyroBiasX = sumX / (float)sampleCount;
        gyroBiasY = sumY / (float)sampleCount;
        gyroBiasZ = sumZ / (float)sampleCount;
    }

    int readData(RawData& rawData) {
        Axis3f rawAcc, rawGyro, rawMag;
        Axis3f mapAcc, mapGyro;

        if (accelDriver.readAccel(rawAcc) != 0 ||
            accelDriver.readGyro(rawGyro) != 0 ||
            magDriver.readMag(rawMag) != 0) {
            return -1;
        }

        remapAxis(rawAcc, mapAcc);
        remapAxis(rawGyro, mapGyro);

        rawData.gyroXDps = mapGyro.x - gyroBiasX;
        rawData.gyroYDps = mapGyro.y - gyroBiasY;
        rawData.gyroZDps = mapGyro.z - gyroBiasZ;
        rawData.accelXG = mapAcc.x;
        rawData.accelYG = mapAcc.y;
        rawData.accelZG = mapAcc.z;
        rawData.airspeedMs = 0.0f;

        // Note: Magnetometer remapping is usually handled by the calibration lib
        // or can be added here if needed.
        rawData.magXGauss = rawMag.x;
        rawData.magYGauss = rawMag.y;
        rawData.magZGauss = rawMag.z;

        return 0;
    }

private:
    AccelPolicy accelDriver;
    MagPolicy magDriver;
    float gyroBiasX, gyroBiasY, gyroBiasZ;
    RlsMagnetometerCalibratorF magCalibrator;
    lsm6dsox_axis_mapping_t vehicleMapping;

    // Helper to replace LSM6DSOX_RemapData
    void remapAxis(const Axis3f& input, Axis3f& output) {
        output.x = getMappedValue(input, vehicleMapping.map_x);
        output.y = getMappedValue(input, vehicleMapping.map_y);
        output.z = getMappedValue(input, vehicleMapping.map_z);
    }

    float getMappedValue(const Axis3f& input, lsm6dsox_axis_source_t source) {
        switch (source) {
            case LSM6DSOX_AXIS_X_POSITIVE: return input.x;
            case LSM6DSOX_AXIS_X_NEGATIVE: return -input.x;
            case LSM6DSOX_AXIS_Y_POSITIVE: return input.y;
            case LSM6DSOX_AXIS_Y_NEGATIVE: return -input.y;
            case LSM6DSOX_AXIS_Z_POSITIVE: return input.z;
            case LSM6DSOX_AXIS_Z_NEGATIVE: return -input.z;
            default: return 0.0f;
        }
    }
};

class Receiver {
public:
    struct Setpoint {
        float rollDeg;
        float pitchDeg;
        float throttle;
    };

    struct StickInput {
        float roll;
        float pitch;
        float yaw;
        float throttle;
    };

    void init();
    void update();
    void getSetpoint(Setpoint& setpoint);
    void getStickInput(StickInput& input);
    uint16_t getChannel(uint8_t channel) const;
    const RC_Channels_t& getCachedData() const;

private:
    RC_Channels_t m_cachedRcData;
};

class Actuators {
public:
    void init();

    /**
     * @brief Sets the normalized output for control surfaces and throttle.
     * @param aileron -1.0 to 1.0
     * @param elevator -1.0 to 1.0
     * @param rudder -1.0 to 1.0
     * @param throttle 0.0 to 100.0
     */
    void setOutputs(float aileron, float elevator, float rudder, float throttle);

    /**
     * @brief Sets the raw output in microseconds (Direct Pass-Through).
     * @param aileronUs Pulse width in microseconds
     * @param elevatorUs Pulse width in microseconds
     * @param rudderUs Pulse width in microseconds
     * @param throttleUs Pulse width in microseconds
     */
    void setRawOutputs(uint16_t aileronUs, uint16_t elevatorUs, uint16_t rudderUs, uint16_t throttleUs);

private:
    firmware::drivers::ServoDriver m_servoDriver;
};

class FlightController {
public:
    enum class ControlMode {
        STABILIZED,
        PASS_THROUGH
    };

    FlightController(float loopTime);
    int init();
    void update();
    void calibrateSensors();
    void setControlMode(ControlMode mode);
    RC_Channels_t getRcData() const;

private:
    void estimateAttitude(const IMU<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData);

    // Concrete instantiation of the Policy-Based IMU
    IMU<Lsm6dsoxAdapter, Lis3mdlAdapter> imu;

    Receiver receiver;
    Actuators actuators;
    AttitudeController attitudeController;
    float loopDt;
    float currentRollDeg, currentPitchDeg, currentYawDeg;
    FusionAhrs ahrs;
    int teleUpdate;
    int teleCounter;
    ControlMode currentControlMode;
};

// --- Globals ---
extern TaskHandle_t g_state_manager_task_handle;
extern TaskHandle_t g_command_handler_task_handle;
extern TaskHandle_t g_idle_task_handle;
extern TaskHandle_t g_flight_task_handle;
extern TaskHandle_t g_calibrate_task_handle;
extern TaskHandle_t g_logging_task_handle;
extern TaskHandle_t g_heartbeat_task_handle;

extern QueueHandle_t g_controls_data_queue;
extern QueueHandle_t g_command_data_queue;
extern QueueHandle_t g_state_change_request_queue;

extern volatile TickType_t g_heartbeat_frequency;

extern volatile FlightState_t g_flight_state;
extern FlightController g_flightController;

#endif // FLIGHT_CONTROLLER_H
