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
#include "servo_driver.h" //

extern "C" {
#include <lis3mdl.h>
#include <lsm6dsox.h>
}

// --- Utility Functions ---
constexpr float degToRad(float deg) { return deg * PI / 180.0f; }
constexpr float radToDeg(float rad) { return rad * 180.0f / PI; }

// Channel Mapping (Assumes AETR: 0=Roll, 1=Pitch, 2=Throttle, 3=Yaw)
#define RC_CH_ROLL     0
#define RC_CH_PITCH    1
#define RC_CH_THROTTLE 2
#define RC_CH_YAW      3
#define RC_CH_AUX1     4 // Switch for Mode Selection
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

typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_raw_t  mag_data;
} sensor_data_raw_t;

typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_data_t  mag_data;
} sensor_data_vehicle_t;

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

class IMU {
public:
    struct RawData {
        float gyroXDps, gyroYDps, gyroZDps;
        float accelXG, accelYG, accelZG;
        float magXGauss, magYGauss, magZGauss;
        float airspeedMs;
    };

    lsm6dsox_axis_mapping_t vehicleMapping = {
        .map_x = LSM6DSOX_AXIS_X_POSITIVE,
        .map_y = LSM6DSOX_AXIS_Y_NEGATIVE,
        .map_z = LSM6DSOX_AXIS_Z_NEGATIVE
    };

    IMU();
    int init();
    int readData(RawData& rawData);
    void calibrateGyro();

private:
    float gyroBiasX, gyroBiasY, gyroBiasZ;
    RlsMagnetometerCalibratorF magCalibrator;
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
    void estimateAttitude(const IMU::RawData& rawData);

    IMU imu;
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
extern lsm6dsox_handle_t g_sensor_handle;
extern lis3mdl_handle_t g_mag_handle;

extern TaskHandle_t g_state_manager_task_handle;
extern TaskHandle_t g_command_handler_task_handle;
extern TaskHandle_t g_idle_task_handle;
extern TaskHandle_t g_flight_task_handle;
extern TaskHandle_t g_calibrate_task_handle;
extern TaskHandle_t g_logging_task_handle;
extern TaskHandle_t g_heartbeat_task_handle;

extern QueueHandle_t g_sensor_data_queue;
extern QueueHandle_t g_controls_data_queue;
extern QueueHandle_t g_command_data_queue;
extern QueueHandle_t g_state_change_request_queue;

extern volatile TickType_t g_heartbeat_frequency;

extern volatile FlightState_t g_flight_state;
extern FlightController g_flightController;

#endif // FLIGHT_CONTROLLER_H
