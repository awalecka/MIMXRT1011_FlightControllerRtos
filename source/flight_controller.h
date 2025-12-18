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
#include "fsl_debug_console.h"
#include "rls_mag_calibration.h"

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
#define IBUS_LPUART_INSTANCE    LPUART1
#define IBUS_LPUART_IRQn        LPUART1_IRQn
#define IBUS_DMA_BASE           DMA0
#define IBUS_DMAMUX_BASE        DMAMUX
#define IBUS_DMA_CHANNEL        0U
#define IBUS_DMA_SOURCE         kDmaRequestMuxLPUART1Rx

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

// --- Globals ---
extern volatile FlightState_t g_flight_state;
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
    /**
     * @brief Construct a new PIDController object.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param alpha Derivative Low Pass Filter coefficient (precomputed).
     * alpha = dt / (tau + dt), where tau = 1 / (2*pi*cutoff).
     */
    PIDController(float p, float i, float d, float alpha);

    /**
     * @brief Calculates PID output with Conditional Integration and Derivative Filtering.
     * @param setpoint Desired value.
     * @param currentValue Measured value.
     * @param dt Delta time in seconds (still needed for I and raw D term).
     * @param minLimit Lower saturation limit of the actuator.
     * @param maxLimit Upper saturation limit of the actuator.
     * @return float Clamped control output.
     */
    float calculate(float setpoint, float currentValue, float dt, float minLimit, float maxLimit);

    /**
     * @brief Resets the internal state (integral and previous error).
     */
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prevError;

    // Derivative Filter State
    float alpha;         ///< Precomputed filter coefficient
    float dTermFiltered; ///< Previous filtered derivative value
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
 * @brief Interface for Inertial Measurement Unit operations.
 * Handles driver initialization, remapping, and calibration (bias/iron).
 */
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

    /**
     * @brief Initializes the IMU hardware drivers.
     * @return 0 on success, non-zero on failure.
     */
    int init();

    /**
     * @brief Reads sensor data, applies bias corrections and runs Mag calibration.
     * @param rawData Output structure.
     * @return 0 on success, non-zero on error.
     */
    int readData(RawData& rawData);

    /**
     * @brief Blocking routine to calculate gyro bias.
     * Assumes vehicle is stationary.
     */
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
        float roll;     // Normalized -1.0 to 1.0
        float pitch;    // Normalized -1.0 to 1.0
        float yaw;      // Normalized -1.0 to 1.0
        float throttle; // Normalized 0.0 to 100.0 (or 0.0 to 1.0)
    };

    void init();

    /**
     * @brief Reads the latest data from the queue and updates the internal cache.
     * This should be called once per flight loop iteration.
     */
    void update();

    /**
     * @brief Gets the setpoint for stabilized modes (Angles).
     * @param setpoint Output structure.
     */
    void getSetpoint(Setpoint& setpoint);

    /**
     * @brief Gets normalized stick inputs for pass-through modes.
     * @param input Output structure with normalized values.
     */
    void getStickInput(StickInput& input);

    /**
     * @brief Returns the raw value of a specific channel from the cache.
     * @param channel Index of the channel.
     * @return Channel value in microseconds (typically 1000-2000).
     */
    uint16_t getChannel(uint8_t channel) const;

    /**
     * @brief Returns the entire cached RC data frame.
     * @return Const reference to the internal cache.
     */
    const RC_Channels_t& getCachedData() const;

private:
    RC_Channels_t m_cachedRcData;
};

class Actuators {
public:
    void init();
    void setOutputs(float aileron, float elevator, float rudder, float throttle);
};

class FlightController {
public:
    enum class ControlMode {
        STABILIZED,
        PASS_THROUGH
    };

    FlightController(float loopTime);

    /**
     * @brief Performs full system initialization (sensors, drivers, AHRS).
     * @return 0 on success, non-zero on failure.
     */
    int init();

    void update();
    void calibrateSensors();
    void setControlMode(ControlMode mode);

    /**
     * @brief Retrieves the latest RC channel data.
     * @return A copy of the current RC channel data.
     */
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

// Global instance exposed for tasks (e.g., calibrateTask)
extern FlightController g_flightController;

#endif // FLIGHT_CONTROLLER_H
