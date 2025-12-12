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

#include "board.h"
#include "peripherals.h"
#include <fsl_debug_console.h>

extern "C" {
#include <lis3mdl.h>
#include <lsm6dsox.h>
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Utility Functions ---

/**
 * @brief Converts degrees to radians.
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
constexpr float degToRad(float deg) { return deg * M_PI / 180.0f; }

/**
 * @brief Converts radians to degrees.
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
constexpr float radToDeg(float rad) { return rad * 180.0f / M_PI; }

#define IBUS_MAX_CHANNELS 14

/**
 * @brief Structure holding raw RC channel data.
 */
typedef struct {
    unsigned short channels[IBUS_MAX_CHANNELS];
} RC_Channels_t;

/**
 * @brief Enumeration of possible flight controller states.
 */
typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_FLIGHT,
    STATE_FAILSAFE,
    STATE_CALIBRATE
} FlightState_t;

/**
 * @brief Raw sensor data directly from the IMU drivers.
 */
typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_raw_t  mag_data;
} sensor_data_raw_t;

/**
 * @brief Sensor data remapped to the vehicle frame.
 */
typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_data_t  mag_data;
} sensor_data_vehicle_t;

/**
 * @brief Processed sensor data used for control loop calculations.
 */
struct FullSensorData {
    float rollDeg;
    float pitchDeg;
    float yawDeg;
    float rollRateDps;
    float pitchRateDps;
    float yawRateDps;
    float trueAirspeedMs;
};

/**
 * @brief Normalized output commands for the control surfaces.
 */
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

/**
 * @brief Task responsible for managing global state transitions.
 * @param pvParameters Task parameters.
 */
void stateManagerTask(void *pvParameters);

/**
 * @brief Task responsible for handling incoming commands (RC/Telemetry).
 * @param pvParameters Task parameters.
 */
void commandHandlerTask(void *pvParameters);

/**
 * @brief Idle task running when no other flight modes are active.
 * @param pvParameters Task parameters.
 */
void idleTask(void *pvParameters);

/**
 * @brief Main flight control loop task.
 * @param pvParameters Task parameters.
 */
void flightTask(void *pvParameters);

/**
 * @brief Sensor calibration task.
 * @param pvParameters Task parameters.
 */
void calibrateTask(void *pvParameters);

/**
 * @brief Data logging task.
 * @param pvParameters Task parameters.
 */
void loggingTask(void *pvParameters);

/**
 * @brief System heartbeat LED task.
 * @param pvParameters Task parameters.
 */
void heartbeatTask(void *pvParameters);

// --- Controller Components ---

/**
 * @brief Proportional-Integral-Derivative controller implementation.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PIDController object.
     * @param p Proportional gain.
     * @param i Integral gain.
     * @param d Derivative gain.
     * @param integralLimit Maximum absolute value for the integral term (anti-windup).
     */
    PIDController(float p, float i, float d, float integralLimit = 1.0f);

    /**
     * @brief Calculates the control output based on error.
     * @param setpoint Desired value.
     * @param currentValue Actual value.
     * @param dt Time delta since last update.
     * @return Calculated control output.
     */
    float calculate(float setpoint, float currentValue, float dt);

    /**
     * @brief Resets the integral accumulator and previous error.
     */
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prevError;
    float integralLimit;
};

/**
 * @brief Cascaded controller for vehicle attitude.
 * * Manages the outer loop (Angle) and inner loop (Rate) PIDs to determine
 * moment commands based on desired roll/pitch angles.
 */
class AttitudeController {
public:
    AttitudeController();

    /**
     * @brief Sets the desired attitude setpoints.
     * @param roll Desired roll angle in degrees.
     * @param pitch Desired pitch angle in degrees.
     */
    void setSetpoints(float roll, float pitch);

    /**
     * @brief Updates the controller to produce actuator outputs.
     * @param sensorData Current state of the vehicle.
     * @param dt Time delta.
     * @return Calculated actuator outputs.
     */
    ActuatorOutput update(const FullSensorData& sensorData, float dt);

    /**
     * @brief Gets the current target pitch.
     * @return Target pitch in degrees.
     */
    float getTargetPitchDeg() const { return targetPitchDeg; }

    /**
     * @brief Gets the current target roll.
     * @return Target roll in degrees.
     */
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
 */
class IMU {
public:
    /**
     * @brief Container for normalized sensor readings.
     */
    struct RawData {
        float gyroXDps, gyroYDps, gyroZDps;
        float accelXG, accelYG, accelZG;
        float airspeedMs;
    };

    lsm6dsox_axis_mapping_t vehicleMapping = {
        .map_x = LSM6DSOX_AXIS_X_POSITIVE,
        .map_y = LSM6DSOX_AXIS_Y_NEGATIVE,
        .map_z = LSM6DSOX_AXIS_Z_NEGATIVE
    };

    /**
     * @brief Initializes the IMU hardware.
     */
    void init();

    /**
     * @brief Reads current data from the sensors.
     * @param rawData Output structure to populate.
     * @return 0 on success, non-zero on error.
     */
    int readData(RawData& rawData);
};

/**
 * @brief Interface for Radio Control Receiver operations.
 */
class Receiver {
public:
    /**
     * @brief Desired control setpoints from the pilot.
     */
    struct Setpoint {
        float rollDeg;
        float pitchDeg;
        float throttle;
    };

    /**
     * @brief Initializes the receiver interface.
     */
    void init();

    /**
     * @brief Retrieves the latest pilot setpoints.
     * @param setpoint Output structure to populate.
     */
    void getSetpoint(Setpoint& setpoint);
};

/**
 * @brief Interface for Actuator (Servo/Motor) control.
 */
class Actuators {
public:
    /**
     * @brief Initializes the actuator hardware (PWM).
     */
    void init();

    /**
     * @brief Sets the physical output for control surfaces.
     * @param aileron Normalized aileron command.
     * @param elevator Normalized elevator command.
     * @param rudder Normalized rudder command.
     * @param throttle Normalized throttle command.
     */
    void setOutputs(float aileron, float elevator, float rudder, float throttle);
};

/**
 * @brief Main Flight Controller logic coordinator.
 */
class FlightController {
public:
    /**
     * @brief Construct a new Flight Controller.
     * @param loopTime The expected control loop duration in seconds.
     */
    FlightController(float loopTime);

    /**
     * @brief Main update step, called cyclically.
     */
    void update();

private:
    /**
     * @brief Estimates current vehicle attitude using Sensor Fusion.
     * @param rawData Raw IMU sensor data.
     */
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
};

#endif // FLIGHT_CONTROLLER_H
