#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <vector>
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
constexpr float degToRad(float deg) { return deg * M_PI / 180.0f; }
constexpr float radToDeg(float rad) { return rad * 180.0f / M_PI; }

#define IBUS_MAX_CHANNELS 14

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
void state_manager_task(void *pvParameters);
void command_handler_task(void *pvParameters);
void idle_task(void *pvParameters);
void flight_task(void *pvParameters);
void calibrate_task(void *pvParameters);
void logging_task(void *pvParameters);
void heartbeat_task(void *pvParameters);

// --- Controller Components ---

class PIDController {
public:
    PIDController(float p, float i, float d);
    float calculate(float setpoint, float currentValue, float dt);
    void reset();

private:
    float kp, ki, kd;
    float integral;
    float prevError;
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
        float airspeedMs;
    };

    lsm6dsox_axis_mapping_t vehicleMapping = {
        .map_x = LSM6DSOX_AXIS_X_POSITIVE,
        .map_y = LSM6DSOX_AXIS_Y_NEGATIVE,
        .map_z = LSM6DSOX_AXIS_Z_NEGATIVE
    };

    void init();
    int readData(RawData& rawData);
};

class Receiver {
public:
    struct Setpoint {
        float rollDeg;
        float pitchDeg;
        float throttle;
    };
    void init();
    void getSetpoint(Setpoint& setpoint);
};

class Actuators {
public:
    void init();
    void setOutputs(float aileron, float elevator, float rudder, float throttle);
};

class FlightController {
public:
    FlightController(float loopTime);
    void update();

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
};

#endif // FLIGHT_CONTROLLER_H
