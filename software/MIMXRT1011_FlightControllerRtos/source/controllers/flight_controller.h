/**
 * @file flight_controller.h
 * @brief Coordinator for flight subsystems.
 */
#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "common_types.h"

// --- ESTIMATOR SELECTION ---
// Uncomment the following line to use the UKF (gnc/attitude_ukf.hpp)
// instead of the default Fusion AHRS.
#define ENABLE_UKF_ESTIMATOR
// ---------------------------

#ifdef ENABLE_UKF_ESTIMATOR
#include "attitude_ukf.hpp"
#else
#include "fusion.h" // FusionAhrs
#endif

// Subsystems
#include "controllers/attitude_controller.h"
#include "io/receiver.h"
#include "io/actuators.h"
#include "sensors/sensor_system.h"
#include "telemetry/telemetry_manager.h"
#include "system/settings.h"
#include "system/state_manager.h"

// Drivers & Adapters
#include "drivers/lsm6dsox_adapter.hpp"
#include "drivers/lis3mdl_adapter.hpp"

// Gesture Thresholds
#define GESTURE_STICK_LOW   1050
#define GESTURE_STICK_HIGH  1900
#define GESTURE_TIME_MS     1000

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

    bool calibrateMagnetometerStep();
    void saveCalibration();
    void getStickInput(Receiver::StickInput& input);

private:
    void estimateAttitude(const SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData);

    // Subsystem Composition
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter> sensorSystem;
    Receiver receiver;
    Actuators actuators;
    AttitudeController attitudeController;
    TelemetryManager telemetry;

    // Estimator State
#ifdef ENABLE_UKF_ESTIMATOR
    gnc::AttitudeUkf ukf;
    gnc::AttitudeUkf::Vector3 magRefVector; // Local magnetic field reference
#else
    FusionAhrs ahrs;
#endif

    float loopDt;
    float currentRollDeg, currentPitchDeg, currentYawDeg;
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
