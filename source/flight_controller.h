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
#include "fusion.h" // FusionAhrs

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

// IBUS Configuration Constants
#define IBUS_DMA_BUFFER_SIZE    128U
#define IBUS_LPUART_INSTANCE    LPUART4
#define IBUS_LPUART_IRQn        LPUART4_IRQn
#define IBUS_DMA_BASE           DMA0
#define IBUS_DMAMUX_BASE        DMAMUX
#define IBUS_DMA_CHANNEL        0U
#define IBUS_DMA_SOURCE         kDmaRequestMuxLPUART4Rx

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

    FusionAhrs ahrs;

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
