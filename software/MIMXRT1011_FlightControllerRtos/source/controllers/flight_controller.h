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

// Estimator
#include "attitude_ukf.hpp"

// Subsystems
#include "attitude_controller.h"
#include "receiver.h"
#include "actuators.h"
#include "sensor_system.h"
#include "telemetry_manager.h"
#include "settings.h"
#include "state_manager.h"
#include "nmea.h"

// Drivers & Adapters
#include "lsm6dsox_adapter.hpp"
#include "lis3mdl_adapter.hpp"

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

    // --- Loop Timing Configuration ---
    static constexpr float LOOP_RATE_HZ = 100.0f;
    static constexpr float LOOP_DT_S = 1.0f / LOOP_RATE_HZ;
    static constexpr uint32_t LOOP_DT_MS = static_cast<uint32_t>(1000.0f / LOOP_RATE_HZ);

    using SensorData = SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData;

    /**
     * @brief Constructor for the Flight Controller.
     * @param loopTime The execution period of the flight loop in seconds.
     */
    FlightController(float loopTime);

    /**
     * @brief Initializes all subsystems (Sensors, Receiver, Actuators, UKF).
     * @return 0 on success, -1 on failure.
     */
    int init();

    /**
     * @brief Main update loop. Reads sensors, runs estimator, updates controllers.
     */
    void update();

    /**
     * @brief Triggers gyroscope calibration.
     */
    void calibrateSensors();

    /**
     * @brief Sets the flight control mode (Stabilized vs Pass-Through).
     * @param mode The desired ControlMode.
     */
    void setControlMode(ControlMode mode);

    /**
     * @brief Retrieves the latest RC channel data.
     * @return RC_Channels_t struct.
     */
    RC_Channels_t getRcData() const;

    /**
     * @brief Retrieves the latest GPS data.
     * @return GpsData struct.
     */
    const firmware::sensors::GpsData& getGpsData() const { return m_latestGps; }

    /**
     * @brief Performs one step of the interactive magnetometer calibration.
     * @return true if successful.
     */
    bool calibrateMagnetometerStep();

    /**
     * @brief Saves the current magnetometer calibration to settings.
     */
    void saveCalibration();

    /**
     * @brief Gets the current normalized stick input.
     * @param input Reference to StickInput struct to fill.
     */
    void getStickInput(Receiver::StickInput& input);

    /**
     * @brief Reads the latest sensor data into the provided structure.
     * @param data Reference to store the data.
     * @return 0 on success.
     */
    int readSensors(SensorData& data);

private:
    /**
     * @brief Runs the Attitude Estimator (UKF).
     * @param rawData The raw sensor data from the SensorSystem.
     */
    void estimateAttitude(const SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData& rawData);

    // Subsystem Composition
    SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter> sensorSystem;
	Receiver receiver;
	Actuators actuators;
	AttitudeController attitudeController;
	TelemetryManager telemetry;

	// Estimator State (UKF)
	gnc::AttitudeUkf ukf;
	gnc::AttitudeUkf::Vector3 magRefVector; // Local magnetic field reference

	// State and Timing Variables
	float loopDt;
	float currentRollDeg;
	float currentPitchDeg;
	float currentYawDeg;
	ControlMode currentControlMode;
	uint32_t lastSensorTimestamp;
	int staleDataCounter;
	uint32_t lastUpdateTick;
	QueueHandle_t sensorQueue;
	
	firmware::sensors::GpsData m_latestGps;
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
extern QueueHandle_t g_sensor_data_queue;

extern volatile TickType_t g_heartbeat_frequency;
extern volatile FlightState_t g_flight_state;
extern FlightController g_flightController;

#endif // FLIGHT_CONTROLLER_H
