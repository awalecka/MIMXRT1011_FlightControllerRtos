/**
 * @file flight_controller.h
 * @brief Central header file for the flight controller project.
 *
 * This file contains all shared definitions, global variable declarations,
 * and function prototypes used across the different source files.
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "attitudeController.h"

// Include the sensor driver header
extern "C" {
#include "LIS3MDL.h"
#include "LSM6DSOX.h"
}

// --- Flight Controller State Machine ---
typedef enum {
    STATE_BOOT,
    STATE_IDLE,
    STATE_FLIGHT,
    STATE_FAILSAFE,
    STATE_CALIBRATE
} FlightState_t;

// --- Data Structures ---
// Structure to hold a single snapshot of sensor data for the logging queue
typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_raw_t  mag_data;
} sensor_log_data_raw_t;

typedef struct {
    TickType_t timestamp;
    lsm6dsox_3axis_data_t acc_data;
    lsm6dsox_3axis_data_t gyro_data;
    lis3mdl_3axis_data_t  mag_data;
} sensor_log_data_vehicle_t;

// --- Global Handles and Variables (declared as 'extern') ---

// Global state variable
extern volatile FlightState_t g_flight_state;

// Global handle for the sensor
extern lsm6dsox_handle_t g_sensor_handle;
extern lis3mdl_handle_t g_mag_handle;

// Task handles
extern TaskHandle_t g_state_manager_task_handle;
extern TaskHandle_t g_command_handler_task_handle;
extern TaskHandle_t g_idle_task_handle;
extern TaskHandle_t g_flight_task_handle;
extern TaskHandle_t g_calibrate_task_handle;
extern TaskHandle_t g_logging_task_handle;

// Queue handle for logging sensor data
extern QueueHandle_t g_sensor_data_queue;
extern QueueHandle_t g_controls_data_queue;
extern QueueHandle_t g_command_data_queue;

// Global heartbeat
extern volatile TickType_t g_heartbeat_frequency;

// --- Task Function Prototypes ---
void state_manager_task(void *pvParameters);
void command_handler_task(void *pvParameters);
void idle_task(void *pvParameters);
void flight_task(void *pvParameters);
void calibrate_task(void *pvParameters);
void logging_task(void *pvParameters);
void heartbeat_task(void *pvParameters);

#endif // FLIGHT_CONTROLLER_H
