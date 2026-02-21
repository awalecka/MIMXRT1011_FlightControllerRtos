/**
 * @file    main.cpp
 * @brief   Application entry point.
 */

#include "pin_mux.h"
#include "command_handler.h"
#include "logging_task.h"
#include "heartbeat_task.h"
#include "flight_controller.h"
#include "gps_handler.h"
#include "nmea.h"

extern "C" {
#include "i2c_sync.h"
}

// --- Global Variable Definitions ---
// Flight State
volatile FlightState_t g_flight_state = STATE_BOOT;

// Task Handles
TaskHandle_t g_heartbeat_task_handle = NULL;
TaskHandle_t g_command_handler_task_handle = NULL;
TaskHandle_t g_logging_task_handle = NULL;
TaskHandle_t g_idle_task_handle = NULL;
TaskHandle_t g_flight_task_handle = NULL;
TaskHandle_t g_calibrate_task_handle = NULL;
TaskHandle_t g_gps_task_handle = NULL;

// Main State Manager Handle
TaskHandle_t g_state_manager_task_handle = NULL;

// Queue Handles (Definitions restored here)
QueueHandle_t g_controls_data_queue = NULL;
QueueHandle_t g_command_data_queue = NULL;
QueueHandle_t g_state_change_request_queue = NULL;
QueueHandle_t g_sensor_data_queue = NULL;
QueueHandle_t g_gps_rx_queue = NULL;
QueueHandle_t g_gps_data_queue = NULL;

// Heartbeat frequency
volatile TickType_t g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

// --- Queue Static Allocation ---

// Controls Data Queue
#define CONTROLS_QUEUE_LENGTH 10
#define CONTROLS_QUEUE_ITEM_SIZE sizeof(LogMessage_t)
static StaticQueue_t xControlsQueueControlBlock;
static uint8_t ucControlsQueueStorageArea[CONTROLS_QUEUE_LENGTH * CONTROLS_QUEUE_ITEM_SIZE];

// Command Data Queue
#define COMMAND_QUEUE_LENGTH 1
#define COMMAND_QUEUE_ITEM_SIZE sizeof(RC_Channels_t)
static StaticQueue_t xCommandQueueControlBlock;
static uint8_t ucCommandQueueStorageArea[COMMAND_QUEUE_LENGTH * COMMAND_QUEUE_ITEM_SIZE];

// State Change Request Queue
#define STATE_CHANGE_QUEUE_LENGTH 2
#define STATE_CHANGE_QUEUE_ITEM_SIZE sizeof(FlightState_t)
static StaticQueue_t xStateChangeQueueControlBlock;
static uint8_t ucStateChangeQueueStorageArea[STATE_CHANGE_QUEUE_LENGTH * STATE_CHANGE_QUEUE_ITEM_SIZE];

// Sensor Data Queue
#define SENSOR_QUEUE_LENGTH 1
#define SENSOR_QUEUE_ITEM_SIZE sizeof(SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::RawData)
static StaticQueue_t xSensorQueueControlBlock;
static uint8_t ucSensorQueueStorageArea[SENSOR_QUEUE_LENGTH * SENSOR_QUEUE_ITEM_SIZE];

// GPS Queues
#define GPS_RX_QUEUE_LENGTH 128
#define GPS_RX_QUEUE_ITEM_SIZE sizeof(uint8_t)
static StaticQueue_t xGpsRxQueueControlBlock;
static uint8_t ucGpsRxQueueStorageArea[GPS_RX_QUEUE_LENGTH * GPS_RX_QUEUE_ITEM_SIZE];

#define GPS_DATA_QUEUE_LENGTH 1
#define GPS_DATA_QUEUE_ITEM_SIZE sizeof(firmware::sensors::GpsData)
static StaticQueue_t xGpsDataQueueControlBlock;
static uint8_t ucGpsDataQueueStorageArea[GPS_DATA_QUEUE_LENGTH * GPS_DATA_QUEUE_ITEM_SIZE];

// State Manager Task Allocation
#define STATE_MANAGER_TASK_PRIORITY     (tskIDLE_PRIORITY + 4)
#define STATE_MGR_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xStateMgrStack[STATE_MGR_STACK_SIZE];
static StaticTask_t xStateMgrTaskControlBlock;

// Other Task Allocations
#define CMD_HANDLER_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
static StackType_t s_cmdHandlerStack[CMD_HANDLER_STACK_SIZE];
static StaticTask_t s_cmdHandlerTaskControlBlock;

#define HEARTBEAT_STACK_SIZE (configMINIMAL_STACK_SIZE)
static StackType_t s_heartbeatStack[HEARTBEAT_STACK_SIZE];
static StaticTask_t s_heartbeatTaskControlBlock;

#define LOGGING_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t s_loggingStack[LOGGING_STACK_SIZE];
static StaticTask_t s_loggingTaskControlBlock;

#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t s_sensorStack[SENSOR_TASK_STACK_SIZE];
static StaticTask_t s_sensorTaskTCB;
static TaskHandle_t s_sensorTaskHandle = NULL;

#define GPS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
static StackType_t s_gpsStack[GPS_TASK_STACK_SIZE];
static StaticTask_t s_gpsTaskControlBlock;

#define SENSOR_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
#define COMMAND_HANDLER_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define GPS_TASK_PRIORITY               (tskIDLE_PRIORITY + 2)
#define LOGGING_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

/**
 * @brief Main Application Entry Point.
 * Initializes the board hardware, sets up FreeRTOS queues and tasks, and starts the scheduler.
 * @return Does not return.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    // Initialize custom hardware for IBUS via BSP
    BOARD_InitIBUS(ibus_idle_interrupt_callback);
    
    // Initialize GPS UART via BSP
    BOARD_InitGPS(gps_rx_callback);

    // Create Queues
    g_controls_data_queue = xQueueCreateStatic(CONTROLS_QUEUE_LENGTH, CONTROLS_QUEUE_ITEM_SIZE, ucControlsQueueStorageArea, &xControlsQueueControlBlock);
    g_command_data_queue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_ITEM_SIZE, ucCommandQueueStorageArea, &xCommandQueueControlBlock);
    g_state_change_request_queue = xQueueCreateStatic(STATE_CHANGE_QUEUE_LENGTH, STATE_CHANGE_QUEUE_ITEM_SIZE, ucStateChangeQueueStorageArea, &xStateChangeQueueControlBlock);
    g_sensor_data_queue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE, ucSensorQueueStorageArea, &xSensorQueueControlBlock);
    g_gps_rx_queue = xQueueCreateStatic(GPS_RX_QUEUE_LENGTH, GPS_RX_QUEUE_ITEM_SIZE, ucGpsRxQueueStorageArea, &xGpsRxQueueControlBlock);
    g_gps_data_queue = xQueueCreateStatic(GPS_DATA_QUEUE_LENGTH, GPS_DATA_QUEUE_ITEM_SIZE, ucGpsDataQueueStorageArea, &xGpsDataQueueControlBlock);

    if (g_controls_data_queue == NULL || g_command_data_queue == NULL || g_state_change_request_queue == NULL || g_sensor_data_queue == NULL || g_gps_rx_queue == NULL || g_gps_data_queue == NULL) {
        while(1);
    }

    // Create the State Manager Task
    g_state_manager_task_handle = xTaskCreateStatic(stateManagerTask, "StateMgrTask", STATE_MGR_STACK_SIZE, NULL, STATE_MANAGER_TASK_PRIORITY, xStateMgrStack, &xStateMgrTaskControlBlock);
    if (g_state_manager_task_handle == NULL) {
        while(1);
    }

    // Create the Other Tasks
    g_heartbeat_task_handle = xTaskCreateStatic(heartbeatTask, "HeartbeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, s_heartbeatStack, &s_heartbeatTaskControlBlock);
    g_command_handler_task_handle = xTaskCreateStatic(commandHandlerTask, "CommandTask", CMD_HANDLER_STACK_SIZE, NULL, COMMAND_HANDLER_TASK_PRIORITY, s_cmdHandlerStack, &s_cmdHandlerTaskControlBlock);
    g_logging_task_handle = xTaskCreateStatic(loggingTask, "LoggingTask", LOGGING_STACK_SIZE, NULL, LOGGING_TASK_PRIORITY, s_loggingStack, &s_loggingTaskControlBlock);
    s_sensorTaskHandle = xTaskCreateStatic(sensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, s_sensorStack, &s_sensorTaskTCB);
    g_gps_task_handle = xTaskCreateStatic(gpsTask, "GpsTask", GPS_TASK_STACK_SIZE, NULL, GPS_TASK_PRIORITY, s_gpsStack, &s_gpsTaskControlBlock);

    // Start Scheduler
    vTaskStartScheduler();

    // Should not reach here
    while(1);

    return 0;
}
