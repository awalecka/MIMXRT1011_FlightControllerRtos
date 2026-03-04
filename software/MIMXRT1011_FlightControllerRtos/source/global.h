/*
 * global.h
 *
 *  Created on: Feb 27, 2026
 *      Author: Andrew
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

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

// Queue Handles
QueueHandle_t g_controls_data_queue = NULL;
QueueHandle_t g_command_data_queue = NULL;
QueueHandle_t g_state_change_request_queue = NULL;
QueueHandle_t g_imu_data_queue = NULL;
QueueHandle_t g_mag_data_queue = NULL;
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

// Sensor Data Queues
#define IMU_QUEUE_LENGTH 1
#define IMU_QUEUE_ITEM_SIZE sizeof(SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::ImuData)
static StaticQueue_t xImuQueueControlBlock;
static uint8_t ucImuQueueStorageArea[IMU_QUEUE_LENGTH * IMU_QUEUE_ITEM_SIZE];

#define MAG_QUEUE_LENGTH 1
#define MAG_QUEUE_ITEM_SIZE sizeof(SensorSystem<Lsm6dsoxAdapter, Lis3mdlAdapter>::MagData)
static StaticQueue_t xMagQueueControlBlock;
static uint8_t ucMagQueueStorageArea[MAG_QUEUE_LENGTH * MAG_QUEUE_ITEM_SIZE];

// GPS Queues
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

#define IMU_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t s_imuStack[IMU_TASK_STACK_SIZE];
static StaticTask_t s_imuTaskTCB;
static TaskHandle_t s_imuTaskHandle = NULL;

#define MAG_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
static StackType_t s_magStack[MAG_TASK_STACK_SIZE];
static StaticTask_t s_magTaskTCB;
static TaskHandle_t s_magTaskHandle = NULL;

#define GPS_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
static StackType_t s_gpsStack[GPS_TASK_STACK_SIZE];
static StaticTask_t s_gpsTaskControlBlock;

#define SENSOR_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
#define COMMAND_HANDLER_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define GPS_TASK_PRIORITY               (tskIDLE_PRIORITY + 2)
#define LOGGING_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

#endif /* GLOBAL_H_ */
