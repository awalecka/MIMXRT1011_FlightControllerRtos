/**
 * @file    MIMXRT1011_FlightControllerRtos.cpp
 * @brief   Application entry point.
 */

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "peripherals.h"

// Include the user-provided C driver header
extern "C" {
#include "LSM6DSOX.h"
#include "i2c_sync.h"
#include "command_handler.h"
}

// --- Global Variable Definitions ---
volatile FlightState_t g_flight_state = STATE_BOOT;
lsm6dsox_handle_t g_sensor_handle = {0};
lis3mdl_handle_t g_mag_handle = {0};

TaskHandle_t g_state_manager_task_handle = NULL;
TaskHandle_t g_command_handler_task_handle = NULL;
TaskHandle_t g_heartbeat_task_handle = NULL;

TaskHandle_t g_idle_task_handle = NULL;
TaskHandle_t g_flight_task_handle = NULL;
TaskHandle_t g_calibrate_task_handle = NULL;
TaskHandle_t g_logging_task_handle = NULL;
QueueHandle_t g_sensor_data_queue = NULL;

// Task priorities
#define STATE_MANAGER_TASK_PRIORITY     (tskIDLE_PRIORITY + 4)
#define FLIGHT_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
#define COMMAND_HANDLER_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define LOGGING_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define IDLE_TASK_PRIORITY              (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

// --- Static Allocation for Tasks and Queues ---

// Sensor Data Queue
#define SENSOR_QUEUE_LENGTH 50
#define SENSOR_QUEUE_ITEM_SIZE sizeof(sensor_log_data_vehicle_t)
static StaticQueue_t xSensorQueueControlBlock;
static uint8_t ucSensorQueueStorageArea[SENSOR_QUEUE_LENGTH * SENSOR_QUEUE_ITEM_SIZE];

// State Manager Task
#define STATE_MGR_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xStateMgrStack[STATE_MGR_STACK_SIZE];
static StaticTask_t xStateMgrTaskControlBlock;

// Command Handler Task
#define CMD_HANDLER_STACK_SIZE (configMINIMAL_STACK_SIZE + 128)
static StackType_t xCmdHandlerStack[CMD_HANDLER_STACK_SIZE];
static StaticTask_t xCmdHandlerTaskControlBlock;

// Heartbeat Task
#define HEARTBEAT_STACK_SIZE (configMINIMAL_STACK_SIZE)
static StackType_t xHeartbeatStack[HEARTBEAT_STACK_SIZE];
static StaticTask_t xHeartbeatTaskControlBlock;

// Logging Task
#define LOGGING_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xLoggingStack[LOGGING_STACK_SIZE];
static StaticTask_t xLoggingTaskControlBlock;

// --- Static Allocation for State-Specific Tasks ---

// Idle Task
#define IDLE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 128)
static StackType_t xIdleStack[IDLE_TASK_STACK_SIZE];
static StaticTask_t xIdleTaskTCB;

// Flight Task
#define FLIGHT_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
static StackType_t xFlightStack[FLIGHT_TASK_STACK_SIZE];
static StaticTask_t xFlightTaskTCB;

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    PRINTF("Flight Controller Initializing...\r\n");

    // Create the heartbeat task using static allocation
    g_heartbeat_task_handle = xTaskCreateStatic(heartbeat_task, "HeartbeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, xHeartbeatStack, &xHeartbeatTaskControlBlock);
    if (g_heartbeat_task_handle == NULL) {
        PRINTF("Failed to create heartbeat task.\r\n");
        return -1;
    }

    // Create the queue for logging sensor data using static allocation.
    g_sensor_data_queue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE, ucSensorQueueStorageArea, &xSensorQueueControlBlock);
    if (g_sensor_data_queue == NULL) {
        PRINTF("Failed to create sensor data queue.\r\n");
        return -1;
    }

    // Create the main state manager task using static allocation
    g_state_manager_task_handle = xTaskCreateStatic(state_manager_task, "StateMgrTask", STATE_MGR_STACK_SIZE, NULL, STATE_MANAGER_TASK_PRIORITY, xStateMgrStack, &xStateMgrTaskControlBlock);
    if (g_state_manager_task_handle == NULL) {
        PRINTF("Failed to create state manager task.\r\n");
        return -1;
    }

    // Create the command handler task using static allocation
    g_command_handler_task_handle = xTaskCreateStatic(command_handler_task, "CommandTask", CMD_HANDLER_STACK_SIZE, NULL, COMMAND_HANDLER_TASK_PRIORITY, xCmdHandlerStack, &xCmdHandlerTaskControlBlock);
    if (g_command_handler_task_handle == NULL) {
        PRINTF("Failed to create command handler task.\r\n");
        return -1;
    }


    // Create the logging task
    g_logging_task_handle = xTaskCreateStatic(logging_task, "LoggingTask", LOGGING_STACK_SIZE, NULL, LOGGING_TASK_PRIORITY, xLoggingStack, &xLoggingTaskControlBlock);
    if (g_logging_task_handle == NULL) {
        PRINTF("Failed to create logging task.\r\n");
        return -1;
    }

    // Create state-specific tasks using static allocation, but keep them suspended initially.
    g_idle_task_handle = xTaskCreateStatic(idle_task, "IdleTask", IDLE_TASK_STACK_SIZE, NULL, IDLE_TASK_PRIORITY, xIdleStack, &xIdleTaskTCB);
    if (g_idle_task_handle == NULL) {
        PRINTF("Failed to create idle task.\r\n");
        return -1;
    }

    g_flight_task_handle = xTaskCreateStatic(flight_task, "FlightTask", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, xFlightStack, &xFlightTaskTCB);
    if (g_flight_task_handle == NULL) {
        PRINTF("Failed to create flight task.\r\n");
        return -1;
    }
    vTaskSuspend(g_idle_task_handle);
    vTaskSuspend(g_flight_task_handle);

    vTaskStartScheduler();

    // Should not reach here
    while(1);

    return 0;
}
