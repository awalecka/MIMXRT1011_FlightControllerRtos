/**
 * @file    main.cpp
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
#include "command_handler.h"
#include <vector>

// Include the user-provided C driver header
extern "C" {
#include "i2c_sync.h"
}

// --- Global Variable Definitions ---
// These are the actual memory definitions for the externs in flight_controller.h
volatile FlightState_t g_flight_state = STATE_BOOT;
lsm6dsox_handle_t g_sensor_handle = {0};
lis3mdl_handle_t g_mag_handle = {0};

// Main State Manager Handle
TaskHandle_t g_state_manager_task_handle = NULL;

// Queue Handles (Definitions restored here)
QueueHandle_t g_sensor_data_queue = NULL;
QueueHandle_t g_controls_data_queue = NULL;
QueueHandle_t g_command_data_queue = NULL;
QueueHandle_t g_state_change_request_queue = NULL;

volatile TickType_t g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

// --- Queue Static Allocation ---

// Sensor Data Queue
#define SENSOR_QUEUE_LENGTH 1
#define SENSOR_QUEUE_ITEM_SIZE sizeof(sensor_data_vehicle_t)
static StaticQueue_t xSensorQueueControlBlock;
static uint8_t ucSensorQueueStorageArea[SENSOR_QUEUE_LENGTH * SENSOR_QUEUE_ITEM_SIZE];

// Controls Data Queue
#define CONTROLS_QUEUE_LENGTH 1
#define CONTROLS_QUEUE_ITEM_SIZE sizeof(ActuatorOutput)
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

// State Manager Task Allocation
#define STATE_MANAGER_TASK_PRIORITY     (tskIDLE_PRIORITY + 4)
#define STATE_MGR_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xStateMgrStack[STATE_MGR_STACK_SIZE];
static StaticTask_t xStateMgrTaskControlBlock;

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

    // --- 1. Create Queues ---
    g_sensor_data_queue = xQueueCreateStatic(SENSOR_QUEUE_LENGTH, SENSOR_QUEUE_ITEM_SIZE, ucSensorQueueStorageArea, &xSensorQueueControlBlock);
    g_controls_data_queue = xQueueCreateStatic(CONTROLS_QUEUE_LENGTH, CONTROLS_QUEUE_ITEM_SIZE, ucControlsQueueStorageArea, &xControlsQueueControlBlock);
    g_command_data_queue = xQueueCreateStatic(COMMAND_QUEUE_LENGTH, COMMAND_QUEUE_ITEM_SIZE, ucCommandQueueStorageArea, &xCommandQueueControlBlock);
    g_state_change_request_queue = xQueueCreateStatic(STATE_CHANGE_QUEUE_LENGTH, STATE_CHANGE_QUEUE_ITEM_SIZE, ucStateChangeQueueStorageArea, &xStateChangeQueueControlBlock);

    if (g_sensor_data_queue == NULL || g_controls_data_queue == NULL || g_command_data_queue == NULL || g_state_change_request_queue == NULL) {
        PRINTF("FATAL: Failed to create one or more queues.\r\n");
        while(1);
    }

    // --- 2. Create the State Manager Task ---
    g_state_manager_task_handle = xTaskCreateStatic(state_manager_task, "StateMgrTask", STATE_MGR_STACK_SIZE, NULL, STATE_MANAGER_TASK_PRIORITY, xStateMgrStack, &xStateMgrTaskControlBlock);
    if (g_state_manager_task_handle == NULL) {
        PRINTF("FATAL: Failed to create state manager task.\r\n");
        while(1);
    }

    // --- 3. Start Scheduler ---
    vTaskStartScheduler();

    // Should not reach here
    while(1);

    return 0;
}
