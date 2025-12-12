/**
 * @file state_manager.cpp
 * @brief Implements the main state machine controller task.
 */

#include "flight_controller.h"
#include "fsl_debug_console.h"
#include "Driver_I2C.h"
#include "peripherals.h"
#include "i2c_sync.h"

// CMSIS I2C Driver Instance
extern ARM_DRIVER_I2C LPI2C1_SENSORS_CMSIS_DRIVER;

// --- Task Handle Definitions ---
// These must be defined here (or in any single .cpp) to resolve the externs in flight_controller.h
TaskHandle_t g_heartbeat_task_handle = NULL;
TaskHandle_t g_command_handler_task_handle = NULL;
TaskHandle_t g_logging_task_handle = NULL;
TaskHandle_t g_idle_task_handle = NULL;
TaskHandle_t g_flight_task_handle = NULL;
TaskHandle_t g_calibrate_task_handle = NULL;

// --- Task Priorities ---
#define FLIGHT_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
#define COMMAND_HANDLER_TASK_PRIORITY   (tskIDLE_PRIORITY + 2)
#define LOGGING_TASK_PRIORITY           (tskIDLE_PRIORITY + 1)
#define IDLE_TASK_PRIORITY              (tskIDLE_PRIORITY + 1)
#define HEARTBEAT_TASK_PRIORITY         (tskIDLE_PRIORITY + 1)

// --- Static Allocation for Subordinate Tasks ---

// Command Handler Task
#define CMD_HANDLER_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
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

// Idle Task
#define IDLE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 128)
static StackType_t xIdleStack[IDLE_TASK_STACK_SIZE];
static StaticTask_t xIdleTaskTCB;

// Flight Task (Optimized size)
#define FLIGHT_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
static StackType_t xFlightStack[FLIGHT_TASK_STACK_SIZE];
static StaticTask_t xFlightTaskTCB;

// Calibrate Task
#define CALIBRATE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
static StackType_t xCalibrateStack[CALIBRATE_TASK_STACK_SIZE];
static StaticTask_t xCalibrateTaskTCB;


/**
 * @brief Main state machine controller task.
 * Manages the lifecycle of state-specific tasks.
 */
void stateManagerTask(void *pvParameters) {

    // --- BOOT SEQUENCE ---
    PRINTF("State: BOOT\r\n");

    volatile FlightState_t last_known_state = STATE_BOOT;
    g_flight_state = STATE_BOOT;

    i2c_sync_global_init();

    if (LSM6DSOX_Init(&g_sensor_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LSM6DSOX_I2C_ADDR) != 0 ||
        LSM6DSOX_SetAccConfig(&g_sensor_handle, LSM6DSOX_ACC_ODR_52Hz, LSM6DSOX_ACC_FS_2G) != 0 ||
        LSM6DSOX_SetGyroConfig(&g_sensor_handle, LSM6DSOX_GYRO_ODR_52Hz, LSM6DSOX_GYRO_FS_250DPS) != 0 ||
		LIS3MDL_Init(&g_mag_handle, &LPI2C1_SENSORS_CMSIS_DRIVER, LIS3MDL_I2C_ADDR) != 0 ||
		LIS3MDL_SetConfig(&g_mag_handle, LIS3MDL_ODR_80_HZ_HP, LIS3MDL_FS_4_GAUSS) != 0)
    {
        PRINTF("Error: Sensor initialization failed! Entering FAILSAFE.\r\n");
        g_flight_state = STATE_FAILSAFE;
    }
    else
    {
        PRINTF("Sensor initialized successfully.\r\n");
        g_flight_state = STATE_IDLE;
    }

    // --- TASK CREATION ---

    // Create the persistent system tasks
    g_heartbeat_task_handle = xTaskCreateStatic(heartbeatTask, "HeartbeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, xHeartbeatStack, &xHeartbeatTaskControlBlock);
    if (g_heartbeat_task_handle == NULL) { PRINTF("FATAL: Failed to create heartbeat task.\r\n"); }

    g_command_handler_task_handle = xTaskCreateStatic(commandHandlerTask, "CommandTask", CMD_HANDLER_STACK_SIZE, NULL, COMMAND_HANDLER_TASK_PRIORITY, xCmdHandlerStack, &xCmdHandlerTaskControlBlock);
    if (g_command_handler_task_handle == NULL) { PRINTF("FATAL: Failed to create command handler task.\r\n"); }

    g_logging_task_handle = xTaskCreateStatic(loggingTask, "LoggingTask", LOGGING_STACK_SIZE, NULL, LOGGING_TASK_PRIORITY, xLoggingStack, &xLoggingTaskControlBlock);
    if (g_logging_task_handle == NULL) { PRINTF("FATAL: Failed to create logging task.\r\n"); }

    // 2. Create and suspend the state-specific tasks
    g_idle_task_handle = xTaskCreateStatic(idleTask, "IdleTask", IDLE_TASK_STACK_SIZE, NULL, IDLE_TASK_PRIORITY, xIdleStack, &xIdleTaskTCB);
    if (g_idle_task_handle == NULL) { PRINTF("FATAL: Failed to create idle task.\r\n"); }
    vTaskSuspend(g_idle_task_handle);

    g_flight_task_handle = xTaskCreateStatic(flightTask, "FlightTask", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, xFlightStack, &xFlightTaskTCB);
    if (g_flight_task_handle == NULL) { PRINTF("FATAL: Failed to create flight task.\r\n"); }
    vTaskSuspend(g_flight_task_handle);

    g_calibrate_task_handle = xTaskCreateStatic(calibrateTask, "CalibrateTask", CALIBRATE_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, xCalibrateStack, &xCalibrateTaskTCB);
    if (g_calibrate_task_handle == NULL) { PRINTF("FATAL: Failed to create calibrate task.\r\n"); }
    vTaskSuspend(g_calibrate_task_handle);

    // --- END TASK CREATION ---

    // --- Handle the FIRST transition ---
    if (last_known_state != g_flight_state) {
        PRINTF("State transition: %d -> %d\r\n", last_known_state, g_flight_state);

        switch (g_flight_state) {
            case STATE_IDLE:
                vTaskResume(g_idle_task_handle);
                break;
            case STATE_FAILSAFE:
                PRINTF("State: FAILSAFE - System halted. Please reset.\r\n");
                g_heartbeat_frequency = pdMS_TO_TICKS(125);
                vTaskSuspend(g_command_handler_task_handle);
                vTaskSuspend(g_logging_task_handle);
                vTaskSuspend(NULL);
                break;
            default: break;
        }
        last_known_state = g_flight_state;
    }

    // --- Main State-Change Event Loop ---
    FlightState_t requested_state;
    while (1) {
        if (xQueueReceive(g_state_change_request_queue, &requested_state, portMAX_DELAY) == pdPASS) {

            if (requested_state == g_flight_state) {
                continue;
            }

            PRINTF("State transition requested: %d -> %d\r\n", last_known_state, requested_state);

            switch (last_known_state) {
                case STATE_IDLE:      vTaskSuspend(g_idle_task_handle); break;
                case STATE_FLIGHT:    vTaskSuspend(g_flight_task_handle); break;
                case STATE_CALIBRATE: vTaskSuspend(g_calibrate_task_handle); break;
                default: break;
            }

            g_flight_state = requested_state;

            switch (g_flight_state) {
                case STATE_IDLE:
                	vTaskResume(g_idle_task_handle);
                	break;
                case STATE_FLIGHT:
                	vTaskResume(g_flight_task_handle);
                	break;
                case STATE_CALIBRATE:
                	vTaskResume(g_calibrate_task_handle);
                	break;
                case STATE_FAILSAFE:
                    PRINTF("State: FAILSAFE - System halted. Please reset.\r\n");
                	g_heartbeat_frequency = pdMS_TO_TICKS(125);
                    vTaskSuspend(g_command_handler_task_handle);
                    vTaskSuspend(g_logging_task_handle);
                    vTaskSuspend(NULL);
                    break;
                default: break;
            }
            last_known_state = g_flight_state;
        }
    }
}
