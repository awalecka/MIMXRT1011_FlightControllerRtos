/**
 * @file state_manager.cpp
 * @brief Implements the main state machine controller task.
 */
#include "flight_controller.h"

// Include Task Definitions
#include "system/state_manager.h"
#include "system/state_tasks.h"
#include "system/heartbeat_task.h"
#include "system/logging_task.h"
#include "radio/command_handler.h"

// --- Task Handle Definitions ---
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

// --- Static Allocation ---
#define CMD_HANDLER_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
static StackType_t xCmdHandlerStack[CMD_HANDLER_STACK_SIZE];
static StaticTask_t xCmdHandlerTaskControlBlock;

#define HEARTBEAT_STACK_SIZE (configMINIMAL_STACK_SIZE)
static StackType_t xHeartbeatStack[HEARTBEAT_STACK_SIZE];
static StaticTask_t xHeartbeatTaskControlBlock;

#define LOGGING_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t xLoggingStack[LOGGING_STACK_SIZE];
static StaticTask_t xLoggingTaskControlBlock;

#define IDLE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 128)
static StackType_t xIdleStack[IDLE_TASK_STACK_SIZE];
static StaticTask_t xIdleTaskTCB;

#define FLIGHT_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 1024)
static StackType_t xFlightStack[FLIGHT_TASK_STACK_SIZE];
static StaticTask_t xFlightTaskTCB;

#define CALIBRATE_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 256)
static StackType_t xCalibrateStack[CALIBRATE_TASK_STACK_SIZE];
static StaticTask_t xCalibrateTaskTCB;


void stateManagerTask(void *pvParameters) {

    // --- BOOT SEQUENCE ---
    volatile FlightState_t last_known_state = STATE_BOOT;
    g_flight_state = STATE_BOOT;

    // Use the FlightController to initialize all hardware
    if (g_flightController.init() != 0) {
        g_flight_state = STATE_FAILSAFE;
    } else {
        g_flight_state = STATE_IDLE;
    }

    // --- TASK CREATION ---
    g_heartbeat_task_handle = xTaskCreateStatic(heartbeatTask, "HeartbeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, xHeartbeatStack, &xHeartbeatTaskControlBlock);
    g_command_handler_task_handle = xTaskCreateStatic(commandHandlerTask, "CommandTask", CMD_HANDLER_STACK_SIZE, NULL, COMMAND_HANDLER_TASK_PRIORITY, xCmdHandlerStack, &xCmdHandlerTaskControlBlock);
    g_logging_task_handle = xTaskCreateStatic(loggingTask, "LoggingTask", LOGGING_STACK_SIZE, NULL, LOGGING_TASK_PRIORITY, xLoggingStack, &xLoggingTaskControlBlock);

    g_idle_task_handle = xTaskCreateStatic(idleTask, "IdleTask", IDLE_TASK_STACK_SIZE, NULL, IDLE_TASK_PRIORITY, xIdleStack, &xIdleTaskTCB);
    vTaskSuspend(g_idle_task_handle);

    g_flight_task_handle = xTaskCreateStatic(flightTask, "FlightTask", FLIGHT_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, xFlightStack, &xFlightTaskTCB);
    vTaskSuspend(g_flight_task_handle);

    g_calibrate_task_handle = xTaskCreateStatic(calibrateTask, "CalibrateTask", CALIBRATE_TASK_STACK_SIZE, NULL, FLIGHT_TASK_PRIORITY, xCalibrateStack, &xCalibrateTaskTCB);
    vTaskSuspend(g_calibrate_task_handle);

    // --- MAIN STATE LOOP ---
    // Handle the initial state set by the boot process
    if (last_known_state != g_flight_state) {
        switch (g_flight_state) {
            case STATE_IDLE: vTaskResume(g_idle_task_handle); break;
            case STATE_FAILSAFE:
                g_heartbeat_frequency = pdMS_TO_TICKS(125);
                vTaskSuspend(g_command_handler_task_handle);
                break;
            default: break;
        }
        last_known_state = g_flight_state;
    }

    FlightState_t requested_state;
    while (true) {
        if (xQueueReceive(g_state_change_request_queue, &requested_state, portMAX_DELAY) == pdPASS) {
            if (requested_state == g_flight_state) continue;

            // Suspend current
            switch (last_known_state) {
                case STATE_IDLE:      vTaskSuspend(g_idle_task_handle); break;
                case STATE_FLIGHT:    vTaskSuspend(g_flight_task_handle); break;
                case STATE_CALIBRATE: vTaskSuspend(g_calibrate_task_handle); break;
                default: break;
            }

            g_flight_state = requested_state;

            // Resume next
            switch (g_flight_state) {
                case STATE_IDLE:
                    vTaskResume(g_idle_task_handle);
                    g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz
                    break;
                case STATE_FLIGHT:
                    vTaskResume(g_flight_task_handle);
                    g_heartbeat_frequency = pdMS_TO_TICKS(250); // 2Hz
                    break;
                case STATE_CALIBRATE:
                    vTaskResume(g_calibrate_task_handle);
                    g_heartbeat_frequency = pdMS_TO_TICKS(50); // Fast blink for calibration
                    break;
                case STATE_FAILSAFE:
                    g_heartbeat_frequency = pdMS_TO_TICKS(125);
                    vTaskSuspend(g_command_handler_task_handle);
                    vTaskSuspend(g_logging_task_handle);
                    break;
                default: break;
            }
            last_known_state = g_flight_state;
        }
    }
}
