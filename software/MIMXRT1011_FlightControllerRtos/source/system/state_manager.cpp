/**
 * @file state_manager.cpp
 * @brief Implements the main state machine controller task.
 */
#include "flight_controller.h"

// Include Task Definitions
#include "system/state_manager.h"
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
#define SENSOR_TASK_PRIORITY            (tskIDLE_PRIORITY + 3)
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

#define SENSOR_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE + 512)
static StackType_t s_sensorStack[SENSOR_TASK_STACK_SIZE];
static StaticTask_t s_sensorTaskTCB;

TaskHandle_t g_sensor_task_handle = nullptr;


void sensorTask(void *pvParameters) {
    FlightController::SensorData data;
    while (true) {
        if (g_flightController.readSensors(data) == 0) {
            xQueueOverwrite(g_sensor_data_queue, &data);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static FlightState_t checkGestures(const Receiver::StickInput& s);

// --- State Loop Helpers ---
static void runFlightState() {
    const TickType_t xFlightLoopFrequency = pdMS_TO_TICKS(FlightController::LOOP_DT_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    static uint32_t s_disarmStart = 0;

    g_heartbeat_frequency = pdMS_TO_TICKS(250); // 2Hz

    while (g_flight_state == STATE_FLIGHT) {
        vTaskDelayUntil(&xLastWakeTime, xFlightLoopFrequency);

        // --- Disarm Check (Throttle Low + Yaw Left) ---
        Receiver::StickInput s;
        g_flightController.getStickInput(s);
        if (s.throttle < 5.0f && s.yaw < -0.9f) {
             if (s_disarmStart == 0) s_disarmStart = xTaskGetTickCount();
             else if ((xTaskGetTickCount() - s_disarmStart) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
                 FlightState_t next = STATE_IDLE;
                 xQueueSend(g_state_change_request_queue, &next, 0);
                 s_disarmStart = 0;
             }
        } else {
            s_disarmStart = 0;
        }

        // Use the global controller instance
        USER_TIMING_ON();
        g_flightController.update();
        USER_TIMING_OFF();
        
        // Check for state change requests
        FlightState_t requestedState;
        if (xQueuePeek(g_state_change_request_queue, &requestedState, 0) == pdTRUE) {
            if (requestedState != g_flight_state) return;
        }
    }
}

static void runIdleState() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    g_heartbeat_frequency = pdMS_TO_TICKS(500); // 1Hz

    while (g_flight_state == STATE_IDLE) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FlightController::LOOP_DT_MS));

        // Run Controller (Keeps AHRS updated, sensors read, telemetry sending)
        g_flightController.update();

        // Check Gestures
        Receiver::StickInput sticks;
        g_flightController.getStickInput(sticks);
        FlightState_t nextState = checkGestures(sticks);

        if (nextState != STATE_IDLE) {
            xQueueSend(g_state_change_request_queue, &nextState, 0);
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for switch
        }
        
        // Check for state change requests
        FlightState_t requestedState;
        if (xQueuePeek(g_state_change_request_queue, &requestedState, 0) == pdTRUE) {
             if (requestedState != g_flight_state) return;
        }
    }
}

static void runCalibrateState() {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t entryTime = xTaskGetTickCount();
    const uint32_t TIMEOUT = pdMS_TO_TICKS(60000); // 60s
    
    g_heartbeat_frequency = pdMS_TO_TICKS(50); // Fast blink

    while (g_flight_state == STATE_CALIBRATE) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));

        g_flightController.update();

        // Interactive Calibration Step
        g_flightController.calibrateMagnetometerStep();

        // Timeout Check
        bool timedOut = (xTaskGetTickCount() - entryTime) > TIMEOUT;

        if (timedOut) {
            FlightState_t newState = STATE_IDLE;
            xQueueSend(g_state_change_request_queue, &newState, 0);
            entryTime = xTaskGetTickCount(); // Reset
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        // Check for state change requests
        FlightState_t requestedState;
        if (xQueuePeek(g_state_change_request_queue, &requestedState, 0) == pdTRUE) {
             if (requestedState != g_flight_state) return;
        }
    }
}

static void runFailsafeState() {
    g_heartbeat_frequency = pdMS_TO_TICKS(125);
    vTaskSuspend(g_command_handler_task_handle);
    vTaskSuspend(g_logging_task_handle); // Optional but good for safety
    
    while (g_flight_state == STATE_FAILSAFE) {
        // Blink fast, minimal activity
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


/**
 * @brief Main state machine task.
 * Handles state transitions and dispatches to specific state loops.
 */
void stateManagerTask(void *pvParameters) {

    // --- BOOT SEQUENCE ---
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
    g_sensor_task_handle = xTaskCreateStatic(sensorTask, "SensorTask", SENSOR_TASK_STACK_SIZE, NULL, SENSOR_TASK_PRIORITY, s_sensorStack, &s_sensorTaskTCB);

    // --- MAIN STATE LOOP ---
    while (true) {
        // Process any pending state changes in main loop as fallback
        FlightState_t requestedState;
        if (xQueueReceive(g_state_change_request_queue, &requestedState, 0) == pdPASS) {
             g_flight_state = requestedState;
        }

        switch (g_flight_state) {
            case STATE_IDLE:
            	runIdleState();
            	break;
            case STATE_FLIGHT:
            	runFlightState();
            	break;
            case STATE_CALIBRATE:
            	runCalibrateState();
            	break;
            case STATE_FAILSAFE:
            	runFailsafeState();
            	break;
            default:
            	runIdleState();
            	break;
        }
    }
}

// --- Gesture Helper Implementation ---
static uint32_t s_gestureStartTime = 0;
static bool s_gestureActive = false;

// Stick Patterns (Mode 2):
// ARM: Throttle Low + Yaw Right
// CALIBRATE: Throttle Low + Yaw Left + Pitch Up
static FlightState_t checkGestures(const Receiver::StickInput& s) {
    bool thrLow  = (s.throttle < 5.0f);
    bool yawRight = (s.yaw > 0.9f);
    bool yawLeft  = (s.yaw < -0.9f);
    bool pitchUp  = (s.pitch > 0.9f);

    bool isArm = thrLow && yawRight;
    bool isCal = thrLow && yawLeft && pitchUp;

    if (isArm || isCal) {
        uint32_t now = xTaskGetTickCount();
        if (!s_gestureActive) {
            s_gestureStartTime = now;
            s_gestureActive = true;
        } else if ((now - s_gestureStartTime) > pdMS_TO_TICKS(GESTURE_TIME_MS)) {
            s_gestureActive = false; // Reset
            return isArm ? STATE_FLIGHT : STATE_CALIBRATE;
        }
    } else {
        s_gestureActive = false;
    }

    return STATE_IDLE; 
}
